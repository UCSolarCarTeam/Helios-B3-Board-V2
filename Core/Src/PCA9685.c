/*
 * pca9685.c
 *
 *  Created on: Sep 1, 2025
 *      Author: jazebzafar
 */

#include "pca9685.h"
#include <math.h>

// ========== Internal helpers ==========
static HAL_StatusTypeDef wr8(PCA9685_HandleTypeDef *hpca, uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(hpca->i2c, (hpca->addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

HAL_StatusTypeDef rd8(PCA9685_HandleTypeDef *hpca, uint8_t reg, uint8_t *val) {
    return HAL_I2C_Mem_Read(hpca->i2c, (hpca->addr7 << 1) | 0x01, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 100);
}

static HAL_StatusTypeDef wrN(PCA9685_HandleTypeDef *hpca, uint8_t start_reg, const uint8_t *buf, uint16_t len) {
    return HAL_I2C_Mem_Write(hpca->i2c, (hpca->addr7 << 1), start_reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf, len,100);
}

// ========== Public API ==========

HAL_StatusTypeDef PCA9685_Init(PCA9685_HandleTypeDef *hpca, I2C_HandleTypeDef *hi2c, uint8_t addr7) {
    if (!hpca || !hi2c) return HAL_ERROR;
    hpca->i2c   = hi2c;
    hpca->addr7 = addr7;
    hpca->osc_hz = 25000000.0f; // 25 MHz internal osc (typical)

    // MODE1: set AI (auto-increment), clear sleep
    uint8_t mode1 = 0;
    if (rd8(hpca, PCA9685_MODE1, &mode1) != HAL_OK) return HAL_ERROR;
    mode1 &= ~BIT(MODE1_SLEEP_Pos);           // ensure oscillator on
    mode1 |=  BIT(MODE1_AI_Pos);              // auto-increment
    // leave ALLCALL default (1) unless user changes it
    if (wr8(hpca, PCA9685_MODE1, mode1) != HAL_OK) return HAL_ERROR;

    // MODE2: default totem-pole, update on STOP (OCH=0), not inverted
    uint8_t mode2 = 0;
    mode2 |= BIT(MODE2_OUTDRV_Pos);           // totem-pole
    // OUTNE=00 (LEDn=0 when OE=1), INVRT=0, OCH=0
    if (wr8(hpca, PCA9685_MODE2, mode2) != HAL_OK) return HAL_ERROR;

    // Default: 200 Hz (datasheet’s example for PRE_SCALE 0x1E @ 25 MHz)
    // You can call SetPWMFreq() later to change.
    return PCA9685_SetPWMFreq(hpca, 200.0f);
}

HAL_StatusTypeDef PCA9685_SoftwareReset(I2C_HandleTypeDef *hi2c) {
    // SWRST General Call address = 0x00 with data 0x06 (write only)
    // The HAL expects 8-bit address; general call write is 0x00.
    uint8_t swrst = 0x06;
    // Use HAL_I2C_Master_Transmit to 0x00
    return HAL_I2C_Master_Transmit(hi2c, 0x00, &swrst, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef PCA9685_SetOutputMode(PCA9685_HandleTypeDef *hpca, bool totem_pole, bool invert, bool update_on_ack) {
    uint8_t mode2 = 0;
    if (rd8(hpca, PCA9685_MODE2, &mode2) != HAL_OK) return HAL_ERROR;

    if (totem_pole) mode2 |=  BIT(MODE2_OUTDRV_Pos);
    else            mode2 &= ~BIT(MODE2_OUTDRV_Pos);

    if (invert)     mode2 |=  BIT(MODE2_INVRT_Pos);
    else            mode2 &= ~BIT(MODE2_INVRT_Pos);

    if (update_on_ack) mode2 |=  BIT(MODE2_OCH_Pos);
    else               mode2 &= ~BIT(MODE2_OCH_Pos);

    return wr8(hpca, PCA9685_MODE2, mode2);
}

HAL_StatusTypeDef PCA9685_Sleep(PCA9685_HandleTypeDef *hpca, bool sleep) {
    uint8_t mode1 = 0;
    if (rd8(hpca, PCA9685_MODE1, &mode1) != HAL_OK) return HAL_ERROR;

    if (sleep) mode1 |=  BIT(MODE1_SLEEP_Pos);
    else       mode1 &= ~BIT(MODE1_SLEEP_Pos);

    HAL_StatusTypeDef st = wr8(hpca, PCA9685_MODE1, mode1);
    if (st != HAL_OK) return st;

    if (!sleep) {
        // datasheet: allow oscillator to stabilize (max 500 µs) before touching PWM regs
        // (You can tighten if you know actual osc; keep safe.)
        HAL_Delay(1);
    }
    return HAL_OK;
}

HAL_StatusTypeDef PCA9685_SetPWMFreq(PCA9685_HandleTypeDef *hpca, float hz) {
    if (hz < 24.0f)   hz = 24.0f;     // device typical min
    if (hz > 1526.0f) hz = 1526.0f;   // device typical max

    // prescale = round(osc/(4096*hz)) - 1 ; but datasheet uses floor-ish.
    float prescale_f = (hpca->osc_hz / (4096.0f * hz)) - 1.0f;
    uint8_t prescale = (uint8_t)floorf(prescale_f + 0.5f); // round to nearest

    // Go to sleep, write prescale, wake, delay, then (optionally) restart
    uint8_t oldmode = 0;
    if (rd8(hpca, PCA9685_MODE1, &oldmode) != HAL_OK) return HAL_ERROR;

    uint8_t sleepmode = (oldmode & ~BIT(MODE1_RESTART_Pos)) | BIT(MODE1_SLEEP_Pos);
    if (wr8(hpca, PCA9685_MODE1, sleepmode) != HAL_OK) return HAL_ERROR;

    if (wr8(hpca, PCA9685_PRE_SCALE, prescale) != HAL_OK) return HAL_ERROR;

    if (wr8(hpca, PCA9685_MODE1, oldmode) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1); // wait for osc

    // Optional: set RESTART to quickly apply previous PWM states
    return PCA9685_Restart(hpca);
}

HAL_StatusTypeDef PCA9685_SetPWM(PCA9685_HandleTypeDef *hpca, uint8_t channel, uint16_t on_count, uint16_t off_count) {
    if (channel > 15) return HAL_ERROR;
    on_count  &= 0x0FFF;
    off_count &= 0x0FFF;

    uint8_t reg = PCA9685_ChannelOnL(channel);
    uint8_t buf[4];
    buf[0] = (uint8_t)(on_count & 0xFF);
    buf[1] = (uint8_t)((on_count >> 8) & 0x0F);   // bit4 is full-ON flag (leave 0)
    buf[2] = (uint8_t)(off_count & 0xFF);
    buf[3] = (uint8_t)((off_count >> 8) & 0x0F);  // bit4 is full-OFF flag (leave 0)

    return wrN(hpca, reg, buf, sizeof(buf));
}

HAL_StatusTypeDef PCA9685_SetDuty(PCA9685_HandleTypeDef *hpca, uint8_t channel, float duty_0_to_1, uint16_t phase_count) {
    if (duty_0_to_1 < 0.0f) duty_0_to_1 = 0.0f;
    if (duty_0_to_1 > 1.0f) duty_0_to_1 = 1.0f;

    uint16_t on  = (uint16_t)(phase_count & 0x0FFF);
    uint16_t off = (uint16_t)((on + (uint16_t)lroundf(duty_0_to_1 * 4096.0f)) & 0x0FFF);

    // Handle corner cases for full ON / full OFF for best behavior:
    if (duty_0_to_1 <= 0.0f) {
        // Force full-OFF
        uint8_t regH = PCA9685_ChannelOffH(channel);
        HAL_StatusTypeDef st = PCA9685_SetPWM(hpca, channel, 0, 0);
        if (st != HAL_OK) return st;
        uint8_t offh = 0x10; // bit4=1 -> full OFF
        return wr8(hpca, regH, offh);

    }
    else if (duty_0_to_1 >= 1.0f) {
        // Force full-ON (with optional phase delay)
        uint8_t regH = PCA9685_ChannelOnH(channel);
        HAL_StatusTypeDef st = PCA9685_SetPWM(hpca, channel, on, 0);
        if (st != HAL_OK) return st;
        uint8_t onh = (uint8_t)(((on >> 8) & 0x0F) | 0x10); // bit4=1 -> full ON
        return wr8(hpca, regH, onh);
    }
    else {
        return PCA9685_SetPWM(hpca, channel, on, off);
    }
}

HAL_StatusTypeDef PCA9685_SetAllPWM(PCA9685_HandleTypeDef *hpca, uint16_t on_count, uint16_t off_count) {
    on_count  &= 0x0FFF;
    off_count &= 0x0FFF;
    uint8_t buf[4];
    buf[0] = (uint8_t)(on_count & 0xFF);
    buf[1] = (uint8_t)((on_count >> 8) & 0x0F);
    buf[2] = (uint8_t)(off_count & 0xFF);
    buf[3] = (uint8_t)((off_count >> 8) & 0x0F);

    return wrN(hpca, PCA9685_ALL_LED_ON_L, buf, sizeof(buf));
}

HAL_StatusTypeDef PCA9685_Restart(PCA9685_HandleTypeDef *hpca) {
    // RESTART sequence per datasheet: only valid if SLEEP=0 and RESTART previously set.
    uint8_t mode1 = 0;

    if (rd8(hpca, PCA9685_MODE1, &mode1) != HAL_OK) return HAL_ERROR;

    // Ensure SLEEP=0
    if (mode1 & BIT(MODE1_SLEEP_Pos)) {
        mode1 &= ~BIT(MODE1_SLEEP_Pos);
        if (wr8(hpca, PCA9685_MODE1, mode1) != HAL_OK) return HAL_ERROR;
        HAL_Delay(1);
    }

    // Set RESTART=1 to kick PWM logic
    mode1 |= BIT(MODE1_RESTART_Pos);
    return wr8(hpca, PCA9685_MODE1, mode1);
}