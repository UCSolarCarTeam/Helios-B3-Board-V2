#ifndef PCA9685_H
#define PCA9685_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l1xx_hal.h"   // change if your project uses a different STM32 family HAL header
#include <stdint.h>
#include <stdbool.h>

// =========================
// Register map (PCA9685)
// =========================
#define PCA9685_MODE1        0x00
#define PCA9685_MODE2        0x01
#define PCA9685_SUBADR1      0x02
#define PCA9685_SUBADR2      0x03
#define PCA9685_SUBADR3      0x04
#define PCA9685_ALLCALLADR   0x05

#define PCA9685_LED0_ON_L    0x06
#define PCA9685_LED0_ON_H    0x07
#define PCA9685_LED0_OFF_L   0x08
#define PCA9685_LED0_OFF_H   0x09

#define PCA9685_ALL_LED_ON_L  0xFA
#define PCA9685_ALL_LED_ON_H  0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD
#define PCA9685_PRE_SCALE     0xFE

// =========================
// Bit helpers / bit positions
// =========================
#ifndef BIT
#define BIT(pos) (1U << (pos))
#endif

// MODE1 bit positions
#define MODE1_ALLCALL_Pos    0
#define MODE1_SUB3_Pos       1
#define MODE1_SUB2_Pos       2
#define MODE1_SUB1_Pos       3
#define MODE1_SLEEP_Pos      4
#define MODE1_AI_Pos         5
#define MODE1_EXTCLK_Pos     6
#define MODE1_RESTART_Pos    7

// MODE2 bit positions
// (bits 7:5 reserved)
#define MODE2_OUTNE0_Pos     0
#define MODE2_OUTNE1_Pos     1
#define MODE2_OUTDRV_Pos     2
#define MODE2_OCH_Pos        3
#define MODE2_INVRT_Pos      4

// =========================
// Channel register helpers
// =========================
static inline uint8_t PCA9685_ChannelOnL(uint8_t ch)   { return (uint8_t)(PCA9685_LED0_ON_L  + 4U * ch); }
static inline uint8_t PCA9685_ChannelOnH(uint8_t ch)   { return (uint8_t)(PCA9685_LED0_ON_H  + 4U * ch); }
static inline uint8_t PCA9685_ChannelOffL(uint8_t ch)  { return (uint8_t)(PCA9685_LED0_OFF_L + 4U * ch); }
static inline uint8_t PCA9685_ChannelOffH(uint8_t ch)  { return (uint8_t)(PCA9685_LED0_OFF_H + 4U * ch); }

// =========================
// Types
// =========================
typedef struct {
    I2C_HandleTypeDef *i2c;   // HAL I2C handle
    uint8_t  addr7;           // 7-bit I2C address (e.g. 0x40)
    float    osc_hz;          // oscillator frequency (default 25 MHz)
} PCA9685_HandleTypeDef;

// =========================
// Public API (matches your .c)
// =========================
HAL_StatusTypeDef PCA9685_Init(PCA9685_HandleTypeDef *hpca,
                               I2C_HandleTypeDef *hi2c,
                               uint8_t addr7);

HAL_StatusTypeDef PCA9685_SoftwareReset(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef PCA9685_SetOutputMode(PCA9685_HandleTypeDef *hpca,
                                        bool totem_pole,
                                        bool invert,
                                        bool update_on_ack);

HAL_StatusTypeDef PCA9685_Sleep(PCA9685_HandleTypeDef *hpca, bool sleep);

HAL_StatusTypeDef PCA9685_SetPWMFreq(PCA9685_HandleTypeDef *hpca, float hz);

HAL_StatusTypeDef PCA9685_SetPWM(PCA9685_HandleTypeDef *hpca,
                                 uint8_t channel,
                                 uint16_t on_count,
                                 uint16_t off_count);

HAL_StatusTypeDef PCA9685_SetDuty(PCA9685_HandleTypeDef *hpca,
                                  uint8_t channel,
                                  float duty_0_to_1,
                                  uint16_t phase_count);

HAL_StatusTypeDef PCA9685_SetAllPWM(PCA9685_HandleTypeDef *hpca,
                                    uint16_t on_count,
                                    uint16_t off_count);

HAL_StatusTypeDef PCA9685_Restart(PCA9685_HandleTypeDef *hpca);

#ifdef __cplusplus
}
#endif

#endif // PCA9685_H