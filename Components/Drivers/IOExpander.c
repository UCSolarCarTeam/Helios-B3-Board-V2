 /*
 * IOExpander.c
 *
 *  Created on: Nov 1, 2025
 *      Author: Omar Hassan
 */

#include "IOExpander.h"

// ---------------- Defines ----------------
#define I2C_TIMEOUT_MS 1000

// ---------------- Internal State -----------------
static I2C_HandleTypeDef *hi2c_ = NULL;
static uint8_t dev_ = NULL;

static uint8_t last_write[2] = {0xFF, 0xFF};
static uint8_t pending_write[2] = {0xFF, 0xFF};
static uint8_t last_read[2] = {0xFF, 0xFF};


// ----------------- Low-level I2C Access -----------------

// Write a buffer into sequential registers
static bool IOE_WriteBuf(uint8_t dest, uint8_t* data, uint16_t size) {
	if ( HAL_I2C_Mem_Write(hi2c_, dev_, dest, I2C_MEMADD_SIZE_8BIT, data, size, I2C_TIMEOUT_MS) == HAL_OK) {
		return true;
	}
	return false;

}

// Read a buffer from sequential registers
static bool IOE_ReadBuf(uint8_t dest, uint8_t* data, uint16_t size) {
    if (HAL_I2C_Mem_Read(hi2c_, dev_, dest, I2C_MEMADD_SIZE_8BIT, data, size, I2C_TIMEOUT_MS) == HAL_OK) {
		return true;
	}
	return false;
}

// Write a value into a single register
static bool IOE_WriteReg(uint8_t dest, uint8_t data) {
	uint8_t temp = data;
	if ( HAL_I2C_Mem_Write(hi2c_, dev_ << 1, dest, I2C_MEMADD_SIZE_8BIT, &temp, 1, I2C_TIMEOUT_MS) == HAL_OK) {
		return true;
	}
	return false;

}

// Read a value from a single register
static bool IOE_ReadReg(uint8_t dest, uint8_t* data) {
	uint8_t temp = data;
    if (HAL_I2C_Mem_Read(hi2c_, dev_ << 1, dest, I2C_MEMADD_SIZE_8BIT, &temp, 1, I2C_TIMEOUT_MS) == HAL_OK) {
		return true;
	}
	return false;
}

// ----------------- Public API -----------------
void IOE_Init(I2C_HandleTypeDef *hi2c, uint8_t dev) {
    // Save I2C peripheral handler and address
    hi2c_ = hi2c;
    dev_ = dev;

    // Configure Port A and Port B as inputs
    IOE_WriteReg(IODIRA, 0xFF);
    IOE_WriteReg(IODIRB, 0xFF);

    // Inverse Pin polarity
//    IOE_WriteReg(IPOLA, 0xFF);
//    IOE_WriteReg(IPOLB, 0xFF);



    // Initialize internal state to 0
    last_write[0] = last_write[1] = 0x00;
    pending_write[0] = pending_write[1] = 0x00;
    last_read[0] = last_read[1] = 0x00;
}

bool IOE_SetPin(IOExpanderPin pin, IOState state) {
    uint8_t port = (pin < 8) ? 0 : 1;
    uint8_t bit = (pin % 8);

    if(state == IOE_HIGH)
        pending_write[port] |= (1 << bit);
    else
        pending_write[port] &= ~(1 << bit);

    return true;
}

bool IOE_SetPinNow(IOExpanderPin pin, IOState state) {
    IOE_SetPin(pin, state);
    return IOE_Commit();
}

bool IOE_TogglePin(IOExpanderPin pin) {
    uint8_t port = (pin < 8) ? 0 : 1;
    uint8_t bit = (pin % 8);
    pending_write[port] ^= (1 << bit);
    return true;
}

bool IOE_TogglePinNow(IOExpanderPin pin) {
    IOE_TogglePin(pin);
    return IOE_Commit();
}

bool IOE_Commit(void) {
    IOE_WriteReg(GPIOA, pending_write[0]);
    IOE_WriteReg(GPIOB, pending_write[1]);

    last_write[0] = pending_write[0];
    last_write[1] = pending_write[1];

    return true;
}

bool IOE_Update(void) {
	IOE_ReadReg(GPIOA, &last_read[0]);
	IOE_ReadReg(GPIOB, &last_read[1]);
    return true;
}

IOState IOE_GetPinState(IOExpanderPin pin) {
    uint8_t port = (pin < 8) ? 0 : 1;
    uint8_t bit = (pin % 8);
    return (last_read[port] & (1 << bit)) ? IOE_HIGH : IOE_LOW;
}

IOState IOE_GetPinStateNow(IOExpanderPin pin) {
    IOE_Update();
    return IOE_GetPinState(pin);
}
