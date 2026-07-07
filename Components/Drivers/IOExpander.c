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
static uint8_t last_write[2] = {0xFF, 0xFF};
static uint8_t pending_write[2] = {0xFF, 0xFF};
static uint8_t last_read[2] = {0xFF, 0xFF};

// ----------------- Low-level I2C Access -----------------

// Write a buffer into sequential registers
static bool IOE_WriteBuf(MCP23017_HandleTypeDef* hiox, uint8_t dest, uint8_t* data, uint16_t size) {
	if ( HAL_I2C_Mem_Write(hiox->hi2c, (hiox->addr) << 1, dest, I2C_MEMADD_SIZE_8BIT, data, size, I2C_TIMEOUT_MS) == HAL_OK) {
		return true;
	}
	return false;

}

// Read a buffer from sequential registers
static bool IOE_ReadBuf(MCP23017_HandleTypeDef* hiox, uint8_t dest, uint8_t* data, uint16_t size) {
    if (HAL_I2C_Mem_Read(hiox->hi2c, (hiox->addr) << 1, dest, I2C_MEMADD_SIZE_8BIT, data, size, I2C_TIMEOUT_MS) == HAL_OK) {
		return true;
	}
	return false;
}

// Write a value into a single register
static bool IOE_WriteReg(MCP23017_HandleTypeDef* hiox, uint8_t dest, uint8_t data) {
	uint8_t temp = data;
	if ( HAL_I2C_Mem_Write(hiox->hi2c, (hiox->addr) << 1, dest, I2C_MEMADD_SIZE_8BIT, &temp, 1, I2C_TIMEOUT_MS) == HAL_OK) {
		return true;
	}
	return false;

}

// Read a value from a single register
static bool IOE_ReadReg(MCP23017_HandleTypeDef* hiox, uint8_t dest, uint8_t* data) {
	uint8_t temp = data;
    if (HAL_I2C_Mem_Read(hiox->hi2c, (hiox->addr) << 1, dest, I2C_MEMADD_SIZE_8BIT, &temp, 1, I2C_TIMEOUT_MS) == HAL_OK) {
		return true;
	}
	return false;
}

// ----------------- Public API -----------------
void IOE_Init(MCP23017_HandleTypeDef* hiox) {

    // Configure Port A and Port B as inputs
    IOE_WriteReg(hiox, IODIRA, 0xFF);
    IOE_WriteReg(hiox, IODIRB, 0xFF);

    // Inverse Pin polarity
//    IOE_WriteReg(hiox, IPOLA, 0xFF);
//    IOE_WriteReg(hiox, IPOLB, 0xFF);

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

bool IOE_SetPinNow(MCP23017_HandleTypeDef* hiox, IOExpanderPin pin, IOState state) {
    IOE_SetPin(pin, state);
    return IOE_Commit(hiox);
}

bool IOE_TogglePin(IOExpanderPin pin) {
    uint8_t port = (pin < 8) ? 0 : 1;
    uint8_t bit = (pin % 8);
    pending_write[port] ^= (1 << bit);
    return true;
}

bool IOE_TogglePinNow(MCP23017_HandleTypeDef* hiox, IOExpanderPin pin) {
    IOE_TogglePin(pin);
    return IOE_Commit(hiox);
}

bool IOE_Commit(MCP23017_HandleTypeDef* hiox) {
    IOE_WriteReg(hiox, IOXA, pending_write[0]);
    IOE_WriteReg(hiox, IOXB, pending_write[1]);

    last_write[0] = pending_write[0];
    last_write[1] = pending_write[1];

    return true;
}

bool IOE_Update(MCP23017_HandleTypeDef* hiox) {
	IOE_ReadReg(hiox, IOXA, &last_read[0]);
	IOE_ReadReg(hiox, IOXB, &last_read[1]);
    return true;
}

IOState IOE_GetPinState(IOExpanderPin pin) {
    uint8_t port = (pin < 8) ? 0 : 1;
    uint8_t bit = (pin % 8);
    return (last_read[port] & (1 << bit)) ? IOE_HIGH : IOE_LOW;
}

IOState IOE_GetPinStateNow(MCP23017_HandleTypeDef* hiox, IOExpanderPin pin) {
    IOE_Update(hiox);
    return IOE_GetPinState(pin);
}
