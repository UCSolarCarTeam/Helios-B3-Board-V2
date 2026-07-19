 /*
 * IOExpander.c
 *
 *  Created on: Nov 1, 2025
 *      Author: Omar Hassan
 */

#include "IOExpander.h"

// ---------------- Defines ----------------
#define I2C_TIMEOUT_MS 1000

// ----------------- Low-level I2C Access -----------------

// Write a buffer into sequential registers
static bool IOE_WriteBuf(MCP23017_HandleTypeDef* hiox, uint8_t dest, uint8_t* data, uint16_t size) {
	if ( HAL_I2C_Mem_Write(hiox->hi2c, hiox->addr, dest, I2C_MEMADD_SIZE_8BIT, data, size, I2C_TIMEOUT_MS) == HAL_OK) {
		return true;
	}
	return false;

}

// Read a buffer from sequential registers
static bool IOE_ReadBuf(MCP23017_HandleTypeDef* hiox, uint8_t dest, uint8_t* data, uint16_t size) {
    if (HAL_I2C_Mem_Read(hiox->hi2c, hiox->addr, dest, I2C_MEMADD_SIZE_8BIT, data, size, I2C_TIMEOUT_MS) == HAL_OK) {
		return true;
	}
	return false;
}

// Write a value into a single register
static bool IOE_WriteReg(MCP23017_HandleTypeDef* hiox, uint8_t dest, uint8_t data) {
	uint8_t temp = data;
	if ( HAL_I2C_Mem_Write(hiox->hi2c, hiox->addr, dest, I2C_MEMADD_SIZE_8BIT, &temp, 1, I2C_TIMEOUT_MS) == HAL_OK) {
		return true;
	}
	return false;

}

// Read a value from a single register
static bool IOE_ReadReg(MCP23017_HandleTypeDef* hiox, uint8_t dest, uint8_t* data) {
	uint8_t temp = 0;
    if (HAL_I2C_Mem_Read(hiox->hi2c, ((hiox->addr) | 0x01), dest, I2C_MEMADD_SIZE_8BIT, &temp, 1, I2C_TIMEOUT_MS) == HAL_OK) {
    	*data = temp;
    	return true;
	}
	return false;
}

uint16_t getLastRead(MCP23017_HandleTypeDef* hiox) {
//	uint8_t gpA = 0;
//	uint8_t gpB = 0;
//	IOE_ReadReg(hiox, IOXA, &gpA);
//	IOE_ReadReg(hiox, IOXB, &gpB);
	IOE_Update(hiox);
	uint16_t value = ((uint16_t)hiox->last_read[0] << 8) |
	                 ((uint16_t)hiox->last_read[1]);
	return value;
}

// ----------------- Public API -----------------
void IOE_Init(MCP23017_HandleTypeDef* hiox) {
    // Configure Port A and Port B as inputs
    IOE_WriteReg(hiox, IODIRA, 0xFF);
    IOE_WriteReg(hiox, IODIRB, 0xFF);

    // Inverse Pin polarity
    IOE_WriteReg(hiox, IPOLA, 0xFF);
    IOE_WriteReg(hiox, IPOLB, 0xFF);

    // Internal Pull-up Enable
	IOE_WriteReg(hiox, GPPUA, 0xFF);
	IOE_WriteReg(hiox, GPPUB, 0xFF);
}

bool IOE_SetPin(MCP23017_HandleTypeDef* hiox, IOExpanderPin pin, IOState state) {
    uint8_t port = (pin < 8) ? 0 : 1;
    uint8_t bit = (pin % 8);

    if(state == IOE_HIGH)
    	hiox->pending_write[port] |= (1 << bit);
    else
    	hiox->pending_write[port] &= ~(1 << bit);

    return true;
}

bool IOE_SetPinNow(MCP23017_HandleTypeDef* hiox, IOExpanderPin pin, IOState state) {
    IOE_SetPin(hiox, pin, state);
    return IOE_Commit(hiox);
}

bool IOE_TogglePin(MCP23017_HandleTypeDef* hiox, IOExpanderPin pin) {
    uint8_t port = (pin < 8) ? 0 : 1;
    uint8_t bit = (pin % 8);
    hiox->pending_write[port] ^= (1 << bit);
    return true;
}

bool IOE_TogglePinNow(MCP23017_HandleTypeDef* hiox, IOExpanderPin pin) {
    IOE_TogglePin(hiox, pin);
    return IOE_Commit(hiox);
}

bool IOE_Commit(MCP23017_HandleTypeDef* hiox) {
    IOE_WriteReg(hiox, IOXA, hiox->pending_write[0]);
    IOE_WriteReg(hiox, IOXB, hiox->pending_write[1]);
    hiox->last_write[0] = hiox->pending_write[0];
    hiox->last_write[1] = hiox->pending_write[1];

    return true;
}

bool IOE_Update(MCP23017_HandleTypeDef* hiox) {
	IOE_ReadReg(hiox, IOXA, &hiox->last_read[0]);
	IOE_ReadReg(hiox, IOXB, &hiox->last_read[1]);
    return true;
}

IOState IOE_GetPinState(MCP23017_HandleTypeDef* hiox, IOExpanderPin pin) {
    uint8_t port = (pin < 8) ? 0 : 1;
    uint8_t bit = (pin % 8);
    return (hiox->last_read[port] & (1 << bit)) ? IOE_HIGH : IOE_LOW;
}

IOState IOE_GetPinStateNow(MCP23017_HandleTypeDef* hiox, IOExpanderPin pin) {
    IOE_Update(hiox);
    return IOE_GetPinState(hiox, pin);
}
