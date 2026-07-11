/*
 * IOExpander.h
 *
 *  Created on: Nov 1, 2025
 *      Author: Omar Hassan
 */

#ifndef INC_IOEXPANDER_H_
#define INC_IOEXPANDER_H_

#include "main.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ----------- Register Defines ------------

// MCP23S17 opcodes
#define DEVICE_OPCODE_WRITE 0x40   // A2:A0 tied to GND
#define DEVICE_OPCODE_READ  0x41

// MCP23S17 register addresses (Bank = 0)
#define IODIRA    0x00
#define IODIRB    0x01

#define IPOLA     0x02
#define IPOLB     0x03

#define GPINTENA  0x04
#define GPINTENB  0x05

#define DEFVALA   0x06
#define DEFVALB   0x07

#define INTCONA   0x08
#define INTCONB   0x09

#define IOCON     0x0A
#define IOCON2    0x0B

#define GPPUA     0x0C
#define GPPUB     0x0D

#define INTFA     0x0E
#define INTFB     0x0F

#define INTCAPA   0x10
#define INTCAPB   0x11

#define GPIOA     0x12
#define GPIOB     0x13

#define OLATA     0x14
#define OLATB     0x15

// ----------------- Enums -----------------
typedef enum {
    IOE_LOW = 0,
    IOE_HIGH = 1
} IOState;

typedef enum {
    PA0 = 0, PA1, PA2, PA3, PA4, PA5, PA6, PA7,
    PB0 = 8, PB1, PB2, PB3, PB4, PB5, PB6, PB7
} IOExpanderPin;

// ----------------- Public Functions -----------------
void IOE_Init(I2C_HandleTypeDef *hi2c, uint8_t dev);
bool IOE_SetPin(IOExpanderPin pin, IOState state);        // stage only
bool IOE_SetPinNow(IOExpanderPin pin, IOState state);     // stage + commit
bool IOE_TogglePin(IOExpanderPin pin);                    // stage only
bool IOE_TogglePinNow(IOExpanderPin pin);                 // stage + commit
bool IOE_Commit(void);                                    // commit staged changes
bool IOE_Update(void);                                    // read from device
IOState IOE_GetPinState(IOExpanderPin pin);               // last read state
IOState IOE_GetPinStateNow(IOExpanderPin pin);            // read immediately

#ifdef __cplusplus
}
#endif

#endif /* INC_IOEXPANDER_H_ */
