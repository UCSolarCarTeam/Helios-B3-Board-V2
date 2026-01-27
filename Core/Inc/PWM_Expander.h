/*
 * PWM_Expander.h
 *
 *  Created on: Jan 10, 2026
 *      Author: Karim
 */

#include <stdint.h>
#include "stm32l1xx_hal.h"

#ifndef INC_PWM_EXPANDER_H_
#define INC_PWM_EXPANDER_H_

// PWM I2C Interface Functions

void PWM_I2C_READ_REGISTER(uint8_t address,uint8_t *buffer);
void PWM_I2C_WRITE_REGISTER(uint8_t address,uint8_t value);
void PWM_I2C_WRITE_REGISTER_BITWISE(uint8_t address,uint8_t mask,uint8_t value);

void PCA9685_SetPWM(uint8_t channel, uint16_t value);
void PCA9685_LED0_On(void);


void PWM_I2C_RESET();
void PWM_I2C_SLEEP_ENABLE();
void PWM_I2C_SLEEP_DISABLE();

// PWM I2C Operation Functions
void ConfigurePWMI2C();
void PWM_SetFrequency(uint16_t frequency());
void PWM_SetDutyCycle(uint8_t channel,uint16_t duty);
void PWM_SetOnOffCounts(uint8_t channel,uint16_t on_count,uint16_t off_count);
void PWM_EnableChannel(uint8_t channel);
void PWM_DisableChannel(uint8_t channel);
void PWM_AllChannelsOff();


#endif
