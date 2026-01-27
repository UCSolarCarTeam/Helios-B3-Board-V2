/*
 * PWM_Expander.c
 *
 *  Created on: Jan 10, 2026
 *      Author: Karim
 */

#include "PWM_Expander.h"
#include "PWM_Registers.h"

extern I2C_HandleTypeDef hi2c1;

void PWM_I2C_READ_REGISTER(uint8_t address, uint8_t *buffer)
{
    HAL_I2C_Mem_Read( &hi2c1,
    					0x11,
                     address,
                     I2C_MEMADD_SIZE_8BIT,
                     buffer,
                     1,
                     100U);
}

void PWM_I2C_WRITE_REGISTER(uint8_t address, uint8_t value)
{
    HAL_I2C_Mem_Write( &hi2c1,
    					0x10,
                      address,
                      I2C_MEMADD_SIZE_8BIT,
                      &value,
                      1,
                      100U);
}

void PCA9685_LED0_On(void)
{
    uint8_t data = 0x10; // FULL ON
    HAL_I2C_Mem_Write(&hi2c1,
                      0x40 << 1,
                      0x07,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      200);

    __NOP();
}

void PCA9685_SetPWM(uint8_t channel, uint16_t value)
{
    uint8_t data[4];
    uint8_t reg = 0x06 + 4 * channel;  // LEDn_ON_L base

    if (value > 4095) value = 4095;

    // ON = 0
    data[0] = 0x00;        // ON_L
    data[1] = 0x00;        // ON_H

    // OFF = value
    data[2] = value & 0xFF;        // OFF_L
    data[3] = (value >> 8) & 0x0F; // OFF_H

    HAL_I2C_Mem_Write(
        &hi2c1,
        0x40 << 1,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        4,
        100
    );
}


//void PCA9685_LED0_Off(void)
//{
//    uint8_t data = 0x10; // bit 4 = 1 → FULL OFF
//
//    HAL_I2C_Mem_Write(&hi2c1,
//                      0x40 << 1,
//                      0x09,                 // LED0_OFF_H
//                      I2C_MEMADD_SIZE_8BIT,
//                      &data,
//                      1,
//                      200);
//}


//void PWM_I2C_WRITE_REGISTER_BITWISE(uint8_t address,uint8_t mask,uint8_t value)
//	{
//	uint8_t reg = 0;
//
//	    PWM_I2C_READ_REGISTER(address, &reg, peripheral);
//
//	    reg &= ~mask;          // clear the bits we want to change
//	    reg |= (value & mask); // set them to requested value
//
//	    PWM_I2C_WRITE_REGISTER(address, reg, peripheral);
//	}
//
//void PWM_I2C_RESET()
//	{
//	uint8_t cmd = 0x06; // General Call "Software Reset"
//
//	    // Send to General Call address 0x00
//	    HAL_I2C_Master_Transmit(peripheral->hi2c,
//	                            0x00,
//	                            &cmd,
//	                            1,
//	                            100U);
//	    HAL_Delay(1);
//	}

//void PWM_I2C_SLEEP_ENABLE()
//	{
//	 uint8_t mode1 = 0;
//	    PWM_I2C_READ_REGISTER(PCA9685_MODE1, &mode1, peripheral);
//
//	    mode1 |= PCA9685_MODE1_SLEEP;
//	    PWM_I2C_WRITE_REGISTER(PCA9685_MODE1, mode1, peripheral);
//
//	    HAL_Delay(1);
//	}
//
//void PWM_I2C_SLEEP_DISABLE()
// {
//	uint8_t mode1 = 0;
//	    PWM_I2C_READ_REGISTER(PCA9685_MODE1, &mode1, peripheral);
//
//	    mode1 &= ~PCA9685_MODE1_SLEEP;
//	    PWM_I2C_WRITE_REGISTER(PCA9685_MODE1, mode1, peripheral);
//
//	    HAL_Delay(1);
// }
