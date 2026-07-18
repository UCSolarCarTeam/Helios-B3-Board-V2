/*
 * Imu.c
 *
 *  Created on: Apr 17, 2026
 *      Author: mkom0
 *
 *  Imu driver (.c): Contains functions for accelerometer and gyro data
 *  calculated using fifo.
 */

#include "Imu.h"

uint8_t who_am_i(I2C_HandleTypeDef* hi2c_)
{
    uint8_t whoami = 0;
    HAL_StatusTypeDef status;
    // char msg[100];

    HAL_Delay(100);
    status = HAL_I2C_Mem_Read(hi2c_, IMU_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &whoami, 1, 1000);

    return whoami;
    // CUBE_PRINT("who_am_i status=%d value=0x%02X\r\n", status, who);
}

void imu_init(I2C_HandleTypeDef* hi2c_)
{
//#if 0
//    HAL_StatusTypeDef status;
//    uint8_t data;
//
//    data = 0x80; //reset device
//    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
//    HAL_Delay(100);
//
//    data = 0x01; //wake up, set clock source
//    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
//    HAL_Delay(10);
//
//    data = 0x09; //sample rate dvider
//    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x19, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
//
//    data = 0x03; //gyro dlpf
//    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x1A, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
//
//    data = 0x08; // gyro ±500 dps
//    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
//
//    data = 0x08; // accel ±4g
//    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
//
//    data = 0x03; //accel dlpf
//    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x1D, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
//
//    data = 0x00; //disable fifo
//    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6A, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
//
//    data = 0x04; //reset fifo
//    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6A, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
//    HAL_Delay(10);
//
//    data = 0x78; //set gyro and accel in fifo
//    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x23, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
//
//    data = 0x40; //enable fifo
//    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6A, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
//#endif
//
//
//    uint8_t data;
//
//	// 1. Reset device
//	data = 0x80; // DEVICE_RESET
//	if (HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6B, 1, &data, 1, 100) != HAL_OK) {
//		return HAL_ERROR;
//	}
//
//	HAL_Delay(100); // wait for reset to complete
//
//	// 2. Wake up + select clock source (PLL or gyro X)
//	data = 0x01; // auto PLL clock
//	if (HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6B, 1, &data, 1, 100) != HAL_OK) {
//		return HAL_ERROR;
//	}
//
//	HAL_Delay(10);
//
//	// 3. Disable sleep + disable cycle mode
//	data = 0x00;
//	HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6C, 1, &data, 1, 100);
//
//	// 4. Configure sample rate
//	// Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV)
//	// Gyro ODR = 1 kHz (default when DLPF enabled)
//	data = 0x07; // 125 Hz output
//	HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x19, 1, &data, 1, 100);
//
//	// 5. Configure DLPF (gyro + accel bandwidth)
//	// 0x03 ≈ ~44 Hz bandwidth (stable for control systems)
//	data = 0x03;
//	HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x1A, 1, &data, 1, 100);
//
//	// 6. Gyro full scale ±500 dps
//	data = 0x08;
//	HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x1B, 1, &data, 1, 100);
//
//	// 7. Accel full scale ±2g (best resolution)
//	data = 0x00;
//	HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x1C, 1, &data, 1, 100);
//
//	// 8. Accel DLPF enabled + ~44 Hz bandwidth
//	data = 0x03;
//	HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x1D, 1, &data, 1, 100);
//
//	// 9. Disable FIFO completely (important!)
//	data = 0x00;
//	HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6A, 1, &data, 1, 100); // USER_CTRL
//
//	// 10. Ensure FIFO disabled sources
//	data = 0x00;
//	HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x23, 1, &data, 1, 100); // FIFO_EN
//
//	HAL_Delay(10);
//
//	return HAL_OK;
}

void fifo_reset(I2C_HandleTypeDef* hi2c_)
{
	/*-------------------------------------------*/
	// Original
#if 0
    HAL_StatusTypeDef status;
    uint8_t data;

    data = 0x00; //disable fifo
    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6A, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    data = 0x04; //reset fifo
    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6A, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    data = 0x78;   // set gyro and accel
    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x23, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    data = 0x40; //enable FIFO
    status = HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6A, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    HAL_Delay(20);   //wait
#endif

	/*-------------------------------------------*/
	// Testing proper FIFO clear
    uint8_t data;

    // 1. Disable FIFO (clear FIFO_EN bit only)
    HAL_I2C_Mem_Read(hi2c_, IMU_ADDR, 0x6A, 1, &data, 1, 100);
    data &= ~0x40; // clear FIFO_EN bit only
    HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6A, 1, &data, 1, 100);

    HAL_Delay(5);

    // 2. Reset FIFO (set FIFO_RST bit only)
    HAL_I2C_Mem_Read(hi2c_, IMU_ADDR, 0x6A, 1, &data, 1, 100);
    data |= 0x04;
    HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6A, 1, &data, 1, 100);

    HAL_Delay(5);

    // 3. Re-enable FIFO mode (preserve other bits!)
    HAL_I2C_Mem_Read(hi2c_, IMU_ADDR, 0x6A, 1, &data, 1, 100);
    data |= 0x40;
    HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x6A, 1, &data, 1, 100);

    // 4. Re-enable accel+gyro FIFO sources
    data = 0x78;
    HAL_I2C_Mem_Write(hi2c_, IMU_ADDR, 0x23, 1, &data, 1, 100);

    HAL_Delay(10);
}
