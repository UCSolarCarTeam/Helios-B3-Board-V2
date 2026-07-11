/*
 * Imu.h
 *
 *  Created on: Apr 17, 2026
 *      Author: mkom0
 *
 *  Imu driver header (.h): Declares functions for accelerometer and gyro
 *  access using fifo.
 */

#ifndef DRIVERS_IMU_H_
#define DRIVERS_IMU_H_

#include "main.h"

#define IMU_ADDR (0x68 << 1)
#define IMU_I2U_ADDR_68 (0x68 << 1) //adding bits for r/w
#define IMU_I2U_ADDR_69 (0x69 << 1)
#define WHO_AM_I_REG  0x75

#ifdef __cplusplus
extern "C" {
#endif

uint8_t who_am_i(I2C_HandleTypeDef* hi2c_);
void imu_init(I2C_HandleTypeDef* hi2c_);
void fifo_reset(I2C_HandleTypeDef* hi2c_);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_IMU_H_ */
