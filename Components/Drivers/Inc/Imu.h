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

void who_am_i(void);
void imu_init(void);
void fifo_reset(void);

#endif /* DRIVERS_IMU_H_ */
