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
#include "main_system.hpp"
#include <stdio.h>
#include <string.h>


I2C_HandleTypeDef hi2c1;
extern UARTDriver uart1;

void who_am_i(void)
{
    uint8_t who = 0;
    HAL_StatusTypeDef status;
    char msg[100];

    HAL_Delay(100);

    status = HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &who, 1, 1000);

    sprintf(msg, "who_am_i status=%d value=0x%02X\r\n", status, who);
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
}

void imu_init(void)
{
    HAL_StatusTypeDef status;
    uint8_t data;
    char msg[80];

    data = 0x80; //reset device
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    HAL_Delay(100);

    data = 0x01; //wake up, set clock source
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    HAL_Delay(10);

    data = 0x09; //sample rate dvider
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x19, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

    data = 0x03; //gyro dlpf
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x1A, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

    data = 0x08; // gyro ±500 dps
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

    data = 0x08; // accel ±4g
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

    data = 0x03; //accel dlpf
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x1D, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

    data = 0x00; //disable fifo
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x6A, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

    data = 0x04; //reset fifo
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x6A, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    HAL_Delay(10);

    data = 0x78; //set gyro and accel in fifo
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x23, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

    data = 0x40; //enable fifo
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x6A, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
}

void fifo_reset(void)
{
    HAL_StatusTypeDef status;
    uint8_t data;
    char msg[80];

    data = 0x00; //disable fifo
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x6A, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

    data = 0x04; //reset fifo
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x6A, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

    data = 0x78;   // set gyro and accel
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x23, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

    data = 0x40; //enable FIFO
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x6A, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

    HAL_Delay(20);   //wait
}
