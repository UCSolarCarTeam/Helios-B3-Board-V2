/*
 * ImuReadTask.cpp
 *
 * Imu read task (.cpp): Retrieves accelerometer and gyro data from fifo.
 *
 */

#include "ImuReadTask.hpp"
#include "SystemDefines.hpp"
#include "Imu.h"

#define IMU_ADDR (0x68 << 1)
#define IMU_I2U_ADDR_68 (0x68 << 1) //adding bits for r/w
#define IMU_I2U_ADDR_69 (0x69 << 1)
#define WHO_AM_I_REG  0x75

/**
 * @brief Constructor for ImuReadTask
 */
ImuReadTaskTask::ImuReadTask() : Task(IMU_READ_TASK_QUEUE_DEPTH_OBJS)
{
}

/**
 * @brief Initialize the ImuReadTask
 */
void ImuReadTaskTask::InitTask()
{
    // Make sure the task is not already initialized
    CUBE_ASSERT(rtTaskHandle == nullptr, "Cannot initialize ImuReadTask task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)ImuReadTask::RunTask,
            (const char*)"ImuReadTask",
            (uint16_t)IMU_READ_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)IMU_READ_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    CUBE_ASSERT(rtValue == pdPASS, "ImuReadTask::InitTask() - xTaskCreate() failed");
}

void ImuReadTask::Run(void * pvParams)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    imu_init();
    who_am_i();

    while (1)
    {
    	uint8_t count_H, count_L;
    	uint16_t count;

    	HAL_I2C_Mem_Read(I2C2_Handle, IMU_ADDR, 0x72, I2C_MEMADD_SIZE_8BIT, &count_H, 1, 100); //0x72 = fifo_countH
    	HAL_I2C_Mem_Read(I2C2_Handle, IMU_ADDR, 0x73, I2C_MEMADD_SIZE_8BIT, &count_L, 1, 100); //0x73 = fifo countL

    	count = (count_H << 8) | count_L;
    	CUBE_PRINT("FIFO count: %d\n", count);

        if (count >= 500)
        {
        	CUBE_PRINT("FIFO is almost full, resetting\r\n");
        	fifo_reset();
            continue; //keep going
        }

    	if (count >= 12) //need atleast 12 bytes to start giving results for accel (59 to 64) and gyro (67 to 72)
    	{
    	    uint8_t buf[12];
    	    HAL_I2C_Mem_Read(I2C2_Handle, IMU_ADDR, 0x74, I2C_MEMADD_SIZE_8BIT, buf, 12, 1000); //0x74 = to r/w from the fifo buf (contains sensor data reg 59-72)

    	    int16_t ax = (buf[0] << 8) | buf[1];
    	    int16_t ay = (buf[2] << 8) | buf[3];
    	    int16_t az = (buf[4] << 8) | buf[5];

    	    int16_t gx = (buf[6] << 8) | buf[7];
    	    int16_t gy = (buf[8] << 8) | buf[9];
    	    int16_t gz = (buf[10] << 8) | buf[11];

    	    //converting g to m/s^2
    	    float ax_ms2 = (ax * 9.80665f) / 8192.0f;
    	    float ay_ms2 = (ay * 9.80665f) / 8192.0f;
    	    float az_ms2 = (az * 9.80665f) / 8192.0f;

    	    //converting counts to degrees per second
    	    float gx_dps = gx / 65.5f;
    	    float gy_dps = gy / 65.5f;
    	    float gz_dps = gz / 65.5f;

    	    CUBE_PRINT("Accel [m/s^2]: X: %.3f Y: %.3f Z: %.3f | Gyro [dps]: X: %.3f Y: %.3f Z: %.3f\r\n", ax_ms2, ay_ms2, az_ms2,
    	    		gx_dps, gy_dps, gz_dps);
    	}
	}
}