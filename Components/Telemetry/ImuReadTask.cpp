/**
 ******************************************************************************
 * File Name          : ImuReadTask.hpp
 * Description        : Accesses IAM-20680HT to poll accelerometer, gyroscope, and temperature.
 ******************************************************************************
*/

#include "ImuReadTask.hpp"
#include "SystemDefines.hpp"
#include "Imu.h"

/**
 * @brief Constructor for ImuReadTask
 */
ImuReadTask::ImuReadTask() : Task(IMU_READ_TASK_QUEUE_DEPTH_OBJS)
{
}

/**
 * @brief Initialize the ImuReadTask
 */
void ImuReadTask::InitTask()
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

#if 0
void ImuReadTask::Run(void * pvParams)
{
	//
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    imu_init(SystemHandles::I2C2_Handle);

    // Verify proper IMU function through WHO_AM_I register
    // Default WHO_AM_I register value is 0xFA
    uint8_t whoami = who_am_i(SystemHandles::I2C2_Handle);
    CUBE_PRINT("who_am_i value=0x%02X\r\n", whoami);

    while (1)
    {
    	uint8_t count_H, count_L;
    	uint16_t count;

    	HAL_I2C_Mem_Read(SystemHandles::I2C2_Handle, IMU_ADDR, 0x72, I2C_MEMADD_SIZE_8BIT, &count_H, 1, 100); //0x72 = fifo_countH
    	HAL_I2C_Mem_Read(SystemHandles::I2C2_Handle, IMU_ADDR, 0x73, I2C_MEMADD_SIZE_8BIT, &count_L, 1, 100); //0x73 = fifo countL

    	count = (count_H << 8) | count_L;
    	CUBE_PRINT("FIFO count: %d\n", count);

    	// Reset FIFO
        if (count >= 4000)
        {
        	CUBE_PRINT("FIFO is almost full, resetting\r\n");
        	fifo_reset(SystemHandles::I2C2_Handle);
            continue; //keep going
        }

    	if (count >= 12) //need atleast 12 bytes to start giving results for accel (59 to 64) and gyro (67 to 72)
    	{
    	    uint8_t buf[12];
    	    HAL_I2C_Mem_Read(SystemHandles::I2C2_Handle, IMU_ADDR, 0x74, I2C_MEMADD_SIZE_8BIT, buf, 12, 1000); //0x74 = to r/w from the fifo buf (contains sensor data reg 59-72)

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

    	    osDelay(500);
    	}
	}
}
#endif

void ImuReadTask::Run(void * pvParams)
{
    imu_init(SystemHandles::I2C2_Handle);

    uint8_t whoami = who_am_i(SystemHandles::I2C2_Handle);
    CUBE_PRINT("WHO_AM_I = 0x%02X\r\n", whoami);

    uint8_t buf[14];

    while (1)
    {
        HAL_I2C_Mem_Read(SystemHandles::I2C2_Handle,
                         IMU_ADDR,
                         0x3B,   // ACCEL_XOUT_H
                         I2C_MEMADD_SIZE_8BIT,
                         buf,
                         14,
                         100);

        int16_t ax = (buf[0] << 8) | buf[1];
        int16_t ay = (buf[2] << 8) | buf[3];
        int16_t az = (buf[4] << 8) | buf[5];

        int16_t temp_raw = (buf[6] << 8) | buf[7];

        int16_t gx = (buf[8] << 8) | buf[9];
        int16_t gy = (buf[10] << 8) | buf[11];
        int16_t gz = (buf[12] << 8) | buf[13];

        float ax_ms2 = (ax / 16384.0f) * 9.80665f;
        float ay_ms2 = (ay / 16384.0f) * 9.80665f;
        float az_ms2 = (az / 16384.0f) * 9.80665f;

 	    float gx_dps = gx / 65.5f;
		float gy_dps = gy / 65.5f;
		float gz_dps = gz / 65.5f;


        CUBE_PRINT("A: %.2f %.2f %.2f | G: %.2f %.2f %.2f\r\n",
                   ax_ms2, ay_ms2, az_ms2,
                   gx_dps, gy_dps, gz_dps);

        osDelay(250);
    }
}

