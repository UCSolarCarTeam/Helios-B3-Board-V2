/**
 ******************************************************************************
 * File Name          : GnssReadTask.cpp
 * Description        : Primary GNSS read task.
 ******************************************************************************
 */

#include "SystemDefines.hpp"
#include "GnssReadTask.hpp"
#include "GnssDriver.hpp"
#include "UbloxDriver.hpp"

/* Macros ------------------------------------------------------------------*/
#define GNSS_TASK_FREQUENCY_HZ 1

constexpr uint32_t GNSS_TASK_DELAY = 1000 / GNSS_TASK_FREQUENCY_HZ;

/**
 * @brief Constructor for GnssReadTask
 */
GnssReadTask::GnssReadTask() : Task(GNSSREAD_TASK_QUEUE_DEPTH_OBJS)
{
}

/**
 * @brief Initialize the GnssReadTask
 */
void GnssReadTask::InitTask()
{
    CUBE_ASSERT(rtTaskHandle == nullptr, "Cannot initialize GNSS read task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)GnssReadTask::RunTask,
                    (const char*)"GnssReadTask",
                    (uint16_t)GNSSREAD_TASK_STACK_DEPTH_WORDS,
                    (void*)this,
                    (UBaseType_t)GNSSREAD_TASK_RTOS_PRIORITY,
                    (TaskHandle_t*)&rtTaskHandle);

    CUBE_ASSERT(rtValue == pdPASS, "GnssReadTask::InitTask() - xTaskCreate() failed");

    /*
     * GNSS hardware/module initialization belongs here,
     * because this is the task startup.
     */
    GPSDevice::GPS_Initialization();

    CUBE_PRINT("GNSS Configurations complete\r\n");
}

uint8_t* GnssReadTask::PollData()
{
    return GnssDriver::GetLatestBuffer();
}

uint16_t GnssReadTask::CheckSum()
{
    return GnssDriver::CheckSum();
}

const NavData& GnssReadTask::LatestData()
{
    return GnssDriver::GetLatestData();
}

const uint8_t* GnssReadTask::GpsTimeData()
{
    return GnssDriver::GetGpsTimeData();
}

const uint8_t* GnssReadTask::GpsFlagsData()
{
    return GnssDriver::GetGpsFlagsData();
}

const uint8_t* GnssReadTask::GpsPositionData()
{
    return GnssDriver::GetGpsPositionData();
}

/**
 * @brief Instance Run loop for the GNSS task.
 */
void GnssReadTask::Run(void* pvParams)
{
    CUBE_PRINT("GnssReadTask - Starting run loop\r\n");

    while (1)
    {
        GnssDriver::Poll();

    	CUBE_PRINT("GnssReadTask polling GPS...\r\n");

                bool gpsOk = GnssDriver::Poll();

                if (gpsOk)
                {
                    CUBE_PRINT("GPS poll OK\r\n");
                }
                else
                {
                    CUBE_PRINT("GPS poll failed\r\n");
                }

                osDelay(GNSS_TASK_DELAY);
    }
}
