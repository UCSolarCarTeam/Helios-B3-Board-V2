/**
 ******************************************************************************
 * File Name          : GnssReadTask.hpp
 * Description        : Polls and updates GNSS/GPS data.
 ******************************************************************************
 */
#ifndef HELIOS_GNSSREADTASK_HPP_
#define HELIOS_GNSSREADTASK_HPP_

#include "Task.hpp"
#include "SystemDefines.hpp"
#include "GnssDriver.hpp"

/* Macros/Enums ------------------------------------------------------------*/
enum GNSSREAD_COMMANDS
{
    GNSSREAD_NONE = 0,
    GNSSREAD_STATE,
    GNSSREAD_RESET
};

/* Task Implementation -----------------------------------------------------*/
class GnssReadTask : public Task
{
public:
    static GnssReadTask& Inst()
    {
        static GnssReadTask inst;
        return inst;
    }

    void InitTask();

    /** Getters for CAN formatting */
    uint8_t GPSData();
    uint8_t* PollData();
    uint16_t CheckSum();
    const NavData& LatestData();

    const uint8_t* GpsTimeData();
    const uint8_t* GpsFlagsData();
    const uint8_t* GpsPositionData();

protected:
    static void RunTask(void* pvParams) { GnssReadTask::Inst().Run(pvParams); }
    void Run(void* pvParams);

private:
    // Private Functions
    GnssReadTask();                                      // Private constructor
    GnssReadTask(const GnssReadTask&);                   // Prevent copy-construction
    GnssReadTask& operator=(const GnssReadTask&);        // Prevent assignment
};

#endif // HELIOS_GNSSREADTASK_HPP_
