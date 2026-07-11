#ifndef HELIOS_GNSSDRIVER_HPP_
#define HELIOS_GNSSDRIVER_HPP_

#include <stdint.h>
#include "UbloxDriver.hpp"

class GnssDriver
{
public:
    static bool Poll();

    static uint8_t* GetLatestBuffer();
    static uint16_t CheckSum();

    static const NavData& GetLatestData();

    static const uint8_t* GetGpsTimeData();
    static const uint8_t* GetGpsFlagsData();
    static const uint8_t* GetGpsPositionData();

private:
    GnssDriver();
};

#endif
