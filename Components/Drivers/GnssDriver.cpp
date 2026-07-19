/**
 ******************************************************************************
 * File Name          : GnssDriver.cpp
 * Description        : GNSS driver
 ******************************************************************************
 */

#include "SystemDefines.hpp"
#include "GnssDriver.hpp"
#include "UbloxDriver.hpp"
#include "CubeUtils.hpp"

#include <string.h>
#include <stdio.h>

/*----------------------- Macros -----------------------*/
#define BUFFER_SIZE 36

/*----------------------- Private Data -----------------------*/
namespace
{
    uint8_t GPS_BUFFER[BUFFER_SIZE] = {0};
    NavData gpsData;

    uint8_t gps_time_data[7] = {0};
    uint8_t gps_flags_data[3] = {0};
    uint8_t gps_position_data[8] = {0};

    void PackGpsPosition()
    {
        /*
         * 0x732
         * GPS Position
         * 8 bytes
         * Longitude -> bits 0-31, Float
         * Latitude  -> bits 32-63, Float
         */
        float gps_longitude = gpsData.lon / 10000000.0f;
        float gps_latitude  = gpsData.lat / 10000000.0f;

        memcpy(&gps_position_data[0], &gps_longitude, sizeof(float));
        memcpy(&gps_position_data[4], &gps_latitude, sizeof(float));
    }
}

/*----------------------- Public Driver Functions -----------------------*/

bool GnssDriver::Poll()
{
    GPSDevice::UBX_Transmit(UBX_POLL_POSLLH, UBX_POLL_POSLLH_LEN);

    osDelay(10);

    GPSDevice::UBX_Receive(GPS_BUFFER, BUFFER_SIZE);

    uint16_t ck = UBXMessage::calculateChecksum(GPS_BUFFER, BUFFER_SIZE);

    uint16_t exp = (static_cast<uint16_t>(GPS_BUFFER[BUFFER_SIZE - 2]) << 8) |
                   (static_cast<uint16_t>(GPS_BUFFER[BUFFER_SIZE - 1]));

    if (ck == exp)
    {
        GPSDevice::UBX_M8N_NAV_POSLLH_Parsing(GPS_BUFFER, &gpsData);

        PackGpsPosition();

        CUBE_PRINT("GPS CHECKSUM OK\r\n");
        CUBE_PRINT("iTOW: %lu\r\n", (unsigned long)gpsData.iTOW);
        CUBE_PRINT("Longitude raw: %ld\r\n", (long)gpsData.lon);
        CUBE_PRINT("Latitude raw: %ld\r\n", (long)gpsData.lat);
        CUBE_PRINT("Height mm: %ld\r\n", (long)gpsData.height);
        CUBE_PRINT("hMSL mm: %ld\r\n", (long)gpsData.hMSL);
        CUBE_PRINT("hAcc mm: %lu\r\n", (unsigned long)gpsData.hAcc);
        CUBE_PRINT("vAcc mm: %lu\r\n", (unsigned long)gpsData.vAcc);
        CUBE_PRINT("-------------------------\r\n");
    }
    else
    {
    	CUBE_PRINT("GPS CHECKSUM FAILED\r\n");
    }

    CUBE_PRINT(
        "Data, iTOW: %u\n lon: %ld\n lat: %ld\n height: %ld\n hMSL: %ld\n hAcc: %u\n vAcc: %u\n",
        gpsData.iTOW,
        gpsData.lon,
        gpsData.lat,
        gpsData.height,
        gpsData.hMSL,
        gpsData.hAcc,
        gpsData.vAcc
    );

    return ck == exp;
}

uint8_t* GnssDriver::GetLatestBuffer()
{
    return GPS_BUFFER;
}

uint16_t GnssDriver::CheckSum()
{
    return UBXMessage::calculateChecksum(GPS_BUFFER, BUFFER_SIZE);
}

const NavData& GnssDriver::GetLatestData()
{
    return gpsData;
}

const uint8_t* GnssDriver::GetGpsTimeData()
{
    return gps_time_data;
}

const uint8_t* GnssDriver::GetGpsFlagsData()
{
    return gps_flags_data;
}

const uint8_t* GnssDriver::GetGpsPositionData()
{
    return gps_position_data;
}
