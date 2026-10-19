/**
 * 
 * File Name    : CAN.hpp
 * Description  :
 *      
 *      Contains struct definitions and other utilities for ease of use when using FDCAN peripheral
 * 
 */

#ifndef HELIOS_CAN_H
#define HELIOS_CAN_H

typedef struct {
    uint16_t ID;
    uint32_t extendedID;
    uint8_t DLC;
    uint8_t data[8];
} CANMsg;

#endif 