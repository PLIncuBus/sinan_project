#ifndef __UWB_H
#define __UWB_H


#include "main.h"
#include "CRC8_CRC16.h"

#define Modbus_buffer_Length 17


typedef struct 
{
    int16_t x;
    int16_t y;
    int16_t z;
}UWB_Pos_t;

typedef struct 
{
    uint8_t Modbus_buffer[255];
    UWB_Pos_t Pos;

}UWB_t;


extern UWB_t UWB;

void UWB_Init(UWB_t *UWB_Init);
void UWB_Modbus_data_processing(UWB_t *UWB_Init);

#endif