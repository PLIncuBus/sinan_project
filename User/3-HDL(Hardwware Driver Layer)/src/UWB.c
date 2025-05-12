#include "UWB.h"

UWB_t UWB;


void UWB_Init(UWB_t *UWB_Init)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2,UWB.Modbus_buffer,sizeof(UWB.Modbus_buffer));
}


void UWB_Modbus_data_processing(UWB_t *UWB_Init)
{
    if((UWB_Init->Modbus_buffer[0] == 0X01) && 
    (UWB_Init->Modbus_buffer[1] == 0X03) &&
    (UWB_Init->Modbus_buffer[3] == 0XAC) &&
    (UWB_Init->Modbus_buffer[4] == 0XDA) &&
    (UWB_Init->Modbus_buffer[5] == 0X00))// &&(1 == verify_CRC16_check_sum(UWB_Init->Modbus_buffer,Modbus_buffer_Length)))
    {   
        UWB_Init->Pos.x = (UWB_Init->Modbus_buffer[9] << 8) | UWB_Init->Modbus_buffer[10];
        UWB_Init->Pos.y = (UWB_Init->Modbus_buffer[11] << 8 )| UWB_Init->Modbus_buffer[12];
        UWB_Init->Pos.z = (UWB_Init->Modbus_buffer[13] << 8) | UWB_Init->Modbus_buffer[14];
    }

}