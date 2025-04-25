#include "Callback_Uart.h"


//串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == USART1)
    {
			IMU_uart_callback(&IMU931);
			HAL_UART_Receive_IT(&huart1,&IMU931.rx_origin_data,1);
		}
}
//串口空闲中断回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    if(huart->Instance == USART2)
    {
        UWB_Modbus_data_processing(&UWB);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2,UWB.Modbus_buffer,sizeof(UWB.Modbus_buffer));
    }

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2,UWB.Modbus_buffer,sizeof(UWB.Modbus_buffer));
    }
}



