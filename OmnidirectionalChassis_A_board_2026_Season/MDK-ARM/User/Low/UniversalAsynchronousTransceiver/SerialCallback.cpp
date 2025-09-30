#include "SerialCallback.hpp"

extern uint8_t RX_Data[1+sizeof(pData)+sizeof(int)+sizeof(float)];
extern Clicker::DR16 dr16;
extern uint8_t pData[18];
extern uint8_t IMU_RX_Data[164];
extern IMU::IMU_ HI12;
extern ADRC::First_LADRC_ M6020;
extern int accumulate;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART6)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, RX_Data, sizeof(RX_Data));
        Communication_AtoC_Receive();
        dr16.DR16DataUpdata(pData);
    }
    
    else if(huart->Instance == UART8)
    {
        HI12.whatch_imu = accumulate;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart8, IMU_RX_Data, sizeof(IMU_RX_Data));
        HI12.HI12Recive();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE)
    {     
        if(huart->Instance == USART6)
        __HAL_UART_CLEAR_OREFLAG(&huart6);
        if(huart->Instance == UART8)
        __HAL_UART_CLEAR_OREFLAG(&huart8);
    }
}

extern "C" void RxEventCallback()
{
    Communication_AtoC_Tansmit();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, IMU_RX_Data, sizeof(IMU_RX_Data));
    HI12.HI12Recive();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, RX_Data, sizeof(RX_Data));
    Communication_AtoC_Receive();
    dr16.DR16DataUpdata(pData);
}

