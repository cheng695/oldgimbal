#include "BoardCommunication_AtoC.hpp"

uint8_t TX_Data[1+sizeof(float)];
uint8_t RX_Data[1+sizeof(pData)+sizeof(int)+sizeof(float)];
uint8_t send_str2[sizeof(float) * 8]; // 分配8个float空间（32字节）
int tick_e;

extern "C" void Communication_AtoC_Receive()
{ 
    if(RX_Data[0] == 0X55)
    {
        memcpy(pData, RX_Data+sizeof(uint8_t), sizeof(pData));
        memcpy(&tick_e, RX_Data+sizeof(uint8_t)+sizeof(pData), sizeof(int));
        memcpy(&YAW.chassis_gyro, RX_Data+sizeof(uint8_t)+sizeof(pData)+sizeof(int), sizeof(float));
    }
}        

int Delay = 0;
bool Transmit_flag = true;
extern "C" void Communication_AtoC_Tansmit()
{ 
    Delay++;
    TX_Data[0] = 0X50;
    memcpy(TX_Data+sizeof(uint8_t), &M6020_206.angle_8191, sizeof(float));
    if(Transmit_flag == true)
    {
        HAL_UART_Transmit_DMA(&huart6, TX_Data, sizeof(TX_Data));
        Transmit_flag = false;
    }
    else if (Transmit_flag == false)
    {
        // if(Delay >= 10)
        // {
        //     Delay = 0;
            HAL_UART_Transmit_DMA(&huart6, TX_Data, sizeof(TX_Data));
        // }
    }
}

extern "C" void Restart()
{ 
    if(HI12.whatch_imu - accumulate > 100)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart8, IMU_RX_Data, sizeof(IMU_RX_Data));
        HI12.HI12Recive();
    }
}

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6) 
{
    const uint8_t sendSize = sizeof(float); // 单浮点数占4字节

    // 将6个浮点数据写入缓冲区（小端模式）
    *((float*)&send_str2[sendSize * 0]) = x1;
    *((float*)&send_str2[sendSize * 1]) = x2;
    *((float*)&send_str2[sendSize * 2]) = x3;
    *((float*)&send_str2[sendSize * 3]) = x4;
    *((float*)&send_str2[sendSize * 4]) = x5;
    *((float*)&send_str2[sendSize * 5]) = x6;

    // 写入帧尾（协议要求 0x00 0x00 0x80 0x7F）
    *((uint32_t*)&send_str2[sizeof(float) * 6]) = 0x7F800000; // 小端存储为 00 00 80 7F

    // 通过DMA发送完整帧（6数据 + 1帧尾 = 7个float，共28字节）
    HAL_UART_Transmit_DMA(&huart7, send_str2, sizeof(float) * 7);
}

