#include "IMU.hpp"

uint8_t IMU_RX_Data[54];
IMU::IMU_ HI12;
float IMU::IMU_::R4(uint8_t *p) 
{
    float r; 
    memcpy(&r,p,4); 
    return r;
};

int16_t IMU::IMU_::Init16(uint8_t *p) 
{
    int16_t r; 
    memcpy(&r,p,2); 
    return r;
};

int32_t IMU::IMU_::Init32(uint8_t *p) 
{
    int32_t r; 
    memcpy(&r,p,4); 
    return r;
};

void IMU::IMU_::HI12Recive()
{
    if(IMU_RX_Data[0] == 0X5A && IMU_RX_Data[1] == 0XA5)
    {
        this->FrameHeader_flag = true;
    }

    if(this->FrameHeader_flag == true)
    {

        // 获取数据域长度 (小端格式)
        uint16_t payload_len = IMU_RX_Data[2] | (IMU_RX_Data[3] << 8);
        
        // 计算整个数据包应有的长度: 帧头(2) + 长度字段(2) + CRC(2) + 数据域(payload_len)
        uint16_t expected_packet_length = 6 + payload_len;
        
        // 检查接收缓冲区是否有足够的数据
        // if(IMU_RX_Index < expected_packet_length) {
        //     // 数据不完整，等待更多数据
        //     return;
        // }
        
        // 计算CRC校验值
        uint16_t crc_calculated = 0;
        
        // 计算帧头和长度字段的CRC (前4字节)
        IMU::IMU_::crc16_update(&crc_calculated, IMU_RX_Data, 4);
        
        // 计算数据域的CRC (从第6字节开始，长度为payload_len)
        IMU::IMU_::crc16_update(&crc_calculated, IMU_RX_Data + 6, payload_len);
        
        // 从数据包中提取接收到的CRC值 (小端格式，位于第4-5字节)
        uint16_t crc_received = IMU_RX_Data[4] | (IMU_RX_Data[5] << 8);
        
        // 验证CRC
        if(crc_calculated != crc_received)
        {
            // CRC校验失败，丢弃数据包
            this->FrameHeader_flag = false;
            // // IMU_RX_Index = 0; // 重置接收索引
            // // 可以在这里添加错误计数或日志记录
            // return;
        }

        if(this->FrameHeader_flag == true)
        {
            this->X_Acc = Init16(IMU_RX_Data+6+16) * 0.0048828f;
            this->Y_Acc = Init16(IMU_RX_Data+6+18) * 0.0048828f;
            this->Z_Acc = Init16(IMU_RX_Data+6+20) * 0.0048828f;

            this->Pitch_Gyro = Init16(IMU_RX_Data+6+10) * 0.001f * 57.3f;
            this->Roll_Gyro  = Init16(IMU_RX_Data+6+12) * 0.001f * 57.3f;
            this->Yaw_Gyro   = Init16(IMU_RX_Data+6+14) * 0.001f * 57.3f;

            this->Roll  = Init32(IMU_RX_Data+6+28) * 0.001f;
            this->Pitch = Init32(IMU_RX_Data+6+32) * 0.001f;
            this->Yaw   = Init32(IMU_RX_Data+6+36) * 0.001f;

            this->q_w = Init16(IMU_RX_Data+6+40) * 0.0001f;
            this->q_x = Init16(IMU_RX_Data+6+42) * 0.0001f;
            this->q_y = Init16(IMU_RX_Data+6+44) * 0.0001f;
            this->q_z = Init16(IMU_RX_Data+6+46) * 0.0001f;

            this->Pitch_8192 = (this->Pitch + 90.0f) / 180.0f * 8192.0f;
            this->Pitch_rad  = (this->Pitch + 90.0f) * (MY_PI / 180.0f);
            this->Yaw_8192   = (this->Yaw + 180.0f) / 360.0f * 8192.0f;
            this->Yaw_rpm    = this->Yaw_Gyro /6.0f; 
            Add_angle();

            this->DataReady_flag = true;
            this->FrameHeader_flag = false;
        }
    }   
}

void IMU::IMU_::Add_angle()
{
    if (this->Init_Flag == false)
    {
        this->last_Yaw_8192 = this->Yaw_8192;
        this->Yaw_add = this->Yaw_8192;
        this->Init_Flag = true;
    }

    if (this->Yaw_8192 - this->last_Yaw_8192 < -4096.0f) // 正转
        this->Yaw_add += (8192.0f - this->last_Yaw_8192 + this->Yaw_8192);
    else if (this->Yaw_8192 - this->last_Yaw_8192 > 4096.0f) // 反转
        this->Yaw_add += -(8192.0f - this->Yaw_8192 + this->last_Yaw_8192);
    else
        this->Yaw_add += (this->Yaw_8192 - this->last_Yaw_8192);

    this->last_Yaw_8192 = this->Yaw_8192;
}

void IMU::IMU_::crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j=0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *currectCrc = crc;
}
