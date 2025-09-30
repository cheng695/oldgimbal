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

        // ��ȡ�����򳤶� (С�˸�ʽ)
        uint16_t payload_len = IMU_RX_Data[2] | (IMU_RX_Data[3] << 8);
        
        // �����������ݰ�Ӧ�еĳ���: ֡ͷ(2) + �����ֶ�(2) + CRC(2) + ������(payload_len)
        uint16_t expected_packet_length = 6 + payload_len;
        
        // �����ջ������Ƿ����㹻������
        // if(IMU_RX_Index < expected_packet_length) {
        //     // ���ݲ��������ȴ���������
        //     return;
        // }
        
        // ����CRCУ��ֵ
        uint16_t crc_calculated = 0;
        
        // ����֡ͷ�ͳ����ֶε�CRC (ǰ4�ֽ�)
        IMU::IMU_::crc16_update(&crc_calculated, IMU_RX_Data, 4);
        
        // �����������CRC (�ӵ�6�ֽڿ�ʼ������Ϊpayload_len)
        IMU::IMU_::crc16_update(&crc_calculated, IMU_RX_Data + 6, payload_len);
        
        // �����ݰ�����ȡ���յ���CRCֵ (С�˸�ʽ��λ�ڵ�4-5�ֽ�)
        uint16_t crc_received = IMU_RX_Data[4] | (IMU_RX_Data[5] << 8);
        
        // ��֤CRC
        if(crc_calculated != crc_received)
        {
            // CRCУ��ʧ�ܣ��������ݰ�
            this->FrameHeader_flag = false;
            // // IMU_RX_Index = 0; // ���ý�������
            // // ������������Ӵ����������־��¼
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

    if (this->Yaw_8192 - this->last_Yaw_8192 < -4096.0f) // ��ת
        this->Yaw_add += (8192.0f - this->last_Yaw_8192 + this->Yaw_8192);
    else if (this->Yaw_8192 - this->last_Yaw_8192 > 4096.0f) // ��ת
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
