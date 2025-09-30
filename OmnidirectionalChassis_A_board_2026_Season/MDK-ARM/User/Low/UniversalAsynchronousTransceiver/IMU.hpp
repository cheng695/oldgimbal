#ifndef _IMU_Hpp_
#define _IMU_Hpp_

#include "usart.h"
#include "dma.h"
#include "string.h"
#include "../User/Low/MotorLibrary/DJIMotor.hpp"

namespace IMU
{
    class IMU_
    {
        public:
            bool FrameHeader_flag = false;
            bool Init_Flag = false;
            bool DataReady_flag = false;

            float Roll;              //角度
            float Pitch;
            float Yaw;

            float Pitch_8192;
            float Pitch_rad;
            float Yaw_8192;
            float last_Yaw_8192;
            float Yaw_add;
            float Yaw_rpm;

            float Roll_Gyro;        //角速度
            float Pitch_Gyro;
            float Yaw_Gyro;

            float X_Acc;            //加速度
            float Y_Acc;
            float Z_Acc;

            float q_w;              //四元数
            float q_x;
            float q_y;
            float q_z;

            int whatch_imu; 

            void HI12Recive(void);
            void Add_angle(void);
            void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes);
        private:
            float R4(uint8_t *p);
            int32_t Init32(uint8_t *p);
            int16_t Init16(uint8_t *p); 
    };
}


#endif
