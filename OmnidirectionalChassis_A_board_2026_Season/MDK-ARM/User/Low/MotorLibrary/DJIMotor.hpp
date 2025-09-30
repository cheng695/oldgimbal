#ifndef _DJIMOTOR_HPP_
#define _DJIMOTOR_HPP_

#include <stdint.h>
#include <can.h>
#include "../User/Middle/RemoteControl/RemoteOperation.hpp"

#define MY_PI 3.1415926535897932384626433832795f

extern CAN_TxHeaderTypeDef DJI_txHeader;
extern uint8_t DJI_TX_Data[8];
extern uint32_t txMailbox_DJI;

typedef struct
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxdata[8];
}Can_RX_T;
extern Can_RX_T Can_RX;

namespace motor
{
    class DJIMotor
    {
    public:
        DJIMotor(uint32_t recv_id) { recv_id_ = recv_id; };
        
        uint32_t recv_id_;              //接收id
        float angle_8191;               //转子机械角度  0-8191
        int16_t speed_rpm;                //转子速度      rpm
        float torqueCurrent;            //实际扭矩电流
        uint8_t  temperate;             //电机温度      摄氏度
        float torque;                   //电机扭矩

        float angel_360;                //转子机械角度  0-360
        float speed_rad;                //转子速度      rad
        float last_angle_8191;          //上一次转子机械角度  8191-0
        float last_angle_360;           //上一次转子机械角度  0-360
        float first_angle_360;          //第一次转子机械角度  0-360
        float add_angle;                //电机角度增量
        //float 

        bool add_angle_stFlag = false;
        bool Init_Flag;

        float MotorSet;                 //电机设置值
        void DataUpdate(uint8_t *data);
        void Add_angle();
				
    };

    class DJIMotorController
    {
    public:
        DJIMotorController() = default;

        bool SendMotorData(CAN_HandleTypeDef* hcan, uint16_t StdId, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
        
        bool SendMotorData_0x200(CAN_HandleTypeDef* hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
        {
            return SendMotorData(hcan, 0x200, motor1, motor2, motor3, motor4);
        }

        bool SendMotorData_0x1FF(CAN_HandleTypeDef* hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
        {
            return SendMotorData(hcan, 0x1FF, motor1, motor2, motor3, motor4);
        }

        bool SendMotorData_0x1FE(CAN_HandleTypeDef* hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
        {
            return SendMotorData(hcan, 0x1FE, motor1, motor2, motor3, motor4);
        }

    private:
        bool send_can_message(CAN_HandleTypeDef* hcan, uint16_t id, const uint8_t* data, uint8_t length);
    };
}

void RmMotorRxData(void);
void DJImotorTX_0x200(CAN_HandleTypeDef* hcan);
void DJImotorTX_0x1FF(CAN_HandleTypeDef* hcan);
void DJImotorTX_0x1FE(CAN_HandleTypeDef* hcan);

#endif
