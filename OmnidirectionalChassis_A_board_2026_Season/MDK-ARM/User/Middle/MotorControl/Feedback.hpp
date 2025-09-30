#ifndef _FEEDBACK_HPP_
#define _FEEDBACK_HPP_ 

#include "math.h"
#include "../User/Middle/RemoteControl/RemoteOperation.hpp"
#include "../User/Middle/MotorControl/ADRC.hpp"
#include "../User/Low/UniversalAsynchronousTransceiver/IMU.hpp"

extern RemoteOperation::Motortarget M6020_206_Remove;
extern ADRC::Second_ADRC_ TD_chassis_imu;
extern IMU::IMU_ HI12;

namespace feedback
{
    class Acceleration
    {
        public:
            float chassis_gyro;             //底盘角速度 dps

            float chassis_angularacc;       //底盘角加速度
            float U_ff;                     //前馈
            float K_ff_final = 0.0f;//-0.18f;               //前馈系数
            float angular_acceleration_feedforward();
    };

    class gravity
    {
        public: //mgr*cos(pitch)
            float Parameter = -0.03f;   //m*r
            float g = 9.8f;
            float pitch_rad;
            float Feedforward;
            float gravity_feedback();
    };
}


#endif
