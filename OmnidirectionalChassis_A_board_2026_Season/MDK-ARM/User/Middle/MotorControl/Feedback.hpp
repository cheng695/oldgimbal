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
            float chassis_gyro;             //���̽��ٶ� dps

            float chassis_angularacc;       //���̽Ǽ��ٶ�
            float U_ff;                     //ǰ��
            float K_ff_final = 0.0f;//-0.18f;               //ǰ��ϵ��
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
