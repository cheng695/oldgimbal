#ifndef _BOARDCOMMUNICATION_AtoC_HPP_
#define _BOARDCOMMUNICATION_AtoC_HPP_ 

#include "string.h"
#include "usart.h"
#include "../User/Low/MotorLibrary/DJIMotor.hpp"
#include "../User/Low/UniversalAsynchronousTransceiver/Clicker.hpp"
#include "../User/Low/UniversalAsynchronousTransceiver/IMU.hpp"
#include "../User/Middle/MotorControl/Feedback.hpp"

extern motor::DJIMotor M6020_206;
extern RemoteOperation::Motortarget M6020_206_Remove;
extern IMU::IMU_ HI12;
extern feedback::Acceleration YAW;
extern int tick_e;
extern int accumulate;
extern uint8_t IMU_RX_Data[164];

#ifdef __cplusplus
extern "C" {
#endif
	
void Communication_AtoC_Receive(void);
void Communication_AtoC_Tansmit(void);
void Restart(void);

#ifdef __cplusplus
}
#endif

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);

#endif

//发：遥控器，帧头，帧尾，四个电机状态（需要设计）
//收：yaw轴角度，帧头，帧尾，上面5个电机状态，陀螺仪状态（需要设计）