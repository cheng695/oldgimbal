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

//����ң������֡ͷ��֡β���ĸ����״̬����Ҫ��ƣ�
//�գ�yaw��Ƕȣ�֡ͷ��֡β������5�����״̬��������״̬����Ҫ��ƣ�