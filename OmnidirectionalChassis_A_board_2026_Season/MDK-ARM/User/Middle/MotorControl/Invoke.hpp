#ifndef _INVOKE_HPP_
#define _INVOKE_HPP_ 

#include "PID.hpp"
#include "../User/Low/UniversalAsynchronousTransceiver/Clicker.hpp"
#include "../User/Low/MotorLibrary/DJIMotor.hpp"
#include "../User/Low/MotorLibrary/DaMiao.hpp"
#include "../User/Low/UniversalAsynchronousTransceiver/IMU.hpp"
#include "../User/Low/UniversalAsynchronousTransceiver/SerialCallback.hpp"
#include "../User/Middle/MotorControl/ADRC.hpp"

extern int tick_e;

extern motor::DJIMotor M6020_206;
extern motor::DJIMotor M2006_203;
extern motor::DJIMotor M3508_204;
extern motor::DJIMotor M3508_201;
extern motor::DaMiaoMotor M4310_0x01;

extern RemoteOperation::Motortarget M3508_201_Remove;
extern RemoteOperation::Motortarget M3508_204_Remove;
extern RemoteOperation::Motortarget M2006_203_Remove;
extern RemoteOperation::Motortarget M6020_206_Remove;
extern RemoteOperation::Motortarget M4310_0x01_Remove;

extern PID::PID_ M3508_201_PidInstance;
extern PID::PID_ M3508_204_PidInstance;
extern PID::PID_ M2006_203_PidInstance;
extern PID::PID_ M6020_206_PidInstance;
extern PID::PID_ M4310_0x01_PidInstance;
extern PID::PID_ Counter_rotation_PidInstance;
extern PID::PID_ M6020_206_PidInstance_Vision;
extern PID::PID_ M6020_206_PidInstance_speed;

extern IMU::IMU_ HI12;
extern ADRC::First_LADRC_ M6020;
extern ADRC::First_LADRC_ M6020_IMU;
extern feedback::gravity PITCH;

#ifdef __cplusplus
extern "C" {
#endif
	
void Invoke(void);

#ifdef __cplusplus
}
#endif

#endif
