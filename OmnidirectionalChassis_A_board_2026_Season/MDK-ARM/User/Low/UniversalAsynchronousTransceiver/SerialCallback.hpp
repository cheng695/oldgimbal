#ifndef _SERIALCALLBACK_HPP_
#define _SERIALCALLBACK_HPP_ 

#include "../User/Low/UniversalAsynchronousTransceiver/Clicker.hpp"
#include "../User/Middle/Communication/BoardCommunication_AtoC.hpp"
#include "../User/Middle/RemoteControl/RemoteOperation.hpp"
#include "../User/Low/UniversalAsynchronousTransceiver/IMU.hpp"
#include "../User/Middle/MotorControl/ADRC.hpp"

#ifdef __cplusplus
extern "C" {
#endif
	
void RxEventCallback(void);

#ifdef __cplusplus
}
#endif



#endif
