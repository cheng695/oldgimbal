#ifndef _TASKPROCESS_HPP_
#define _TASKPROCESS_HPP_ 

#include <FreeRTOS.h>
#include <task.h>

#include "../User/Low/MotorLibrary/DJIMotor.hpp"
#include "../User/Middle/Communication/BoardCommunication_AtoC.hpp"
#include "../User/Middle/MotorControl/Invoke.hpp" 
#include "../User/Middle/Buzzer/buzzer.hpp"

extern float tick_now;

#endif
