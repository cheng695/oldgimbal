#ifndef _REMOTEOPERATION_HPP_
#define _REMOTEOPERATION_HPP_ 

#include "stdbool.h"
#include "math.h"
#include "../User/Low/UniversalAsynchronousTransceiver/Clicker.hpp"
#include "../User/Low/MotorLibrary/DJIMotor.hpp"
#include "../User/Middle/MotorControl/ADRC.hpp"

extern Clicker::DR16 dr16;
extern ADRC::Second_ADRC_ TD_gimbal_x;
extern ADRC::Second_ADRC_ TD_gimbal_y;

namespace RemoteOperation
{
    class RemoteOperation_
    {
        public:
            float vx;
            float vy;
            float w;
    };

    class Motortarget
    {
        public:
            float target;
            void mySaturate(float *in,float min,float max);
    };
}

#ifdef __cplusplus
extern "C" {
#endif
	
void ChassisSolution();

#ifdef __cplusplus
}
#endif

void StandardizationOfWeightsAndMeasures_DR16();
void ModeSelection();
void Dead_zone();
float generate_sine(float freq);
void Judgment();
void Key_Mou();

#endif
