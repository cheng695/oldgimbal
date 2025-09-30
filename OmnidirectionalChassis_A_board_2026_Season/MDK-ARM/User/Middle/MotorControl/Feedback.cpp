#include "Feedback.hpp"

feedback::Acceleration YAW;
feedback::gravity PITCH;

float feedback::Acceleration::angular_acceleration_feedforward()
{
     this->chassis_gyro *= 0.00551156f; 
    // TD_chassis_imu.TdFilter(this->chassis_gyro); 
    // this->chassis_angularacc = TD_chassis_imu.V2;
    // if(TD_chassis_imu.V2 > 0.01f || TD_chassis_imu.V2 < -0.01f)
    // {
    //     this->U_ff = this->K_ff_final * this->chassis_angularacc;
    // }
    // else
    // {
    //     this->U_ff = 0.0f;
    // }
    this->U_ff = this->K_ff_final * this->chassis_gyro;

    return this->U_ff;
}

float feedback::gravity::gravity_feedback()
{
    this->pitch_rad = HI12.Pitch_rad;
    this->Feedforward = this->Parameter * this->g * this->pitch_rad;
    return this->Feedforward;
}
