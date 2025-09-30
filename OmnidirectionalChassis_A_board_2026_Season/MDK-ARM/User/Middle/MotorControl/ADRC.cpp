#include "ADRC.hpp"

ADRC::First_LADRC_ M6020(0.001f, 80.0f, 60.0f, 0.15f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
ADRC::First_LADRC_ M6020_IMU(0.005f, 88.0f, 22.0f, 0.065f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
ADRC::Second_ADRC_ TD_chassis_imu(250.0f, 0.001f, 0.0f, 0.0f, 0.0f);
ADRC::Second_ADRC_ TD_gimbal_x(50.0f, 0.001f, 0.0f, 0.0f, 0.0f);
ADRC::Second_ADRC_ TD_gimbal_y(80.0f, 0.001f, 0.0f, 0.0f, 0.0f);

void ADRC::Second_ADRC_::TdFilter(float Input)//滤波的值，调r，r越大滤波越差，r越小越滞后 H0.001，
{
    float fh= -this->R * this->R * (this->V1 - Input) - 2 * this->R * this->V2;
    this->V1 += this->V2 * this->H;
    this->V2 += fh * this->H;
}

float ADRC::Second_ADRC_::Second_LADRC(float input, float feedback)
{
    ADRC::Second_ADRC_::TdFilter(input);
    ADRC::Second_ADRC_::Second_LESO(feedback);
    ADRC::Second_ADRC_::Second_LSEF();
    this->u = (this->u0 - this->z3) / this->b0;
    return this->u;
}

void ADRC::Second_ADRC_::Second_LESO(float feedback)
{
    this->z1 += this->H * (this->z2 + this->beta1 * (feedback - this->z1));
    this->z2 += this->H * (this->z3 + this->beta2 * (feedback - this->z1) + this->b0 * this->u);
    this->z3 += this->H * (this->beta3 * (feedback - this->z1));
}

void ADRC::Second_ADRC_::Second_LSEF()
{
    this->kp = this->wc * this->wc;
    this->kd = 2.0f*this->wc;
    this->u0 = this->kp * (this->V1 - this->z1) + this->kd * (this->V2 - this->z2);
}


// 前馈控制量 (需要根据系统调整前馈增益k_ff)
float k_ff = 0.0f; // 前馈增益，需要根据系统调整
float u_feedforward = 0.0f;
float prev_input = 0.0f;
float input_derivative = 0.0f;


float ADRC::First_LADRC_::First_LADRC(float input, float feedback)
{
    this->input = input;
    this->feedback = feedback;
    ADRC::First_LADRC_::First_LSEF();
    ADRC::First_LADRC_::First_LESO(this->feedback);
    this->e2 = input - this->z1;
    this->e4 = this->kp * this->e2;
    this->e3 = this->kp * this->e2 - this->z2;
    this->u = this->e3 / this->b0;

    input_derivative = (input - prev_input) / this->H;
    prev_input = input;
    u_feedforward = k_ff * input_derivative;
    this->u = this->u + u_feedforward;

    ADRC::First_LADRC_::Limit_max(&this->u, 25000.0f);
    return this->u;
}
    
void ADRC::First_LADRC_::First_LESO(float feedback)
{
    this->e1 = feedback - this->z1;
    this->z1 += this->H * (this->beta1 * this->e1 + this->z2 + this->b0 * this->u);
    this->z2 += this->H * (this->beta2 * this->e1);
}

void ADRC::First_LADRC_::First_LSEF()
{
    this->kp = this->wc;
    this->beta1 = 2.0f * this->w0;
    this->beta2 = this->w0 * this->w0;
}

void ADRC::First_LADRC_::Limit_max(float *out, float max_out)//限幅
{
    if(*out >= max_out)
    {
        *out = max_out;
    }
    else if(*out <= -max_out)
    {
        *out = -max_out;
    }
}
