#include "TaskProcess.hpp"
#include <cmsis_os.h>
#include <tim.h>

extern float u_feedforward;

// extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
// 	if(htim->Instance == TIM2)
// 	{
// 		Communication_AtoC_Tansmit();
// 		Invoke();
// 		ChassisSolution();
// 		vofa_send(M6020_206_Remove.target, HI12.Yaw_rpm, HI12.Yaw_add, 0.0f, u_feedforward, M6020_IMU.z1);
// 	}
// }
int accumulate = 0;
extern float set_sin;
#ifdef __cplusplus
extern "C" {
#endif
	
void StartTask03(void const * argument)
{
	while(1)
	{
        accumulate++;
		// Communication_AtoC_Tansmit();
 		// Invoke();
 		// ChassisSolution();
		// vofa_send(M6020_206_Remove.target, HI12.Yaw_rpm, HI12.Yaw_add, 0.0f, u_feedforward, M6020_IMU.z1);
        vofa_send(M6020_IMU.z1, M6020_206_Remove.target, HI12.Yaw_rpm, M4310_0x01.angle_rad, M4310_0x01_Remove.target, PITCH.gravity_feedback());
		osDelay(1);
	}
}

void WGD(void const * argument)
{
    while(1)
    {
        if(dr16.EnablePosition_flag == true)
        {
            B_star();
            M6020_206_Remove.target = HI12.Yaw_add;
            dr16.PositionControl_flag = true;
            dr16.EnablePosition_flag = false;
        }
        else if(dr16.DisablePosition_flag == true)
        {
            B_stop();
            dr16.SpeedControl_flag = true;
            dr16.DisablePosition_flag = false;
        }

        if(HI12.whatch_imu - accumulate > 100)
        {
            B_IMU();
        }
        osDelay(1);
    }
}

#ifdef __cplusplus
}
#endif