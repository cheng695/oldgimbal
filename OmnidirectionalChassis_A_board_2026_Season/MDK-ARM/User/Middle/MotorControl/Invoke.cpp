#include "Invoke.hpp"

extern "C" void Invoke()
{
    static uint8_t cycle_counter = 0;
	cycle_counter++;
    if(dr16.rc.s2 == 2 || tick_e >= 200)
    {
        M3508_201.MotorSet = M3508_201_PidInstance.NormalPID(M3508_201.speed_rpm, 0.0f);
        M3508_204.MotorSet = M3508_204_PidInstance.NormalPID(M3508_204.speed_rpm, 0.0f);
        M2006_203.MotorSet = M2006_203_PidInstance.NormalPID(M2006_203.speed_rpm, 0.0f);
        M6020_206.MotorSet = M6020_206_PidInstance.NormalPID(M6020_206.speed_rpm, 0.0f);
        if(M4310_0x01.Enable_Flag == true)
        {
            M4310_0x01.Disability(&hcan2, 0x01);
            M4310_0x01.Enable_Flag = false;
        }
    }

    else if(dr16.rc.s2 != 2)
    {
        if(M4310_0x01.Enable_Flag == false)
        {
            M4310_0x01.Enable(&hcan2, 0x01);
            M4310_0x01.Enable_Flag = true;
        }
        M3508_201.MotorSet = M3508_201_PidInstance.NormalPID(M3508_201.speed_rpm, M3508_201_Remove.target);
        M3508_204.MotorSet = M3508_204_PidInstance.NormalPID(M3508_204.speed_rpm, M3508_204_Remove.target);
        M2006_203.MotorSet = M2006_203_PidInstance.NormalPID(M2006_203.speed_rpm, M2006_203_Remove.target);
        M4310_0x01.MotorSet = M4310_0x01_PidInstance.Double_Ring_PID(M4310_0x01.angle_rad, M4310_0x01.speed_rad_s, M4310_0x01_Remove.target) + PITCH.gravity_feedback();
        if(dr16.SpeedControl_flag == true || dr16.normal == true)
        {
            if (cycle_counter >= 2)
            {
                cycle_counter = 0;
                
                //检查 IMU 数据是否就绪
                if (HI12.DataReady_flag == true)
                {
                    HI12.DataReady_flag = false;
                    // 执行 ADRC 计算
                    M6020_206.MotorSet = M6020_IMU.First_LADRC(M6020_206_Remove.target, HI12.Yaw_rpm) + YAW.angular_acceleration_feedforward(); 
                }
                //M6020_206.MotorSet = M6020_206_PidInstance_speed.NormalPID(HI12.Yaw_rpm, M6020_206_Remove.target) + YAW.angular_acceleration_feedforward();
            }
        }
        else if(dr16.PositionControl_flag == true)
        {
            M6020_206.MotorSet = M6020_206_PidInstance_Vision.Double_Ring_PID(HI12.Yaw_add, HI12.Yaw_rpm, M6020_206_Remove.target);
        }

    }


    DJImotorTX_0x200(&hcan1);
    DJImotorTX_0x1FF(&hcan1);
    M4310_0x01.ctrl_MIT(&hcan2, 0x01, 0.0f, 0.0f, 0.0f, 0.0f, M4310_0x01.MotorSet);
    //M4310_0x01.ctrl_MIT(&hcan2, 0x01, M4310_0x01_Remove.target, 0.0f, 20.0f, 1.0f, 0.0f);
}
