#include "RemoteOperation.hpp"

RemoteOperation::RemoteOperation_ VehicleDirection;
RemoteOperation::Motortarget M3508_201_Remove;
RemoteOperation::Motortarget M3508_204_Remove;
RemoteOperation::Motortarget M2006_203_Remove;
RemoteOperation::Motortarget M6020_206_Remove;
RemoteOperation::Motortarget M4310_0x01_Remove;

extern motor::DJIMotor M6020_206;
bool dead_zone_flag0 = false;
bool dead_zone_flag1 = false;
bool dead_zone_flag2 = false;
bool dead_zone_flag3 = false;
bool dead_zone_flag4 = false;

extern "C" void ChassisSolution()
{
  StandardizationOfWeightsAndMeasures_DR16();
  Judgment();
  ModeSelection();
}


void StandardizationOfWeightsAndMeasures_DR16()
{
    Dead_zone();

    //if()
    //{
      Key_Mou();
    //}
    
    if(dead_zone_flag3 == true)
    {
      dr16.vx_left = 0.0f;//vx,vx是对遥控器而言
    }
    else
    {
      dr16.vx_left = (dr16.rc.ch3 - 1024.0f)/660.0f;//vx,vx是对遥控器而言
    }

    if(dead_zone_flag2 == true)
    {
      dr16.vy_left = 0.0f;
    }
    else
    {
      dr16.vy_left = (dr16.rc.ch2 - 1024.0f)/660.0f;
    }

    if(dead_zone_flag1 == true)
    {
      dr16.vx_right = 0.0f;
    }
    else
    {
      dr16.vx_right = (dr16.rc.ch1 - 1024.0f)/660.0f;
    }

    if(dead_zone_flag0 == true)
    {
      dr16.vy_right = 0.0f;
    }
    else
    {
      dr16.vy_right = (dr16.rc.ch0 - 1024.0f)/660.0f;
    }

    if(dead_zone_flag4 == true)
    {
      dr16.Roller = (dr16.reserved_.reserved - 1024.0f)/660.0f;
    }
    else
    {
      dr16.Roller = (dr16.reserved_.reserved - 1024.0f)/660.0f;
    }      
}

void Dead_zone()
{
    if(dr16.rc.ch0 - 1024.0f < 15.0f && dr16.rc.ch0 - 1024.0f > -15.0f)
    {
      dead_zone_flag0 = true;
    }
    else
    {
      dead_zone_flag0 = false;
    }

    if(dr16.rc.ch1 - 1024.0f < 15.0f && dr16.rc.ch1 - 1024.0f > -15.0f)
    {
      dead_zone_flag1 = true;
    }
    else
    {
      dead_zone_flag1 = false;
    }

    if(dr16.rc.ch2 - 1024.0f < 15.0f && dr16.rc.ch2 - 1024.0f > -15.0f)
    {
      dead_zone_flag2 = true;
    }
    else
    {
      dead_zone_flag2 = false;
    }

    if(dr16.rc.ch3 - 1024.0f < 15.0f && dr16.rc.ch3 - 1024.0f > -15.0f)
    {
      dead_zone_flag3 = true;
    }
    else
    {
      dead_zone_flag3 = false;
    }

    if(dr16.reserved_.reserved - 1024.0f < 15.0f && dr16.reserved_.reserved - 1024.0f > -15.0f)
    {
      dead_zone_flag4 = true;
    }
    else
    {
      dead_zone_flag4 = false;
    }
}

int i = 0;

void ModeSelection()
{
    if(dr16.rc.s2 == 3)//不发
    {
        dr16.transform_flag = 0;

        M3508_201_Remove.target = 0.0f;
        M3508_204_Remove.target = 0.0f;
        M2006_203_Remove.target = 0.0f;
        if(i == 0)
        {
          if(dr16.SpeedControl_flag == true || dr16.normal == true)
          {
            dr16.PositionControl_flag = false;
            M6020_206_Remove.target = -132.0f / 4.0f * dr16.vy_right + (-132.0f * 80.0f * dr16.mou_x); //rpm
          }
          
          else if(dr16.PositionControl_flag == true)
          {
            dr16.SpeedControl_flag = false;
            M6020_206_Remove.target -= 8191.0f * dr16.vy_right * 0.001f * 2.0f + (8191.0f * dr16.mou_x * 0.001f * 80.0f);
          }
        }

        else if(i == 1)
        {
          M6020_206_Remove.target = generate_sine(1.0f);
        }

        M4310_0x01_Remove.target -= dr16.vx_right / 1000.0f * 4.0f + (dr16.mou_y / 1000.0f * 3000.0f);
        M4310_0x01_Remove.mySaturate(&M4310_0x01_Remove.target, -0.42f, 0.58f);
    }

    if(dr16.rc.s2 == 1)//发
    {
        M3508_201_Remove.target = 6000.0f;
        M3508_204_Remove.target = -6000.0f;
        M2006_203_Remove.target = 4400.0f * dr16.Roller;
        if(i == 0)
        {
          M6020_206_Remove.target = -132.0f / 4.0f * dr16.vy_right + (-132.0f * dr16.mou_x);
        }
        else if(i == 1)
        {
          M6020_206_Remove.target = generate_sine(5.0f);
        }
        M4310_0x01_Remove.target -= dr16.vx_right / 1000.0f * 4.0f + (dr16.mou_y / 1000.0f * 4.0f);
        M4310_0x01_Remove.mySaturate(&M4310_0x01_Remove.target, -0.42f, 0.58f);
    }

    if(dr16.rc.s2 == 2)
    {
      if(dr16.transform_flag == 0)
      {
        dr16.SpeedControl_flag = false;
        dr16.PositionControl_flag = false;
      }
      dr16.transform_flag = 1;
    }
}

void RemoteOperation::Motortarget::mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}

float phase = 0.0f;  // 静态变量保持相位连续
#define SAMPLE_RATE   1000.0f  // 采样率1kHz
float set_sin;
float generate_sine(float freq)	
{

    float phase_increment;
		phase_increment = 2 * MY_PI * freq / SAMPLE_RATE;
    

		set_sin = 132.0f / 2.0f * sinf(phase);  // 计算当前相位正弦值
    phase += phase_increment; // 更新相位
        
        // 相位回绕（避免浮点溢出）
    if(phase >= 2*MY_PI)
		{
			phase -= 2*MY_PI;
		}
    return set_sin;
}

void Judgment()
{
    if(dr16.SpeedControl_flag == false && dr16.PositionControl_flag == false)
    {
        dr16.normal = true;
    }

    if(dr16.rc.s2 == 2)
    {
        if(dr16.rc.ch1 >= 1354)
        {
            dr16.right_above = true;
        }
        else if(dr16.rc.ch1 <= 694)
        {
            dr16.right_under = true;
        }

        else if(dr16.rc.ch0 >= 1354)
        {
            dr16.right_left = true;
        }
        else if(dr16.rc.ch0 <= 694)
        {
            dr16.right_right = true;
        }

        if(dr16.right_above == true && dr16.right_under == true)
        {
            dr16.EnablePosition_flag = true;
            dr16.normal = false;
            dr16.right_above = false;
            dr16.right_under = false;
        }

        if(dr16.right_left  == true && dr16.right_right == true)
        {
            dr16.DisablePosition_flag = true;
            dr16.normal = false;
            dr16.right_left  = false;
            dr16.right_right = false;
        }
    }
}

void Key_Mou()
{
  dr16.mou_x = dr16.mouse.x / 32767.0f;
  dr16.mou_y = dr16.mouse.y / 32767.0f;
  dr16.mou_z = dr16.mouse.z / 32767.0f; 
  TD_gimbal_x.TdFilter(dr16.mou_x);
  TD_gimbal_y.TdFilter(dr16.mou_y);
  dr16.mou_x = TD_gimbal_x.V1;
  dr16.mou_y = TD_gimbal_y.V1;
}

