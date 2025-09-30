#include "DJIMotor.hpp"

Can_RX_T Can_RX;
CAN_TxHeaderTypeDef DJI_txHeader;
uint8_t DJI_TX_Data[8];
uint32_t txMailbox_DJI;

motor::DJIMotor M6020_206(0x206);
motor::DJIMotor M2006_203(0x203);
motor::DJIMotor M3508_204(0x204);
motor::DJIMotor M3508_201(0x201);

motor::DJIMotorController DjimotorController;

extern RemoteOperation::Motortarget M6020_206_Remove;

void motor::DJIMotor::DataUpdate(uint8_t *data)
{
    angle_8191      =(int16_t)(data[0]<<8|data[1]);       //角度值
    speed_rpm       =(int16_t)(data[2]<<8|data[3]);       //速度
    torqueCurrent   =(int16_t)(data[4]<<8|data[5]);       //电流
    temperate       =data[6];  

    angel_360       = angle_8191 / 8191.0f * 360.0f;
    speed_rad       = speed_rpm * (MY_PI / 30.0f);  
    last_angle_360  = angel_360;
    first_angle_360 = angel_360;
}

void RmMotorRxData()
{
    switch (Can_RX.rxHeader.StdId)
    {
		case 0x201:
            M3508_201.DataUpdate(Can_RX.rxdata);
            break;

        case 0x203:
            M2006_203.DataUpdate(Can_RX.rxdata);
            M2006_203.Add_angle();
            break;

        case 0x204:
            M3508_204.DataUpdate(Can_RX.rxdata);
            break;

        case 0x206:
            M6020_206.DataUpdate(Can_RX.rxdata);
            M6020_206.Add_angle();
            break;
    }
}

bool motor::DJIMotorController::SendMotorData(CAN_HandleTypeDef* hcan, uint16_t StdId, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint8_t data[8];
    
    data[0] = static_cast<uint8_t>(motor1 >> 8);
    data[1] = static_cast<uint8_t>(motor1 & 0xFF);
    data[2] = static_cast<uint8_t>(motor2 >> 8);
    data[3] = static_cast<uint8_t>(motor2 & 0xFF);
    data[4] = static_cast<uint8_t>(motor3 >> 8);
    data[5] = static_cast<uint8_t>(motor3 & 0xFF);
    data[6] = static_cast<uint8_t>(motor4 >> 8);
    data[7] = static_cast<uint8_t>(motor4 & 0xFF);

    return send_can_message(hcan, StdId, data, sizeof(data));
}

bool motor::DJIMotorController::send_can_message(CAN_HandleTypeDef* hcan, uint16_t id, const uint8_t* data, uint8_t length)
{
    DJI_txHeader.StdId = id;
    DJI_txHeader.IDE = CAN_ID_STD;
    DJI_txHeader.RTR = CAN_RTR_DATA;
    DJI_txHeader.DLC = length;

    for (int i = 0; i < length; ++i) {
        DJI_TX_Data[i] = data[i];
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        return false;
    }

    if (HAL_CAN_AddTxMessage(hcan, &DJI_txHeader, DJI_TX_Data, &txMailbox_DJI) != HAL_OK) {
        return false;
    }

    return true;
}

void DJImotorTX_0x200(CAN_HandleTypeDef* hcan)
{
    int16_t motor1_value = (short)M3508_201.MotorSet;
    int16_t motor2_value = 0;
    int16_t motor3_value = (short)M2006_203.MotorSet;
    int16_t motor4_value = (short)M3508_204.MotorSet;

    if (!DjimotorController.SendMotorData_0x200(hcan, motor1_value, motor2_value, motor3_value, motor4_value))
    {
        // 发送失败处理
    }
}

void DJImotorTX_0x1FF(CAN_HandleTypeDef* hcan)
{
    int16_t motor1_value = 0;
    int16_t motor2_value = (short)M6020_206.MotorSet;
    int16_t motor3_value = 0;
    int16_t motor4_value = 0;

    if (!DjimotorController.SendMotorData_0x1FF(hcan, motor1_value, motor2_value, motor3_value, motor4_value))
    {
        // 发送失败处理
    }
}

void DJImotorTX_0x1FE(CAN_HandleTypeDef* hcan)
{ 
    int16_t motor1_value = 0;
    int16_t motor2_value = 0;
    int16_t motor3_value = 0;
    int16_t motor4_value = 0;

    if (!DjimotorController.SendMotorData_0x1FE(hcan, motor1_value, motor2_value, motor3_value, motor4_value))
    {
        // 发送失败处理
    }
}

void motor::DJIMotor::Add_angle()
{
    if(this->add_angle_stFlag == false)
    {
        this->Init_Flag = false;
        M6020_206_Remove.target = M6020_206.angle_8191;
        this->add_angle_stFlag = true;
    }
    if (this->Init_Flag == false)
    {
        this->last_angle_8191 = this->angle_8191;
        this->add_angle = this->angle_8191;
        this->Init_Flag = true;
    }

    if (this->angle_8191 - this->last_angle_8191 < -4096.0f) // 正转
        this->add_angle += (8192.0f - this->last_angle_8191 + this->angle_8191);
    else if (this->angle_8191 - this->last_angle_8191 > 4096.0f) // 反转
        this->add_angle += -(8192.0f - this->angle_8191 + this->last_angle_8191);
    else
        this->add_angle += (this->angle_8191 - this->last_angle_8191);

    this->last_angle_8191 = this->angle_8191;
}
