#include "DaMiao.hpp"

CAN_TxHeaderTypeDef DaMiao_txHeader;
uint8_t DaMiao_TX_Data[8];
uint32_t txMailbox_DaMiao;
Can_RX_T2 Can_RX2;

motor::DaMiaoMotor M4310_0x01
(    
    -12.56f, 12.56f,  // P_MIN, P_MAX: 根据上位机PMAX=12.56设置
    -30.0f, 30.0f,    // V_MIN, V_MAX: 根据上位机VMAX=30设置
    -10.0f, 10.0f,    // T_MIN, T_MAX: 根据上位机TMAX=10设置
    0.0f, 500.0f,     // KP_MIN, KP_MAX (这些是MIT模式的增益范围，与上位机"控制幅值"无关，通常保持默认即可)
    0.0f, 5.0f,       // KD_MIN, KD_MAX (同上，保持默认)
    0x02              // recv_id: 这个值需要根据上位机"驱动参数"中的Master ID来设置
);

float motor::DaMiaoMotor::uint_to_float(int x_int, float min, float max, int bits) const 
{
    const float span = max - min;
    const float max_value = (1 << bits) - 1;
    return ((float)x_int) * span / max_value + min;
}

int motor::DaMiaoMotor::float_to_uint(float x, float min, float max, int bits) const 
{
    const float span = max - min;
    const float max_value = (1 << bits) - 1;
    return (int)((x - min) * max_value / span);
}

bool motor::DaMiaoMotor::send_can_message(CAN_HandleTypeDef* hcan, uint16_t id, const uint8_t* data, uint8_t length)
{
    DaMiao_txHeader.StdId = id;
    DaMiao_txHeader.IDE = CAN_ID_STD;
    DaMiao_txHeader.RTR = CAN_RTR_DATA;
    DaMiao_txHeader.DLC = length;

    for (int i = 0; i < length; ++i) {
        DaMiao_TX_Data[i] = data[i];
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        return false;
    }

    if (HAL_CAN_AddTxMessage(hcan, &DaMiao_txHeader, DaMiao_TX_Data, &txMailbox_DaMiao) != HAL_OK) {
        return false;
    }

    return true;
}

void motor::DaMiaoMotor::ctrl_MIT(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,
    float _KP, float _KD, float _torq)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;

    if (!send_can_message(hcan, id, data, sizeof(data))) 
    {

    }
}

void motor::DaMiaoMotor::ctrl_AngleSpeed(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel)
{

    uint8_t data[8];
    uint8_t *pbuf,*vbuf;

    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;

    for (int i = 0; i < 4; ++i) 
    {
        data[i] = pbuf[i];
        data[4 + i] = vbuf[i];
    }

    if (!send_can_message(hcan, id, data, sizeof(data)))
    {
        // 错误处理，可根据需要添加日志或错误处理逻辑
    }
}

void motor::DaMiaoMotor::ctrl_Velocity(CAN_HandleTypeDef* hcan,uint16_t id, float _vel)
{

    uint8_t data[8];
    uint8_t *vbuf;

    for (int i = 0; i < 4; ++i) 
    {
        data[i] = vbuf[i];
    }

    if (!send_can_message(hcan, id, data, sizeof(data)))
    {
        // 错误处理，可根据需要添加日志或错误处理逻辑
    }

}

void motor::DaMiaoMotor::Enable(CAN_HandleTypeDef* hcan,uint16_t id)
{
    uint8_t data[8];

    for (int i = 0; i < 7; ++i) 
    {
        data[i] = 0xFF;
    }
    data[7] = 0xFC;

    if (!send_can_message(hcan, id, data, sizeof(data)))
    {
        // 错误处理，可根据需要添加日志或错误处理逻辑
    }
}

void motor::DaMiaoMotor::Disability(CAN_HandleTypeDef* hcan,uint16_t id)
{
    uint8_t data[8];

    for (int i = 0; i < 7; ++i) 
    {
        data[i] = 0xFF;
    }
    data[7] = 0xFD;

    if (!send_can_message(hcan, id, data, sizeof(data)))
    {
        // 错误处理，可根据需要添加日志或错误处理逻辑
    }
}

void motor::DaMiaoMotor::ClearError(CAN_HandleTypeDef* hcan,uint16_t id)
{
    uint8_t data[8];

    for (int i = 0; i < 7; ++i) 
    {
        data[i] = 0xFF;
    }
    data[7] = 0xFB;

    if (!send_can_message(hcan, id, data, sizeof(data)))
    {
        // 错误处理，可根据需要添加日志或错误处理逻辑
    }
}

void motor::DaMiaoMotor::DataUpdate(uint8_t *data)
{
    id        = data[0] & 0x0F;                                 // 取低4位
    error     = (data[0] >> 4) & 0x0F;                          // 取高4位
    angle_raw = (data[1] << 8) | data[2];                       // 拼接成16位有符号整数
    speed_raw = (data[3] << 4) | (data[4] >> 4);
    torque_raw = ((data[4] & 0x0F) << 8) | data[5];
    MOS_temperate  = data[6];
    coil_temperate = data[7];

    angle_rad   = uint_to_float(angle_raw , P_MIN, P_MAX, 16);
    speed_rad_s = uint_to_float(speed_raw , V_MIN, V_MAX, 12);
    torque_nm   = uint_to_float(torque_raw, T_MIN, T_MAX, 12);
}

void DaMiaoMotorRxData()
{
    switch (Can_RX2.rxHeader2.StdId)
    {
		case 0x02:
            M4310_0x01.DataUpdate(Can_RX2.rxdata2);
            M4310_0x01.RemoteInit();
            break;
    }
}

void motor::DaMiaoMotor::RemoteInit()
{
    if(this->Init_Flag == false)
    {
        M4310_0x01_Remove.target = M4310_0x01.angle_rad;
        this->Init_Flag = true;
    }
}
