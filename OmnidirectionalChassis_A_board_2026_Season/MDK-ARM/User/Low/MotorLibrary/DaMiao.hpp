#ifndef _DAMIAO_HPP_
#define _DAMIAO_HPP_ 

#include "DJIMotor.hpp"

extern CAN_TxHeaderTypeDef DaMiao_txHeader;
extern uint8_t DaMiao_TX_Data[8];
extern uint32_t txMailbox_DaMiao;
extern RemoteOperation::Motortarget M4310_0x01_Remove;

typedef struct
{
    CAN_RxHeaderTypeDef rxHeader2;
    uint8_t rxdata2[8];
}Can_RX_T2;
extern Can_RX_T2 Can_RX2;

namespace motor
{
    class DaMiaoMotor
    {
        public:
            DaMiaoMotor(
            float p_min = -12.5f, float p_max = 12.5f,
            float v_min = -45.0f, float v_max = 45.0f,
            float t_min = -18.0f, float t_max = 18.0f,
            float kp_min = 0.0f, float kp_max = 500.0f,
            float kd_min = 0.0f, float kd_max = 5.0f, 
            uint32_t recv_id = 0x01)
            : P_MIN(p_min), P_MAX(p_max),
            V_MIN(v_min), V_MAX(v_max),
            T_MIN(t_min), T_MAX(t_max),
            KP_MIN(kp_min), KP_MAX(kp_max),
            KD_MIN(kd_min), KD_MAX(kd_max) { recv_id_ = recv_id; };//���캯��

            uint32_t recv_id_;                    //����ID

            uint16_t angle_raw;                    //ԭʼת�ӻ�е�Ƕ�  
            uint16_t speed_raw;                    //ԭʼת���ٶ�      
            uint16_t torque_raw;                   //ԭʼת������      

            uint8_t error;                        //�����־ 8��ѹ��9Ƿѹ��A��������B MOS���£�C��Ȧ���£�DͨѶ��ʧ��E����
            uint8_t id;                           //���id
            float angle_rad;                      //ת�ӻ�е�Ƕ�  rad
            float speed_rad_s;                    //ת���ٶ�      rad/s
            float torque_nm;                      //ת������      N*m
            uint8_t  MOS_temperate;               //MOS���¶�     ���϶�
            uint8_t  coil_temperate;              //��Ȧ�¶�      ���϶�

            float MotorSet;                       //�������ֵ
            bool Init_Flag = false;
            bool Enable_Flag = false;

            float uint_to_float(int x_int, float min, float max, int bits) const;
            int float_to_uint(float x, float min, float max, int bits) const;

            void ctrl_MIT(CAN_HandleTypeDef* hcan, uint16_t id, 
                            float _pos, float _vel, float _KP, float _KD, float _torq); 
            void ctrl_AngleSpeed(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel);
            void ctrl_Velocity(CAN_HandleTypeDef* hcan,uint16_t id, float _vel);
            void Enable(CAN_HandleTypeDef* hcan,uint16_t id);
            void Disability(CAN_HandleTypeDef* hcan,uint16_t id);
            void ClearError(CAN_HandleTypeDef* hcan,uint16_t id);
            void DataUpdate(uint8_t *data);
            void RemoteInit(void);

        private:
            const float P_MIN;
            const float P_MAX;
            const float V_MIN;
            const float V_MAX;
            const float T_MIN;
            const float T_MAX;

            const float KP_MIN;
            const float KP_MAX;
            const float KD_MIN;
            const float KD_MAX;

            bool send_can_message(CAN_HandleTypeDef* hcan, uint16_t id, const uint8_t* data, uint8_t length);
    };

}

void DaMiaoMotorRxData(void);

#endif
