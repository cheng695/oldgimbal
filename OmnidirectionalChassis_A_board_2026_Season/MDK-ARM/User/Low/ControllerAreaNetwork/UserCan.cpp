#include "UserCan.hpp"

extern Can_RX_T Can_RX;
extern Can_RX_T2 Can_RX2;
extern "C" void CAN_ConfigFilter()//���˺������ر�������
{
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;         //����������
  sFilterConfig.FilterBank           = 0;                         //ʹ�ù�����0
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;          //������0 ��0��1����
  sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;     //����ģʽ
  sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;     //�����׼ID����չID
  sFilterConfig.FilterIdHigh         = 0x00;                      //�������б�ʾ��������ID
  sFilterConfig.FilterIdLow          = 0x00;
  sFilterConfig.FilterMaskIdHigh     = 0x00;
  sFilterConfig.FilterMaskIdLow      = 0x00;
  sFilterConfig.SlaveStartFilterBank = 14;                
  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

    sFilterConfig.FilterBank = 14;
  if(HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//���պ���
{
  if(hcan == &hcan1)
  {
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Can_RX.rxHeader, Can_RX.rxdata);//��ȡ�����������   
    RmMotorRxData(); 
  }
  if(hcan == &hcan2)
  {
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &Can_RX2.rxHeader2, Can_RX2.rxdata2);//��ȡ�����������   
    DaMiaoMotorRxData();
  }
}
