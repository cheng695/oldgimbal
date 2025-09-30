#include "UserCan.hpp"

extern Can_RX_T Can_RX;
extern Can_RX_T2 Can_RX2;
extern "C" void CAN_ConfigFilter()//过滤函数，关闭屏蔽器
{
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;         //启动过滤器
  sFilterConfig.FilterBank           = 0;                         //使用过滤器0
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;          //缓冲器0 有0，1两个
  sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;     //掩码模式
  sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;     //处理标准ID和扩展ID
  sFilterConfig.FilterIdHigh         = 0x00;                      //以下四行表示接受所有ID
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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//接收函数
{
  if(hcan == &hcan1)
  {
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Can_RX.rxHeader, Can_RX.rxdata);//获取电机返回数据   
    RmMotorRxData(); 
  }
  if(hcan == &hcan2)
  {
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &Can_RX2.rxHeader2, Can_RX2.rxdata2);//获取电机返回数据   
    DaMiaoMotorRxData();
  }
}
