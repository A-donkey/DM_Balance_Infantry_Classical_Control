#include "bsp_can.h"

//INCLUDE部分
#include "cmsis_os.h"

/*******************************************************************************************************
CAN滤波器设置以及CAN初始化
********************************************************************************************************/
void FDCAN_Init(void)
{
	FDCAN_FilterTypeDef filter;
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x0000;
  filter.FilterID2 = 0x0000; // 掩码为0，接收所有ID
	
	HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
	
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  HAL_FDCAN_ConfigFilter(&hfdcan3, &filter);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_Start(&hfdcan3);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/*******************************************************************************************************
CAN的发送函数
********************************************************************************************************/
uint8_t Can_TxMessage(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t len, uint8_t *data)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; 	// 8字节数据长度
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;   // 关闭波特率切换
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;    // 指定为经典CAN模式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0) {
        osDelay(1);
    }
    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) == HAL_OK) ? 0 : 1;
}




