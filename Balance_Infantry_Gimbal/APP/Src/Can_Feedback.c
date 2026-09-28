#include "Can_Feedback.h"

/*
     Forward
    0       1 (3508)
				0 (YAW DM4310)
						1 (PITCH DM4310)
*/

//INCLUDE部分
#include "Gimbal_Task.h"
//全局变量定义部分
Gimbal_Motor_t Gimbal_Motor;
Down_Cboard_Info_t Down_Cboard_Info;

/*******************************************************************************************************
接收电机的反馈值并处理(CAN1)
********************************************************************************************************/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	static uint8_t Rx_Data1[8];
	static FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;
	
	HAL_StatusTypeDef RxState;
	RxState = HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&FDCAN1_RxHeader,Rx_Data1);
	
	while(RxState!=HAL_OK){;}

	if (hfdcan->Instance == FDCAN3)
	{
		switch(FDCAN1_RxHeader.Identifier)
		{
			case 0x105:
			{
				Down_Cboard_Info.fall_flag  = 0;
//				Down_Cboard_Info.fall_flag  = Rx_Data1[0]&0x01;//暂时取消倒地标志位便于调试<*_*>
				Down_Cboard_Info.enem_color = Rx_Data1[0]&0x02;
				
				Down_Cboard_Info.heat_limit    = (int16_t)(Rx_Data1[1] << 8 | Rx_Data1[2]);
				Down_Cboard_Info.barrel_heat   = (int16_t)(Rx_Data1[3] << 8 | Rx_Data1[4]);
				Down_Cboard_Info.cooling_value =  Rx_Data1[5];
				
				Down_Cboard_Info.down_dyaw     = (float)((int16_t)(Rx_Data1[6] << 8 | Rx_Data1[7]))/1000.f;
				break;
			}
			case 0x0B: // YAW
			{
				Receive_Motor_Data(&Gimbal_Motor.DM_4310[0], Rx_Data1, DM4310);
				break;
			}
			case 0x206: // 拨弹
			{
				Get_Motor_Measure(&Gimbal_Motor.Dji_2006, Rx_Data1);
				break;
			}
			default:
				break;
		}
	}
}

/*******************************************************************************************************
接收电机的反馈值并处理(CAN2)
********************************************************************************************************/
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	static uint8_t Rx_Data2[8]; 
  static FDCAN_RxHeaderTypeDef FDCAN2_RxHeader;
	
	HAL_StatusTypeDef RxState;
	RxState=HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO1,&FDCAN2_RxHeader,Rx_Data2);

	while(RxState!=HAL_OK){;}
		
	switch(FDCAN2_RxHeader.Identifier)
	{
		case 0x0D://PITCH
		{
			Receive_Motor_Data(&Gimbal_Motor.DM_4310[1],Rx_Data2,DM4310);
			break;
		}
		case 0x208://摩擦轮右
		{
			Get_Motor_Measure(&Gimbal_Motor.Dji_3508[1],Rx_Data2);
			break;
		}
		case 0x207://摩擦轮左
		{
			Get_Motor_Measure(&Gimbal_Motor.Dji_3508[0],Rx_Data2);
			break;
		}
		default:
		{
			break;
		}
	}
}


