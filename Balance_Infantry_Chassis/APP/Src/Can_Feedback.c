#include "Can_Feedback.h"

/*
     Forward
	  0       2    DM_8009
            
   0         1   DJI_3508

    1       3    DM_8009
*/

//全局变量定义部分
Chassis_Motor_t Chassis_Motor;
Up_Cboard_Info_t Up_Cboard_Info;

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
			// DT7遥控
			case DT7_TO_CHASSIS_CAN_ID_1:
			{
				Rc_Ctrl.rc.ch[1]  = (Rx_Data1[0] << 8 | Rx_Data1[1]);
				Rc_Ctrl.rc.ch[4]  = (Rx_Data1[2] << 8 | Rx_Data1[3]);
				Rc_Ctrl.key.v     = (Rx_Data1[4] << 8 | Rx_Data1[5]);
				Rc_Ctrl.rc.s[0]   =  Rx_Data1[6];
				Rc_Ctrl.rc.s[1]   =  Rx_Data1[7];
				break;
			}
			// DT7遥控
			case DT7_TO_CHASSIS_CAN_ID_2:
			{
				Rc_Ctrl.rc.ch[0]  = (Rx_Data1[0] << 8 | Rx_Data1[1]);
				break;
			}
			// 上板信息1
			case GIMBAL_TO_CHASSIS_CAN_ID_1:
			{
				Up_Cboard_Info.Auto_Aim_Flag    = Rx_Data1[0];
				Up_Cboard_Info.Friction_Status  = Rx_Data1[1];
				Up_Cboard_Info.Friction_Speed   = Rx_Data1[2] << 8 | Rx_Data1[3];
				VT03.key                        = Rx_Data1[4] << 8 | Rx_Data1[5];
				VT03.mode_sw                    = Rx_Data1[6];
				break;
			}
			// 上板信息2
			case GIMBAL_TO_CHASSIS_CAN_ID_2:
			{
				Up_Cboard_Info.up_err_num        = (Rx_Data1[0] << 8 | Rx_Data1[1]);
				Up_Cboard_Info.ecd_yaw_DM4310    = (Rx_Data1[2] << 8 | Rx_Data1[3]);
				Up_Cboard_Info.up_pitch          = (float)((int16_t)(Rx_Data1[4] << 8 | Rx_Data1[5]));
				Up_Cboard_Info.Number_Of_Bullets = (Rx_Data1[6] << 8 | Rx_Data1[7]);
				break;
			}
			default:
				break;
		}
	}
	else if (hfdcan->Instance == FDCAN1)
	{
		switch(FDCAN1_RxHeader.Identifier)
		{
			// DM_8009反馈
			case LEFT_FRONT_MOTOR_FEEDBACK_ID:
			{
				Receive_Motor_Data(&Chassis_Motor.DM_8009[0], Rx_Data1, DM8009);
				break;
			}
			case LEFT_BACK_MOTOR_FEEDBACK_ID:
			{
				Receive_Motor_Data(&Chassis_Motor.DM_8009[1], Rx_Data1, DM8009);
				break;
			}
			// DJI_3508反馈
			case LEFT_WHEEL_MOTOR_FEEDBACK_ID:
			{
				Get_Motor_Measure(&Chassis_Motor.Dji_3508[0], Rx_Data1);
				break;
			}
			default:
				break;
		}
	}
}
uint8_t Rx_Data2[8];
/*******************************************************************************************************
接收电机的反馈值并处理(CAN2)
********************************************************************************************************/
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	 
  static FDCAN_RxHeaderTypeDef FDCAN2_RxHeader;
	
	HAL_StatusTypeDef RxState;
	RxState=HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO1,&FDCAN2_RxHeader,Rx_Data2);

	while(RxState!=HAL_OK){;}
		
	switch(FDCAN2_RxHeader.Identifier)
	{
		//超电数据
		case CAP_TO_CHASSIS_CAN_ID:
		{
			pm_od.voltage 			= (uint16_t)(Rx_Data2[1] << 8 | Rx_Data2[0]);
			pm_od.chassis_power = ( int16_t)(Rx_Data2[3] << 8 | Rx_Data2[2]);
			pm_od.referee_power = (uint16_t)(Rx_Data2[5] << 8 | Rx_Data2[4]);
			pm_od.reserve				=	(uint16_t)(Rx_Data2[7] << 8 | Rx_Data2[6]);
			//判断超级电容是否离线
			if(super_link[0]>=1000) super_link[0]=0;
			else              	 		super_link[0]++;
			
			break;
		}
		//DM_8009数据
		case RIGHT_FRONT_MOTOR_FEEDBACK_ID:
		{
			Receive_Motor_Data(&Chassis_Motor.DM_8009[2],Rx_Data2,DM8009);
			break;
		}
		case RIGHT_BACK_MOTOR_FEEDBACK_ID:
		{
			Receive_Motor_Data(&Chassis_Motor.DM_8009[3],Rx_Data2,DM8009);
			break;
		}
		//DJI_3508数据
		case RIGHT_WHEEL_MOTOR_FEEDBACK_ID:
		{
			Get_Motor_Measure(&Chassis_Motor.Dji_3508[1],Rx_Data2);
			break;
		}
		default:
		{
			break;
		}
	}
}

