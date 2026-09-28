#include "Vofa.h"

//INCLUDE部分
#include "VMC.h"
#include "Ref_Task.h"
#include "Chassis_Task.h"
#include "LQR.h"
//EXTERN部分
extern UART_HandleTypeDef huart1;
//全局变量定义部分
TypedefVofa Send_Array[7];

/*******************************************************************************************************
对VOFA传输的数据结构体进行初始化
********************************************************************************************************/
void Vofa_Init(void)
{
	Send_Array[6].v_u8[0] = 0x00;
	Send_Array[6].v_u8[1] = 0x00;
	Send_Array[6].v_u8[2] = 0x80;
	Send_Array[6].v_u8[3] = 0x7F;
}

/*******************************************************************************************************
VOFA发送函数
********************************************************************************************************/
void Vofa_Send_Message(void)
{
	Send_Array[0].v_f = joint_m_ptr[0]->A;        
	Send_Array[1].v_f = Leg[0].theta;
	Send_Array[2].v_f = Leg[0].d_theta;
	Send_Array[3].v_f = five_link_ptr[0]->d_phi0;
 	Send_Array[4].v_f =	Body.d_theta;
	Send_Array[5].v_f = joint_m_ptr[1]->A;
	
//	Send_Array[0].v_f = pm_od.chassis_power/100.0;
//	Send_Array[1].v_f = joint_m_ptr[0]->T_Wheel;
//	Send_Array[2].v_f = joint_m_ptr[1]->T_Wheel;
//	Send_Array[3].v_f = joint_m_ptr[0]->Tp;
// 	Send_Array[4].v_f =	joint_m_ptr[1]->Tp;
//	Send_Array[5].v_f = pm_od.referee_power/100.0;
	
//	Send_Array[0].v_f = INS.Roll;        
//	Send_Array[1].v_f = INS.Pitch;
//	Send_Array[2].v_f = INS.Yaw;
//	Send_Array[3].v_f = Roll_L_Pid.Output;
// 	Send_Array[4].v_f =	Leg[0].abs_leg_theta;
//	Send_Array[5].v_f = Leg[1].abs_leg_theta;

//  CDC_Transmit_FS(&Send_Array[0].v_u8[0],4*7);
	HAL_UART_Transmit_DMA(&huart1, &Send_Array[0].v_u8[0], 4*7);
}

