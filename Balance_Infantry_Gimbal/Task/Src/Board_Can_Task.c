#include "Board_Can_Task.h"

//INCLUDE部分
#include "VT03.h"
#include "Ins_Task.h"
#include "Check_Task.h"
#include "Limit_Task.h"
#include "Gimbal_Task.h"
//全局变量定义部分
bool board_ready_flag; //当前线程初始化完成标志 

/*******************************************************************************************************
Board_Can任务初始化
********************************************************************************************************/
void Board_Can_Init(void)
{
	board_ready_flag = 1;
	
	while(!all_ready_flag) {osDelay(1);}
}

/*******************************************************************************************************
Board_Can任务
********************************************************************************************************/
void Board_Can_Task(void)
{
	static uint16_t board_can_tick = 0;
	board_can_tick++;
	if (board_can_tick >= 1000) board_can_tick = 0; 
	
	if (board_can_tick % 2 == 0)
	{
		Send_Message_1();
	}

	if (board_can_tick % 2 == 0)
	{
		Send_Rc_DT7();
		Send_Message_2();
	}
}

/*******************************************************************************************************
向下板发送遥控器信息
********************************************************************************************************/
void Send_Rc_DT7(void)
{
	uint8_t Data[8];
	
	Data[0] = Rc_Ctrl.rc.ch[1]>>8;
	Data[1] = Rc_Ctrl.rc.ch[1]&0x00FF;
	Data[2] = Rc_Ctrl.rc.ch[4]>>8;
	Data[3] = Rc_Ctrl.rc.ch[4]&0x00FF;
	Data[4] = Rc_Ctrl.key.v>>8;
	Data[5] = Rc_Ctrl.key.v&0x00FF;
	Data[6] = Rc_Ctrl.rc.s[0];	
	Data[7] = Rc_Ctrl.rc.s[1];
	
	Can_TxMessage(&hfdcan3,0x080,8,Data);
}

/*******************************************************************************************************
板间通讯
********************************************************************************************************/
void Send_Message_1(void)
{
	uint8_t Data[8];
	
	static int up_pitch = 0;
	
	up_pitch = -INS.Pitch;
	
	Data[0] = Link_Sit.err_num>>8;
	Data[1] = Link_Sit.err_num&0x00FF;
	Data[2] = Gimbal_Motor.DM_4310[0].p_int>>8;
	Data[3] = Gimbal_Motor.DM_4310[0].p_int&0x00FF;
	Data[4] = up_pitch>>8; 
	Data[5] = up_pitch&0x00FF; 
	Data[6] = Heat_Control.Number_Of_Bullets>>8; //弹舱内弹丸数量
	Data[7] = Heat_Control.Number_Of_Bullets&0x00FF; 

	Can_TxMessage(&hfdcan3,0x120,8,Data);
}

void Send_Message_2(void)
{
	uint8_t Data[8];
	
	static int friction_speed = 0;
	
	friction_speed = Friction_Speed + Friction_Speed_Comp;
	
	Data[0] = aim_rx.detect_number;       //自瞄标志位
	Data[1] = Shoot_Condition;   //摩擦轮状态
	Data[2] = friction_speed>>8; //当前弹速
	Data[3] = friction_speed&0x00FF;
	Data[4] = VT03.key>>8;
	Data[5] = VT03.key&0x00FF; 
	Data[6] = VT03.mode_sw; 
	Data[7] = 0;  

	Can_TxMessage(&hfdcan3,0x100,8,Data);
}

