#include "Super_Cap_Task.h"

//全局变量定义部分
bool super_ready_flag; //当前线程初始化完成标志 

/*******************************************************************************************************
Super_Cap任务初始化
********************************************************************************************************/
void Super_Cap_Init(void)
{
	//初始化PID
	Power_Limit_Init();
	
	super_ready_flag = 1;
	
	//等待
	while(!all_ready_flag) {osDelay(1);}
}

/*******************************************************************************************************
Super_Cap任务
********************************************************************************************************/
void Super_Cap_Task(void)
{
	Super_Cap_Control(&Controlled_State);
}

/*******************************************************************************************************
超电控制
********************************************************************************************************/
void Super_Cap_Control(Controlled_State_t *cs)
{
	if(*cs!=ERO && *cs!=STOP)
	{
		// 能量环：PD控制器计算 P_max
		Energy_Ring_Calc(Robot_Status.chassis_power_limit, pm_od.voltage);
	}	
	Pm_Power_Set(&hfdcan2,Robot_Status.chassis_power_limit*100,Power_Heat_Data.buffer_energy); //设置输入功率
}

