#ifndef __CAN_FEEDBACK_H
#define __CAN_FEEDBACK_H

#include "main.h"

//INCLUDE部分
#include "fdcan.h"
#include "VT03.h"
#include "Super_Cap.h"
#include "Dji_Motor.h"
#include "Damiao_Motor.h"
#include "Remote_Control.h"

typedef struct
{
	motor_fbpara_t DM_8009[4];
	motor_measure_t Dji_3508[2];
}Chassis_Motor_t;

typedef struct
{
	int Friction_Speed; //摩擦轮转速
	
	float up_pitch;  //上板的PITCH值
	
	bool Auto_Aim_Flag;    //自瞄锁定标志位
	bool Friction_Status;  //摩擦轮状态
	
	uint16_t up_err_num;   //上板报错码
	
	float ecd_yaw_DM4310;    //YAW4310的转子位置
	
	int Number_Of_Bullets; //弹舱内弹丸数量
}Up_Cboard_Info_t;

//EXTERN部分
extern Chassis_Motor_t Chassis_Motor;
extern Up_Cboard_Info_t Up_Cboard_Info;

#endif
