#ifndef __CAN_FEEDBACK_H
#define __CAN_FEEDBACK_H

#include "main.h"

//INCLUDE部分
#include "fdcan.h"
#include "stdbool.h"
#include "Dji_Motor.h"
#include "Damiao_Motor.h"

typedef struct
{
	motor_fbpara_t  DM_4310[2];  //云台电机[0:YAW 1:PITCH]
	motor_measure_t Dji_3508[2]; //摩擦轮电机[0:左 1:右]
	motor_measure_t Dji_2006;		 //拨盘电机
}Gimbal_Motor_t;

typedef struct
{
	bool fall_flag;  //倒地标志位
	bool enem_color; //0:red 1:blue
	
	float down_dyaw; //底盘DYAW
	
	uint16_t heat_limit;    //热量上限
	uint16_t barrel_heat;   //当前热量
	uint16_t cooling_value; //每秒热量冷却值
}Down_Cboard_Info_t;

//EXTERN部分
extern Gimbal_Motor_t Gimbal_Motor;
extern Down_Cboard_Info_t Down_Cboard_Info;

#endif
