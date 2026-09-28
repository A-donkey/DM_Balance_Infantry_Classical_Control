#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "main.h"

//INCLUDE部分
#include "PID.h"
#include "VT03.h"
#include "Aim_Task.h"
#include "Ins_Task.h"
#include "Limit_Task.h"
#include "Check_Task.h"
#include "Some_Functions.h"
#include "Board_Can_Task.h"
#include "RLS_Identification.h"
#include "ZeroCheck.h"

//角度转弧度
#define Ang_PI 0.01745329f
//弧度转角度
#define PI_Ang 57.2957805f

//正负方向零点位置
#define ZERO_HEAD_YAW   -2.17543435f
#define ZERO_BACK_YAW   0.961483002f

//拨盘电机减速比
#define DIAL_REDUCTION_RATIO 36.0f

//摩擦轮转速
#define Friction_Speed   6000

//云台俯仰角限位
#define PITCH_UP_LIMIT_POSITION     30
#define PITCH_DOWN_LIMIT_POSITION  -30

//云台电机输出极性
#define PITCH_MOTOR_SIGN -1
#define YAW_MOTOR_SIGN 1

//控制器灵敏度以及死区设置
#define RC_PITCH_SENSITIVITY        0.0004
#define RC_YAW_SENSITIVITY          0.0006
#define RC_DEADBAND                 5

#define PC_PITCH_SENSITIVITY        0.0016
#define PC_YAW_SENSITIVITY          0.0020
#define PC_DEADBAND                 1

//拨盘过零检测阈值
#define DIR_THRESHOLD 0.7f

typedef enum
{
	Close,
	Open
}Shoot_Condition_t; //发射机构状态

typedef struct
{
	bool run_flag;         //启动标志
	
	float Yaw_Zero_Target; //零点目标位置
}Self_Rescue_t;

// 系统辨识相关
typedef struct
{
    // 系统辨识数据
    TD_t    td_omega;      // 速度/加速度 TD 滤波器
    RLS   rls_sysid;     // RLS 递推最小二乘
    float   sysid_timer;   // 辨识计时器
    uint8_t sysid_done;    // 辨识完成标志
    
    // 辨识结果
    float J;  // 转动惯量
    float B;  // 粘性阻尼
    float C;  // 库伦摩擦
} Gimbal_SysID_t;

typedef struct 
{
	int16_t Rc_Pitch; //遥控器数据
	int16_t Rc_Yaw;
	int16_t Pc_Pitch;
	int16_t Pc_Yaw;
	
	float abs_yaw;   //绝对数据
	
	float yaw;       //陀螺仪数据
	float d_yaw;
	float pitch; 
	float d_pitch;
	
	float abs_yaw_ref; //绝对控制下的目标值
	
	float yaw_ref;  //相对控制下的目标值
	float pitch_ref; 
	
	float Yaw_Motor_Out;   //电机输出
	float Pitch_Motor_Out; 
	
	/*系统辨识*/
	uint32_t sys_cnt;
  float sys_delta_t; // 两帧计算之间的时间差
	float sys_yaw_speed_ref;
	float sys_pitch_speed_ref;
	Gimbal_SysID_t Yaw_SysID;
	Gimbal_SysID_t Pitch_SysID;
}Gimbal_Status_t;

typedef struct 
{
	int16_t L_Rpm; //摩擦轮RPM
	int16_t R_Rpm;
	
	float D_Pos; //拨盘位置
	float D_Spd; //拨盘转速
	
	float Target_Rpm; //设定摩擦轮电机目标转速
	float Target_Pos; //设定拨盘电机目标位置
	
	float L_Motor_Out; //摩擦轮电机输出
	float R_Motor_Out;
	float Toggle_Motor_Out;
}Shoot_Status_t;

// 新增：前馈参数结构体
typedef struct
{
    float J;  // 转动惯量
    float B;  // 粘性阻尼系数
    float C;  // 库伦摩擦系数
    float Cb; // PITCH轴阻尼系数（如果与B不同）
}Feedforward_Param_t;

// 新增：重力补偿参数
typedef struct
{
    float A;  // sin分量系数
    float B;  // cos分量系数
    float C;  // 库伦摩擦
}Gravity_Comp_Param_t;

void Gimbal_Init(void);
void Gimbal_Task(void);
void Auto_Aim(Aim_Rx *aim,Gimbal_Status_t *gs);
void Gimbal_Target_Limit(Gimbal_Status_t *gs);
void Gimbal_Can_Data_Send(Controlled_State_t *cs,Shoot_Status_t *ss,Gimbal_Status_t *gs);
void Shoot_Control(Heat_Control_t *hc,Shoot_Status_t *ss,Shoot_Condition_t *sc);
void Variable_Information_Acquisition(INS_t *ins,Gimbal_Motor_t *gm,Shoot_Status_t *ss,Gimbal_Status_t *gs);
void Gimbal_Controllor(Shoot_Status_t *ss,Gimbal_Status_t *gs,Self_Rescue_t *self_re,Controlled_State_t *cs,Aim_Rx *aim);
void Gimbal_Control(RC_Ctrl_t *rc_ctrl,PC_Ctrl_t *pc_ctrl,Shoot_Status_t *ss,Gimbal_Status_t *gs,Shoot_Condition_t *sc,Self_Rescue_t *self_re,Controlled_State_t *cs,Aim_Tx* aim_tx);

//EXTERN部分
extern bool gimbal_ready_flag;
extern float Friction_Speed_Comp;

extern Shoot_Status_t Shoot_Status;
extern Gimbal_Status_t Gimbal_Status;
extern Shoot_Condition_t Shoot_Condition; 
extern PID_t Pitch_S_Pid;
extern PID_t Yaw_S_Pid;
extern Gravity_Comp_Param_t Gravity_Param;
extern Feedforward_Param_t Yaw_FF_Param;
extern Feedforward_Param_t Pitch_FF_Param;
extern TD_t Pos_Pitch_TD;
extern TD_t Pos_Yaw_TD;
extern bool Aim_Permission;

#endif
