#include "Gimbal_Task.h"
#include "System_Identification.h"
#include "GimbalSystemIDConfig.h"
#include "GimbalSystemID.h"

//全局变量定义部分
Self_Rescue_t Self_Rescue; //自救
Shoot_Status_t Shoot_Status;
Gimbal_Status_t Gimbal_Status; 
Shoot_Condition_t Shoot_Condition;

//建立控制器结构体
PID_t Yaw_P_Pid; //云台
PID_t Yaw_S_Pid;
PID_t Pitch_P_Pid; 
PID_t Pitch_S_Pid; 
PID_t Abs_Yaw_P_Pid;
PID_t Abs_Yaw_S_Pid;

PID_t L_Rpm_Pid; //发射机构
PID_t R_Rpm_Pid;
PID_t D_Pos_Pid;
PID_t D_Spd_Pid;

// TD跟踪微分器
TD_t Pos_Pitch_TD;
TD_t Pos_Yaw_TD;

// 前馈参数（需要实测/辨识）
//Feedforward_Param_t Yaw_FF_Param  = {0.0115128597,  0.0286347487f,  0.23262085f};//{J_yaw,  B_yaw,  C_yaw};
Feedforward_Param_t Yaw_FF_Param  = {0.000115128597,  0.286347487f,  0.0f};//{J_yaw,  B_yaw,  C_yaw};
Feedforward_Param_t Pitch_FF_Param = {0.0064, 0, 0};//{J_pitch, Cb_pitch, C_pitch}// Pitch用Cb代替B

// 重力补偿参数
Gravity_Comp_Param_t Gravity_Param = {-0.2899f, 3.00 , 1.543f};//{A, B_g, C}//C用不上

// 前馈和重力补偿中间变量
float Yaw_FF_Output;
float Pitch_FF_Output;
float Pitch_Gravity_Comp;

//过零检测初始化
ZeroCheck_Typedef dial_zero_check;

float Friction_Speed_Comp = 0;

bool Aim_Permission    = 0;  //自瞄许可
bool Fire_Permission   = 0;  //开火许可
bool Aim_Converge_Flag = 0;  //自瞄收敛标志位

bool gimbal_ready_flag; //当前线程初始化完成标志 


/*******************************************************************************************************
Gimbal任务初始化
********************************************************************************************************/
void Gimbal_Init(void)
{
	//初始化PID
	//更改云台PID以适配新电机
	PID_Init(&Pitch_P_Pid   ,    10,   	5, 0,    50,     5,     0,  0,0,0,0, 4,RADIAN,Integral_Limit); //云台
	PID_Init(&Pitch_S_Pid   , 		9, 		5, 0,  		6,     0,     0,  0,0,0,0, 0,NO_CIRCLE,Integral_Limit);
	PID_Init(&Yaw_P_Pid     ,    10,   	5, 0,    30,     0,     0,  0,0,0,0, 4,RADIAN,Integral_Limit);
	PID_Init(&Yaw_S_Pid     , 		9, 		3, 0,  		1,     0,     0,  0,0,0,0, 0,NO_CIRCLE,Integral_Limit);
	
	PID_Init(&Abs_Yaw_P_Pid , 	 10, 		5, 0,  		1,     0,   	0,  0,0,0,0, 0,RADIAN,NONE);
	PID_Init(&Abs_Yaw_S_Pid , 		9, 		5, 0,  		1,     0,   	0,  0,0,0,0, 0,RADIAN,NONE);
	
	PID_Init(&L_Rpm_Pid     , 16000, 1000, 0,  16.8,     0,     0,  0,0,0,0,10,NO_CIRCLE,NONE); //发射机构
	PID_Init(&R_Rpm_Pid     , 16000, 1000, 0,  16.8,     0,     0,  0,0,0,0,10,NO_CIRCLE,NONE);
	PID_Init(&D_Pos_Pid     ,	 1900, 	100, 0,   100,     0,   	1,  0,0,0,0, 2,NO_CIRCLE,NONE);
	PID_Init(&D_Spd_Pid     ,	 9500, 1000, 0,  	  5,     0,   	0,  0,0,0,0, 2,NO_CIRCLE,NONE);

	// TD初始化（跟踪微分器）
	TD_Init(&Pos_Yaw_TD,   700, 0.005);  // r=700, h0=0.005s
	TD_Init(&Pos_Pitch_TD, 1000, 0.005); // r=1000, h0=0.005s
	
	// RLS系统辨识初始化
	RLS_Init(&Gimbal_Status.Yaw_SysID.rls_sysid,   2, 1, 0.98f);  // x=[B, C], y=[torque]
	RLS_Init(&Gimbal_Status.Pitch_SysID.rls_sysid, 3, 1, 0.98f);  // x=[B, A, C], y=[torque]

	//发射机构状态关闭
	Shoot_Condition = Close;
	
	//弹丸估计量重置
	Heat_Control.Number_Of_Bullets = 450;
	
	//过零检测初始化
	dial_zero_check.Circle = 0;
	dial_zero_check.LastValue = 0;

	//初始化完成
	gimbal_ready_flag  = 1;
	
	//等待(进行目标值初始化)
	while(!all_ready_flag) 
	{
		Gimbal_Status.yaw_ref   = INS.YawTotalAngle; //云台目标重置
		Gimbal_Status.pitch_ref = -INS.Pitch;
		Shoot_Status.Target_Pos = ZeroCheck(&dial_zero_check, Gimbal_Motor.Dji_2006.ecd / 8191.0f * 2 * PI , 2*PI,DIR_THRESHOLD ,Shoot_Status.D_Spd);  // 累计后的绝对角度
		osDelay(1);
	}
	TD_Clear(&Pos_Yaw_TD,   Gimbal_Status.yaw_ref);
	TD_Clear(&Pos_Pitch_TD, PITCH_DOWN_LIMIT_POSITION);
}

/*******************************************************************************************************
Gimbal任务
********************************************************************************************************/
void Gimbal_Task(void)
{
	Gimbal_Status.sys_delta_t = DWT_GetDeltaT(&Gimbal_Status.sys_cnt);
	Variable_Information_Acquisition(&INS,&Gimbal_Motor,&Shoot_Status,&Gimbal_Status);
	Gimbal_Control(&Rc_Ctrl,&Pc_Ctrl,&Shoot_Status,&Gimbal_Status,&Shoot_Condition,&Self_Rescue,&Controlled_State,&aim_tx);
	Auto_Aim(&aim_rx,&Gimbal_Status);
	Gimbal_Target_Limit(&Gimbal_Status);
	Shoot_Control(&Heat_Control,&Shoot_Status,&Shoot_Condition);
	Gimbal_Controllor(&Shoot_Status,&Gimbal_Status,&Self_Rescue,&Controlled_State,&aim_rx);
	Gimbal_Can_Data_Send(&Controlled_State,&Shoot_Status,&Gimbal_Status);
}

/*******************************************************************************************************
云台关键数据获取
********************************************************************************************************/
void Variable_Information_Acquisition(INS_t *ins,
																	    Gimbal_Motor_t *gm,
																			Shoot_Status_t *ss,
																			Gimbal_Status_t *gs)
{
	//获取摩擦轮RPM
	ss->L_Rpm = gm->Dji_3508[0].speed_rpm; 
	ss->R_Rpm = gm->Dji_3508[1].speed_rpm;
	
	//获取拨盘POS
	ss->D_Pos = ZeroCheck(&dial_zero_check, Gimbal_Motor.Dji_2006.ecd / 8191.0f * 2 * PI , 2*PI,DIR_THRESHOLD ,Shoot_Status.D_Spd);  // 累计后的绝对角度
	ss->D_Spd = gm->Dji_2006.speed_rpm*2*PI/60.0f;
	 
	//计算获取云台电机绝对位置
	gs->abs_yaw = gm->DM_4310[0].pos;
	
	//计算获取云台陀螺仪数据
	gs->yaw     = ins->YawTotalAngle;
	gs->d_yaw   = ins->Gyro[2];
	gs->pitch   = -ins->Pitch;
	gs->d_pitch = -ins->Gyro[0];
}

/*******************************************************************************************************
云台控制各项参数初始化
********************************************************************************************************/
void Gimbal_Control_Init(Shoot_Status_t *ss,
										     Gimbal_Status_t *gs,
												 Shoot_Condition_t *sc)
{ 
	*sc = Close; //发射机构状态重置
	
	//顺序不要更改，PID清零会将ref置0
	PID_Clear(&Pitch_P_Pid);
	PID_Clear(&Pitch_S_Pid);
	PID_Clear(&Yaw_P_Pid);
	PID_Clear(&Yaw_S_Pid);
	PID_Clear(&Abs_Yaw_P_Pid);
	PID_Clear(&Abs_Yaw_S_Pid);
	PID_Clear(&L_Rpm_Pid);
	PID_Clear(&R_Rpm_Pid);
	PID_Clear(&D_Pos_Pid);
	PID_Clear(&D_Spd_Pid);
	
	gs->yaw_ref   = gs->yaw; //云台目标重置
	gs->pitch_ref = gs->pitch;
	TD_Clear(&Pos_Yaw_TD,   gs->yaw_ref);
	// TD_Clear(&Pos_Pitch_TD, PITCH_DOWN_LIMIT_POSITION);
	TD_Clear(&Pos_Pitch_TD, gs->pitch_ref);
	
	Yaw_FF_Output = 0;
	Pitch_FF_Output = 0;
	Pitch_Gravity_Comp = 0;
	gs->Yaw_Motor_Out = 0;
	gs->Pitch_Motor_Out = 0;

	ss->Target_Pos = ss->D_Pos; //拨盘目标重置
	
	Aim_Permission  = 0; //许可重置
	Fire_Permission = 0; 
}

/*******************************************************************************************************
遥控器模式
********************************************************************************************************/
void Rc_Mode(RC_Ctrl_t *rc_ctrl,
						 Shoot_Condition_t *sc)
{	
	if     (switch_is_down(rc_ctrl->rc.s[1])){*sc = Close;} //关闭发射机构
	else if(switch_is_mid (rc_ctrl->rc.s[1])){*sc = Open;} //开启摩擦轮
	
	if(switch_is_up(rc_ctrl->rc.s[1]) && abs(rc_ctrl->rc.ch[0]) == 660) //左上右上开启连发开火
	{
		Fire_Permission = 1; //允许开火
	}
	else Fire_Permission = 0;
	
//	*sc = Close; //关火
//	Fire_Permission = 0;
	
	if(switch_is_mid (rc_ctrl->rc.s[1])) 
	{
		Aim_Permission = 1; //左中右上开启自瞄跟随
		if(abs(rc_ctrl->rc.ch[0]) == 660){
			Fire_Permission = 1; //允许开火
		}
		else Fire_Permission = 0;
	}
	else
	{
		Aim_Permission = 0;
	}
}

/*******************************************************************************************************
键鼠模式初始化
********************************************************************************************************/
void Pc_Init(PC_Ctrl_t *pc_ctrl)
{
	pc_ctrl->KEY_F = 0;
	pc_ctrl->KEY_G = 0;
	pc_ctrl->KEY_Z = 0;
	pc_ctrl->KEY_X = 0;
	pc_ctrl->KEY_V = 0;
	
	pc_ctrl->f_t = 0;
	pc_ctrl->g_t = 0;
	pc_ctrl->z_t = 0;
	pc_ctrl->x_t = 0;
	pc_ctrl->v_t = 0;
}

/*******************************************************************************************************
键鼠模式
********************************************************************************************************/
uint16_t Dial_block_Count = 0; //拨盘卡死计数

void Pc_Mode(RC_Ctrl_t *rc_ctrl,
						 PC_Ctrl_t *pc_ctrl,
						Aim_Tx *aim_tx)
{
	if     (switch_is_down(rc_ctrl->rc.s[1])){Shoot_Condition = Close;} //关闭发射机构
	else if(switch_is_mid (rc_ctrl->rc.s[1])){Shoot_Condition = Open;} //开启摩擦轮
	else if(switch_is_up  (rc_ctrl->rc.s[1])){Shoot_Condition = Open;} //开启摩擦轮

	static bool single_flag = 1; //单发标志位
	
	/******左键检测,允许开火******/
	if(aim_tx->mode_want == STD_AUTO_AIM){
		if(rc_ctrl->mouse.press_l) 
		{
			Fire_Permission = 1;
		}
		else if(!rc_ctrl->mouse.press_l)
		{
			Fire_Permission = 0;
		}
		else Fire_Permission = 0;
	}
	else{
		if(rc_ctrl->mouse.press_l && single_flag) 
		{
			single_flag = 0;
			Fire_Permission = 1;
		}
		else if(!rc_ctrl->mouse.press_l && !single_flag)
		{
			single_flag = 1;
			Fire_Permission = 0;
		}
		else Fire_Permission = 0;
	}
	
	/******右键检测,开启自瞄******/
	if(rc_ctrl->mouse.press_r) Aim_Permission = 1;
	else Aim_Permission = 0;

	/******ctrl键检测,拨盘复原******/
	if((rc_ctrl->key.v&KEY_PRESSED_OFFSET_CTRL) && !pc_ctrl->KEY_CTRL)
	{
		pc_ctrl->KEY_CTRL = 1;
		pc_ctrl->ctrl_t = HAL_GetTick();
	}
	else if(!(rc_ctrl->key.v&KEY_PRESSED_OFFSET_CTRL) && pc_ctrl->KEY_CTRL)
	{
		pc_ctrl->KEY_CTRL = 0;
		if(HAL_GetTick()-pc_ctrl->ctrl_t < 500)
		{
			Shoot_Status.Target_Pos = Shoot_Status.D_Pos; //拨盘目标重置
			Dial_block_Count = 0; //拨盘卡死计数清零
		}
	}
	
	/******Z键检测,切换大符自瞄******/
	if((rc_ctrl->key.v&KEY_PRESSED_OFFSET_Z) && !pc_ctrl->KEY_Z)
	{
		pc_ctrl->KEY_Z = 1;
		pc_ctrl->z_t = HAL_GetTick();
	}
	else if(!(rc_ctrl->key.v&KEY_PRESSED_OFFSET_Z) && pc_ctrl->KEY_Z)
	{
		pc_ctrl->KEY_Z = 0;
		if(HAL_GetTick()-pc_ctrl->z_t < 500)
		{
			aim_tx->mode_want = BIG_BUFF;
		}
	}

	/******X键检测,切换小符自瞄******/
	if((rc_ctrl->key.v&KEY_PRESSED_OFFSET_X) && !pc_ctrl->KEY_X)
	{
		pc_ctrl->KEY_X = 1;
		pc_ctrl->x_t = HAL_GetTick();
	}
	else if(!(rc_ctrl->key.v&KEY_PRESSED_OFFSET_X) && pc_ctrl->KEY_X)
	{
		pc_ctrl->KEY_X = 0;
		if(HAL_GetTick()-pc_ctrl->x_t < 500)
		{
			aim_tx->mode_want = SMALL_BUFF;
		}
	}

	/******V键检测,切换打车自瞄******/
	if((rc_ctrl->key.v&KEY_PRESSED_OFFSET_V) && !pc_ctrl->KEY_V)
	{
		pc_ctrl->KEY_V = 1;
		pc_ctrl->v_t = HAL_GetTick();
	}
	else if(!(rc_ctrl->key.v&KEY_PRESSED_OFFSET_V) && pc_ctrl->KEY_V)
	{
		pc_ctrl->KEY_V = 0;
		if(HAL_GetTick()-pc_ctrl->v_t < 500)
		{
			aim_tx->mode_want = STD_AUTO_AIM;
		}
	}

	/******G键检测,摩擦轮转速减******/
	if((rc_ctrl->key.v&KEY_PRESSED_OFFSET_G) && !pc_ctrl->KEY_G)
	{
		pc_ctrl->KEY_G = 1;
		pc_ctrl->g_t = HAL_GetTick();
	}
	else if(!(rc_ctrl->key.v&KEY_PRESSED_OFFSET_G) && pc_ctrl->KEY_G)
	{
		pc_ctrl->KEY_G = 0;
		if(HAL_GetTick()-pc_ctrl->g_t < 500)
		{
			Friction_Speed_Comp -= 20;
		}
	}
	
	/******F键检测,摩擦轮转速加******/
	if((rc_ctrl->key.v&KEY_PRESSED_OFFSET_F) && !pc_ctrl->KEY_F)
	{
		pc_ctrl->KEY_F = 1;
		pc_ctrl->f_t = HAL_GetTick();
	}
	else if(!(rc_ctrl->key.v&KEY_PRESSED_OFFSET_F) && pc_ctrl->KEY_F)
	{
		pc_ctrl->KEY_F = 0;
		if(HAL_GetTick()-pc_ctrl->f_t < 500)
		{
			Friction_Speed_Comp += 20;
		}
	}
	
}

/*******************************************************************************************************
VT03的键鼠模式
********************************************************************************************************/
void Vt03_Pc_Mode(PC_Ctrl_t *pc_ctrl,Aim_Tx *aim_tx)
{
	//挡位左右两侧均能开启摩擦轮
	if     (VT03.mode_sw == 0){Shoot_Condition = Close;} //关闭发射机构
	else if(VT03.mode_sw == 1){Shoot_Condition = Open;} //开启摩擦轮
	else if(VT03.mode_sw == 2){Shoot_Condition = Open;} //开启摩擦轮

	static bool single_flag = 1; //单发标志位
	
	/******左键检测,允许开火******/
	if(aim_tx->mode_want == STD_AUTO_AIM){
		if(VT03.mouse_left) 
		{
			Fire_Permission = 1;
		}
		else if(!VT03.mouse_left)
		{
			Fire_Permission = 0;
		}
		else Fire_Permission = 0;
	}
	else{
		if(VT03.mouse_left && single_flag) 
		{
			single_flag = 0;
			Fire_Permission = 1;
		}
		else if(!VT03.mouse_left && !single_flag)
		{
			single_flag = 1;
			Fire_Permission = 0;
		}
		else Fire_Permission = 0;
	}
	
	/******右键检测,开启自瞄******/
	if(VT03.mouse_right) Aim_Permission = 1;
	else                 Aim_Permission = 0;

	/******ctrl键检测,拨盘复原******/
	if((VT03.key&KEY_PRESSED_OFFSET_CTRL) && !pc_ctrl->KEY_CTRL)
	{
		pc_ctrl->KEY_CTRL = 1;
		pc_ctrl->ctrl_t = HAL_GetTick();
	}
	else if(!(VT03.key&KEY_PRESSED_OFFSET_CTRL) && pc_ctrl->KEY_CTRL)
	{
		pc_ctrl->KEY_CTRL = 0;
		if(HAL_GetTick()-pc_ctrl->ctrl_t < 500)
		{
			Shoot_Status.Target_Pos = Shoot_Status.D_Pos; //拨盘目标重置
			Dial_block_Count = 0; //拨盘卡死计数清零
		}
	}

	/******Z键检测,切换大符自瞄******/
	if((VT03.key&KEY_PRESSED_OFFSET_Z) && !pc_ctrl->KEY_Z)
	{
		pc_ctrl->KEY_Z = 1;
		pc_ctrl->z_t = HAL_GetTick();
	}
	else if(!(VT03.key&KEY_PRESSED_OFFSET_Z) && pc_ctrl->KEY_Z)
	{
		pc_ctrl->KEY_Z = 0;
		if(HAL_GetTick()-pc_ctrl->z_t < 500)
		{
			aim_tx->mode_want = BIG_BUFF;
		}
	}

	/******X键检测,切换小符自瞄******/
	if((VT03.key&KEY_PRESSED_OFFSET_X) && !pc_ctrl->KEY_X)
	{
		pc_ctrl->KEY_X = 1;
		pc_ctrl->x_t = HAL_GetTick();
	}
	else if(!(VT03.key&KEY_PRESSED_OFFSET_X) && pc_ctrl->KEY_X)
	{
		pc_ctrl->KEY_X = 0;
		if(HAL_GetTick()-pc_ctrl->x_t < 500)
		{
			aim_tx->mode_want = SMALL_BUFF;
		}
	}

	/******V键检测,切换打车自瞄******/
	if((VT03.key&KEY_PRESSED_OFFSET_V) && !pc_ctrl->KEY_V)
	{
		pc_ctrl->KEY_V = 1;
		pc_ctrl->v_t = HAL_GetTick();
	}
	else if(!(VT03.key&KEY_PRESSED_OFFSET_V) && pc_ctrl->KEY_V)
	{
		pc_ctrl->KEY_V = 0;
		if(HAL_GetTick()-pc_ctrl->v_t < 500)
		{
			aim_tx->mode_want = STD_AUTO_AIM;
		}
	}
	
	/******G键检测,摩擦轮转速减******/

	if((VT03.key&KEY_PRESSED_OFFSET_G) && !pc_ctrl->KEY_G)
	{
		pc_ctrl->KEY_G = 1;
		pc_ctrl->g_t = HAL_GetTick();
	}
	else if(!(VT03.key&KEY_PRESSED_OFFSET_G) && pc_ctrl->KEY_G)
	{
		pc_ctrl->KEY_G = 0;
		if(HAL_GetTick()-pc_ctrl->g_t < 500)
		{
			Friction_Speed_Comp -= 20;
		}
	}
	
	/******F键检测,摩擦轮转速加******/
	if((VT03.key&KEY_PRESSED_OFFSET_F) && !pc_ctrl->KEY_F)
	{
		pc_ctrl->KEY_F = 1;
		pc_ctrl->f_t = HAL_GetTick();
	}
	else if(!(VT03.key&KEY_PRESSED_OFFSET_F) && pc_ctrl->KEY_F)
	{
		pc_ctrl->KEY_F = 0;
		if(HAL_GetTick()-pc_ctrl->f_t < 500)
		{
			Friction_Speed_Comp += 20;
		}
	}
}

/*******************************************************************************************************
寻找合适的YAW轴复位零点
********************************************************************************************************/
void Chose_Zero_Target(Gimbal_Status_t *gs,
											 Self_Rescue_t *self_re)
{
	static float dis_to_head = 0; //到正方向零点位置最小角度值
	static float dis_to_back = 0; //到负方向零点位置最小角度值
	
	dis_to_head = fabs(Find_Min_RADIAN(gs->abs_yaw,ZERO_HEAD_YAW));
	dis_to_back = fabs(Find_Min_RADIAN(gs->abs_yaw,ZERO_BACK_YAW));
	
	if(dis_to_head <= dis_to_back) self_re->Yaw_Zero_Target = ZERO_HEAD_YAW;
	else                           self_re->Yaw_Zero_Target = ZERO_BACK_YAW;
	
	gs->abs_yaw_ref = self_re->Yaw_Zero_Target; //对绝对位置目标进行赋值
}

/*******************************************************************************************************
云台控制逻辑
********************************************************************************************************/
void Gimbal_Control(RC_Ctrl_t *rc_ctrl,
										PC_Ctrl_t *pc_ctrl,
								    Shoot_Status_t *ss,
										Gimbal_Status_t *gs,
										Shoot_Condition_t *sc,
										Self_Rescue_t *self_re,		
										Controlled_State_t *cs,
										Aim_Tx *aim_tx)
{
	if(Remote_Select == UNLINK) //未连接遥控器
	{
		gs->Pc_Pitch = 0;
		gs->Pc_Yaw   = 0;
	}
	else if(Remote_Select == DT7) //连接DT7遥控器
	{
		gs->Pc_Pitch = Limit_Min(rc_ctrl->mouse.y,PC_DEADBAND);
		gs->Pc_Yaw   = Limit_Min(rc_ctrl->mouse.x,PC_DEADBAND);
	}
	else if(Remote_Select == VT_03) //连接VT03遥控器
	{
		gs->Pc_Pitch = Limit_Min(VT03.mouse_y,PC_DEADBAND);
		gs->Pc_Yaw   = Limit_Min(VT03.mouse_x,PC_DEADBAND);
	}
	
	gs->Rc_Pitch = Limit_Min(rc_ctrl->rc.ch[3],RC_DEADBAND);
	gs->Rc_Yaw   = Limit_Min(rc_ctrl->rc.ch[2],RC_DEADBAND);
	
	if(*cs!=ERO && *cs!=STOP)
	{
		if(Remote_Select == UNLINK) //未连接遥控器
		{
			self_re->run_flag = 0; //重启复位
			
			gs->abs_yaw_ref = gs->abs_yaw; //绝对位置目标值重置
			
			Pc_Init(pc_ctrl); //初始化
			Gimbal_Control_Init(ss,gs,sc);
		}
		else if(Remote_Select == DT7) //连接DT7遥控器
		{
			if(Down_Cboard_Info.fall_flag) //倒地
			{
				if(((rc_ctrl->key.v&KEY_PRESSED_OFFSET_R) || rc_ctrl->rc.ch[1]==-660) && !self_re->run_flag)//触发自救
				{
					Chose_Zero_Target(gs,self_re);
					self_re->run_flag = 1;
				}
				
				Pc_Init(pc_ctrl); //初始化
				Gimbal_Control_Init(ss,gs,sc);
			}
			else //正常
			{
				self_re->run_flag = 0; //重启复位
				
				if(*cs == RC) //遥控器模式
				{
					gs->yaw_ref    = gs->yaw_ref - RC_YAW_SENSITIVITY*gs->Rc_Yaw; //云台
					gs->pitch_ref += RC_PITCH_SENSITIVITY * gs->Rc_Pitch; 
					
					Pc_Init(pc_ctrl);
					Rc_Mode(rc_ctrl,sc);
				}
				else if(*cs == MOUSE) //键鼠模式
				{
					gs->yaw_ref    =  gs->yaw_ref - PC_YAW_SENSITIVITY*gs->Pc_Yaw; //云台
					gs->pitch_ref +=  PC_PITCH_SENSITIVITY * gs->Pc_Pitch; 
					
					Pc_Mode(rc_ctrl,pc_ctrl,aim_tx);
				}
			}
		}
		else if(Remote_Select == VT_03) //连接VT03遥控器
		{
			if(Down_Cboard_Info.fall_flag) //倒地
			{
				if((VT03.key&KEY_PRESSED_OFFSET_R) && !self_re->run_flag)//触发自救
				{
					Chose_Zero_Target(gs,self_re);
					self_re->run_flag = 1;
				}
				
				Pc_Init(pc_ctrl); //初始化
				Gimbal_Control_Init(ss,gs,sc);
			}
			else //正常
			{
				self_re->run_flag = 0; //重启复位
				
				if(*cs == MOUSE) //键鼠模式
				{
					gs->yaw_ref    = gs->yaw_ref - PC_YAW_SENSITIVITY*gs->Pc_Yaw; //云台
					gs->pitch_ref +=  PC_PITCH_SENSITIVITY * gs->Pc_Pitch; 
					
					Vt03_Pc_Mode(pc_ctrl,aim_tx);
				}
			}
		}
	}
	else
	{
		self_re->run_flag = 0; //重启复位
		
		gs->abs_yaw_ref = gs->abs_yaw; //绝对位置目标值重置
		
		Pc_Init(pc_ctrl); //初始化
		Gimbal_Control_Init(ss,gs,sc);
	}
}

/*******************************************************************************************************
自瞄控制逻辑
********************************************************************************************************/
void Auto_Aim(Aim_Rx *aim,
							Gimbal_Status_t *gs)
{
	if(Aim_Permission && aim->detect_number != 0 && aim->yaw_setpoint!= 0 && aim->pitch_setpoint!=0)
	{
		float single_yaw = fmod(gs->yaw,360.0f);
		gs->yaw_ref = gs->yaw + Find_Min_Angle(single_yaw, aim->yaw_setpoint);
		gs->pitch_ref = aim->pitch_setpoint;
	}
}

/*******************************************************************************************************
对目标值进行限制
********************************************************************************************************/
void Gimbal_Target_Limit(Gimbal_Status_t *gs)
{
	if		 (gs->pitch_ref > PITCH_UP_LIMIT_POSITION  ) gs->pitch_ref = PITCH_UP_LIMIT_POSITION;
	else if(gs->pitch_ref < PITCH_DOWN_LIMIT_POSITION) gs->pitch_ref = PITCH_DOWN_LIMIT_POSITION;
}

/*******************************************************************************************************
发射机构控制逻辑
********************************************************************************************************/
bool Dial_Status = 0;//拨盘状态(0运行 1就绪)

void Shoot_Control(Heat_Control_t *hc,
									 Shoot_Status_t *ss,
									 Shoot_Condition_t *sc)
{
	if(fabs(ss->Target_Pos-ss->D_Pos)<=2.0f && !Dial_Status && !hc->dial_flag) //判断拨盘是否就绪
	{
		Dial_Status = 1;
		hc->dial_flag = 1;
	}

	if(fabs(ss->Target_Pos-ss->D_Pos)>2.0f) //拨盘未就绪
	{
		Dial_block_Count++;
		if(Dial_block_Count >= 150) //拨盘卡死
		{
			ss->Target_Pos = ss->D_Pos; //拨盘目标重置
		}
	}
	else
	{
		Dial_block_Count = 0;
	}
	
	if(*sc == Close) //关闭
	{
		Dial_Status = 1; //拨盘就绪
		ss->Target_Rpm = 0;
		Friction_Speed_Comp = 0;
	}
	else if(*sc == Open) //单开摩擦轮
	{
		ss->Target_Rpm = Friction_Speed + Friction_Speed_Comp;
		
		if(Aim_Permission && Fire_Permission) //自瞄情况下需手动确认判断开火
		{
			if(aim_rx.detect_number==1 && aim_rx.shoot_flag && Dial_Status && !hc->dial_flag && hc->Perm_Bullets_Num >= 2.05f) //允许开火
			{
				Dial_Status = 0; //拨盘运行
				ss->Target_Pos = ss->Target_Pos + PI/4.0f*DIAL_REDUCTION_RATIO;
			}
		}
		else //手动控制下判断开火
		{
			if(Fire_Permission && Dial_Status && !hc->dial_flag && hc->Perm_Bullets_Num >= 2.05f) //允许开火
			{
				Dial_Status = 0; //拨盘运行
				ss->Target_Pos = ss->Target_Pos + PI/4.0f*DIAL_REDUCTION_RATIO;
			}
		}
	}
}

/*******************************************************************************************************
Pitch轴计算逻辑
********************************************************************************************************/
void Gimbal_Pitch_Calculate(Gimbal_Status_t *gs,Aim_Rx *aim){
	float pitch_alpha;float pitch_omega;float sign_dpitch=0;
	// 1. TD
	TD_Calculate(&Pos_Pitch_TD, gs->pitch_ref);

	// 2. 前馈（惯量 + 阻尼）自瞄模式下使用上位机目标值
	if(Aim_Permission && aim->detect_number != 0){
		pitch_alpha = aim->pitch_acc_setpoint * Ang_PI;
		pitch_omega = aim->pitch_omega_setpoint * Ang_PI;
	}
	else{
		pitch_alpha = Pos_Pitch_TD.ddx * Ang_PI;
		pitch_omega = Pos_Pitch_TD.dx * Ang_PI;
	}
	Pitch_FF_Output = Pitch_FF_Param.J * pitch_alpha
                + Pitch_FF_Param.Cb * pitch_omega;
	// 3. 重力补偿
	float pitch_rad = gs->pitch * Ang_PI;  // 度 → 弧度
	if(fabs(gs->d_pitch)>5.0f*Ang_PI)
	{
		sign_dpitch = sign(gs->d_pitch);
	}
	Pitch_Gravity_Comp = Gravity_Param.A * sin(pitch_rad)
                   + Gravity_Param.B * cos(pitch_rad)
                   + Gravity_Param.C * sign_dpitch;

	// 4. PID反馈
	if(Aim_Permission && aim->detect_number != 0){
		PID_Calculate(&Pitch_P_Pid, gs->pitch * Ang_PI, gs->pitch_ref * Ang_PI);
		PID_Calculate(&Pitch_S_Pid, gs->d_pitch, aim->pitch_omega_setpoint * Ang_PI);
	}
	else{
		PID_Calculate(&Pitch_P_Pid, gs->pitch * Ang_PI, Pos_Pitch_TD.x * Ang_PI);
		PID_Calculate(&Pitch_S_Pid, gs->d_pitch, Pos_Pitch_TD.dx * Ang_PI);
	}

	// 5. 总输出
	gs->Pitch_Motor_Out = Pitch_FF_Output + Pitch_Gravity_Comp
                    + Pitch_P_Pid.Output + Pitch_S_Pid.Output;
}

/*******************************************************************************************************
Yaw轴计算逻辑
********************************************************************************************************/
void Gimbal_Yaw_Calculate(Gimbal_Status_t *gs,Aim_Rx *aim){
	float yaw_alpha;float yaw_omega;
	// 1. TD计算目标角度的平滑值、角速度、角加速度
	TD_Calculate(&Pos_Yaw_TD, gs->yaw_ref);  // 输入：度

	// 2. 物理模型前馈（单位需要统一）
	//    TD输出是度、°/s、°/s?，需要转换为电机电流单位
	if(Aim_Permission && aim->detect_number != 0){
		yaw_alpha = aim->yaw_acc_setpoint * Ang_PI;  // °/s? → rad/s?
		yaw_omega = aim->yaw_omega_setpoint * Ang_PI;   // °/s → rad/s
	}
	else{
		yaw_alpha = Pos_Yaw_TD.ddx * Ang_PI;  // °/s? → rad/s?
		yaw_omega = Pos_Yaw_TD.dx * Ang_PI;   // °/s → rad/s
	}
	Yaw_FF_Output = Yaw_FF_Param.J * yaw_alpha           // 惯量 × 角加速度
              + Yaw_FF_Param.B * yaw_omega            // 阻尼 × 角速度
              + Yaw_FF_Param.C * sign(yaw_omega);     // 库伦摩擦

	// 3. PID反馈（TD滤波后的值作为参考）
	if(Aim_Permission && aim->detect_number != 0){
		PID_Calculate(&Yaw_P_Pid,   gs->yaw*Ang_PI,   gs->yaw_ref * Ang_PI);
		PID_Calculate(&Yaw_S_Pid,   gs->d_yaw,         aim->yaw_omega_setpoint * Ang_PI);
	}
	else{
		PID_Calculate(&Yaw_P_Pid,   gs->yaw*Ang_PI,   Pos_Yaw_TD.x * Ang_PI);
		PID_Calculate(&Yaw_S_Pid,   gs->d_yaw,         Pos_Yaw_TD.dx * Ang_PI);
	}

	// 4. 总输出
	gs->Yaw_Motor_Out = Yaw_FF_Output + Yaw_P_Pid.Output + Yaw_S_Pid.Output;
}

/*******************************************************************************************************
云台控制器
********************************************************************************************************/
void Gimbal_Controllor(Shoot_Status_t *ss,
											 Gimbal_Status_t *gs,
											 Self_Rescue_t *self_re,
											 Controlled_State_t *cs,
											Aim_Rx *aim)
{
	if(*cs!=ERO && *cs!=STOP)
	{
		if(Down_Cboard_Info.fall_flag) //倒地
		{
			gs->Yaw_Motor_Out   = 0; //云台
			gs->Pitch_Motor_Out = 0; 
			
			ss->L_Motor_Out = 0; //发射机构
			ss->R_Motor_Out = 0;
			ss->Toggle_Motor_Out = 0;
			
			if(self_re->run_flag) //启动 云台复位
			{
				PID_Calculate(&Abs_Yaw_P_Pid,gs->abs_yaw,gs->abs_yaw_ref);//先清零再计算,纯P控制器
				gs->Yaw_Motor_Out = PID_Calculate(&Abs_Yaw_S_Pid,gs->d_yaw,Abs_Yaw_P_Pid.Output);
				gs->Pitch_Motor_Out = 0; 
			}
		}
		else //正常
		{

		#if GIMBAL_SYSID == GIMBAL_YAW_SYSID
			if (!gimbal_sysid.yaw.sysid_done)
			{
				GimbalSystemID_Run();
				gs->Yaw_Motor_Out = PID_Calculate(&Yaw_S_Pid,gs->d_yaw,gs->sys_yaw_speed_ref* Ang_PI);
			 }
			else
			{
				gs->Yaw_Motor_Out = PID_Calculate(&Yaw_S_Pid,gs->d_yaw,0.0f);				
			}

			Gimbal_Pitch_Calculate(gs,aim);
		#elif GIMBAL_SYSID == GIMBAL_PITCH_SYSID
			if(!gimbal_sysid.pitch.sysid_done)	
			{
				GimbalSystemID_Run();
			 } 		
			gs->Pitch_Motor_Out = PID_Calculate(&Pitch_S_Pid,   gs->d_pitch,  gs->sys_pitch_speed_ref * Ang_PI);
		#else
		    Gimbal_Pitch_Calculate(gs,aim);
		    Gimbal_Yaw_Calculate(gs,aim);
		#endif

			ss->L_Motor_Out = PID_Calculate(&L_Rpm_Pid,ss->L_Rpm, ss->Target_Rpm); //发射机构
			ss->R_Motor_Out = PID_Calculate(&R_Rpm_Pid,ss->R_Rpm,-ss->Target_Rpm);
			PID_Calculate(&D_Pos_Pid,ss->D_Pos,ss->Target_Pos);
			ss->Toggle_Motor_Out = PID_Calculate(&D_Spd_Pid,ss->D_Spd,D_Pos_Pid.Output);
		}
	}
	else
	{
		gs->Pitch_Motor_Out = 0; //云台
		gs->Yaw_Motor_Out   = 0;
		
		ss->L_Motor_Out = 0; //发射机构
		ss->R_Motor_Out = 0;
		ss->Toggle_Motor_Out = 0;
	}
}

/*******************************************************************************************************
向电机发送消息
********************************************************************************************************/
void Gimbal_Can_Data_Send(Controlled_State_t *cs,
													Shoot_Status_t *ss,
													Gimbal_Status_t *gs)
{
	if(*cs==ERO || *cs==STOP)
	{
		//下电时失能电机缓释
		Disable_Motor_Mode(&hfdcan3,0x0A);
		osDelay(1);
		Disable_Motor_Mode(&hfdcan2,0x0C);
		osDelay(1);
		Dji_Motor_Ctrl(&hfdcan3,0x1FF,0,0,0,0);
		Dji_Motor_Ctrl(&hfdcan2,0x1FF,0,0,0,0);
	}
	else{
		//Yaw电机
	if(Gimbal_Motor.DM_4310[0].state!=1) //状态异常
	{
		if(Gimbal_Motor.DM_4310[0].state==0) Enable_Motor_Mode(&hfdcan3,0x0A); //失能后重新使能
		else Clear_Err(&hfdcan3,0x0A); //清除异常
	}
	else Mit_Ctrl(&hfdcan3,0x0A,0,0,0,0,Max_Output(YAW_MOTOR_SIGN*gs->Yaw_Motor_Out,9),DM4310); 
//	else Mit_Ctrl(&hfdcan1,0x0A,0,0,0,0,0,DM4310);//暂时失能yaw电机<*_*> 
	osDelay(1);
	
	//Pitch电机
	if(Gimbal_Motor.DM_4310[1].state!=1) //状态异常
	{
		if(Gimbal_Motor.DM_4310[1].state==0) Enable_Motor_Mode(&hfdcan2,0x0C); //失能后重新使能
		else Clear_Err(&hfdcan2,0x0C); //清除异常
	}
	else Mit_Ctrl(&hfdcan2,0x0C,0,0,0,0,Max_Output(PITCH_MOTOR_SIGN*gs->Pitch_Motor_Out,9),DM4310);
//	else Mit_Ctrl(&hfdcan2,0x0C,0,0,0,0,0,DM4310);//暂时失能pitch电机<*_*>
	osDelay(1);
	
	//拨盘电机与摩擦轮电机
	Dji_Motor_Ctrl(&hfdcan3,0x1FF,0,Max_Output(ss->Toggle_Motor_Out,10000),																 0,																 0);
	Dji_Motor_Ctrl(&hfdcan2,0x1FF,0,																		 0,Max_Output(ss->L_Motor_Out,16384),Max_Output(ss->R_Motor_Out,16384));
	}
}

