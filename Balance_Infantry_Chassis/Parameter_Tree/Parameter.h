#ifndef __PARAMETER_TREE_H
#define __PARAMETER_TREE_H

//车体参数

#define FL       300.0f //气弹簧的力(N)
#define M_car     23.85f //整车重量(KG)
#define R_wheel   0.06f //轮子半径(m)
#define L_wheel  0.4662f //两个驱动轮之间距离(m)

#define Kt    0.232463677f //减速箱的转矩常数(Nm/A)
// #define Gear_Ratio  14.88f //自制减速箱的减速比
#define Gear_Ratio  15.7647f //XROLL减速箱的减速比

//云台和底盘是反过来的
#define ZERO_HEAD_YAW  0.961483002f  //正方向零点位置
#define ZERO_BACK_YAW  -2.17543435f   //负方向零点位置
 
//目标参数

#define DX_UP_MAX       1.00f     //高腿杆最大X速度(m/s)
#define DX_MID_MAX      1.70f     //中腿杆最大X速度
#define DX_DOWN_MAX     2.10f     //低腿杆最大X速度  
#define DX_MAX_PRO 			2.30f     //最大X速度(m/s)(超电)

#define DYAW_UP_MAX     6.0f     //高腿杆最大YAW速度(rad/s)
#define DYAW_MID_MAX    9.0f     //中腿杆最大YAW速度  
#define DYAW_DOWN_MAX   11.0f    //低腿杆最大YAW速度  
#define DYAW_MAX_PRO 		12.0f    //最大YAW速度(rad/s)(超电)

//为了克服静态误差把腿长调小
#define UP_LEG_LENGTH   0.34f    //高腿杆腿长(m)
#define MID_LEG_LENGTH  0.27f    //中腿杆腿长
#define DOWN_LEG_LENGTH 0.22f    //低腿杆腿长

#define PC_DX_RAMP_SENS    0.018f   //键鼠DX阶跃灵敏度
#define RC_DX_RAMP_SENS    0.020f   //遥控器DX阶跃灵敏度
#define PC_DYAW_RAMP_SENS  0.010f   //键鼠DYAW阶跃灵敏度
#define RC_DYAW_RAMP_SENS  0.005f   //遥控器DYAW阶跃灵敏度
#define LEG_RAMP_SENS      0.010f   //腿杆阶跃灵敏度

//离地参数
#define OFF_GROUND_FN_MAX    100.0f  //地面对车体最大支持力(离地判断阈值)
//暂时修改离地检测参数<*_*>
//#define OFF_GROUND_FN_MAX    135.0f  //地面对车体最大支持力(离地判断阈值)
#define OFF_GROUND_FN_COMP    40.0f  //离地后向下补偿的推力

//倒地自救参数

#define SELF_RESCUE_FN_COMP      80.0f  //倒地自救时收腿补偿力

#define BACK_SELF_RESCUE_SPEED  0.006f  //车体已翻倒自救速度(bz:调整此参数需要搭配调整PD值)
#define FRONT_SELF_RESCUE_SPEED 0.008f  //车体未翻倒自救速度(bz:调整此参数需要搭配调整PD值)

//判断倒地的各项参数

#define LEG_THETA_HEAD_MAX      -1.1f //腿杆向前的最大摆角
#define LEG_THETA_BACK_MAX       1.1f //腿杆向后的最大摆角
#define BUMP_LEG_THETA_BACK_MAX  2.4f //磕台阶模式下腿杆向后的最大摆角

//判断磕台阶的各项参数

#define BUMP_TWHEEL_MAX     5.2f //触发磕台阶判断的最大轮力矩值
#define BUMP_LEG_THETA_MAX  0.3f //触发磕台阶判断的最大腿杆摆角

#define BACK_LEG_TP         5.0f //磕上台阶腿杆往回摆的力
#define FRONT_LEG_TP      -17.0f //磕上台阶腿杆归正的力

#define RETRACT_LEG_LENGTH 0.18f //磕上台阶后收腿腿长

//判断打滑的各项参数

#define LRW_THRESHOLD 0.2f   //左右轮速差阈值
#define DVB_THRESHOLD 1.2f   //轮速与车体估计速度差值阈值
#define DBY_THRESHOLD 0.9f   //轮估计YAW速度与陀螺仪YAW速度差值阈值
#define DVW_THRESHOLD 0.06f  //加速度估计的瞬时速度与轮部速度估计的瞬时速度的差值阈值
#define DYAW_EXIT_THRESHOLD 1.0f //YAW速度退出阈值

//判断卡腿的各项参数

#define BST_THRESHOLD        2       //卡腿判断时间阈值
#define BRING_TWHEEL_MAX     5.2f    //触发卡腿判断的最大轮力矩值
#define BRING_LEG_THETA_MAX  0.5f    //触发卡腿判断的最大腿杆摆角
#define BRING_LEG_THETA_MIN  0.15f   //退出卡腿判断的最小腿杆摆角

#define BRING_COMP_FN_MAX    28.0f    //卡腿后收腿补偿力最大值
#define BRING_COMP_TL_COEF 	 0.008f  //卡腿后收腿补偿腿长系数
#define BRING_COMP_FN_COEF   6.0f    //卡腿后收腿补偿力系数

#endif
