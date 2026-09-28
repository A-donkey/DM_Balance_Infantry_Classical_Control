#ifndef __LQR_H
#define __LQR_H

#include "main.h"

//INCLUDE部分
#include "stdbool.h"
#include "Chassis_Task.h"
#include "Some_Functions.h"
#include "Power_Limit.h"

#define X_MAX       6.00f
//#define Yaw_MAX     1.25f
#define Yaw_MAX     0.5f//不同地面光滑程度不同
#define Theta_L_MAX 0.45f
#define Theta_R_MAX 0.45f
#define Theta_B_MAX 0.25f

void Offset_Calc(Flag_Bit_t *flag,Compensation_Amount_t *comp,float (*Fit_Coefficients),float TL);
void Fitting_K_Calc(float (*fitting_k)[10],float (*p)[6],float L_l,float L_r);
void LQR_Calc(Flag_Bit_t *flag,Goal_Setting_t *goal,Compensation_Amount_t *comp,Body_Current_Situation_t *body,Leg_Current_Situation_t *leg[2],Joint_Motor_Status_t *joint_m[2]);

//变腿长下多项式拟合的全部系数
extern float u[10];
extern float P[40][6];
extern float Fitting_K[4][10];
extern float Offset_Fit_Coefficients[4];
#endif
