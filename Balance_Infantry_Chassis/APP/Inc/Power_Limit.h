#ifndef __POWER_LIMIT_H
#define __POWER_LIMIT_H

#include "main.h"
#include "stdbool.h"
#include "PID.h"

// 电机功率模型常数（轮端，需根据实测标定）
#define PL_K0			1.0f		 // 机械功率系数
#define PL_K1     0.10f	 	 // 转速项系数
#define PL_K2     1.38f	 	 // 扭矩平方项系数
#define PL_K3     3.5f     // 单电机基底损耗(W)
#define MAX_PREDICT_POWER 250.0f//最大预测功率(W)

// 能量环参数
#define CAP_TOTAL_ENERGY   2800.0f  // 超级电容总容量(电压V*100)，根据实际标定
#define CAP_E_RATIO     	 0.6f     // 目标剩余 60%
#define CAP_E_MIN_RATIO    0.3f     // 最低保留 30%
#define ENERGY_KP          0.05f    // 能量环 P 增益
#define ENERGY_KD          0.0f     // 能量环 D 增益
#define POWER_FLOOR        35.0f    // 最低功率限制(W)

// 功率限制状态
typedef enum {
    UNLIMIT = 0,  // 无需限制
    LIMIT  = 1    // 需要限制
} Limit_State_t;

// 功率限制下的期望速度/角速度
typedef struct {
    float	goal_dx;
    float goal_dyaw;
} Power_Limit_Goal_t;

void Power_Limit_Init(void);
void Energy_Ring_Calc(float P_r, float E_c);
Limit_State_t Power_Limit_Apply(float omega_l, float omega_r ,float T_wl ,float T_wr);

extern PID_t Energy_Pid;
extern float P_max_limit;
extern float P_cmd;
extern Limit_State_t Limit_State;
extern Power_Limit_Goal_t PL_Goal;

#endif
