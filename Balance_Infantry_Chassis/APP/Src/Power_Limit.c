#include "Power_Limit.h"
#include "LQR.h"
#include "VMC.h"
#include "Parameter.h"
#include "Some_Functions.h"
#include "arm_math.h"
#include <math.h>

PID_t Energy_Pid;
float P_max_limit = 100.0f;
float P_cmd = 0.0f;
Limit_State_t Limit_State = UNLIMIT;
Power_Limit_Goal_t PL_Goal;

void Power_Limit_Init(void)
{
    PID_Init(&Energy_Pid, 80.0f, 0, 0, ENERGY_KP, 0, ENERGY_KD, 0, 0, 0, 0, 0, NO_CIRCLE, NONE);
}

/*******************************************************************************
 能量环：PD控制器计算 P_max
 P_max = P_r - Kp*(E_target - E_c)
 限制 P_max >= POWER_FLOOR
*******************************************************************************/
void Energy_Ring_Calc(float P_r, float E_c)
{
    float E_target;
    E_target = CAP_TOTAL_ENERGY * CAP_E_RATIO;
    
    float adjustment = PID_Calculate(&Energy_Pid, E_c, E_target);
		
    float P_max = P_r - adjustment;//功率环输出为负
    
    if (P_max < POWER_FLOOR) {
        P_max = POWER_FLOOR;
    }
    if (P_max > 200.0f) {
        P_max = 200.0f;
    }
    
    P_max_limit = P_max;
}

/*******************************************************************************
 功率环
*******************************************************************************/
Limit_State_t Power_Limit_Apply(float omega_l, float omega_r ,float T_wl ,float T_wr)
{
    float k0 = PL_K0;
    float k1 = PL_K1;
    float k2 = PL_K2;
    float k3 = PL_K3;
    float P_max = P_max_limit;
    
		P_cmd = (k0 * T_wl*omega_l + k1 * fabsf(omega_l) + k2 * T_wl * T_wl + k3)
          + (k0 * T_wr*omega_r + k1 * fabsf(omega_r) + k2 * T_wr * T_wr + k3);
    
		P_cmd = Max_Output(P_cmd,MAX_PREDICT_POWER);//极限工况下功率预测偏差大,故限制最大预测功率
		
    if (P_cmd <= P_max) {
        return UNLIMIT;
    }
    else{
				return LIMIT;
		}
}
