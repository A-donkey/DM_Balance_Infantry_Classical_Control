/**
 * @file    LESO.c
 * @brief   双腿耦合轮足机器人 (10状态-4输入) 离散线性扩张状态观测器实现
 */

#include "LESO.h"

// 全局 LESO 控制器实例
LESO_t Chassis_LESO;

/**
 * @brief  初始化 LESO 观测器
 */
void LESO_Init(LESO_t *leso)
{
    if (leso == NULL) return;
    
    memset(leso->x_hat, 0, sizeof(leso->x_hat));
    memset(leso->d_hat, 0, sizeof(leso->d_hat));
    memset(leso->d_hat_lpf, 0, sizeof(leso->d_hat_lpf));
    
    // 一阶滤波系数: 1kHz 频率下 alpha=0.25f 对应约 45Hz 截止频率，既平滑又无明显滞后
    leso->lpf_alpha = 0.25f;
    
    // 扰动限幅保护 (防止异常撞击或离地瞬间力矩突变打折机构)
    leso->dist_limit[0] = 20.0f; // T_r_to_b 右关节摆动力矩限幅 (N*m)
    leso->dist_limit[1] = 20.0f; // T_l_to_b 左关节摆动力矩限幅 (N*m)
    leso->dist_limit[2] = 6.0f;  // T_wr_to_r 右轮驱动力矩限幅 (N*m)
    leso->dist_limit[3] = 6.0f;  // T_wl_to_l 左轮驱动力矩限幅 (N*m)
    
    leso->enable = true;
    leso->inited = true;
    leso->update_cnt = 0;
}

/**
 * @brief  复位 LESO 观测器状态
 */
void LESO_Reset(LESO_t *leso)
{
    if (leso == NULL) return;
    
    memset(leso->x_hat, 0, sizeof(leso->x_hat));
    memset(leso->d_hat, 0, sizeof(leso->d_hat));
    memset(leso->d_hat_lpf, 0, sizeof(leso->d_hat_lpf));
    leso->update_cnt = 0;
}

/**
 * @brief  设置观测器是否使能补偿
 */
void LESO_Set_Enable(LESO_t *leso, bool enable)
{
    if (leso == NULL) return;
    leso->enable = enable;
    if (!enable) {
        // 关闭补偿时重置扰动项，防止再次开启时产生力矩跳变
        memset(leso->d_hat, 0, sizeof(leso->d_hat));
        memset(leso->d_hat_lpf, 0, sizeof(leso->d_hat_lpf));
    }
}

/**
 * @brief  LESO 离散递推更新 (1kHz 周期调用)
 * 
 * 递推数学方程拆解优化:
 *   1. 创新残差:
 *      e[k] = y[k] - x_hat[k]  (10维向量)
 * 
 *   2. 扰动更新积分:
 *      d_hat[k+1] = d_hat[k] + L_d * e[k]  (4维向量)
 * 
 *   3. 状态预测更新:
 *      u_eff = u_applied + d_hat[k]
 *      x_hat[k+1] = G * x_hat[k] + H * u_eff + L_x * e[k]  (10维向量)
 */
void LESO_Update(LESO_t *leso, const float y[LESO_STATE_DIM], const float u_applied[LESO_CTRL_DIM])
{
    if (leso == NULL || !leso->inited) return;
    
    // 首次调用或状态未同步时，先用传感器测量值同步初始状态估计
    if (leso->update_cnt == 0) {
        for (int i = 0; i < LESO_STATE_DIM; i++) {
            leso->x_hat[i] = y[i];
        }
        leso->update_cnt++;
        return;
    }
    
    // 1. 计算测量创新误差 e = y - x_hat (10维)
    float e[LESO_STATE_DIM];
    for (int i = 0; i < LESO_STATE_DIM; i++) {
        e[i] = y[i] - leso->x_hat[i];
    }
    
    // 2. 更新扰动估计 d_hat[k+1] = d_hat[k] + L_d * e
    float d_new[LESO_DISTURB_DIM];
    for (int i = 0; i < LESO_DISTURB_DIM; i++) {
        float l_dot_e = 0.0f;
        for (int j = 0; j < LESO_STATE_DIM; j++) {
            l_dot_e += LESO_Ld[i][j] * e[j];
        }
        d_new[i] = leso->d_hat[i] + l_dot_e;
        
        // 幅值限幅安全钳位
        if (d_new[i] > leso->dist_limit[i]) {
            d_new[i] = leso->dist_limit[i];
        } else if (d_new[i] < -leso->dist_limit[i]) {
            d_new[i] = -leso->dist_limit[i];
        }
        
        // 一阶低通滤波
        leso->d_hat_lpf[i] = (1.0f - leso->lpf_alpha) * leso->d_hat_lpf[i] + leso->lpf_alpha * d_new[i];
    }
    
    // 3. 计算等效控制输入 u_eff = u_applied + d_hat
    float u_eff[LESO_CTRL_DIM];
    for (int j = 0; j < LESO_CTRL_DIM; j++) {
        u_eff[j] = u_applied[j] + leso->d_hat[j];
    }
    
    // 4. 更新状态估计 x_hat[k+1] = G * x_hat + H * u_eff + L_x * e
    float x_new[LESO_STATE_DIM];
    for (int i = 0; i < LESO_STATE_DIM; i++) {
        float sum_g = 0.0f;
        for (int j = 0; j < LESO_STATE_DIM; j++) {
            sum_g += LESO_G[i][j] * leso->x_hat[j];
        }
        
        float sum_h = 0.0f;
        for (int j = 0; j < LESO_CTRL_DIM; j++) {
            sum_h += LESO_H[i][j] * u_eff[j];
        }
        
        float sum_lx = 0.0f;
        for (int j = 0; j < LESO_STATE_DIM; j++) {
            sum_lx += LESO_Lx[i][j] * e[j];
        }
        
        x_new[i] = sum_g + sum_h + sum_lx;
    }
    
    // 5. 将计算好的新状态写入句柄
    for (int i = 0; i < LESO_STATE_DIM; i++) {
        leso->x_hat[i] = x_new[i];
    }
    for (int i = 0; i < LESO_DISTURB_DIM; i++) {
        leso->d_hat[i] = d_new[i];
    }
    
    leso->update_cnt++;
}

/**
 * @brief  对 LQR 控制量进行前馈补偿
 *         u_out = u_lqr - d_hat_lpf
 */
void LESO_Compensate(const LESO_t *leso, const float u_lqr[LESO_CTRL_DIM], float u_out[LESO_CTRL_DIM])
{
    if (leso == NULL || !leso->enable || !leso->inited) {
        // 未使能时直通名义输出
        for (int i = 0; i < LESO_CTRL_DIM; i++) {
            u_out[i] = u_lqr[i];
        }
        return;
    }
    
    for (int i = 0; i < LESO_CTRL_DIM; i++) {
        u_out[i] = u_lqr[i] - leso->d_hat_lpf[i];
    }
}
