/**
 * @file    LESO.h
 * @brief   双腿耦合轮足机器人 (10状态-4输入) 离散线性扩张状态观测器 (LESO)
 * 
 * 状态向量 y[10]:
 *   [0] X_b_h      机体水平位移 (m)
 *   [1] V_b_h      机体水平速度 (m/s)
 *   [2] phi        偏航角 Yaw (rad)
 *   [3] dphi       偏航角速度 (rad/s)
 *   [4] theta_l    左腿摆杆角度 (rad)
 *   [5] dtheta_l   左腿摆杆角速度 (rad/s)
 *   [6] theta_r    右腿摆杆角度 (rad)
 *   [7] dtheta_r   右腿摆杆角速度 (rad/s)
 *   [8] theta_b    机体俯仰角 Pitch (rad)
 *   [9] dtheta_b   机体俯仰角速度 (rad/s)
 * 
 * 控制向量 u[4]:
 *   [0] T_r_to_b   右髋关节摆动力矩 (N*m) -> 对应 joint_m[1]->Tp
 *   [1] T_l_to_b   左髋关节摆动力矩 (N*m) -> 对应 joint_m[0]->Tp
 *   [2] T_wr_to_r  右驱动轮电机力矩 (N*m) -> 对应 joint_m[1]->T_Wheel
 *   [3] T_wl_to_l  左驱动轮电机力矩 (N*m) -> 对应 joint_m[0]->T_Wheel
 * 
 * 扩张扰动向量 d[4]:
 *   分别对应上述 4 个控制输入通道上的等效输入总扰动 (内部模型失配 + 外部环境冲击)
 */

#ifndef __LESO_H
#define __LESO_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "leso_parameters.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LESO 观测器控制结构体
 */
typedef struct {
    /* 观测状态 */
    float x_hat[LESO_STATE_DIM];       // 状态估计值 x_hat (10维)
    float d_hat[LESO_DISTURB_DIM];     // 总扰动估计原始值 d_hat (4维, 对应Tr, Tl, Twr, Twl)
    float d_hat_lpf[LESO_DISTURB_DIM]; // 低通滤波后的总扰动估计值 (用于前馈补偿)
    
    /* 算法参数与安全阈值 */
    float lpf_alpha;                   // 一阶低通滤波系数 [0.0 ~ 1.0], 越小滤波越平滑
    float dist_limit[LESO_DISTURB_DIM];// 扰动估计前馈幅值限幅保护 (N*m)
    
    /* 状态与标志位 */
    bool  enable;                      // 观测器补偿使能开关 (true: 开启前馈; false: 仅观测不补偿或停止更新)
    bool  inited;                      // 初始化完成标志
    uint32_t update_cnt;               // 更新周期计数
} LESO_t;

/**
 * @brief 全局 LESO 句柄声明
 */
extern LESO_t Chassis_LESO;

/**
 * @brief  初始化 LESO 观测器
 * @param  leso: 观测器句柄指针
 */
void LESO_Init(LESO_t *leso);

/**
 * @brief  复位 LESO 观测器内部状态 (清零估计值与滤波值)
 * @param  leso: 观测器句柄指针
 * @note   在机器人倒地、失能、跳跃腾空或剧烈碰撞保护时调用
 */
void LESO_Reset(LESO_t *leso);

/**
 * @brief  设置观测器是否使能补偿
 * @param  leso: 观测器句柄指针
 * @param  enable: true 开启补偿, false 关闭补偿
 */
void LESO_Set_Enable(LESO_t *leso, bool enable);

/**
 * @brief  LESO 离散更新计算 (1kHz 周期调用)
 * @param  leso: 观测器句柄指针
 * @param  y: 传感器测量的 10 维系统真实状态
 * @param  u_applied: 上一控制周期实际施加给执行机构的 4 维力矩 [Tr, Tl, Twr, Twl]
 */
void LESO_Update(LESO_t *leso, const float y[LESO_STATE_DIM], const float u_applied[LESO_CTRL_DIM]);

/**
 * @brief  将 LESO 估计的扰动对 LQR 名义控制量进行前馈补偿
 * @param  leso: 观测器句柄指针
 * @param  u_lqr: LQR 控制器计算得到的名义力矩 [Tr, Tl, Twr, Twl]
 * @param  u_out: 经扰动补偿后的最终输出力矩 [Tr, Tl, Twr, Twl]
 * @note   补偿公式: u_out = u_lqr - d_hat_lpf
 */
void LESO_Compensate(const LESO_t *leso, const float u_lqr[LESO_CTRL_DIM], float u_out[LESO_CTRL_DIM]);

#ifdef __cplusplus
}
#endif

#endif // __LESO_H
