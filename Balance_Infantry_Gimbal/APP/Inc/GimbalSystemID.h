/**
 ******************************************************************************
 * @file    GimbalSystemID.h
 * @brief   云台系统辨识模块 — 封装 Yaw / Pitch 轴的辨识测试逻辑
 *          包含公共采样累加器、步进序列执行器、边界往返扫描器
 ******************************************************************************
 */

#ifndef _GIMBAL_SYSTEM_ID_H
#define _GIMBAL_SYSTEM_ID_H

#include "stdint.h"
#include "stdbool.h"
#include "GimbalSystemIDConfig.h"
#include "PID.h"

//struct Gimbal_Status_t;

/*******************************************************************************************************
最大最小输出限制
********************************************************************************************************/
#define LIMIT_MAX_MIN(x, max, min) (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/*
 * 单轴辨识状态与结果。它不属于普通云台控制器，可在调试器中独立查看。
 * 已删除旧实现遗留且未使用的 RLS / SI 成员，降低对其他算法模块的依赖。
 */
typedef struct Gimbal_SI
{
    float sysid_timer;
    TD_t td_omega;
    uint8_t sysid_done;
    float J;
    float B;
    float B_raw;
    float C;
    float G_sin;
    float G_cos;
    float fit_rmse;
    float gravity_rmse;
    float bc_rmse;
    float j_rmse;
    float J_pair_min;
    float J_pair_max;
    float j_alpha_filtered;
    float j_signal_rms;
    float j_residual_ratio;
    float j_velocity_ref;
    float j_switch_angle;
    float j_target_accel;
    float mean_torque;
    float mean_omega;
    float mean_gravity;
    float mean_input;
    float mean_residual;
    uint32_t mean_raw_count;
    uint32_t sample_count;
    uint32_t gravity_valid_bins;
    uint32_t bc_sample_count;
    uint32_t j_sample_count;
    uint8_t mean_point_count;
    uint8_t j_motion_phase;
    uint8_t j_pass_count;
    uint8_t sysid_stage;
    uint8_t sysid_valid;
    uint8_t sysid_error;
} Gimbal_SI;

/* 由 safe_min_deg / safe_max_deg 自动生成，便于调试器检查实际运动区间。 */
typedef struct
{
    float safe_min_deg;
    float safe_max_deg;
    float reverse_min_deg;
    float reverse_max_deg;
    float sample_min_deg;
    float sample_max_deg;
    float j_start_deg;
    float j_end_deg;
    float j_switch_deg[GIMBAL_SYSID_PITCH_J_PASS_COUNT];
    float j_pair_center_deg[GIMBAL_SYSID_PITCH_J_PAIR_COUNT];
} GimbalSysIdPitchAngleLayout;

typedef struct
{
    Gimbal_SI yaw;
    Gimbal_SI pitch;
    GimbalSysIdPitchAngleLayout pitch_angle_layout;
} GimbalSystemIDContext;

/* Debug直接观察 gimbal_sysid.yaw 或 gimbal_sysid.pitch。 */
extern GimbalSystemIDContext gimbal_sysid;

/* Debug 观察值：sysid_stage / sysid_error。 */
typedef enum {
    PITCH_SYSID_STAGE_IDLE = 0,
    PITCH_SYSID_STAGE_GRAVITY,
    PITCH_SYSID_STAGE_BC,
    PITCH_SYSID_STAGE_J_PREPARE,
    PITCH_SYSID_STAGE_J_EXCITE,
    PITCH_SYSID_STAGE_DONE
} PitchSysIdStage;

typedef enum {
    GIMBAL_SYSID_ERROR_NONE = 0,
    GIMBAL_SYSID_ERROR_INSUFFICIENT_EXCITATION,
    GIMBAL_SYSID_ERROR_SAFETY_LIMIT,
    GIMBAL_SYSID_ERROR_NON_PHYSICAL_RESULT,
    GIMBAL_SYSID_ERROR_PAIR_MISMATCH,
    GIMBAL_SYSID_ERROR_POOR_FIT,
    GIMBAL_SYSID_ERROR_INVALID_CONFIG
} GimbalSysIdError;

/* ========== 1. 采样累加器 ========== */
typedef struct {
    float   sum;
    uint32_t count;
} SampleAccumulator;

void   SAcc_Reset(SampleAccumulator *acc);
void   SAcc_Add(SampleAccumulator *acc, float val);
float  SAcc_Mean(const SampleAccumulator *acc);

/* ========== 2. 多速度点步进序列 ========== */
typedef struct {
    /* 配置参数 */
    const float *vel_pts;       /* 速度序列指针        */
    uint8_t      num_pts;       /* 序列长度            */
    float        settle_time;   /* 每点稳定时间 (s)    */
    float        hold_revolutions; /* 每点旋转圈数     */
    float        deg_per_rev;   /* 每圈角度 (360.0f)   */

    /* 运行状态（调用方不应直接读写） */
    uint8_t  current_idx;
    float    step_elapsed;
    float    step_angle;
    uint32_t sample_count;
    float    torque_sum;
    float    omega_sum;
} StepSequencer;

void StepSeq_Init(StepSequencer *seq, const float *pts, uint8_t n,
                  float settle, float revs, float deg_per_rev);
bool StepSeq_Run(StepSequencer *seq, float dt,
                 float omega, float torque,
                 bool *done, float *out_omega, float *out_torque);

/* ========== 3. 角度边界往返扫描器 ========== */
typedef enum {
    SCAN_CONST_VEL  = 0,   /* 恒定速度模式 */
    SCAN_ACCEL_RAMP = 1    /* 加速度斜坡模式 */
} ScanMode;

typedef struct {
    /* 配置参数 */
    float      angle_min;       /* 安全角度下限 (°)    */
    float      angle_max;       /* 安全角度上限 (°)    */
    float      margin;          /* 换向余量 (°)        */
    uint8_t    max_half_cycles; /* 最大半周期数        */

    ScanMode   mode;
    float      vel_forward;     /* 正向速度 (CONST_VEL)     */
    float      vel_backward;    /* 反向速度 (CONST_VEL)     */
    float      accel;           /* 加速度 (ACCEL_RAMP)      */
    float      max_speed;       /* 最大速度限制 (ACCEL_RAMP)*/

    /* 运行状态 */
    uint8_t    half_cycle_count;
    float      current_vel_ref;
    float      ramp_speed;       /* 仅 ACCEL_RAMP 模式 */
} BoundaryScanner;

void BScan_Init_ConstVel(BoundaryScanner *bs,
                         float angle_min, float angle_max, float margin,
                         float vel_forward, float vel_backward,
                         uint8_t max_half_cycles);
void BScan_Init_AccelRamp(BoundaryScanner *bs,
                          float angle_min, float angle_max, float margin,
                          float accel, float max_speed,
                          uint8_t max_half_cycles);
bool BScan_Run(BoundaryScanner *bs, float theta_deg, float dt,
               float *out_vel_ref);
bool BScan_IsDone(const BoundaryScanner *bs);

/* ========== 4. 云台系统辨识顶层接口 ========== */
void GimbalSystemID_Init(Gimbal_Status_t *ctrl);
void GimbalSystemID_Run(void);

#endif /* _GIMBAL_SYSTEM_ID_H */
