/**
 ******************************************************************************
 * @file    GimbalSystemIDConfig.h
 * @brief   系统辨识移植入口：模式选择、固定数组尺寸和用户配置类型
 *
 * 移植者通常只需要修改：
 *   1. 本文件顶部的运行模式与“已完成移植确认”；
 *   2. GimbalSystemIDConfig.c 中带【必须确认】标记的配置对象；
 *   3. Gimbal.c 中对应轴的系统辨识速度环 PID。
 *
 * 不要把所有参数作为 GimbalSystemID_Init() 的形参传入。配置结构体既保留
 * 字段名和单位，又不会增加初始化调用复杂度；只有决定条件编译的选项使用宏。
 ******************************************************************************
 */

#ifndef GIMBAL_SYSTEM_ID_CONFIG_H
#define GIMBAL_SYSTEM_ID_CONFIG_H

#include "stdint.h"

/* ======================== 1. 运行模式（仅此处使用宏） ======================== */
#define GIMBAL_SYSID_DISABLED                  0
#define GIMBAL_YAW_SYSID                       1
#define GIMBAL_PITCH_SYSID                     2

#define GIMBAL_SYSID_STEP_GRAVITY              1
#define GIMBAL_SYSID_STEP_BC                   2
#define GIMBAL_SYSID_STEP_J                    3
#define GIMBAL_SYSID_STEP_ALL                  4

/* 正常运行保持 DISABLED；辨识时改成 YAW_SYSID 或 PITCH_SYSID。 */
#define GIMBAL_SYSID                           GIMBAL_SYSID_DISABLED
#define GIMBAL_SYSID_STEP                      GIMBAL_SYSID_STEP_ALL

/*
 * 【必须确认】新项目第一次启用辨识前，在完成移植清单后改成 1。
 * 关闭辨识时允许保持 0，不影响正常固件编译。
 */
#define GIMBAL_SYSID_PORTING_CONFIRMED         1

#if (GIMBAL_SYSID != GIMBAL_SYSID_DISABLED) && \
    (GIMBAL_SYSID_PORTING_CONFIRMED != 1)
#error "Gimbal System ID: read GimbalSystemID_PORTING.md, verify user config, then set GIMBAL_SYSID_PORTING_CONFIRMED to 1"
#endif

#if (GIMBAL_SYSID != GIMBAL_SYSID_DISABLED) && \
    (GIMBAL_SYSID != GIMBAL_YAW_SYSID) && \
    (GIMBAL_SYSID != GIMBAL_PITCH_SYSID)
#error "GIMBAL_SYSID must be DISABLED, GIMBAL_YAW_SYSID or GIMBAL_PITCH_SYSID"
#endif

#if (GIMBAL_SYSID_STEP < GIMBAL_SYSID_STEP_GRAVITY) || \
    (GIMBAL_SYSID_STEP > GIMBAL_SYSID_STEP_ALL)
#error "Invalid GIMBAL_SYSID_STEP"
#endif

#if (GIMBAL_SYSID == GIMBAL_YAW_SYSID) && \
    (GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_GRAVITY)
#error "Yaw has no gravity stage; select BC, J or ALL"
#endif

/* 固定算法规模；移植者通常不修改。 */
#define GIMBAL_SYSID_YAW_BC_POINT_COUNT        8U
#define GIMBAL_SYSID_YAW_J_PAIR_COUNT          2U
#define GIMBAL_SYSID_PITCH_BC_LEVEL_COUNT      3U
#define GIMBAL_SYSID_PITCH_J_PASS_COUNT        3U
#define GIMBAL_SYSID_PITCH_J_PAIR_COUNT        2U

#if (GIMBAL_SYSID_PITCH_J_PAIR_COUNT + 1U) != \
    GIMBAL_SYSID_PITCH_J_PASS_COUNT
#error "Pitch J layout requires pair_count = pass_count - 1"
#endif

/* ======================== 2. 用户配置（单位写在字段名中） =================== */
typedef struct
{
    /* 电机反馈力矩 -> 模型正方向力矩的系数，通常为 +1 或 -1。【必须确认】 */
    float torque_feedback_coef;

    /* B/C 测试速度，建议正负交替且覆盖低/中/高速度。【必须确认可稳定跟踪】 */
    float bc_speed_dps[GIMBAL_SYSID_YAW_BC_POINT_COUNT];

    /* J 三角速度轨迹与实际速度采样窗口。【必须确认无线缆/机械限位风险】 */
    float j_ref_accel_dps2;
    float j_max_ref_dps;
    float j_pair_center_dps[GIMBAL_SYSID_YAW_J_PAIR_COUNT];
} GimbalSysIdYawUserConfig;

typedef struct
{
    /* 电机反馈力矩 -> Pitch向上为正的模型力矩系数。【必须确认】 */
    float torque_feedback_coef;

    /* IMU Pitch角度硬安全边界，不是期望工作边界。【必须实机确认】 */
    float safe_min_deg;
    float safe_max_deg;

    /* 重力扫描参考速度；目标是让上/下实际速度幅值接近。 */
    float gravity_up_ref_dps;
    float gravity_down_ref_dps;

    /* 三档B/C扫描：参考速度与期望实际速度。【必须先调好速度环并实测】 */
    float bc_up_ref_dps[GIMBAL_SYSID_PITCH_BC_LEVEL_COUNT];
    float bc_down_ref_dps[GIMBAL_SYSID_PITCH_BC_LEVEL_COUNT];
    float bc_expected_actual_dps[GIMBAL_SYSID_PITCH_BC_LEVEL_COUNT];

    /* J 测试角度由 safe_min/safe_max 自动分配，这里只配置速度轨迹。 */
    float j_prepare_up_ref_dps;
    float j_prepare_down_ref_dps;
    float j_actual_peak_speed_dps;
    float j_up_ref_offset_dps;
    float j_up_ref_min_dps;
    float j_up_ref_max_dps;
    float j_return_down_ref_dps;

} GimbalSysIdPitchUserConfig;

typedef struct
{
    GimbalSysIdYawUserConfig yaw;
    GimbalSysIdPitchUserConfig pitch;
} GimbalSystemIDUserConfig;

typedef enum
{
    GIMBAL_SYSID_PITCH_PROFILE_OLD = 0,
    GIMBAL_SYSID_PITCH_PROFILE_NEW,
    GIMBAL_SYSID_PITCH_PROFILE_COUNT
} GimbalSysIdPitchProfileId;

/*
 * 配置档案保存在 Flash 中，gimbal_sysid_user_config 是启动时装载的工作副本。
 * 可在 GimbalSystemID_Init() 前按硬件ID选择，也可只修改默认 profile_id。
 */
extern GimbalSystemIDUserConfig gimbal_sysid_user_config;
extern GimbalSysIdPitchProfileId gimbal_sysid_pitch_profile_id;

uint8_t GimbalSystemID_SelectPitchProfile(GimbalSysIdPitchProfileId profile_id);
void GimbalSystemID_LoadSelectedProfile(void);

#endif /* GIMBAL_SYSTEM_ID_CONFIG_H */
