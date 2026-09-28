/**
 ******************************************************************************
 * @file    GimbalSystemIDConfig.c
 * @brief   系统辨识用户配置。移植时优先检查本文件，不要先改算法实现。
 ******************************************************************************
 */

#include "GimbalSystemIDConfig.h"

/*
 * 每套实测参数作为独立档案保存在 Flash 中，切换硬件不会覆盖另一套数据。
 * Pitch 的所有测试角度只由 safe_min_deg / safe_max_deg 自动生成。
 */
static const GimbalSysIdPitchUserConfig pitch_profiles[GIMBAL_SYSID_PITCH_PROFILE_COUNT] =
{
    [GIMBAL_SYSID_PITCH_PROFILE_OLD] =
    {
        .torque_feedback_coef = -1.0f,
        .safe_min_deg = -30.5f,
        .safe_max_deg = 30.0f,
        .gravity_up_ref_dps = 50.0f,
        .gravity_down_ref_dps = 10.0f,
        .bc_up_ref_dps = {50.0f, 60.0f, 70.0f},
        .bc_down_ref_dps = {10.0f, 20.0f, 30.0f},
        .bc_expected_actual_dps = {20.0f, 30.0f, 40.0f},
				
        .j_prepare_up_ref_dps = 80.0f,
        .j_prepare_down_ref_dps = 10.0f,
        .j_actual_peak_speed_dps = 50.0f,
        .j_up_ref_offset_dps = 30.0f,
        .j_up_ref_min_dps = 35.0f,
        .j_up_ref_max_dps = 80.0f,
        .j_return_down_ref_dps = 1.0f
    },
    [GIMBAL_SYSID_PITCH_PROFILE_NEW] =
    {
        .torque_feedback_coef = -1.0f,
        .safe_min_deg = -13.0f,
        .safe_max_deg = 40.0f,
        .gravity_up_ref_dps = 50.0f,
        .gravity_down_ref_dps = 1.0f,
        .bc_up_ref_dps = {50.0f, 60.0f, 70.0f},
        .bc_down_ref_dps = {1.0f, 12.0f, 20.0f},
        .bc_expected_actual_dps = {20.0f, 30.0f, 40.0f},
        .j_prepare_up_ref_dps = 50.0f,
        .j_prepare_down_ref_dps = 1.0f,
        .j_actual_peak_speed_dps = 30.0f,
        .j_up_ref_offset_dps = 30.0f,
        .j_up_ref_min_dps = 35.0f,
        .j_up_ref_max_dps = 60.0f,
        .j_return_down_ref_dps = 1.0f
    }
};

/* 当前在老云台测试；若能读取硬件ID，应在 Init 前调用选择函数。 */
GimbalSysIdPitchProfileId gimbal_sysid_pitch_profile_id =
    GIMBAL_SYSID_PITCH_PROFILE_OLD;

GimbalSystemIDUserConfig gimbal_sysid_user_config =
{
    .yaw =
    {
        .torque_feedback_coef = 1.0f,
        .bc_speed_dps =
        {
            50.0f, -50.0f, 100.0f, -100.0f,
            150.0f, -150.0f, 200.0f, -200.0f
        },
        .j_ref_accel_dps2 = 80.0f,
        .j_max_ref_dps = 200.0f,
        .j_pair_center_dps = {80.0f, 150.0f}
    },
    .pitch = {0}
};

uint8_t GimbalSystemID_SelectPitchProfile(GimbalSysIdPitchProfileId profile_id)
{
    if ((uint32_t)profile_id >= (uint32_t)GIMBAL_SYSID_PITCH_PROFILE_COUNT)
        return 0U;

    gimbal_sysid_pitch_profile_id = profile_id;
    gimbal_sysid_user_config.pitch = pitch_profiles[profile_id];
    return 1U;
}

void GimbalSystemID_LoadSelectedProfile(void)
{
    if (!GimbalSystemID_SelectPitchProfile(gimbal_sysid_pitch_profile_id))
        (void)GimbalSystemID_SelectPitchProfile(GIMBAL_SYSID_PITCH_PROFILE_OLD);
}
