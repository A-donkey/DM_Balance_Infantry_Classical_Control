/**
 ******************************************************************************
 * @file    GimbalSystemID.c
 * @brief   云台系统辨识模块实现
 *          包含 Yaw / Pitch 轴的辨识测试逻辑以及公共工具
 *
 * 用法：
 *   1. 在合适位置调用 GimbalSystemID_Init(&gimbal_controller)
 *   2. 在循环中调用 GimbalSystemID_Run()
 *   3. 在 GimbalSystemIDConfig.h 中选择轴/步骤
 *   4. 在 GimbalSystemIDConfig.c 中设置机构相关参数
 *   5. 新项目先阅读 GimbalSystemID_PORTING.md
 ******************************************************************************
 */
#include "Gimbal_Task.h" 
#include "GimbalSystemID.h"
#include "PID.h"
#include "arm_math.h"
#include "INS_Task.h"        /* Ang_PI */

/*
 * ========== 算法内部默认值 ==========
 * 与机构相关的运动参数统一来自 gimbal_sysid_user_config；这里保留的主要是
 * 采样质量、滤波和拟合判据，移植者通常不需要修改。
 */
#define YAW_BC_POINT_COUNT                       GIMBAL_SYSID_YAW_BC_POINT_COUNT
#define YAW_BC_SETTLE_S                         0.4f
#define YAW_BC_SAMPLE_REVOLUTIONS               0.5f
#define YAW_BC_SPEED_TRACK_RATIO                0.25f
#define YAW_BC_NEGATIVE_B_TOLERANCE             0.5f

/*
 * J：先正向再反向完成一组加速/减速，净转角接近0。每个方向均在相同
 * 实际速度窗口配对，C自动抵消，只需修正B*delta_omega。
 */
#define YAW_J_PAIR_WINDOW_COUNT                  GIMBAL_SYSID_YAW_J_PAIR_COUNT
#define YAW_J_DIRECTION_COUNT                    2U
#define YAW_J_REF_ACCEL_DPS2                    (gimbal_sysid_user_config.yaw.j_ref_accel_dps2)
#define YAW_J_MAX_REF_DPS                       (gimbal_sysid_user_config.yaw.j_max_ref_dps)
#define YAW_J_PAIR_HALF_WIDTH_DPS               15.0f
#define YAW_J_MIN_TRAVERSAL_SAMPLES             30U
#define YAW_J_MIN_TRAVERSAL_TIME_S              0.08f
#define YAW_J_MIN_MEAN_ALPHA_DPS2                5.0f
#define YAW_J_ZERO_SPEED_DPS                     3.0f
#define YAW_J_ZERO_SETTLE_S                      0.25f
#define YAW_J_MAX_PAIR_RATIO                     3.0f
#define YAW_J_MAX_RMSE_SIGNAL_RATIO              3.0f
#define YAW_J_ALPHA_LPF_HZ                       3.0f
#define YAW_J_RAW_ALPHA_REJECT_DPS2            500.0f

#define PITCH_SAFE_MIN_DEG                 (gimbal_sysid_user_config.pitch.safe_min_deg)
#define PITCH_SAFE_MAX_DEG                 (gimbal_sysid_user_config.pitch.safe_max_deg)
#define PITCH_REVERSE_MARGIN_DEG             1.0f
#define PITCH_SAMPLE_MARGIN_DEG              2.5f
#define PITCH_MIN_USABLE_SPAN_DEG            20.0f
#define PITCH_REVERSAL_SETTLE_S               0.25f
#define PITCH_CONST_ALPHA_MAX_DPS2           80.0f
#define PITCH_SPEED_TRACK_RATIO               0.35f

/*
 * 重力扫描参考速度。纯 P 速度环受重力影响，上升和下降使用独立参考值；
 * 调参目标是让两个方向的实际速度幅值接近，而不是让参考值相等。
 */
#define PITCH_GRAVITY_UP_REF_DPS             (gimbal_sysid_user_config.pitch.gravity_up_ref_dps)
#define PITCH_GRAVITY_DOWN_REF_DPS           (gimbal_sysid_user_config.pitch.gravity_down_ref_dps)
#define PITCH_GRAVITY_MIN_ACTUAL_SPEED_DPS      3.0f
#define PITCH_GRAVITY_PAIR_SPEED_TOL_DPS        3.0f
#define PITCH_GRAVITY_HALF_CYCLES             4U
#define PITCH_GRAVITY_BIN_COUNT              20U
#define PITCH_GRAVITY_MIN_BIN_SAMPLES        20U
#define PITCH_GRAVITY_MIN_VALID_BINS          8U

/* B/C：每个速度幅值都做一次正向和反向全行程扫描。 */
#define PITCH_BC_SPEED_LEVELS                 GIMBAL_SYSID_PITCH_BC_LEVEL_COUNT
#define PITCH_BC_HALF_CYCLES                 (2U * PITCH_BC_SPEED_LEVELS)
#define PITCH_BC_MIN_SPEED_DPS                5.0f
#define PITCH_BC_MIN_SEGMENT_RAW_SAMPLES     50U
#define PITCH_BC_NEGATIVE_B_TOLERANCE         0.5f

/*
 * J：单向上升测量。前半程匀加速、后半程匀减速，采样期间 omega 始终为正，
 * 因而库仑摩擦方向固定；下降过程只用于复位，不参与辨识。
 */
#define PITCH_REVERSE_MIN_DEG               (gimbal_sysid.pitch_angle_layout.reverse_min_deg)
#define PITCH_REVERSE_MAX_DEG               (gimbal_sysid.pitch_angle_layout.reverse_max_deg)
#define PITCH_SAMPLE_MIN_DEG                (gimbal_sysid.pitch_angle_layout.sample_min_deg)
#define PITCH_SAMPLE_MAX_DEG                (gimbal_sysid.pitch_angle_layout.sample_max_deg)
#define PITCH_J_START_DEG                   (gimbal_sysid.pitch_angle_layout.j_start_deg)
#define PITCH_J_END_DEG                     (gimbal_sysid.pitch_angle_layout.j_end_deg)
#define PITCH_J_CENTER_TOL_DEG                0.8f
#define PITCH_J_PREP_FAST_BAND_DEG             2.0f
#define PITCH_J_PREP_UP_REF_DPS              (gimbal_sysid_user_config.pitch.j_prepare_up_ref_dps)
#define PITCH_J_PREP_DOWN_REF_DPS            (gimbal_sysid_user_config.pitch.j_prepare_down_ref_dps)
#define PITCH_J_PREP_HOLD_REF_DPS            25.0f
#define PITCH_J_PREP_POSITION_KP               8.0f
#define PITCH_J_PREP_REF_MIN_DPS              15.0f
#define PITCH_J_PREP_REF_MAX_DPS              40.0f
#define PITCH_J_SETTLE_S                      0.5f
#define PITCH_J_ACTUAL_MIN_SPEED_DPS           5.0f
#define PITCH_J_ACTUAL_PEAK_SPEED_DPS        (gimbal_sysid_user_config.pitch.j_actual_peak_speed_dps)
#define PITCH_J_UP_REF_OFFSET_DPS            (gimbal_sysid_user_config.pitch.j_up_ref_offset_dps)
#define PITCH_J_UP_REF_MIN_DPS               (gimbal_sysid_user_config.pitch.j_up_ref_min_dps)
#define PITCH_J_UP_REF_MAX_DPS               (gimbal_sysid_user_config.pitch.j_up_ref_max_dps)
#define PITCH_J_RETURN_DOWN_REF_DPS          (gimbal_sysid_user_config.pitch.j_return_down_ref_dps)
#define PITCH_J_MEASURED_PASSES              GIMBAL_SYSID_PITCH_J_PASS_COUNT
#define PITCH_J_PAIR_BIN_COUNT               GIMBAL_SYSID_PITCH_J_PAIR_COUNT
#define PITCH_J_PAIR_HALF_WIDTH_DEG              1.5f
#define PITCH_J_MIN_TRAVERSAL_SAMPLES            20U
#define PITCH_J_MIN_TRAVERSAL_TIME_S             0.03f
#define PITCH_J_MIN_MEAN_ALPHA_DPS2              3.0f
#define PITCH_J_SWITCH_SETTLE_S                  0.05f
#define PITCH_J_FINAL_HOLD_TIMEOUT_S            2.0f
#define PITCH_J_MIN_SPEED_DPS                   5.0f
#define PITCH_J_RAW_ALPHA_REJECT_DPS2        250.0f
#define PITCH_J_ALPHA_LPF_HZ                   2.0f
#define PITCH_J_MAX_PAIR_RATIO                  3.0f
#define PITCH_J_MAX_RMSE_SIGNAL_RATIO            3.0f

typedef struct
{
    float s00;
    float s01;
    float s11;
    float b0;
    float b1;
    float yy;
    uint32_t count;
} LeastSquares2;

typedef struct
{
    float torque_pos_sum;
    float torque_neg_sum;
    float omega_pos_sum;
    float omega_neg_sum;
    uint32_t pos_count;
    uint32_t neg_count;
} GravityAngleBin;

typedef struct
{
    float aa;
    float ay;
    float yy;
    uint32_t count;
} LeastSquares1;

/*
 * B/C 行程累加器。实时数据只在这里求和，行程结束后才生成一个
 * 等权平均点参与回归，避免长行程或高采样率主导辨识结果。
 */
typedef struct
{
    float torque_sum;
    float omega_sum;
    float gravity_sum;
    uint32_t count;
} PitchBCMean;

/* 同一角度窗口内，一次上升穿越只形成一个等权平均点。 */
typedef struct
{
    float torque_mean_sum;
    float omega_mean_sum;
    float theta_mean_sum;
    float alpha_mean_sum;
    uint32_t raw_count;
    uint8_t traversal_count;
} PitchJPairGroup;

typedef struct
{
    float torque_sum;
    float omega_sum;
    float theta_sum;
    float omega_start;
    float omega_last;
    float elapsed;
    uint32_t count;
    int8_t bin_index;
    int8_t group_index;
} PitchJTraversal;

typedef struct
{
    float torque_mean_sum;
    float omega_mean_sum;
    float alpha_mean_sum;
    uint32_t raw_count;
    uint8_t traversal_count;
} YawJPairGroup;

typedef struct
{
    float torque_sum;
    float omega_sum;
    float omega_start;
    float omega_last;
    float elapsed;
    uint32_t count;
    int8_t window_index;
    int8_t direction_index;
    int8_t phase_index;
} YawJTraversal;

typedef enum
{
    YAW_SYSID_STAGE_IDLE = 0,
    YAW_SYSID_STAGE_BC,
    YAW_SYSID_STAGE_J_PREPARE,
    YAW_SYSID_STAGE_J_EXCITE,
    YAW_SYSID_STAGE_DONE
} YawSysIdStage;

typedef enum
{
    YAW_J_MOTION_PREPARE = 0,
    YAW_J_MOTION_ACCEL,
    YAW_J_MOTION_DECEL,
    YAW_J_MOTION_ZERO_HOLD,
    YAW_J_MOTION_FINAL_HOLD
} YawJMotionPhase;

typedef enum
{
    PITCH_J_MOTION_PREPARE = 0,
    PITCH_J_MOTION_UP_ACCEL,
    PITCH_J_MOTION_UP_DECEL,
    PITCH_J_MOTION_RETURN_DOWN,
    PITCH_J_MOTION_FINAL_HOLD
} PitchJMotionPhase;

/* ========== 模块级全局状态 ========== */
static Gimbal_Status_t *ctrl = NULL;
GimbalSystemIDContext gimbal_sysid;

/* ---------- Yaw 轴测试状态 ---------- */
static StepSequencer    yaw_bc_seq;
static LeastSquares2    yaw_bc_ls;
static LeastSquares1    yaw_j_all;
static YawJPairGroup    yaw_j_pair[YAW_J_PAIR_WINDOW_COUNT]
                                  [YAW_J_DIRECTION_COUNT][2];
static YawJTraversal    yaw_j_traversal;
static YawSysIdStage    yaw_stage;
static YawJMotionPhase  yaw_j_motion_phase;
static bool             yaw_run_all;
static float            yaw_j_ref_magnitude;
static float            yaw_j_zero_elapsed;
static float            yaw_j_alpha_filtered;
static uint32_t         yaw_j_raw_total;
static uint8_t          yaw_j_direction_index;

/* ---------- Pitch 轴测试状态 ---------- */
static BoundaryScanner  pitch_scanner;
static PitchSysIdStage  pitch_stage;
static bool             pitch_run_all;
static uint8_t          pitch_last_half_cycle;
static float            pitch_reversal_elapsed;
static float            pitch_j_centered_elapsed;
static LeastSquares2    pitch_bc_ls;
static GravityAngleBin  pitch_gravity_bins[PITCH_GRAVITY_BIN_COUNT];
static LeastSquares1    pitch_j_all;
static PitchJPairGroup  pitch_j_pair[PITCH_J_PAIR_BIN_COUNT][2];
static PitchJTraversal  pitch_j_traversal;
static float            pitch_j_alpha_filtered;
static float            pitch_j_phase_elapsed;
static PitchJMotionPhase pitch_j_motion_phase;
static uint8_t          pitch_j_pass_count;
static PitchBCMean      pitch_bc_mean;
static uint32_t         pitch_stage_raw_total;

/* 与机构相关的数组集中在 GimbalSystemIDConfig.c，此处只保留短别名。 */
#define PITCH_J_SWITCH_DEG          (gimbal_sysid.pitch_angle_layout.j_switch_deg)
#define PITCH_J_PAIR_CENTER_DEG     (gimbal_sysid.pitch_angle_layout.j_pair_center_deg)
#define PITCH_BC_UP_REF_DPS         (gimbal_sysid_user_config.pitch.bc_up_ref_dps)
#define PITCH_BC_DOWN_REF_DPS       (gimbal_sysid_user_config.pitch.bc_down_ref_dps)
#define PITCH_BC_EXPECTED_DPS       (gimbal_sysid_user_config.pitch.bc_expected_actual_dps)
#define YAW_BC_SPEED_DPS            (gimbal_sysid_user_config.yaw.bc_speed_dps)
#define YAW_J_PAIR_CENTER_DPS       (gimbal_sysid_user_config.yaw.j_pair_center_dps)

/* ==================================================================
 *  1. 采样累加器
 * ================================================================== */
void SAcc_Reset(SampleAccumulator *acc)
{
    acc->sum   = 0.0f;
    acc->count = 0;
}

void SAcc_Add(SampleAccumulator *acc, float val)
{
    acc->sum   += val;
    acc->count ++;
}

float SAcc_Mean(const SampleAccumulator *acc)
{
    if (acc->count == 0)
        return 0.0f;
    return acc->sum / (float)acc->count;
}

/* ==================================================================
 *  2. 多速度点步进序列
 * ================================================================== */
void StepSeq_Init(StepSequencer *seq, const float *pts, uint8_t n,
                  float settle, float revs, float deg_per_rev)
{
    seq->vel_pts         = pts;
    seq->num_pts         = n;
    seq->settle_time     = settle;
    seq->hold_revolutions = revs;
    seq->deg_per_rev     = deg_per_rev;

    seq->current_idx   = 0;
    seq->step_elapsed  = 0.0f;
    seq->step_angle    = 0.0f;
    seq->sample_count  = 0;
    seq->torque_sum    = 0.0f;
    seq->omega_sum     = 0.0f;
}

bool StepSeq_Run(StepSequencer *seq, float dt,
                 float omega, float torque,
                 bool *done, float *out_omega, float *out_torque)
{
    *done = false;

    if (seq->current_idx >= seq->num_pts)
    {
        *done = true;
        return false;
    }

    seq->step_elapsed += dt;

    /* 稳定期过后开始采样 */
    if (seq->step_elapsed > seq->settle_time)
    {
        seq->step_angle  += fabsf(omega) * dt;
        seq->torque_sum  += torque;
        seq->omega_sum   += omega;
        seq->sample_count++;
    }

    /* 转够圈数 → 输出平均数据，推进到下一个速度点 */
    if (seq->step_angle >= seq->deg_per_rev * seq->hold_revolutions)
    {
        if (seq->sample_count > 0)
        {
            *out_omega  = seq->omega_sum  / (float)seq->sample_count;
            *out_torque = seq->torque_sum / (float)seq->sample_count;
        }
        else
        {
            *out_omega  = 0.0f;
            *out_torque = 0.0f;
        }

        /* 重置当前步状态 */
        seq->step_elapsed = 0.0f;
        seq->step_angle   = 0.0f;
        seq->sample_count = 0;
        seq->torque_sum   = 0.0f;
        seq->omega_sum    = 0.0f;
        seq->current_idx++;

        if (seq->current_idx >= seq->num_pts)
            *done = true;

        return true;    /* 调用方可喂 RLS */
    }

    return false;
}

/* ==================================================================
 *  3. 角度边界往返扫描器
 * ================================================================== */
void BScan_Init_ConstVel(BoundaryScanner *bs,
                         float angle_min, float angle_max, float margin,
                         float vel_forward, float vel_backward,
                         uint8_t max_half_cycles)
{
    bs->angle_min       = angle_min;
    bs->angle_max       = angle_max;
    bs->margin          = margin;
    bs->max_half_cycles = max_half_cycles;

    bs->mode          = SCAN_CONST_VEL;
    bs->vel_forward   = vel_forward;
    bs->vel_backward  = vel_backward;
    bs->accel         = 0.0f;
    bs->max_speed     = 0.0f;

    bs->half_cycle_count = 0;
    bs->current_vel_ref  = vel_forward;
    bs->ramp_speed       = 0.0f;
}

void BScan_Init_AccelRamp(BoundaryScanner *bs,
                          float angle_min, float angle_max, float margin,
                          float accel, float max_speed,
                          uint8_t max_half_cycles)
{
    bs->angle_min       = angle_min;
    bs->angle_max       = angle_max;
    bs->margin          = margin;
    bs->max_half_cycles = max_half_cycles;

    bs->mode          = SCAN_ACCEL_RAMP;
    bs->vel_forward   = max_speed;
    bs->vel_backward  = max_speed;
    bs->accel         = accel;
    bs->max_speed     = max_speed;

    bs->half_cycle_count = 0;
    bs->current_vel_ref  = 0.0f;
    bs->ramp_speed       = 0.0f;
}

bool BScan_Run(BoundaryScanner *bs, float theta_deg, float dt,
               float *out_vel_ref)
{
    if (bs->mode == SCAN_CONST_VEL)
    {
        /* 到达上边界且正向 → 换向 */
        if (theta_deg > (bs->angle_max - bs->margin) && bs->current_vel_ref > 0)
        {
            bs->current_vel_ref = -bs->vel_backward;
            bs->half_cycle_count++;
        }
        /* 到达下边界且反向 → 换向 */
        else if (theta_deg < (bs->angle_min + bs->margin) && bs->current_vel_ref < 0)
        {
            bs->current_vel_ref = bs->vel_forward;
            bs->half_cycle_count++;
        }
    }
    else /* SCAN_ACCEL_RAMP */
    {
        /* 到达上边界 → 标记结束 (与原始逻辑一致) */
        if (theta_deg > (bs->angle_max - bs->margin) && bs->ramp_speed > 0)
        {
            bs->half_cycle_count = bs->max_half_cycles;
        }

        /* 斜坡加速 */
        bs->ramp_speed += bs->accel * dt;
        bs->ramp_speed  = LIMIT_MAX_MIN(bs->ramp_speed, bs->max_speed, -bs->max_speed);
        bs->current_vel_ref = bs->ramp_speed;
    }

    *out_vel_ref = bs->current_vel_ref;
    return (bs->half_cycle_count >= bs->max_half_cycles);
}

bool BScan_IsDone(const BoundaryScanner *bs)
{
    return (bs->half_cycle_count >= bs->max_half_cycles);
}

/* ==================================================================
 *  6. Pitch 轴 — 分阶段辨识 G(theta) -> B/C -> J
 * ================================================================== */
static void LS2_Reset(LeastSquares2 *ls)
{
    ls->s00 = 0.0f;
    ls->s01 = 0.0f;
    ls->s11 = 0.0f;
    ls->b0 = 0.0f;
    ls->b1 = 0.0f;
    ls->yy = 0.0f;
    ls->count = 0U;
}

static void LS2_Add(LeastSquares2 *ls, float phi0, float phi1, float y)
{
    ls->s00 += phi0 * phi0;
    ls->s01 += phi0 * phi1;
    ls->s11 += phi1 * phi1;
    ls->b0 += phi0 * y;
    ls->b1 += phi1 * y;
    ls->yy += y * y;
    ls->count++;
}

static bool LS2_Solve(const LeastSquares2 *ls, float *x0, float *x1, float *rmse)
{
    float det;
    float scale;
    float sse;

    if (ls->count < 3U)
        return false;

    det = ls->s00 * ls->s11 - ls->s01 * ls->s01;
    scale = ls->s00 * ls->s11;
    if (scale <= 0.0f || det <= 1.0e-5f * scale)
        return false;

    *x0 = (ls->b0 * ls->s11 - ls->b1 * ls->s01) / det;
    *x1 = (ls->b1 * ls->s00 - ls->b0 * ls->s01) / det;

    sse = ls->yy - 2.0f * ((*x0) * ls->b0 + (*x1) * ls->b1)
        + (*x0) * (*x0) * ls->s00
        + 2.0f * (*x0) * (*x1) * ls->s01
        + (*x1) * (*x1) * ls->s11;
    if (sse < 0.0f)
        sse = 0.0f;
    *rmse = sqrtf(sse / (float)ls->count);
    return true;
}

static void Pitch_SetStage(PitchSysIdStage stage)
{
    pitch_stage = stage;
    gimbal_sysid.pitch.sysid_stage = (uint8_t)stage;
}

static void Pitch_Finish(bool valid, uint8_t error, float rmse, uint32_t sample_count)
{
    ctrl->sys_pitch_speed_ref = 0.0f;
    gimbal_sysid.pitch.fit_rmse = rmse;
    gimbal_sysid.pitch.sample_count = sample_count;
    gimbal_sysid.pitch.sysid_valid = valid ? 1U : 0U;
    gimbal_sysid.pitch.sysid_error = error;
    gimbal_sysid.pitch.sysid_done = 1U;
    Pitch_SetStage(PITCH_SYSID_STAGE_DONE);
}

static void Pitch_ResetGravityBins(void)
{
    uint32_t i;
    for (i = 0U; i < PITCH_GRAVITY_BIN_COUNT; i++)
    {
        pitch_gravity_bins[i].torque_pos_sum = 0.0f;
        pitch_gravity_bins[i].torque_neg_sum = 0.0f;
        pitch_gravity_bins[i].omega_pos_sum = 0.0f;
        pitch_gravity_bins[i].omega_neg_sum = 0.0f;
        pitch_gravity_bins[i].pos_count = 0U;
        pitch_gravity_bins[i].neg_count = 0U;
    }
}

/*
 * 所有Pitch测试角度只由安全上下限生成：
 *   1. 扫描换向点相对硬边界内缩 PITCH_REVERSE_MARGIN_DEG；
 *   2. 有效采样/J运动区间内缩 PITCH_SAMPLE_MARGIN_DEG；
 *   3. 三个J切换点将运动区间四等分；
 *   4. 两个J配对点位于相邻切换点中间。
 */
static void Pitch_BuildAngleLayout(void)
{
    GimbalSysIdPitchAngleLayout *layout = &gimbal_sysid.pitch_angle_layout;
    float span;
    uint32_t i;

    layout->safe_min_deg = PITCH_SAFE_MIN_DEG;
    layout->safe_max_deg = PITCH_SAFE_MAX_DEG;
    layout->reverse_min_deg = PITCH_SAFE_MIN_DEG + PITCH_REVERSE_MARGIN_DEG;
    layout->reverse_max_deg = PITCH_SAFE_MAX_DEG - PITCH_REVERSE_MARGIN_DEG;
    layout->sample_min_deg = PITCH_SAFE_MIN_DEG + PITCH_SAMPLE_MARGIN_DEG;
    layout->sample_max_deg = PITCH_SAFE_MAX_DEG - PITCH_SAMPLE_MARGIN_DEG;
    layout->j_start_deg = layout->sample_min_deg;
    layout->j_end_deg = layout->sample_max_deg;

    span = layout->j_end_deg - layout->j_start_deg;
    for (i = 0U; i < PITCH_J_MEASURED_PASSES; i++)
    {
        layout->j_switch_deg[i] = layout->j_start_deg
            + span * (float)(i + 1U) / (float)(PITCH_J_MEASURED_PASSES + 1U);
    }
    for (i = 0U; i < PITCH_J_PAIR_BIN_COUNT; i++)
    {
        layout->j_pair_center_deg[i] = 0.5f
            * (layout->j_switch_deg[i] + layout->j_switch_deg[i + 1U]);
    }
}

static void Pitch_BeginGravity(void)
{
    Pitch_ResetGravityBins();
    BScan_Init_ConstVel(&pitch_scanner,
                        PITCH_REVERSE_MIN_DEG, PITCH_REVERSE_MAX_DEG,
                        0.0f,
                        PITCH_GRAVITY_UP_REF_DPS, PITCH_GRAVITY_DOWN_REF_DPS,
                        PITCH_GRAVITY_HALF_CYCLES);
    pitch_last_half_cycle = 0U;
    pitch_reversal_elapsed = 0.0f;
    TD_Clear(&gimbal_sysid.pitch.td_omega, ctrl->d_pitch*PI_Ang);
    Pitch_SetStage(PITCH_SYSID_STAGE_GRAVITY);
}

static void Pitch_BeginBC(void)
{
    LS2_Reset(&pitch_bc_ls);
    pitch_bc_mean.torque_sum = 0.0f;
    pitch_bc_mean.omega_sum = 0.0f;
    pitch_bc_mean.gravity_sum = 0.0f;
    pitch_bc_mean.count = 0U;
    pitch_stage_raw_total = 0U;
    gimbal_sysid.pitch.mean_torque = 0.0f;
    gimbal_sysid.pitch.mean_omega = 0.0f;
    gimbal_sysid.pitch.mean_gravity = 0.0f;
    gimbal_sysid.pitch.mean_input = 0.0f;
    gimbal_sysid.pitch.mean_residual = 0.0f;
    gimbal_sysid.pitch.mean_raw_count = 0U;
    gimbal_sysid.pitch.mean_point_count = 0U;
    gimbal_sysid.pitch.bc_sample_count = 0U;
    BScan_Init_ConstVel(&pitch_scanner,
                        PITCH_REVERSE_MIN_DEG, PITCH_REVERSE_MAX_DEG,
                        0.0f,
                        PITCH_BC_UP_REF_DPS[0], PITCH_BC_DOWN_REF_DPS[0],
                        PITCH_BC_HALF_CYCLES);
    pitch_last_half_cycle = 0U;
    pitch_reversal_elapsed = 0.0f;
    TD_Clear(&gimbal_sysid.pitch.td_omega, ctrl->d_pitch*PI_Ang);
    Pitch_SetStage(PITCH_SYSID_STAGE_BC);
}

static void LS1_Reset(LeastSquares1 *ls)
{
    ls->aa = 0.0f;
    ls->ay = 0.0f;
    ls->yy = 0.0f;
    ls->count = 0U;
}

static void LS1_Add(LeastSquares1 *ls, float alpha, float residual)
{
    ls->aa += alpha * alpha;
    ls->ay += alpha * residual;
    ls->yy += residual * residual;
    ls->count++;
}

static bool LS1_Solve(const LeastSquares1 *ls, float *J, float *rmse)
{
    float sse;

    /* J 只有一个未知量；两个独立角度配对点即可拟合并检查一致性。 */
    if (ls->count < 2U || ls->aa < 1.0e-3f)
        return false;

    *J = ls->ay / ls->aa;
    sse = ls->yy - 2.0f * (*J) * ls->ay + (*J) * (*J) * ls->aa;
    if (sse < 0.0f)
        sse = 0.0f;
    *rmse = sqrtf(sse / (float)ls->count);
    return true;
}

static void Pitch_ResetBCMean(void)
{
    pitch_bc_mean.torque_sum = 0.0f;
    pitch_bc_mean.omega_sum = 0.0f;
    pitch_bc_mean.gravity_sum = 0.0f;
    pitch_bc_mean.count = 0U;
}

static void Pitch_AddBCMeanSample(float torque, float omega, float gravity)
{
    pitch_bc_mean.torque_sum += torque;
    pitch_bc_mean.omega_sum += omega;
    pitch_bc_mean.gravity_sum += gravity;
    pitch_bc_mean.count++;
}

static void SysId_PublishMeanPoint(Gimbal_SI *sysid, float torque,
                                   float omega, float gravity, float input,
                                   float residual, uint32_t raw_count,
                                   uint32_t point_count)
{
    sysid->mean_torque = torque;
    sysid->mean_omega = omega;
    sysid->mean_gravity = gravity;
    sysid->mean_input = input;
    sysid->mean_residual = residual;
    sysid->mean_raw_count = raw_count;
    sysid->mean_point_count = (uint8_t)point_count;
}

/* ==================================================================
 *  Yaw 轴 — 分阶段辨识 B/C -> J
 * ================================================================== */
static void Yaw_SetStage(YawSysIdStage stage)
{
    yaw_stage = stage;
    gimbal_sysid.yaw.sysid_stage = (uint8_t)stage;
}

static void Yaw_Finish(bool valid, uint8_t error, float rmse,
                       uint32_t sample_count)
{
    ctrl->sys_yaw_speed_ref = 0.0f;
    gimbal_sysid.yaw.j_velocity_ref = 0.0f;
    gimbal_sysid.yaw.fit_rmse = rmse;
    gimbal_sysid.yaw.sample_count = sample_count;
    gimbal_sysid.yaw.sysid_valid = valid ? 1U : 0U;
    gimbal_sysid.yaw.sysid_error = error;
    gimbal_sysid.yaw.sysid_done = 1U;
    Yaw_SetStage(YAW_SYSID_STAGE_DONE);
}

static void YawJ_ResetTraversal(void)
{
    yaw_j_traversal.torque_sum = 0.0f;
    yaw_j_traversal.omega_sum = 0.0f;
    yaw_j_traversal.omega_start = 0.0f;
    yaw_j_traversal.omega_last = 0.0f;
    yaw_j_traversal.elapsed = 0.0f;
    yaw_j_traversal.count = 0U;
    yaw_j_traversal.window_index = -1;
    yaw_j_traversal.direction_index = -1;
    yaw_j_traversal.phase_index = -1;
}

static void YawJ_ResetPairData(void)
{
    uint32_t i;
    uint32_t j;
    uint32_t k;

    for (i = 0U; i < YAW_J_PAIR_WINDOW_COUNT; i++)
    {
        for (j = 0U; j < YAW_J_DIRECTION_COUNT; j++)
        {
            for (k = 0U; k < 2U; k++)
            {
                yaw_j_pair[i][j][k].torque_mean_sum = 0.0f;
                yaw_j_pair[i][j][k].omega_mean_sum = 0.0f;
                yaw_j_pair[i][j][k].alpha_mean_sum = 0.0f;
                yaw_j_pair[i][j][k].raw_count = 0U;
                yaw_j_pair[i][j][k].traversal_count = 0U;
            }
        }
    }
    YawJ_ResetTraversal();
}

static void Yaw_BeginJ(void)
{
    LS1_Reset(&yaw_j_all);
    YawJ_ResetPairData();
    yaw_j_motion_phase = YAW_J_MOTION_PREPARE;
    yaw_j_ref_magnitude = 0.0f;
    yaw_j_zero_elapsed = 0.0f;
    yaw_j_alpha_filtered = 0.0f;
    yaw_j_raw_total = 0U;
    yaw_j_direction_index = 0U;

    gimbal_sysid.yaw.J_pair_min = 0.0f;
    gimbal_sysid.yaw.J_pair_max = 0.0f;
    gimbal_sysid.yaw.j_alpha_filtered = 0.0f;
    gimbal_sysid.yaw.j_signal_rms = 0.0f;
    gimbal_sysid.yaw.j_residual_ratio = 0.0f;
    gimbal_sysid.yaw.j_velocity_ref = 0.0f;
    gimbal_sysid.yaw.j_switch_angle = YAW_J_MAX_REF_DPS;
    gimbal_sysid.yaw.j_target_accel = 0.0f;
    gimbal_sysid.yaw.j_sample_count = 0U;
    gimbal_sysid.yaw.mean_torque = 0.0f;
    gimbal_sysid.yaw.mean_omega = 0.0f;
    gimbal_sysid.yaw.mean_gravity = 0.0f;
    gimbal_sysid.yaw.mean_input = 0.0f;
    gimbal_sysid.yaw.mean_residual = 0.0f;
    gimbal_sysid.yaw.mean_raw_count = 0U;
    gimbal_sysid.yaw.mean_point_count = 0U;
    gimbal_sysid.yaw.j_motion_phase = (uint8_t)YAW_J_MOTION_PREPARE;
    gimbal_sysid.yaw.j_pass_count = 0U;
    ctrl->sys_yaw_speed_ref = 0.0f;
    TD_Clear(&gimbal_sysid.yaw.td_omega, ctrl->d_yaw*PI_Ang);
    Yaw_SetStage(YAW_SYSID_STAGE_J_PREPARE);
}

static void Yaw_BeginBC(void)
{
    LS2_Reset(&yaw_bc_ls);
    StepSeq_Init(&yaw_bc_seq, YAW_BC_SPEED_DPS, YAW_BC_POINT_COUNT,
                 YAW_BC_SETTLE_S, YAW_BC_SAMPLE_REVOLUTIONS, 360.0f);
    gimbal_sysid.yaw.B_raw = gimbal_sysid.yaw.B;
    gimbal_sysid.yaw.bc_rmse = 0.0f;
    gimbal_sysid.yaw.bc_sample_count = 0U;
    gimbal_sysid.yaw.mean_raw_count = 0U;
    gimbal_sysid.yaw.mean_point_count = 0U;
    ctrl->sys_yaw_speed_ref = YAW_BC_SPEED_DPS[0];
    TD_Clear(&gimbal_sysid.yaw.td_omega, ctrl->d_yaw*PI_Ang);
    Yaw_SetStage(YAW_SYSID_STAGE_BC);
}

static void Yaw_BCRun(float omega, float torque, float dt)
{
    float target;
    float out_omega;
    float out_torque;
    float sign_omega;
    float B;
    float C;
    float rmse = 0.0f;
    bool point_ready;
    bool done;

    target = (yaw_bc_seq.current_idx < YAW_BC_POINT_COUNT)
           ? yaw_bc_seq.vel_pts[yaw_bc_seq.current_idx] : 0.0f;
    ctrl->sys_yaw_speed_ref = target;
    point_ready = StepSeq_Run(&yaw_bc_seq, dt, omega, torque,
                              &done, &out_omega, &out_torque);

    if (point_ready && out_omega * target > 0.0f &&
        fabsf(out_omega - target)
            <= fabsf(target) * YAW_BC_SPEED_TRACK_RATIO)
    {
        sign_omega = (out_omega > 0.0f) ? 1.0f : -1.0f;
        LS2_Add(&yaw_bc_ls, out_omega, sign_omega, out_torque);
        SysId_PublishMeanPoint(&gimbal_sysid.yaw, out_torque, out_omega,
                               0.0f, sign_omega, out_torque, 0U,
                               yaw_bc_ls.count);
        gimbal_sysid.yaw.bc_sample_count = yaw_bc_ls.count;
        TD_Clear(&gimbal_sysid.yaw.td_omega, ctrl->d_yaw*PI_Ang);
    }

    if (!done)
        return;

    if (yaw_bc_ls.count != YAW_BC_POINT_COUNT ||
        !LS2_Solve(&yaw_bc_ls, &B, &C, &rmse))
    {
        gimbal_sysid.yaw.bc_rmse = rmse;
        Yaw_Finish(false, GIMBAL_SYSID_ERROR_INSUFFICIENT_EXCITATION,
                   rmse, yaw_bc_ls.count);
        return;
    }

    gimbal_sysid.yaw.B_raw = B;
    gimbal_sysid.yaw.C = C;
    gimbal_sysid.yaw.bc_rmse = rmse;
    gimbal_sysid.yaw.fit_rmse = rmse;
    gimbal_sysid.yaw.bc_sample_count = yaw_bc_ls.count;
    if (B < -YAW_BC_NEGATIVE_B_TOLERANCE || C < 0.0f)
    {
        gimbal_sysid.yaw.B = B;
        Yaw_Finish(false, GIMBAL_SYSID_ERROR_NON_PHYSICAL_RESULT,
                   rmse, yaw_bc_ls.count);
        return;
    }
    if (B < 0.0f)
        B = 0.0f;
    gimbal_sysid.yaw.B = B;

    if (yaw_run_all)
        Yaw_BeginJ();
    else
        Yaw_Finish(true, GIMBAL_SYSID_ERROR_NONE, rmse, yaw_bc_ls.count);
}

static int8_t YawJ_FindPairWindow(float omega)
{
    uint32_t i;
    float speed = fabsf(omega);

    for (i = 0U; i < YAW_J_PAIR_WINDOW_COUNT; i++)
    {
        if (fabsf(speed - YAW_J_PAIR_CENTER_DPS[i])
            <= YAW_J_PAIR_HALF_WIDTH_DPS)
            return (int8_t)i;
    }
    return -1;
}

static float YawJ_DirectionSign(uint8_t direction_index)
{
    return (direction_index == 0U) ? 1.0f : -1.0f;
}

static void YawJ_FinishTraversal(void)
{
    YawJPairGroup *group;
    float alpha_avg;
    float normalized_alpha;
    float direction;
    float inv_count;
    bool direction_valid;

    if (yaw_j_traversal.window_index < 0 ||
        yaw_j_traversal.direction_index < 0 ||
        yaw_j_traversal.phase_index < 0)
    {
        YawJ_ResetTraversal();
        return;
    }
    if (yaw_j_traversal.count < YAW_J_MIN_TRAVERSAL_SAMPLES ||
        yaw_j_traversal.elapsed < YAW_J_MIN_TRAVERSAL_TIME_S)
    {
        YawJ_ResetTraversal();
        return;
    }

    alpha_avg = (yaw_j_traversal.omega_last
               - yaw_j_traversal.omega_start)
              / yaw_j_traversal.elapsed;
    direction = YawJ_DirectionSign(
                    (uint8_t)yaw_j_traversal.direction_index);
    normalized_alpha = direction * alpha_avg;
    direction_valid = (yaw_j_traversal.phase_index == 0)
                    ? (normalized_alpha > YAW_J_MIN_MEAN_ALPHA_DPS2)
                    : (normalized_alpha < -YAW_J_MIN_MEAN_ALPHA_DPS2);
    if (!direction_valid)
    {
        YawJ_ResetTraversal();
        return;
    }

    group = &yaw_j_pair[(uint32_t)yaw_j_traversal.window_index]
                       [(uint32_t)yaw_j_traversal.direction_index]
                       [(uint32_t)yaw_j_traversal.phase_index];
    inv_count = 1.0f / (float)yaw_j_traversal.count;
    group->torque_mean_sum += yaw_j_traversal.torque_sum * inv_count;
    group->omega_mean_sum += yaw_j_traversal.omega_sum * inv_count;
    group->alpha_mean_sum += alpha_avg;
    group->raw_count += yaw_j_traversal.count;
    group->traversal_count++;
    yaw_j_raw_total += yaw_j_traversal.count;
    gimbal_sysid.yaw.j_sample_count = yaw_j_raw_total;
    YawJ_ResetTraversal();
}

static void YawJ_CollectPairSample(float omega, float torque, float dt,
                                   bool enabled)
{
    int8_t window_index = -1;
    int8_t direction_index = -1;
    int8_t phase_index = -1;
    float direction;

    if (enabled)
    {
        direction = YawJ_DirectionSign(yaw_j_direction_index);
        if (omega * direction > 0.0f)
        {
            window_index = YawJ_FindPairWindow(omega);
            direction_index = (int8_t)yaw_j_direction_index;
            phase_index = (yaw_j_motion_phase == YAW_J_MOTION_ACCEL) ? 0 : 1;
        }
    }

    if (yaw_j_traversal.window_index != window_index ||
        yaw_j_traversal.direction_index != direction_index ||
        yaw_j_traversal.phase_index != phase_index)
        YawJ_FinishTraversal();

    if (window_index < 0 || direction_index < 0 || phase_index < 0)
        return;

    if (yaw_j_traversal.count == 0U)
    {
        yaw_j_traversal.window_index = window_index;
        yaw_j_traversal.direction_index = direction_index;
        yaw_j_traversal.phase_index = phase_index;
        yaw_j_traversal.omega_start = omega;
        yaw_j_traversal.omega_last = omega;
    }
    else
    {
        yaw_j_traversal.elapsed += dt;
        yaw_j_traversal.omega_last = omega;
    }
    yaw_j_traversal.torque_sum += torque;
    yaw_j_traversal.omega_sum += omega;
    yaw_j_traversal.count++;
}

static void YawJ_Finalize(void)
{
    uint32_t i;
    uint32_t j;
    float J;
    float rmse;
    float alpha_rms;
    float pair_ratio;
    float torque_accel;
    float torque_decel;
    float omega_accel;
    float omega_decel;
    float alpha_accel;
    float alpha_decel;
    float delta_torque;
    float delta_omega;
    float delta_alpha;
    float pair_residual;
    float pair_J;
    uint32_t raw_count;
    YawJPairGroup *accel_group;
    YawJPairGroup *decel_group;

    YawJ_FinishTraversal();
    LS1_Reset(&yaw_j_all);
    gimbal_sysid.yaw.J_pair_min = 1.0e6f;
    gimbal_sysid.yaw.J_pair_max = -1.0e6f;

    for (i = 0U; i < YAW_J_PAIR_WINDOW_COUNT; i++)
    {
        for (j = 0U; j < YAW_J_DIRECTION_COUNT; j++)
        {
            accel_group = &yaw_j_pair[i][j][0];
            decel_group = &yaw_j_pair[i][j][1];
            if (accel_group->traversal_count == 0U ||
                decel_group->traversal_count == 0U)
                continue;

            torque_accel = accel_group->torque_mean_sum
                         / (float)accel_group->traversal_count;
            torque_decel = decel_group->torque_mean_sum
                         / (float)decel_group->traversal_count;
            omega_accel = accel_group->omega_mean_sum
                        / (float)accel_group->traversal_count;
            omega_decel = decel_group->omega_mean_sum
                        / (float)decel_group->traversal_count;
            alpha_accel = accel_group->alpha_mean_sum
                        / (float)accel_group->traversal_count;
            alpha_decel = decel_group->alpha_mean_sum
                        / (float)decel_group->traversal_count;
            delta_torque = torque_accel - torque_decel;
            delta_omega = omega_accel - omega_decel;
            delta_alpha = alpha_accel - alpha_decel;
            if (fabsf(delta_alpha) < 2.0f * YAW_J_MIN_MEAN_ALPHA_DPS2)
                continue;

            pair_residual = delta_torque
                          - gimbal_sysid.yaw.B * delta_omega;
            pair_J = pair_residual / delta_alpha;
            raw_count = accel_group->raw_count + decel_group->raw_count;
            LS1_Add(&yaw_j_all, delta_alpha, pair_residual);
            if (pair_J < gimbal_sysid.yaw.J_pair_min)
                gimbal_sysid.yaw.J_pair_min = pair_J;
            if (pair_J > gimbal_sysid.yaw.J_pair_max)
                gimbal_sysid.yaw.J_pair_max = pair_J;
            SysId_PublishMeanPoint(&gimbal_sysid.yaw, delta_torque,
                                   delta_omega, 0.0f, delta_alpha,
                                   pair_residual, raw_count,
                                   yaw_j_all.count);
        }
    }

    if (yaw_j_all.count != YAW_J_PAIR_WINDOW_COUNT * YAW_J_DIRECTION_COUNT ||
        !LS1_Solve(&yaw_j_all, &J, &rmse))
    {
        if (yaw_j_all.count == 0U)
        {
            gimbal_sysid.yaw.J_pair_min = 0.0f;
            gimbal_sysid.yaw.J_pair_max = 0.0f;
        }
        Yaw_Finish(false, GIMBAL_SYSID_ERROR_INSUFFICIENT_EXCITATION,
                   0.0f, yaw_j_raw_total);
        return;
    }

    gimbal_sysid.yaw.J = J;
    gimbal_sysid.yaw.j_rmse = rmse;
    alpha_rms = sqrtf(yaw_j_all.aa / (float)yaw_j_all.count);
    gimbal_sysid.yaw.j_signal_rms = fabsf(J) * alpha_rms;
    if (gimbal_sysid.yaw.j_signal_rms > 1.0e-3f)
        gimbal_sysid.yaw.j_residual_ratio =
            rmse / gimbal_sysid.yaw.j_signal_rms;
    else
        gimbal_sysid.yaw.j_residual_ratio = 1.0e6f;

    if (J <= 0.0f || gimbal_sysid.yaw.J_pair_min <= 0.0f)
    {
        Yaw_Finish(false, GIMBAL_SYSID_ERROR_NON_PHYSICAL_RESULT,
                   rmse, yaw_j_raw_total);
        return;
    }
    pair_ratio = gimbal_sysid.yaw.J_pair_max
               / gimbal_sysid.yaw.J_pair_min;
    if (pair_ratio > YAW_J_MAX_PAIR_RATIO)
    {
        Yaw_Finish(false, GIMBAL_SYSID_ERROR_PAIR_MISMATCH,
                   rmse, yaw_j_raw_total);
        return;
    }
    if (gimbal_sysid.yaw.j_residual_ratio > YAW_J_MAX_RMSE_SIGNAL_RATIO)
    {
        Yaw_Finish(false, GIMBAL_SYSID_ERROR_POOR_FIT,
                   rmse, yaw_j_raw_total);
        return;
    }
    Yaw_Finish(true, GIMBAL_SYSID_ERROR_NONE, rmse, yaw_j_raw_total);
}

static void Yaw_JRun(float omega, float torque, float dt)
{
    float direction;
    float vel_ref;
    float target_accel = 0.0f;
    bool collect_enabled;

    direction = YawJ_DirectionSign(yaw_j_direction_index);

    if (yaw_j_motion_phase == YAW_J_MOTION_PREPARE)
    {
        vel_ref = 0.0f;
        if (fabsf(omega) < YAW_J_ZERO_SPEED_DPS)
            yaw_j_zero_elapsed += dt;
        else
            yaw_j_zero_elapsed = 0.0f;
        if (yaw_j_zero_elapsed >= YAW_J_ZERO_SETTLE_S)
        {
            yaw_j_zero_elapsed = 0.0f;
            yaw_j_motion_phase = YAW_J_MOTION_ACCEL;
            Yaw_SetStage(YAW_SYSID_STAGE_J_EXCITE);
        }
    }
    else if (yaw_j_motion_phase == YAW_J_MOTION_ACCEL)
    {
        yaw_j_ref_magnitude += YAW_J_REF_ACCEL_DPS2 * dt;
        if (yaw_j_ref_magnitude >= YAW_J_MAX_REF_DPS)
        {
            yaw_j_ref_magnitude = YAW_J_MAX_REF_DPS;
            yaw_j_motion_phase = YAW_J_MOTION_DECEL;
        }
        vel_ref = direction * yaw_j_ref_magnitude;
        target_accel = direction * YAW_J_REF_ACCEL_DPS2;
    }
    else if (yaw_j_motion_phase == YAW_J_MOTION_DECEL)
    {
        yaw_j_ref_magnitude -= YAW_J_REF_ACCEL_DPS2 * dt;
        if (yaw_j_ref_magnitude <= 0.0f)
        {
            yaw_j_ref_magnitude = 0.0f;
            yaw_j_motion_phase = YAW_J_MOTION_ZERO_HOLD;
            yaw_j_zero_elapsed = 0.0f;
        }
        vel_ref = direction * yaw_j_ref_magnitude;
        target_accel = -direction * YAW_J_REF_ACCEL_DPS2;
    }
    else if (yaw_j_motion_phase == YAW_J_MOTION_ZERO_HOLD)
    {
        vel_ref = 0.0f;
        if (fabsf(omega) < YAW_J_ZERO_SPEED_DPS)
            yaw_j_zero_elapsed += dt;
        else
            yaw_j_zero_elapsed = 0.0f;
        if (yaw_j_zero_elapsed >= YAW_J_ZERO_SETTLE_S)
        {
            yaw_j_direction_index++;
            gimbal_sysid.yaw.j_pass_count = yaw_j_direction_index;
            yaw_j_zero_elapsed = 0.0f;
            if (yaw_j_direction_index >= YAW_J_DIRECTION_COUNT)
                yaw_j_motion_phase = YAW_J_MOTION_FINAL_HOLD;
            else
            {
                yaw_j_ref_magnitude = 0.0f;
                yaw_j_motion_phase = YAW_J_MOTION_ACCEL;
            }
        }
    }
    else
    {
        ctrl->sys_yaw_speed_ref = 0.0f;
        gimbal_sysid.yaw.j_velocity_ref = 0.0f;
        gimbal_sysid.yaw.j_target_accel = 0.0f;
        gimbal_sysid.yaw.j_motion_phase = (uint8_t)yaw_j_motion_phase;
        YawJ_CollectPairSample(omega, torque, dt, false);
        YawJ_Finalize();
        return;
    }

    ctrl->sys_yaw_speed_ref = vel_ref;
    gimbal_sysid.yaw.j_velocity_ref = vel_ref;
    gimbal_sysid.yaw.j_target_accel = target_accel;
    gimbal_sysid.yaw.j_motion_phase = (uint8_t)yaw_j_motion_phase;
    collect_enabled = (yaw_j_motion_phase == YAW_J_MOTION_ACCEL ||
                       yaw_j_motion_phase == YAW_J_MOTION_DECEL);
    YawJ_CollectPairSample(omega, torque, dt, collect_enabled);
}

static void Yaw_Run(float dt)
{
    const float two_pi = 6.28318530718f;
    float omega_raw;
    float omega_smooth;
    float alpha_raw;
    float alpha_lpf_k;
    float torque;

    omega_raw = ctrl->d_yaw*PI_Ang;
    omega_smooth = TD_Calculate(&gimbal_sysid.yaw.td_omega, omega_raw);
    alpha_raw = gimbal_sysid.yaw.td_omega.dx;
    torque = gimbal_sysid_user_config.yaw.torque_feedback_coef
           * Gimbal_Motor.DM_4310[0].tor;
    if (fabsf(alpha_raw) <= YAW_J_RAW_ALPHA_REJECT_DPS2)
    {
        alpha_lpf_k = two_pi * YAW_J_ALPHA_LPF_HZ * dt;
        alpha_lpf_k = alpha_lpf_k / (1.0f + alpha_lpf_k);
        yaw_j_alpha_filtered += alpha_lpf_k
                              * (alpha_raw - yaw_j_alpha_filtered);
    }
    gimbal_sysid.yaw.j_alpha_filtered = yaw_j_alpha_filtered;

    if (yaw_stage == YAW_SYSID_STAGE_BC)
        Yaw_BCRun(omega_smooth, torque, dt);
    else if (yaw_stage == YAW_SYSID_STAGE_J_PREPARE ||
             yaw_stage == YAW_SYSID_STAGE_J_EXCITE)
        Yaw_JRun(omega_smooth, torque, dt);
    else
        ctrl->sys_yaw_speed_ref = 0.0f;
}

static void PitchJ_ResetTraversal(void)
{
    pitch_j_traversal.torque_sum = 0.0f;
    pitch_j_traversal.omega_sum = 0.0f;
    pitch_j_traversal.theta_sum = 0.0f;
    pitch_j_traversal.omega_start = 0.0f;
    pitch_j_traversal.omega_last = 0.0f;
    pitch_j_traversal.elapsed = 0.0f;
    pitch_j_traversal.count = 0U;
    pitch_j_traversal.bin_index = -1;
    pitch_j_traversal.group_index = -1;
}

static void PitchJ_ResetPairData(void)
{
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < PITCH_J_PAIR_BIN_COUNT; i++)
    {
        for (j = 0U; j < 2U; j++)
        {
            pitch_j_pair[i][j].torque_mean_sum = 0.0f;
            pitch_j_pair[i][j].omega_mean_sum = 0.0f;
            pitch_j_pair[i][j].theta_mean_sum = 0.0f;
            pitch_j_pair[i][j].alpha_mean_sum = 0.0f;
            pitch_j_pair[i][j].raw_count = 0U;
            pitch_j_pair[i][j].traversal_count = 0U;
        }
    }
    PitchJ_ResetTraversal();
}

static void Pitch_BeginJ(void)
{
    LS1_Reset(&pitch_j_all);
    PitchJ_ResetPairData();
    pitch_j_centered_elapsed = 0.0f;
    pitch_j_alpha_filtered = 0.0f;
    pitch_j_phase_elapsed = 0.0f;
    pitch_j_motion_phase = PITCH_J_MOTION_PREPARE;
    pitch_j_pass_count = 0U;
    pitch_stage_raw_total = 0U;

    gimbal_sysid.pitch.J_pair_min = 0.0f;
    gimbal_sysid.pitch.J_pair_max = 0.0f;
    gimbal_sysid.pitch.j_alpha_filtered = 0.0f;
    gimbal_sysid.pitch.j_signal_rms = 0.0f;
    gimbal_sysid.pitch.j_residual_ratio = 0.0f;
    gimbal_sysid.pitch.j_velocity_ref = 0.0f;
    gimbal_sysid.pitch.j_switch_angle = PITCH_J_SWITCH_DEG[0];
    gimbal_sysid.pitch.j_target_accel = 0.0f;
    gimbal_sysid.pitch.j_sample_count = 0U;
    gimbal_sysid.pitch.mean_torque = 0.0f;
    gimbal_sysid.pitch.mean_omega = 0.0f;
    gimbal_sysid.pitch.mean_gravity = 0.0f;
    gimbal_sysid.pitch.mean_input = 0.0f;
    gimbal_sysid.pitch.mean_residual = 0.0f;
    gimbal_sysid.pitch.mean_raw_count = 0U;
    gimbal_sysid.pitch.mean_point_count = 0U;
    gimbal_sysid.pitch.j_motion_phase = (uint8_t)PITCH_J_MOTION_PREPARE;
    gimbal_sysid.pitch.j_pass_count = 0U;
    TD_Clear(&gimbal_sysid.pitch.td_omega, ctrl->d_pitch*PI_Ang);
    Pitch_SetStage(PITCH_SYSID_STAGE_J_PREPARE);
}

static bool Pitch_RunConstScanner(float theta_deg,
                                  float speed_forward_dps,
                                  float speed_backward_dps,
                                  float dt, float *vel_ref)
{
    bool done;
    float direction;

    pitch_scanner.vel_forward = speed_forward_dps;
    pitch_scanner.vel_backward = speed_backward_dps;
    done = BScan_Run(&pitch_scanner, theta_deg, dt, vel_ref);

    if (pitch_scanner.half_cycle_count != pitch_last_half_cycle)
    {
        pitch_last_half_cycle = pitch_scanner.half_cycle_count;
        pitch_reversal_elapsed = 0.0f;
    }
    else
    {
        pitch_reversal_elapsed += dt;
    }

    if (done)
    {
        *vel_ref = 0.0f;
    }
    else
    {
        direction = (pitch_scanner.current_vel_ref >= 0.0f) ? 1.0f : -1.0f;
        pitch_scanner.current_vel_ref = (direction > 0.0f)
                                      ? speed_forward_dps
                                      : -speed_backward_dps;
        *vel_ref = pitch_scanner.current_vel_ref;
    }
    return done;
}

static bool Pitch_ConstSampleValid(float theta_deg, float omega, float alpha,
                                   float vel_ref, bool require_ref_tracking)
{
    float target_speed = fabsf(vel_ref);

    if (pitch_reversal_elapsed < PITCH_REVERSAL_SETTLE_S)
        return false;
    if (theta_deg < PITCH_SAMPLE_MIN_DEG ||
        theta_deg > PITCH_SAMPLE_MAX_DEG)
        return false;
    if (omega * vel_ref <= 0.0f)
        return false;
    if (require_ref_tracking &&
        fabsf(fabsf(omega) - target_speed) > target_speed * PITCH_SPEED_TRACK_RATIO)
        return false;
    if (fabsf(alpha) > PITCH_CONST_ALPHA_MAX_DPS2)
        return false;
    return true;
}

static void Pitch_AddGravitySample(float theta_deg, float omega, float torque)
{
    const float angle_lo = PITCH_SAMPLE_MIN_DEG;
    const float angle_hi = PITCH_SAMPLE_MAX_DEG;
    float position;
    uint32_t index;
    GravityAngleBin *bin;

    position = (theta_deg - angle_lo) / (angle_hi - angle_lo);
    if (position < 0.0f || position > 1.0f)
        return;

    index = (uint32_t)(position * (float)PITCH_GRAVITY_BIN_COUNT);
    if (index >= PITCH_GRAVITY_BIN_COUNT)
        index = PITCH_GRAVITY_BIN_COUNT - 1U;
    bin = &pitch_gravity_bins[index];

    if (omega > 0.0f)
    {
        bin->torque_pos_sum += torque;
        bin->omega_pos_sum += omega;
        bin->pos_count++;
    }
    else
    {
        bin->torque_neg_sum += torque;
        bin->omega_neg_sum += omega;
        bin->neg_count++;
    }
}

static bool Pitch_SolveGravity(float *rmse, uint32_t *valid_bins)
{
    const float angle_lo = PITCH_SAMPLE_MIN_DEG;
    const float angle_hi = PITCH_SAMPLE_MAX_DEG;
    LeastSquares2 gravity_ls;
    uint32_t i;
    float theta_deg;
    float theta_rad;
    float torque_pair;
    float omega_pos;
    float omega_neg;
    float g_sin;
    float g_cos;

    LS2_Reset(&gravity_ls);
    for (i = 0U; i < PITCH_GRAVITY_BIN_COUNT; i++)
    {
        GravityAngleBin *bin = &pitch_gravity_bins[i];
        if (bin->pos_count < PITCH_GRAVITY_MIN_BIN_SAMPLES ||
            bin->neg_count < PITCH_GRAVITY_MIN_BIN_SAMPLES)
            continue;

        omega_pos = bin->omega_pos_sum / (float)bin->pos_count;
        omega_neg = bin->omega_neg_sum / (float)bin->neg_count;
        if (fabsf(omega_pos + omega_neg) > PITCH_GRAVITY_PAIR_SPEED_TOL_DPS)
            continue;

        torque_pair = 0.5f * (bin->torque_pos_sum / (float)bin->pos_count
                            + bin->torque_neg_sum / (float)bin->neg_count);
        theta_deg = angle_lo + ((float)i + 0.5f)
                  * (angle_hi - angle_lo) / (float)PITCH_GRAVITY_BIN_COUNT;
        theta_rad = theta_deg * Ang_PI;
        LS2_Add(&gravity_ls, sinf(theta_rad), cosf(theta_rad), torque_pair);
    }

    *valid_bins = gravity_ls.count;
    if (gravity_ls.count < PITCH_GRAVITY_MIN_VALID_BINS ||
        !LS2_Solve(&gravity_ls, &g_sin, &g_cos, rmse))
        return false;

    gimbal_sysid.pitch.G_sin = g_sin;
    gimbal_sysid.pitch.G_cos = g_cos;
    return true;
}

static void Pitch_GravityRun(float theta_deg, float omega, float alpha,
                             float torque, float dt)
{
    float vel_ref;
    float rmse = 0.0f;
    uint32_t valid_bins = 0U;
    bool done = Pitch_RunConstScanner(theta_deg,
                                      PITCH_GRAVITY_UP_REF_DPS,
                                      PITCH_GRAVITY_DOWN_REF_DPS,
                                      dt, &vel_ref);

    ctrl->sys_pitch_speed_ref = vel_ref;
    if (!done && fabsf(omega) >= PITCH_GRAVITY_MIN_ACTUAL_SPEED_DPS &&
        Pitch_ConstSampleValid(theta_deg, omega, alpha, vel_ref, false))
        Pitch_AddGravitySample(theta_deg, omega, torque);

    if (!done)
        return;

    if (!Pitch_SolveGravity(&rmse, &valid_bins))
    {
        gimbal_sysid.pitch.gravity_rmse = rmse;
        gimbal_sysid.pitch.gravity_valid_bins = valid_bins;
        Pitch_Finish(false, GIMBAL_SYSID_ERROR_INSUFFICIENT_EXCITATION, rmse, valid_bins);
        return;
    }

    gimbal_sysid.pitch.fit_rmse = rmse;
    gimbal_sysid.pitch.sample_count = valid_bins;
    gimbal_sysid.pitch.gravity_rmse = rmse;
    gimbal_sysid.pitch.gravity_valid_bins = valid_bins;
    if (pitch_run_all)
        Pitch_BeginBC();
    else
        Pitch_Finish(true, GIMBAL_SYSID_ERROR_NONE, rmse, valid_bins);
}

static bool Pitch_BCFinalizeMean(void)
{
    float torque_avg;
    float omega_avg;
    float gravity_avg;
    float sign_omega;
    float residual;
    uint32_t raw_count = pitch_bc_mean.count;

    if (raw_count < PITCH_BC_MIN_SEGMENT_RAW_SAMPLES)
    {
        Pitch_ResetBCMean();
        return false;
    }

    torque_avg = pitch_bc_mean.torque_sum / (float)raw_count;
    omega_avg = pitch_bc_mean.omega_sum / (float)raw_count;
    gravity_avg = pitch_bc_mean.gravity_sum / (float)raw_count;
    if (fabsf(omega_avg) < PITCH_BC_MIN_SPEED_DPS)
    {
        Pitch_ResetBCMean();
        return false;
    }

    sign_omega = (omega_avg > 0.0f) ? 1.0f : -1.0f;
    residual = torque_avg - gravity_avg;
    LS2_Add(&pitch_bc_ls, omega_avg, sign_omega, residual);
    pitch_stage_raw_total += raw_count;
    SysId_PublishMeanPoint(&gimbal_sysid.pitch, torque_avg, omega_avg,
                           gravity_avg, sign_omega, residual, raw_count,
                           pitch_bc_ls.count);
    Pitch_ResetBCMean();
    return true;
}

static void Pitch_BCRun(float theta_deg, float omega, float alpha,
                        float torque, float dt)
{
    uint8_t half_cycle_before = pitch_scanner.half_cycle_count;
    uint32_t speed_index = half_cycle_before / 2U;
    float up_ref_dps;
    float down_ref_dps;
    float expected_actual_dps;
    float vel_ref;
    float theta_rad;
    float gravity;
    float B;
    float C;
    float rmse = 0.0f;
    bool done;

    if (speed_index >= PITCH_BC_SPEED_LEVELS)
        speed_index = PITCH_BC_SPEED_LEVELS - 1U;
    up_ref_dps = PITCH_BC_UP_REF_DPS[speed_index];
    down_ref_dps = PITCH_BC_DOWN_REF_DPS[speed_index];
    expected_actual_dps = PITCH_BC_EXPECTED_DPS[speed_index];
    done = Pitch_RunConstScanner(theta_deg,
                                 up_ref_dps, down_ref_dps,
                                 dt, &vel_ref);
    ctrl->sys_pitch_speed_ref = vel_ref;

    /* 边界发生时，先把刚结束的完整行程压缩成一个等权平均点。 */
    if (pitch_scanner.half_cycle_count != half_cycle_before)
        Pitch_BCFinalizeMean();

    if (!done && pitch_scanner.half_cycle_count == half_cycle_before &&
        fabsf(omega) >= PITCH_BC_MIN_SPEED_DPS &&
        fabsf(fabsf(omega) - expected_actual_dps)
            <= expected_actual_dps * PITCH_SPEED_TRACK_RATIO &&
        Pitch_ConstSampleValid(theta_deg, omega, alpha, vel_ref, false))
    {
        theta_rad = theta_deg * Ang_PI;
        gravity = gimbal_sysid.pitch.G_sin * sinf(theta_rad)
                + gimbal_sysid.pitch.G_cos * cosf(theta_rad);
        Pitch_AddBCMeanSample(torque, omega, gravity);
    }

    if (!done)
        return;

    gimbal_sysid.pitch.bc_sample_count = pitch_stage_raw_total;
    if (pitch_bc_ls.count != PITCH_BC_HALF_CYCLES ||
        !LS2_Solve(&pitch_bc_ls, &B, &C, &rmse))
    {
        gimbal_sysid.pitch.bc_rmse = rmse;
        Pitch_Finish(false, GIMBAL_SYSID_ERROR_INSUFFICIENT_EXCITATION,
                     rmse, pitch_stage_raw_total);
        return;
    }

    gimbal_sysid.pitch.B_raw = B;
    gimbal_sysid.pitch.C = C;
    gimbal_sysid.pitch.fit_rmse = rmse;
    gimbal_sysid.pitch.sample_count = pitch_stage_raw_total;
    gimbal_sysid.pitch.bc_rmse = rmse;
    gimbal_sysid.pitch.bc_sample_count = pitch_stage_raw_total;

    /* 轻微负 B 通常来自噪声：保留原始值用于诊断，控制参数钳位为 0。 */
    if (B < -PITCH_BC_NEGATIVE_B_TOLERANCE || C < 0.0f)
    {
        gimbal_sysid.pitch.B = B;
        Pitch_Finish(false, GIMBAL_SYSID_ERROR_NON_PHYSICAL_RESULT,
                     rmse, pitch_stage_raw_total);
        return;
    }
    if (B < 0.0f)
        B = 0.0f;
    gimbal_sysid.pitch.B = B;

    if (pitch_run_all)
        Pitch_BeginJ();
    else
        Pitch_Finish(true, GIMBAL_SYSID_ERROR_NONE, rmse, pitch_stage_raw_total);
}

static float PitchJ_HoldRef(float theta_deg)
{
    float ref = PITCH_J_PREP_HOLD_REF_DPS
              + PITCH_J_PREP_POSITION_KP * (PITCH_J_START_DEG - theta_deg);
    return LIMIT_MAX_MIN(ref, PITCH_J_PREP_REF_MAX_DPS,
                         PITCH_J_PREP_REF_MIN_DPS);
}

static int8_t PitchJ_FindPairBin(float theta_deg)
{
    uint32_t i;

    for (i = 0U; i < PITCH_J_PAIR_BIN_COUNT; i++)
    {
        if (fabsf(theta_deg - PITCH_J_PAIR_CENTER_DEG[i])
            <= PITCH_J_PAIR_HALF_WIDTH_DEG)
            return (int8_t)i;
    }
    return -1;
}

static void PitchJ_FinishTraversal(void)
{
    PitchJPairGroup *group;
    float alpha_avg;
    float inv_count;
    bool direction_valid;

    if (pitch_j_traversal.bin_index < 0 ||
        pitch_j_traversal.group_index < 0)
    {
        PitchJ_ResetTraversal();
        return;
    }

    if (pitch_j_traversal.count < PITCH_J_MIN_TRAVERSAL_SAMPLES ||
        pitch_j_traversal.elapsed < PITCH_J_MIN_TRAVERSAL_TIME_S)
    {
        PitchJ_ResetTraversal();
        return;
    }

    alpha_avg = (pitch_j_traversal.omega_last
               - pitch_j_traversal.omega_start)
              / pitch_j_traversal.elapsed;
    direction_valid = (pitch_j_traversal.group_index == 0)
                    ? (alpha_avg > PITCH_J_MIN_MEAN_ALPHA_DPS2)
                    : (alpha_avg < -PITCH_J_MIN_MEAN_ALPHA_DPS2);
    if (!direction_valid)
    {
        PitchJ_ResetTraversal();
        return;
    }

    group = &pitch_j_pair[(uint32_t)pitch_j_traversal.bin_index]
                         [(uint32_t)pitch_j_traversal.group_index];
    inv_count = 1.0f / (float)pitch_j_traversal.count;
    group->torque_mean_sum += pitch_j_traversal.torque_sum * inv_count;
    group->omega_mean_sum += pitch_j_traversal.omega_sum * inv_count;
    group->theta_mean_sum += pitch_j_traversal.theta_sum * inv_count;
    group->alpha_mean_sum += alpha_avg;
    group->raw_count += pitch_j_traversal.count;
    group->traversal_count++;
    pitch_stage_raw_total += pitch_j_traversal.count;
    gimbal_sysid.pitch.j_sample_count = pitch_stage_raw_total;
    PitchJ_ResetTraversal();
}

static void PitchJ_CollectPairSample(float theta_deg, float omega,
                                     float torque, float dt, bool enabled)
{
    int8_t bin_index = -1;
    int8_t group_index = -1;

    if (enabled)
    {
        bin_index = PitchJ_FindPairBin(theta_deg);
        if (pitch_j_motion_phase == PITCH_J_MOTION_UP_ACCEL)
            group_index = 0;
        else if (pitch_j_motion_phase == PITCH_J_MOTION_UP_DECEL)
            group_index = 1;
    }

    if (pitch_j_traversal.bin_index != bin_index ||
        pitch_j_traversal.group_index != group_index)
        PitchJ_FinishTraversal();

    if (bin_index < 0 || group_index < 0)
        return;

    if (pitch_j_traversal.count == 0U)
    {
        pitch_j_traversal.bin_index = bin_index;
        pitch_j_traversal.group_index = group_index;
        pitch_j_traversal.omega_start = omega;
        pitch_j_traversal.omega_last = omega;
    }
    else
    {
        pitch_j_traversal.elapsed += dt;
        pitch_j_traversal.omega_last = omega;
    }

    pitch_j_traversal.torque_sum += torque;
    pitch_j_traversal.omega_sum += omega;
    pitch_j_traversal.theta_sum += theta_deg;
    pitch_j_traversal.count++;
}

static void PitchJ_FinishAndHold(bool valid, uint8_t error, float rmse)
{
    float hold_ref = gimbal_sysid.pitch.j_velocity_ref;
    Pitch_Finish(valid, error, rmse, gimbal_sysid.pitch.j_sample_count);
    /* 测试最终停在下端，用近似重力平衡参考保持，避免 done 后继续下坠。 */
    ctrl->sys_pitch_speed_ref = hold_ref;
}

static void PitchJ_Finalize(void)
{
    uint32_t i;
    float J;
    float rmse;
    float pair_ratio;
    float alpha_rms;
    float torque_accel;
    float torque_decel;
    float omega_accel;
    float omega_decel;
    float theta_accel;
    float theta_decel;
    float alpha_accel;
    float alpha_decel;
    float gravity_accel;
    float gravity_decel;
    float delta_torque;
    float delta_omega;
    float delta_gravity;
    float delta_alpha;
    float pair_residual;
    float pair_J;
    uint32_t raw_count;
    PitchJPairGroup *accel_group;
    PitchJPairGroup *decel_group;

    PitchJ_FinishTraversal();
    LS1_Reset(&pitch_j_all);
    gimbal_sysid.pitch.J_pair_min = 1.0e6f;
    gimbal_sysid.pitch.J_pair_max = -1.0e6f;

    for (i = 0U; i < PITCH_J_PAIR_BIN_COUNT; i++)
    {
        accel_group = &pitch_j_pair[i][0];
        decel_group = &pitch_j_pair[i][1];
        if (accel_group->traversal_count == 0U ||
            decel_group->traversal_count == 0U)
            continue;

        torque_accel = accel_group->torque_mean_sum
                     / (float)accel_group->traversal_count;
        torque_decel = decel_group->torque_mean_sum
                     / (float)decel_group->traversal_count;
        omega_accel = accel_group->omega_mean_sum
                    / (float)accel_group->traversal_count;
        omega_decel = decel_group->omega_mean_sum
                    / (float)decel_group->traversal_count;
        theta_accel = accel_group->theta_mean_sum
                    / (float)accel_group->traversal_count;
        theta_decel = decel_group->theta_mean_sum
                    / (float)decel_group->traversal_count;
        alpha_accel = accel_group->alpha_mean_sum
                    / (float)accel_group->traversal_count;
        alpha_decel = decel_group->alpha_mean_sum
                    / (float)decel_group->traversal_count;
        delta_alpha = alpha_accel - alpha_decel;
        if (delta_alpha < 2.0f * PITCH_J_MIN_MEAN_ALPHA_DPS2)
            continue;

        gravity_accel = gimbal_sysid.pitch.G_sin
                      * sinf(theta_accel * Ang_PI)
                      + gimbal_sysid.pitch.G_cos
                      * cosf(theta_accel * Ang_PI);
        gravity_decel = gimbal_sysid.pitch.G_sin
                      * sinf(theta_decel * Ang_PI)
                      + gimbal_sysid.pitch.G_cos
                      * cosf(theta_decel * Ang_PI);
        delta_torque = torque_accel - torque_decel;
        delta_omega = omega_accel - omega_decel;
        delta_gravity = gravity_accel - gravity_decel;
        pair_residual = delta_torque - gimbal_sysid.pitch.B * delta_omega
                      - delta_gravity;
        pair_J = pair_residual / delta_alpha;
        raw_count = accel_group->raw_count + decel_group->raw_count;

        LS1_Add(&pitch_j_all, delta_alpha, pair_residual);
        if (pair_J < gimbal_sysid.pitch.J_pair_min)
            gimbal_sysid.pitch.J_pair_min = pair_J;
        if (pair_J > gimbal_sysid.pitch.J_pair_max)
            gimbal_sysid.pitch.J_pair_max = pair_J;
        SysId_PublishMeanPoint(&gimbal_sysid.pitch, delta_torque,
                               delta_omega, delta_gravity, delta_alpha,
                               pair_residual, raw_count, pitch_j_all.count);
    }

    if (pitch_j_all.count != PITCH_J_PAIR_BIN_COUNT ||
        !LS1_Solve(&pitch_j_all, &J, &rmse))
    {
        PitchJ_FinishAndHold(false, GIMBAL_SYSID_ERROR_INSUFFICIENT_EXCITATION,
                            0.0f);
        return;
    }

    gimbal_sysid.pitch.J = J;
    gimbal_sysid.pitch.j_rmse = rmse;
    alpha_rms = sqrtf(pitch_j_all.aa / (float)pitch_j_all.count);
    gimbal_sysid.pitch.j_signal_rms = fabsf(J) * alpha_rms;
    if (gimbal_sysid.pitch.j_signal_rms > 1.0e-3f)
        gimbal_sysid.pitch.j_residual_ratio = rmse / gimbal_sysid.pitch.j_signal_rms;
    else
        gimbal_sysid.pitch.j_residual_ratio = 1.0e6f;

    if (J <= 0.0f || gimbal_sysid.pitch.J_pair_min <= 0.0f)
    {
        PitchJ_FinishAndHold(false, GIMBAL_SYSID_ERROR_NON_PHYSICAL_RESULT, rmse);
        return;
    }

    pair_ratio = gimbal_sysid.pitch.J_pair_max
               / gimbal_sysid.pitch.J_pair_min;
    if (pair_ratio > PITCH_J_MAX_PAIR_RATIO)
    {
        PitchJ_FinishAndHold(false, GIMBAL_SYSID_ERROR_PAIR_MISMATCH, rmse);
        return;
    }

    if (gimbal_sysid.pitch.j_residual_ratio > PITCH_J_MAX_RMSE_SIGNAL_RATIO)
    {
        PitchJ_FinishAndHold(false, GIMBAL_SYSID_ERROR_POOR_FIT, rmse);
        return;
    }

    PitchJ_FinishAndHold(true, GIMBAL_SYSID_ERROR_NONE, rmse);
}

static void Pitch_JRun(float theta_deg, float omega, float torque, float dt)
{
    float error;
    float vel_ref;
    float distance;
    float desired_actual_speed;
    float switch_deg;
    float target_accel;
    float delta_speed_sq;
    bool pair_sample_enabled;

    if (pitch_stage == PITCH_SYSID_STAGE_J_PREPARE)
    {
        error = PITCH_J_START_DEG - theta_deg;
        /*
         * 远距离使用实测能可靠运动的非对称参考值：上升 50、下降 -1。
         * 进入起点附近后再用带重力平衡偏置的参考精定位，避免 20 左右
         * 的参考落入重力与静摩擦共同形成的死区。
         */
        if (theta_deg > PITCH_J_START_DEG + PITCH_J_PREP_FAST_BAND_DEG)
            vel_ref = -PITCH_J_PREP_DOWN_REF_DPS;
        else if (theta_deg < PITCH_J_START_DEG - PITCH_J_PREP_FAST_BAND_DEG)
            vel_ref = PITCH_J_PREP_UP_REF_DPS;
        else
            vel_ref = PitchJ_HoldRef(theta_deg);
        ctrl->sys_pitch_speed_ref = vel_ref;
        gimbal_sysid.pitch.j_velocity_ref = vel_ref;

        if (fabsf(error) < PITCH_J_CENTER_TOL_DEG && fabsf(omega) < 2.0f)
            pitch_j_centered_elapsed += dt;
        else
            pitch_j_centered_elapsed = 0.0f;

        if (pitch_j_centered_elapsed >= PITCH_J_SETTLE_S)
        {
            pitch_j_phase_elapsed = 0.0f;
            pitch_j_centered_elapsed = 0.0f;
            pitch_j_alpha_filtered = 0.0f;
            pitch_j_motion_phase = PITCH_J_MOTION_UP_ACCEL;
            gimbal_sysid.pitch.j_motion_phase = (uint8_t)pitch_j_motion_phase;
            PitchJ_ResetTraversal();
            TD_Clear(&gimbal_sysid.pitch.td_omega, ctrl->d_pitch*PI_Ang);
            Pitch_SetStage(PITCH_SYSID_STAGE_J_EXCITE);
        }
        return;
    }

    pitch_j_phase_elapsed += dt;
    if (pitch_j_pass_count < PITCH_J_MEASURED_PASSES)
        switch_deg = PITCH_J_SWITCH_DEG[pitch_j_pass_count];
    else
        switch_deg = PITCH_J_SWITCH_DEG[PITCH_J_MEASURED_PASSES - 1U];
    delta_speed_sq = PITCH_J_ACTUAL_PEAK_SPEED_DPS *
                     PITCH_J_ACTUAL_PEAK_SPEED_DPS -
                     PITCH_J_ACTUAL_MIN_SPEED_DPS *
                     PITCH_J_ACTUAL_MIN_SPEED_DPS;
    target_accel = 0.0f;
    gimbal_sysid.pitch.j_switch_angle = switch_deg;

    if (pitch_j_motion_phase == PITCH_J_MOTION_UP_ACCEL)
    {
        if (theta_deg >= switch_deg)
        {
            pitch_j_motion_phase = PITCH_J_MOTION_UP_DECEL;
            pitch_j_phase_elapsed = 0.0f;
        }
        distance = LIMIT_MAX_MIN(theta_deg - PITCH_J_START_DEG,
                                 switch_deg - PITCH_J_START_DEG, 0.0f);
        target_accel = delta_speed_sq /
                       (2.0f * (switch_deg - PITCH_J_START_DEG));
        desired_actual_speed = sqrtf(PITCH_J_ACTUAL_MIN_SPEED_DPS *
                                     PITCH_J_ACTUAL_MIN_SPEED_DPS +
                                     2.0f * target_accel * distance);
        vel_ref = PITCH_J_UP_REF_OFFSET_DPS + desired_actual_speed;
    }
    else if (pitch_j_motion_phase == PITCH_J_MOTION_UP_DECEL)
    {
        if (theta_deg >= PITCH_J_END_DEG)
        {
            pitch_j_motion_phase = PITCH_J_MOTION_RETURN_DOWN;
            pitch_j_phase_elapsed = 0.0f;
            pitch_j_pass_count++;
            gimbal_sysid.pitch.j_pass_count = pitch_j_pass_count;
        }
        distance = LIMIT_MAX_MIN(PITCH_J_END_DEG - theta_deg,
                                 PITCH_J_END_DEG - switch_deg, 0.0f);
        target_accel = delta_speed_sq /
                       (2.0f * (PITCH_J_END_DEG - switch_deg));
        desired_actual_speed = sqrtf(PITCH_J_ACTUAL_MIN_SPEED_DPS *
                                     PITCH_J_ACTUAL_MIN_SPEED_DPS +
                                     2.0f * target_accel * distance);
        vel_ref = PITCH_J_UP_REF_OFFSET_DPS + desired_actual_speed;
    }
    else if (pitch_j_motion_phase == PITCH_J_MOTION_RETURN_DOWN)
    {
        vel_ref = -PITCH_J_RETURN_DOWN_REF_DPS;
        if (theta_deg <= PITCH_J_START_DEG + 1.0f)
        {
            pitch_j_phase_elapsed = 0.0f;
            if (pitch_j_pass_count >= PITCH_J_MEASURED_PASSES)
            {
                pitch_j_motion_phase = PITCH_J_MOTION_FINAL_HOLD;
                pitch_j_centered_elapsed = 0.0f;
            }
            else
                pitch_j_motion_phase = PITCH_J_MOTION_UP_ACCEL;
        }
    }
    else
    {
        vel_ref = PitchJ_HoldRef(theta_deg);
        error = PITCH_J_START_DEG - theta_deg;
        if (fabsf(error) < 1.0f && fabsf(omega) < 2.0f)
            pitch_j_centered_elapsed += dt;
        else
            pitch_j_centered_elapsed = 0.0f;

        ctrl->sys_pitch_speed_ref = vel_ref;
        gimbal_sysid.pitch.j_velocity_ref = vel_ref;
        gimbal_sysid.pitch.j_motion_phase = (uint8_t)pitch_j_motion_phase;
        if (pitch_j_centered_elapsed >= PITCH_J_SETTLE_S ||
            pitch_j_phase_elapsed >= PITCH_J_FINAL_HOLD_TIMEOUT_S)
            PitchJ_Finalize();
        return;
    }

    vel_ref = LIMIT_MAX_MIN(vel_ref, PITCH_J_UP_REF_MAX_DPS,
                            -PITCH_J_RETURN_DOWN_REF_DPS);
    if (pitch_j_motion_phase == PITCH_J_MOTION_UP_ACCEL ||
        pitch_j_motion_phase == PITCH_J_MOTION_UP_DECEL)
        vel_ref = LIMIT_MAX_MIN(vel_ref, PITCH_J_UP_REF_MAX_DPS,
                                PITCH_J_UP_REF_MIN_DPS);
    ctrl->sys_pitch_speed_ref = vel_ref;
    gimbal_sysid.pitch.j_velocity_ref = vel_ref;
    gimbal_sysid.pitch.j_target_accel =
        (pitch_j_motion_phase == PITCH_J_MOTION_UP_DECEL)
        ? -target_accel : target_accel;
    gimbal_sysid.pitch.j_motion_phase = (uint8_t)pitch_j_motion_phase;

    pair_sample_enabled = (pitch_j_motion_phase == PITCH_J_MOTION_UP_ACCEL ||
                           pitch_j_motion_phase == PITCH_J_MOTION_UP_DECEL) &&
                          pitch_j_phase_elapsed > PITCH_J_SWITCH_SETTLE_S &&
                          omega > PITCH_J_MIN_SPEED_DPS;
    PitchJ_CollectPairSample(theta_deg, omega, torque, dt,
                             pair_sample_enabled);
}

static void Pitch_Run(void)
{
    const float two_pi = 6.28318530718f;
    float dt = ctrl->sys_delta_t;
    float theta_deg;
    float omega_raw;
    float omega_smooth;
    float alpha_raw;
    float alpha_lpf_k;
    float torque_raw;

    if (dt <= 0.0f || dt > 0.01f)
        dt = 0.002f;

    theta_deg = ctrl->pitch;
    omega_raw = ctrl->d_pitch*PI_Ang;
    torque_raw = gimbal_sysid_user_config.pitch.torque_feedback_coef
               * Gimbal_Motor.DM_4310[1].tor;
    omega_smooth = TD_Calculate(&gimbal_sysid.pitch.td_omega, omega_raw);
    alpha_raw = gimbal_sysid.pitch.td_omega.dx;

    /* TD 微分的偶发尖峰不进入低通器，避免单点通过 alpha^2 支配 J 拟合。 */
    if (fabsf(alpha_raw) <= PITCH_J_RAW_ALPHA_REJECT_DPS2)
    {
        alpha_lpf_k = two_pi * PITCH_J_ALPHA_LPF_HZ * dt;
        alpha_lpf_k = alpha_lpf_k / (1.0f + alpha_lpf_k);
        pitch_j_alpha_filtered += alpha_lpf_k
                                * (alpha_raw - pitch_j_alpha_filtered);
    }
    gimbal_sysid.pitch.j_alpha_filtered = pitch_j_alpha_filtered;

    if (theta_deg <= PITCH_SAFE_MIN_DEG || theta_deg >= PITCH_SAFE_MAX_DEG)
    {
        Pitch_Finish(false, GIMBAL_SYSID_ERROR_SAFETY_LIMIT, 0.0f,
                     gimbal_sysid.pitch.sample_count);
        return;
    }

    switch (pitch_stage)
    {
    case PITCH_SYSID_STAGE_GRAVITY:
        Pitch_GravityRun(theta_deg, omega_smooth, alpha_raw, torque_raw, dt);
        break;
    case PITCH_SYSID_STAGE_BC:
        Pitch_BCRun(theta_deg, omega_smooth, alpha_raw, torque_raw, dt);
        break;
    case PITCH_SYSID_STAGE_J_PREPARE:
    case PITCH_SYSID_STAGE_J_EXCITE:
        Pitch_JRun(theta_deg, omega_smooth, torque_raw, dt);
        break;
    default:
        ctrl->sys_pitch_speed_ref = 0.0f;
        break;
    }
}

#if GIMBAL_SYSID == GIMBAL_YAW_SYSID
static bool Yaw_UserConfigValid(void)
{
    uint32_t i;
    bool has_positive_speed = false;
    bool has_negative_speed = false;

    if (fabsf(gimbal_sysid_user_config.yaw.torque_feedback_coef) < 0.01f ||
        YAW_J_REF_ACCEL_DPS2 <= 0.0f || YAW_J_MAX_REF_DPS <= 0.0f)
        return false;

    for (i = 0U; i < YAW_BC_POINT_COUNT; i++)
    {
        if (YAW_BC_SPEED_DPS[i] > 0.0f)
            has_positive_speed = true;
        else if (YAW_BC_SPEED_DPS[i] < 0.0f)
            has_negative_speed = true;
        else
            return false;
    }
    if (!has_positive_speed || !has_negative_speed)
        return false;

    for (i = 0U; i < YAW_J_PAIR_WINDOW_COUNT; i++)
    {
        if (YAW_J_PAIR_CENTER_DPS[i] <= YAW_J_PAIR_HALF_WIDTH_DPS ||
            YAW_J_PAIR_CENTER_DPS[i] + YAW_J_PAIR_HALF_WIDTH_DPS
                >= YAW_J_MAX_REF_DPS)
            return false;
    }
    return true;
}
#endif

#if GIMBAL_SYSID == GIMBAL_PITCH_SYSID
static bool Pitch_UserConfigValid(void)
{
    uint32_t i;
    uint32_t j;
    bool has_accel;
    bool has_decel;

    if (fabsf(gimbal_sysid_user_config.pitch.torque_feedback_coef) < 0.01f ||
        PITCH_SAFE_MIN_DEG >= PITCH_SAFE_MAX_DEG ||
        PITCH_J_END_DEG - PITCH_J_START_DEG < PITCH_MIN_USABLE_SPAN_DEG ||
        PITCH_REVERSE_MIN_DEG <= PITCH_SAFE_MIN_DEG ||
        PITCH_REVERSE_MAX_DEG >= PITCH_SAFE_MAX_DEG ||
        PITCH_SAMPLE_MIN_DEG <= PITCH_REVERSE_MIN_DEG ||
        PITCH_SAMPLE_MAX_DEG >= PITCH_REVERSE_MAX_DEG ||
        PITCH_J_START_DEG != PITCH_SAMPLE_MIN_DEG ||
        PITCH_J_END_DEG != PITCH_SAMPLE_MAX_DEG ||
        PITCH_J_START_DEG >= PITCH_J_END_DEG ||
        PITCH_GRAVITY_UP_REF_DPS <= 0.0f ||
        PITCH_GRAVITY_DOWN_REF_DPS <= 0.0f ||
        PITCH_J_PREP_UP_REF_DPS <= 0.0f ||
        PITCH_J_PREP_DOWN_REF_DPS <= 0.0f ||
        PITCH_J_ACTUAL_PEAK_SPEED_DPS <= PITCH_J_ACTUAL_MIN_SPEED_DPS ||
        PITCH_J_UP_REF_MIN_DPS <= 0.0f ||
        PITCH_J_UP_REF_MIN_DPS > PITCH_J_UP_REF_MAX_DPS ||
        PITCH_J_RETURN_DOWN_REF_DPS <= 0.0f)
        return false;

    for (i = 0U; i < PITCH_BC_SPEED_LEVELS; i++)
    {
        if (PITCH_BC_UP_REF_DPS[i] <= 0.0f ||
            PITCH_BC_DOWN_REF_DPS[i] <= 0.0f ||
            PITCH_BC_EXPECTED_DPS[i] <= PITCH_BC_MIN_SPEED_DPS)
            return false;
    }

    for (i = 0U; i < PITCH_J_MEASURED_PASSES; i++)
    {
        if (PITCH_J_SWITCH_DEG[i] <= PITCH_J_START_DEG ||
            PITCH_J_SWITCH_DEG[i] >= PITCH_J_END_DEG)
            return false;
    }

    for (i = 0U; i < PITCH_J_PAIR_BIN_COUNT; i++)
    {
        if (PITCH_J_PAIR_CENTER_DEG[i] <= PITCH_J_START_DEG ||
            PITCH_J_PAIR_CENTER_DEG[i] >= PITCH_J_END_DEG ||
            PITCH_J_PAIR_CENTER_DEG[i] - PITCH_J_PAIR_HALF_WIDTH_DEG
                <= PITCH_J_START_DEG ||
            PITCH_J_PAIR_CENTER_DEG[i] + PITCH_J_PAIR_HALF_WIDTH_DEG
                >= PITCH_J_END_DEG)
            return false;
        has_accel = false;
        has_decel = false;
        for (j = 0U; j < PITCH_J_MEASURED_PASSES; j++)
        {
            if (PITCH_J_SWITCH_DEG[j] > PITCH_J_PAIR_CENTER_DEG[i])
                has_accel = true;
            if (PITCH_J_SWITCH_DEG[j] < PITCH_J_PAIR_CENTER_DEG[i])
                has_decel = true;
        }
        if (!has_accel || !has_decel)
            return false;
    }
    return true;
}
#endif

/* ==================================================================
 *  7. 顶层接口
 * ================================================================== */
void GimbalSystemID_Init(Gimbal_Status_t *controller)
{
    ctrl = controller;

#if GIMBAL_SYSID
    if (ctrl == NULL)
        return;

    GimbalSystemID_LoadSelectedProfile();
    /* The two safe limits are the only user-entered Pitch angles. */
    Pitch_BuildAngleLayout();

    /* 通用初始化 */
    TD_Init(&gimbal_sysid.yaw.td_omega,   10000.0f, 0.005f);
    TD_Init(&gimbal_sysid.pitch.td_omega, 10000.0f, 0.005f);

    gimbal_sysid.yaw.sysid_timer   = 0.0f;
    gimbal_sysid.yaw.sysid_done    = 1U;
    gimbal_sysid.pitch.sysid_timer = 0.0f;
    gimbal_sysid.pitch.sysid_done  = 1U;

    gimbal_sysid.yaw.fit_rmse = 0.0f;
    gimbal_sysid.yaw.gravity_rmse = 0.0f;
    gimbal_sysid.yaw.bc_rmse = 0.0f;
    gimbal_sysid.yaw.j_rmse = 0.0f;
    gimbal_sysid.yaw.J_pair_min = 0.0f;
    gimbal_sysid.yaw.J_pair_max = 0.0f;
    gimbal_sysid.yaw.j_alpha_filtered = 0.0f;
    gimbal_sysid.yaw.j_signal_rms = 0.0f;
    gimbal_sysid.yaw.j_residual_ratio = 0.0f;
    gimbal_sysid.yaw.j_velocity_ref = 0.0f;
    gimbal_sysid.yaw.j_switch_angle = 0.0f;
    gimbal_sysid.yaw.j_target_accel = 0.0f;
    gimbal_sysid.yaw.mean_torque = 0.0f;
    gimbal_sysid.yaw.mean_omega = 0.0f;
    gimbal_sysid.yaw.mean_gravity = 0.0f;
    gimbal_sysid.yaw.mean_input = 0.0f;
    gimbal_sysid.yaw.mean_residual = 0.0f;
    gimbal_sysid.yaw.mean_raw_count = 0U;
    gimbal_sysid.yaw.sample_count = 0U;
    gimbal_sysid.yaw.gravity_valid_bins = 0U;
    gimbal_sysid.yaw.bc_sample_count = 0U;
    gimbal_sysid.yaw.j_sample_count = 0U;
    gimbal_sysid.yaw.mean_point_count = 0U;
    gimbal_sysid.yaw.j_motion_phase = (uint8_t)YAW_J_MOTION_PREPARE;
    gimbal_sysid.yaw.j_pass_count = 0U;
    gimbal_sysid.yaw.sysid_valid = 0U;
    gimbal_sysid.yaw.sysid_error = GIMBAL_SYSID_ERROR_NONE;
    yaw_run_all = false;
    Yaw_SetStage(YAW_SYSID_STAGE_IDLE);

    gimbal_sysid.pitch.fit_rmse = 0.0f;
    gimbal_sysid.pitch.gravity_rmse = 0.0f;
    gimbal_sysid.pitch.bc_rmse = 0.0f;
    gimbal_sysid.pitch.j_rmse = 0.0f;
    gimbal_sysid.pitch.J_pair_min = 0.0f;
    gimbal_sysid.pitch.J_pair_max = 0.0f;
    gimbal_sysid.pitch.j_alpha_filtered = 0.0f;
    gimbal_sysid.pitch.j_signal_rms = 0.0f;
    gimbal_sysid.pitch.j_residual_ratio = 0.0f;
    gimbal_sysid.pitch.j_velocity_ref = 0.0f;
    gimbal_sysid.pitch.j_switch_angle = PITCH_J_SWITCH_DEG[0];
    gimbal_sysid.pitch.j_target_accel = 0.0f;
    gimbal_sysid.pitch.mean_torque = 0.0f;
    gimbal_sysid.pitch.mean_omega = 0.0f;
    gimbal_sysid.pitch.mean_gravity = 0.0f;
    gimbal_sysid.pitch.mean_input = 0.0f;
    gimbal_sysid.pitch.mean_residual = 0.0f;
    gimbal_sysid.pitch.mean_raw_count = 0U;
    gimbal_sysid.pitch.sample_count = 0U;
    gimbal_sysid.pitch.gravity_valid_bins = 0U;
    gimbal_sysid.pitch.bc_sample_count = 0U;
    gimbal_sysid.pitch.j_sample_count = 0U;
    gimbal_sysid.pitch.mean_point_count = 0U;
    gimbal_sysid.pitch.j_motion_phase = (uint8_t)PITCH_J_MOTION_PREPARE;
    gimbal_sysid.pitch.j_pass_count = 0U;
    gimbal_sysid.pitch.sysid_valid = 0U;
    gimbal_sysid.pitch.sysid_error = GIMBAL_SYSID_ERROR_NONE;
    pitch_run_all = false;
    Pitch_SetStage(PITCH_SYSID_STAGE_IDLE);

#if GIMBAL_SYSID == GIMBAL_YAW_SYSID
    /* 安全互锁：上电保持 done=1，进入 Debug 后手动置 0 才允许运动。 */
    gimbal_sysid.yaw.sysid_done = 1U;
    if (!Yaw_UserConfigValid())
    {
        Yaw_Finish(false, GIMBAL_SYSID_ERROR_INVALID_CONFIG, 0.0f, 0U);
        return;
    }
  #if GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_BC
    Yaw_BeginBC();
  #elif GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_J
    Yaw_BeginJ();
  #elif GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_ALL
    yaw_run_all = true;
    Yaw_BeginBC();
  #else
    Yaw_Finish(false, GIMBAL_SYSID_ERROR_INSUFFICIENT_EXCITATION,
               0.0f, 0U);
  #endif

#elif GIMBAL_SYSID == GIMBAL_PITCH_SYSID
    /* 安全互锁：上电保持 done=1，进入 Debug 后手动置 0 才允许运动。 */
    gimbal_sysid.pitch.sysid_done = 1U;
    if (!Pitch_UserConfigValid())
    {
        Pitch_Finish(false, GIMBAL_SYSID_ERROR_INVALID_CONFIG, 0.0f, 0U);
        return;
    }
  #if GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_GRAVITY
    Pitch_BeginGravity();
  #elif GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_BC
    Pitch_BeginBC();
  #elif GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_J
    Pitch_BeginJ();
  #elif GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_ALL
    pitch_run_all = true;
    Pitch_BeginGravity();
  #else
    Pitch_Finish(false, GIMBAL_SYSID_ERROR_INSUFFICIENT_EXCITATION, 0.0f, 0U);
  #endif
#endif
#endif /* GIMBAL_SYSID */
}

void GimbalSystemID_Run(void)
{
    float dt;

    if (ctrl == NULL)
        return;

    dt = ctrl->sys_delta_t;
    if (dt <= 0.0f || dt > 0.01f)
        dt = 0.002f;

#if GIMBAL_SYSID == GIMBAL_YAW_SYSID
    if (gimbal_sysid.yaw.sysid_done)
        return;
    gimbal_sysid.yaw.sysid_timer += dt;
    Yaw_Run(dt);

#elif GIMBAL_SYSID == GIMBAL_PITCH_SYSID
    if (gimbal_sysid.pitch.sysid_done)
        return;
    gimbal_sysid.pitch.sysid_timer += dt;
    Pitch_Run();
#endif
}
