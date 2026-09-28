#include "System_Identification.h"

/*******************************************************************************************************
Pitch轴系统辨识计算逻辑
********************************************************************************************************/
void Gimbal_Pitch_SysID_Run(Gimbal_Status_t *gs)
{
    float dt = 0.002f;
    
    if (gs->Pitch_SysID.sysid_done) return;
    gs->Pitch_SysID.sysid_timer += dt;

    const float SAFE_MAX = 25.0f;   // 度
    const float SAFE_MIN = -25.0f;
    const float MARGIN = 2.0f;
    
    static float vel_target = 120.0f * Ang_PI;  // rad/s
    static int half_cycles = 0;
    
    // 1. 边界检测，自动换向
    float pitch_deg = gs->pitch;  // 度
    if (pitch_deg > (SAFE_MAX - MARGIN) && vel_target > 0)
    {
        vel_target = -10.0f * Ang_PI;  // 下降用低速
        half_cycles++;
    }
    else if (pitch_deg < (SAFE_MIN + MARGIN) && vel_target < 0)
    {
        vel_target = 150.0f * Ang_PI;  // 上升用高速
        half_cycles++;
    }
    
    PID_Calculate(&Pitch_S_Pid, gs->d_pitch, vel_target);
    gs->Pitch_Motor_Out = Pitch_S_Pid.Output; 

    // 2. 读取数据，构建 RLS 回归向量
    // 模型: torque = B*omega + A*sin(θ)  + C*sign(ω)
    //        y      = x[0]*H[0] + x[1]*H[1]  + x[2]*H[2]
    float omega = gs->d_pitch;
    float torque = PITCH_MOTOR_SIGN * Gimbal_Motor.DM_4310[1].tor;
    float pitch_rad = pitch_deg * Ang_PI;
    
    // 3. RLS 更新（同时辨识 B, A, C）
    if (fabsf(omega) > 0.5f * Ang_PI)  // 速度足够大，避免静止噪声
    {
        gs->Pitch_SysID.rls_sysid.H_data[0] = omega;               // ω  → B (阻尼)
        gs->Pitch_SysID.rls_sysid.H_data[1] = sin(pitch_rad);      // sinθ → A (重力sin分量)
        gs->Pitch_SysID.rls_sysid.H_data[2] = sign(omega);         // sign(ω) → C (库伦摩擦)
        gs->Pitch_SysID.rls_sysid.y_data[0] = torque;              // 原始扭矩，不做预补偿
        RLS_Update(&gs->Pitch_SysID.rls_sysid);
    }
    
    // 4. 完成条件
    if (half_cycles >= 6)  // 3 个来回
    {
        gs->Pitch_SysID.sysid_done = 1;
        gs->Pitch_SysID.B = gs->Pitch_SysID.rls_sysid.x_data[0];  // 阻尼
        gs->Pitch_SysID.C = gs->Pitch_SysID.rls_sysid.x_data[2];  // 库伦摩擦
        Gravity_Param.A  = gs->Pitch_SysID.rls_sysid.x_data[1];  // 重力 sin 分量
        Gravity_Param.B  = 0.0f;  // 重力 cos 分量
        Gravity_Param.C  = gs->Pitch_SysID.rls_sysid.x_data[2];  // 库伦摩擦
    }
    
    // 5. 紧急保护（启动0.5s后才生效，避免上电误触发）
    if (gs->Pitch_SysID.sysid_timer > 0.5f){
        if (pitch_deg < SAFE_MIN || pitch_deg > SAFE_MAX)
        {
            gs->Pitch_SysID.sysid_done = 1;
            gs->Pitch_SysID.B = gs->Pitch_SysID.rls_sysid.x_data[0];
            gs->Pitch_SysID.C = gs->Pitch_SysID.rls_sysid.x_data[2];
            Gravity_Param.A  = gs->Pitch_SysID.rls_sysid.x_data[1];
            Gravity_Param.B  = 0.0f;
            Gravity_Param.C  = gs->Pitch_SysID.rls_sysid.x_data[2];
        }
    }
}

/*******************************************************************************************************
Yaw轴系统辨识计算逻辑
********************************************************************************************************/
void Gimbal_Yaw_SysID_Run(Gimbal_Status_t *gs)
{
    float dt = 0.002f;  // 你的控制周期
    
    if (gs->Yaw_SysID.sysid_done) return;
    gs->Yaw_SysID.sysid_timer += dt;

#if GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_BC
    // ===== Step 1: 辨识 B, C =====
    const float vel_pts[] = {200, 150, 100, 50, -50, -100, -150, -200};
    const uint8_t NUM_PTS = 8;
    const float SETTLE_TIME = 0.4f;
    const float DEG_PER_REV = 360.0f;
    
    static uint8_t step_idx = 0;
    static float step_timer = 0;
    static float total_angle = 0;
    static float torque_sum = 0, omega_sum = 0;
    static uint32_t sample_count = 0;
    
    step_timer += dt;
    // 设置速度环目标（注意：你的速度环输入是 rad/s，需要转换）
    gs->yaw_ref = gs->yaw;  // 不控制位置，只控制速度
    // 直接用速度环控制 → 设置 Yaw_S_Pid 的 Ref
    PID_Calculate(&Yaw_S_Pid, gs->d_yaw, vel_pts[step_idx] * Ang_PI);
    gs->Yaw_Motor_Out = Yaw_S_Pid.Output;

    if (step_timer > SETTLE_TIME)
    {
        float omega = gs->d_yaw;  // rad/s
        float torque = YAW_MOTOR_SIGN * Gimbal_Motor.DM_4310[0].tor;
        
        total_angle += fabsf(omega) * dt;
        torque_sum += torque;
        omega_sum += omega;
        sample_count++;
    }
    
    if (total_angle >= DEG_PER_REV * Ang_PI)  // 转够 1 圈
    {
        if (sample_count > 0)
        {
            float torque_avg = torque_sum / sample_count;
            float omega_avg = omega_sum / sample_count;
            float sign_w = (omega_avg > 0.01f) ? 1.0f : ((omega_avg < -0.01f) ? -1.0f : 0.0f);
            
            gs->Yaw_SysID.rls_sysid.H_data[0] = omega_avg;
            gs->Yaw_SysID.rls_sysid.H_data[1] = sign_w;
            gs->Yaw_SysID.rls_sysid.y_data[0] = torque_avg;
            RLS_Update(&gs->Yaw_SysID.rls_sysid);
        }
        // 重置，进入下一个速度点
        step_timer = 0; total_angle = 0;
        torque_sum = 0; omega_sum = 0; sample_count = 0;
        step_idx++;
        TD_Init(&gs->Yaw_SysID.td_omega, 10000, 0.005);
        
        if (step_idx >= NUM_PTS)
        {
            gs->Yaw_SysID.sysid_done = 1;
            gs->Yaw_SysID.B = gs->Yaw_SysID.rls_sysid.x_data[0];
            gs->Yaw_SysID.C = gs->Yaw_SysID.rls_sysid.x_data[1];
        }
    }

#elif GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_J
    // ===== Step 2: 辨识 J =====
    const float ACCEL = 80.0f * Ang_PI;   // rad/s?
    const float MAX_SPEED = 250.0f * Ang_PI; // rad/s
    
    static float ramp_speed = 0;
    static uint8_t init_done = 0;
    if (!init_done)
    {
        TD_Init(&gs->Yaw_SysID.td_omega, 10000, 0.005);
        gs->Yaw_SysID.B = Yaw_FF_Param.B;
        gs->Yaw_SysID.C = Yaw_FF_Param.C;
        init_done = 1;
    }
    static float torque_sum = 0, omega_sum = 0, alpha_sum = 0;
    static uint32_t sample_count = 0;
    
    ramp_speed += ACCEL * dt;
    if (ramp_speed > MAX_SPEED) ramp_speed = MAX_SPEED;
    
    PID_Calculate(&Yaw_S_Pid, gs->d_yaw, ramp_speed);
    gs->Yaw_Motor_Out = Yaw_S_Pid.Output;

    if (gs->Yaw_SysID.sysid_timer > 0.1f)
    {
        float omega = gs->d_yaw*PI_Ang;
        float omega_smooth = TD_Calculate(&gs->Yaw_SysID.td_omega, omega);
        float alpha_smooth = gs->Yaw_SysID.td_omega.dx;
        float torque = YAW_MOTOR_SIGN * Gimbal_Motor.DM_4310[0].tor;
        
        torque_sum += torque;
        omega_sum += omega_smooth;
        alpha_sum += alpha_smooth;
        sample_count++;
    }
    
    if (ramp_speed >= MAX_SPEED)
    {
        if (sample_count > 0)
        {
            float torque_avg = torque_sum / sample_count;
            float omega_avg = omega_sum / sample_count;
            float alpha_avg = alpha_sum / sample_count;
            gs->Yaw_SysID.J = (torque_avg - gs->Yaw_SysID.B * omega_avg - gs->Yaw_SysID.C) / alpha_avg;
        }
        gs->Yaw_SysID.sysid_done = 1;
    }
#endif
}
