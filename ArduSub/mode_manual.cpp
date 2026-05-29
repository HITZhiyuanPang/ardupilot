// mode_manual.cpp - 手动直通飞行模式
// 飞手输入直接传递给推进器，无任何姿态稳定和辅助
// 适用于飞手完全自主控制或调试场景

#include "Sub.h"


bool ModeManual::init(bool ignore_checks) {
    // 设置目标高度为 0（仅用于 GCS 显示，手动模式不控制深度）
    position_control->set_pos_desired_U_cm(0);

    // 手动模式下姿态输入变为推力输入
    // 设为中立值，防止模式切换时产生混乱行为（特别是横滚/俯仰）
    sub.set_neutral_controls();

    return true;
}

// manual_run - 手动直通控制器主循环
// 应以 100Hz 或更高频率调用
void ModeManual::run()
{
    // 未解锁时将油门设为零并立即返回（不输出任何推力）
    if (!sub.motors.armed()) {
        sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    // 解锁后允许全油门输出
    sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 直接将飞手输入归一化值传给电机库（-1.0 ~ +1.0）
    sub.motors.set_roll(channel_roll->norm_input());
    sub.motors.set_pitch(channel_pitch->norm_input());
    // 偏航按增益比例缩放，与 ACRO 模式保持一致的手感
    sub.motors.set_yaw(channel_yaw->norm_input() * g.acro_yaw_p / ACRO_YAW_P);
    // 油门通道从 [-1,+1] 映射到 [0,1]（中位对应 50% 推力）
    sub.motors.set_throttle((channel_throttle->norm_input() + 1.0f) / 2.0f);
    sub.motors.set_forward(channel_forward->norm_input());
    sub.motors.set_lateral(channel_lateral->norm_input());
}
