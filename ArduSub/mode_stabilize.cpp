// mode_stabilize.cpp - 稳定飞行模式
// 自动稳定横滚/俯仰姿态，飞手通过摇杆输入目标倾斜角
// 偏航：有偏航输入时按角速率控制，无输入时保持最后航向
// 深度/油门：飞手直接控制（无自动深度保持）

#include "Sub.h"


bool ModeStabilize::init(bool ignore_checks) {
    // 设置目标深度为 0（仅用于 GCS 显示）
    position_control->set_pos_desired_U_cm(0);
    // 记录当前航向作为初始保持航向
    sub.last_pilot_heading_rad = ahrs.get_yaw_rad();

    return true;
    return true;
}

void ModeStabilize::run()
{
  uint32_t tnow = AP_HAL::millis();
    float target_roll, target_pitch;

    // 未解锁时归零输出并释放姿态控制器
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.last_pilot_heading_rad = ahrs.get_yaw_rad();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 将飞手摇杆输入转换为目标倾斜角（横滚、俯仰）
    // 注意：角度单位为 centi-degrees（百分度）
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->lean_angle_max_cd());

    // 将偏航摇杆输入（含死区）转换为偏航角速率
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * sub.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = sub.get_pilot_desired_yaw_rate(yaw_input);

    // 偏航控制逻辑：
    if (!is_zero(target_yaw_rate)) {
        // 飞手有偏航输入：按角速率模式控制偏航
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
        sub.last_pilot_heading_rad = ahrs.get_yaw_rad();
        sub.last_pilot_yaw_input_ms = tnow; // 记录最后一次偏航输入时间

    } else {
        // 飞手无偏航输入：保持当前航向

        // 此检查用于防止快速偏航后的回弹
        // 潜器的惯性会让航向在飞手停止输入后短暂继续偏转
        if (tnow < sub.last_pilot_yaw_input_ms + 250) {
            // 250ms 内仍以角速率=0 减速，避免目标立即跳变
            target_yaw_rate = 0;
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
            sub.last_pilot_heading_rad = ahrs.get_yaw_rad(); // 持续更新保持航向

        } else {
            // 超过 250ms 后锁定到目标航向（绝对角度控制）
            attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll, target_pitch, rad_to_cd(sub.last_pilot_heading_rad), true);
        }
    }

    // 输出飞手油门（从 [-1,1] 映射到 [0,1]）
    attitude_control->set_throttle_out((channel_throttle->norm_input() + 1.0f) / 2.0f, false, g.throttle_filt);

    // 前进/横移由飞手直接控制（control_in 范围 -1000~1000）
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
