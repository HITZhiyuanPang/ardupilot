// mode_acro.cpp - 特技模式（Acro Mode）
// 飞手直接控制角速率（体轴系），无自稳，适合高级操控
// 横滚/俯仰摇杆输入 → 目标角速率；偏航同样直接控制角速率
// 油门直通，前进/横移直通（不受姿态影响）

#include "Sub.h"


bool ModeAcro::init(bool ignore_checks) {
    // 报告用目标深度归零
    position_control->set_pos_desired_U_cm(0);

    // 特技模式下姿态控制输入变为推力输入，进入时先置中立避免乱舞
    sub.set_neutral_controls();

    return true;
}

void ModeAcro::run()
{
    float target_roll, target_pitch, target_yaw;

    // 未解锁：输出中性油门，放松姿态控制器，等待解锁
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 将摇杆输入转换为目标体轴角速率（centi-deg/s）
    get_pilot_desired_angle_rates(channel_roll->get_control_in(), channel_pitch->get_control_in(), channel_yaw->get_control_in(), target_roll, target_pitch, target_yaw);

    // 运行角速率控制器（直接控制体轴角速率，无外环姿态稳定）
    attitude_control->input_rate_bf_roll_pitch_yaw_cds(target_roll, target_pitch, target_yaw);

    // 油门直通（不加姿态增益补偿）：norm_input 范围 -1~+1，转换为 0~1
    attitude_control->set_throttle_out((channel_throttle->norm_input() + 1.0f) / 2.0f, false, g.throttle_filt);

    // 前进/横移直通（control_in 范围 0-1000，norm_input 归一化到 -1~+1）
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
