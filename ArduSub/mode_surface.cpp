// mode_surface.cpp - 上浮模式（Surface Mode）
// 自动上浮至水面，到达水面后自动切换至高度保持模式
// 如果没有气压计（nobaro_mode），使用固定推力参数上浮

#include "Sub.h"


bool ModeSurface::init(bool ignore_checks)
{
    // 检查深度传感器是否可用（nobaro_mode = 无深度传感器）
    nobaro_mode = !sub.control_check_barometer();

    // 初始化垂直速度和加速度限制（所有限制必须为正值）
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->D_set_correction_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // 初始化位置控制器（锁定当前深度作为起始位置）
    position_control->D_init_controller();

    return true;

}

void ModeSurface::run()
{
    float target_roll, target_pitch;

    // 未解锁：停止电机并等待
    if (!motors.armed()) {
        motors.output_min();
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->D_init_controller();
        return;
    }

    // 无气压计模式：使用固定推力参数上浮（SURFACE_NOBARO_THRUST 参数控制推力大小）
    if (nobaro_mode) {
        float thrust_output = 0.5f + g2.surface_nobaro_thrust * 0.005f; // 将 -100~100 映射到 0~1
        attitude_control->set_throttle_out(thrust_output, true, g.throttle_filt);
    } else {
        // 已到达水面：切换至高度保持模式
        if (sub.ap.at_surface) {
            set_mode(Mode::Number::ALT_HOLD, ModeReason::SURFACE_COMPLETE);
        }

        // 将飞手输入转换为倾斜角（在上浮过程中飞手仍可控制姿态）
        sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->lean_angle_max_cd());

        // 获取飞手期望偏航速率
        float target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // 运行姿态控制器
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);

        // 设置目标上升速度（以 WP 默认上升速度为上限，最小 1 cm/s）
        float cmb_rate_cms = constrain_float(fabsf(sub.wp_nav.get_default_speed_up_cms()), 1, position_control->get_max_speed_up_cms());

        // 更新高度目标并运行位置控制器
        position_control->D_set_pos_target_from_climb_rate_cms(cmb_rate_cms);
        position_control->D_update_controller();
    }
    // pilot has control for repositioning
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
