// mode_circle.cpp - 圆形绕飞模式（Circle Mode）
// 潜器绕以当前位置为中心的圆形轨迹运动
// 圆心在进入模式时自动设置（基于当前速度方向）
// 飞手可以：
//   - 用偏航摇杆覆盖自动偏航（朝向圆心）
//   - 用油门控制深度
//   - 半径和角速度由 CIRCLE_RADIUS 和 CIRCLE_RATE 参数控制

#include "Sub.h"

// circle_init - 初始化圆形导航控制器
bool ModeCircle::init(bool ignore_checks)
{
    if (!sub.position_ok()) {
        return false;
    }

    sub.circle_pilot_yaw_override = false;  // 初始不允许飞手覆盖偏航

    // 初始化水平和垂直速度/加速度限制（所有限制必须为正值）
    position_control->NE_set_max_speed_accel_cm(sub.wp_nav.get_default_speed_NE_cms(), sub.wp_nav.get_wp_acceleration_cmss());
    position_control->NE_set_correction_speed_accel_cm(sub.wp_nav.get_default_speed_NE_cms(), sub.wp_nav.get_wp_acceleration_cmss());
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->D_set_correction_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // 初始化圆形控制器（根据当前速度自动设置圆心位置）
    sub.circle_nav.init();

    return true;
}

// circle_run - 运行圆形导航控制器（需 100Hz 或更高频率调用）
void ModeCircle::run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // 实时更新速度/加速度限制（允许运行时修改参数）
    position_control->NE_set_max_speed_accel_cm(sub.wp_nav.get_default_speed_NE_cms(), sub.wp_nav.get_wp_acceleration_cmss());
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // 检查参数变化并实时更新圆形控制器
    sub.circle_nav.check_param_change();

    // 未解锁：禁用推进器，放松姿态控制器，重置圆形控制器
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.circle_nav.init();
        return;
    }

    // 处理飞手偏航输入（飞手可以覆盖自动朝向圆心的偏航）
    target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    if (!is_zero(target_yaw_rate)) {
        sub.circle_pilot_yaw_override = true;  // 标记飞手正在覆盖偏航
    }

    // get pilot desired climb rate
    target_climb_rate = sub.get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run circle controller
    sub.failsafe_terrain_set_status(sub.circle_nav.update_cms());

    ///////////////////////
    // update xy outputs //

    float lateral_out, forward_out;
    sub.translate_circle_nav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call attitude controller
    if (sub.circle_pilot_yaw_override) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), sub.circle_nav.get_yaw_cd(), true);
    }

    // update altitude target and call position controller
    position_control->D_set_pos_target_from_climb_rate_cms(target_climb_rate);
    position_control->D_update_controller();
}
