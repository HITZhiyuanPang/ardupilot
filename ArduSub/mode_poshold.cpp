// mode_poshold.cpp - GPS 位置保持模式（Position Hold）
// 需要 GPS 锁定
// 飞手控制姿态/偏航/深度，飞控自动保持水平位置（经纬度）
// 适合在水下定点悬停观察目标
// 作者：Jacob Walser，2016 年 8 月

#include "Sub.h"

#if POSHOLD_ENABLED

// poshold_init - 初始化位置保持控制器
bool ModePoshold::init(bool ignore_checks)
{
    // 没有 GPS 锁定则无法进入位置保持模式
    if (!sub.position_ok()) {
        return false;
    }

    // 初始化水平和垂直速度/加速度限制（所有限制必须为正值）
    position_control->NE_set_max_speed_accel_cm(g.pilot_speed, g.pilot_accel_z);
    position_control->NE_set_correction_speed_accel_cm(g.pilot_speed, g.pilot_accel_z);
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->D_set_correction_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // 初始化位置控制器（从停止点开始，锁定当前位置为目标）
    position_control->NE_init_controller_stopping_point();
    position_control->D_init_controller();

    // 停止所有推进器，放松姿态控制器
    attitude_control->set_throttle_out(NEUTRAL_THROTTLE ,true, g.throttle_filt);
    attitude_control->relax_attitude_controllers();
    position_control->D_relax_controller(0.5f);

    sub.last_pilot_heading_rad = ahrs.get_yaw_rad();

    return true;
}

// poshold_run - 运行位置保持控制器（需 100Hz 或更高调用）
void ModePoshold::run()
{
    uint32_t tnow = AP_HAL::millis();
    // 未解锁：禁用推进器并放松稳定
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE ,true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->NE_init_controller_stopping_point();
        position_control->D_relax_controller(0.5f);
        sub.last_pilot_heading_rad = ahrs.get_yaw_rad();
        return;
    }

    // 使能推进器全量程输出
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    /////////////////////
    // 姿态控制 //

    // 获取飞手偏航速率输入（考虑死区和增益）
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * sub.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = sub.get_pilot_desired_yaw_rate(yaw_input);

    // 将飞手输入转换为倾斜角
    float target_roll, target_pitch;
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->lean_angle_max_cd());

    // 偏航逻辑：有输入则跟踪速率，无输入则保持上次航向（含 250ms 防回弹）
    if (!is_zero(target_yaw_rate)) { // 有飞手偏航输入：跟踪角速率
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
        sub.last_pilot_heading_rad = ahrs.get_yaw_rad();
        sub.last_pilot_yaw_input_ms = tnow;

    } else { // 无偏航输入：保持当前航向

        // 250ms 内继续减速（车辆惯性导致实际停止稍晚于输入停止）
        if (tnow < sub.last_pilot_yaw_input_ms + 250) {
            target_yaw_rate = 0;
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
            sub.last_pilot_heading_rad = ahrs.get_yaw_rad();

        } else { // 保持绝对航向角
            attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll, target_pitch, rad_to_cd(sub.last_pilot_heading_rad), true);
        }
    }

    // 更新垂直轴（深度控制）
    control_depth();

    // 更新水平轴（位置控制，在深度更新后调用以获得正确的 THR_DZ）
    control_horizontal();
}

void ModePoshold::control_horizontal() {
    float lateral_out = 0;
    float forward_out = 0;

    // get desired rates in the body frame
    Vector2f body_rates_cms = {
        sub.get_pilot_desired_horizontal_rate(channel_forward),
        sub.get_pilot_desired_horizontal_rate(channel_lateral)
    };

    if (sub.position_ok()) {
        if (!position_control->NE_is_active()) {
            // the xy controller timed out, re-initialize
            position_control->NE_init_controller_stopping_point();
        }

        // convert to the earth frame and set target rates
        auto earth_rates_cms = ahrs.body_to_earth2D(body_rates_cms);
        position_control->input_vel_accel_NE_cm(earth_rates_cms, {0, 0});

        // convert pos control roll and pitch angles back to lateral and forward efforts
        sub.translate_pos_control_rp(lateral_out, forward_out);

        // update the xy controller
        position_control->NE_update_controller();
    } else if (g.pilot_speed > 0) {
        // allow the pilot to reposition manually
        forward_out = body_rates_cms.x / (float)g.pilot_speed;
        lateral_out = body_rates_cms.y / (float)g.pilot_speed;
    }

    motors.set_forward(forward_out);
    motors.set_lateral(lateral_out);
}
#endif  // POSHOLD_ENABLED
