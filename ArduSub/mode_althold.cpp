// mode_althold.cpp - 高度保持模式（Altitude Hold）
// 飞手控制滚转/俯仰/偏航和垂直速率，飞控自动保持深度
// 需要深度传感器（气压计）
// 包含三段逻辑：run_pre（姿态控制）+ control_depth（深度控制）+ run_post
//
// 代码逻辑总览：
// 1) init(): 进入模式时检查深度传感器、初始化深度控制器，并锁定当前深度为初始目标值
// 2) run_pre(): 处理姿态控制
//    - 未解锁时：电机保持怠速，姿态/深度控制器放松
//    - 已解锁时：读取驾驶员滚转/俯仰/偏航输入，驱动姿态控制器
//    - 若近期收到外部 set_attitude_target_no_gps，则优先按外部姿态目标控制
// 3) control_depth(): 处理 Z 轴（深度）控制
//    - 驾驶员油门杆被解释为“期望升降速度”
//    - 若油门在死区内，则保持当前深度；靠近水面/海底时做保护修正
// 4) run_post(): 把前进/横移通道写入推进器，完成本周期控制输出

#include "Sub.h"


bool ModeAlthold::init(bool ignore_checks) {
    // ignore_checks 在这个模式下未使用；保留签名是为了和基类接口一致。
    (void)ignore_checks;

    // 检查深度传感器是否可用（高度保持模式依赖深度传感器）
    if(!sub.control_check_barometer()) {
        return false;
    }

    // 初始化垂直最大速度和加速度限制（所有限制必须为正值）
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->D_set_correction_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // 初始化深度控制器：以当前估计深度为目标，进入模式后不会立刻产生深度跳变。
    position_control->D_init_controller();

    // 记录进入模式时的航向，后续在驾驶员松开偏航杆后用于“航向保持”。
    sub.last_pilot_heading_rad = ahrs.get_yaw_rad();

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAlthold::run()
{
    // AltHold 主循环采用三段式：
    // 先处理姿态，再更新深度，最后补充平面推进输出。
    run_pre();
    control_depth();
    run_post();
}

void ModeAlthold::run_pre()
{
    uint32_t tnow = AP_HAL::millis();

    // 每个周期都刷新一遍垂直速度/加速度限制，确保参数变化后能立即生效。
    // 所有限幅都要求为正，方向由控制器内部处理。
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    if (!motors.armed()) {
        // 未解锁状态：
        // - 电机仅保持 GROUND_IDLE
        // - 油门输出回中
        // - 放松姿态与深度控制器，避免在地面/水面待机时积累控制量
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->D_relax_controller(motors.get_throttle_hover());
        // 同步更新“当前要保持的航向”为实时航向，避免解锁后瞬间拉回旧航向。
        sub.last_pilot_heading_rad = ahrs.get_yaw_rad();
        return;
    }

    // 已解锁：允许油门全范围输出。
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 读取驾驶员期望的滚转/俯仰目标。
    float target_roll, target_pitch;

    // 如果近期收到外部无 GPS 姿态目标（例如伴随计算机经 MAVLink 发送），
    // 则优先采用外部姿态目标，覆盖本地摇杆姿态输入。
    if (tnow - sub.set_attitude_target_no_gps.last_message_ms < 5000) {
        float target_yaw;
        Quaternion(
            sub.set_attitude_target_no_gps.packet.q
        ).to_euler(
            target_roll,
            target_pitch,
            target_yaw
        );
        target_roll = degrees(target_roll);
        target_pitch = degrees(target_pitch);
        target_yaw = degrees(target_yaw);

        // 外部姿态目标直接交给姿态控制器，以绝对欧拉角形式控制。
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll * 1e2f, target_pitch * 1e2f, target_yaw * 1e2f, true);
        return;
    }

    // 常规 AltHold：摇杆 roll/pitch 输入转换为目标倾角。
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->get_althold_lean_angle_max_cd());

    // 偏航通道在 AltHold 中解释为“目标偏航角速度”。
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * sub.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = sub.get_pilot_desired_yaw_rate(yaw_input);

    // 姿态控制的偏航部分采用“有输入时角速度控制，无输入时航向保持”的双模式设计。
    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        // 驾驶员正在主动打偏航：按目标偏航速率控制。
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
        sub.last_pilot_heading_rad = ahrs.get_yaw_rad();
        sub.last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // 这个延时判断用于防止“快速甩尾后立刻锁航向”造成的反弹：
        // 载体有惯性，驾驶员刚松杆时机体可能还会继续转一点点。
        if (tnow < sub.last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0; // Stop rotation on yaw axis

            // 先命令偏航角速度为 0，让机体自然减速。
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
            sub.last_pilot_heading_rad = ahrs.get_yaw_rad(); // update heading to hold

        } else { // call attitude controller holding absolute bearing
            // 惯性衰减完成后，切换为绝对航向保持。
            attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll, target_pitch, rad_to_cd(sub.last_pilot_heading_rad), true);
        }
    }
}

void ModeAlthold::run_post()
{
    // AltHold 只自动管姿态、偏航、深度；前进和横移仍直接由驾驶员控制。
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}

void ModeAlthold::control_depth() {
    // 接近水面时限制最大油门，避免因为深度控制器大幅输出而把潜器“顶出水面”。
    // 越靠近水面，可用最大油门越小；离开水面后逐渐恢复到 1.0。
    float distance_to_surface = (g.surface_depth - position_control->get_pos_estimate_U_m() * 100.0f) * 0.01f;
    distance_to_surface = constrain_float(distance_to_surface, 0.0f, 1.0f);
    motors.set_max_throttle(g.surface_max_throttle + (1.0f - g.surface_max_throttle) * distance_to_surface);

    // 油门杆在 AltHold 中不是直接 PWM，而是“期望垂直速度”。
    float target_climb_rate_cms = sub.get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate_cms = constrain_float(target_climb_rate_cms, -sub.get_pilot_speed_dn(), g.pilot_speed_up);

    // 油门回中（死区内）时，驾驶员等价于“没有垂直速度指令”，
    // 此时控制器应保持深度；但若检测到贴近水面/海底，则对目标深度做保护修正。
    if (fabsf(target_climb_rate_cms) < 0.05f)  {
        if (sub.ap.at_surface) {
            // 在水面附近，把目标深度限制在 surface_depth 之下，避免控制器继续上冲。
            position_control->set_pos_desired_U_cm(MIN(position_control->get_pos_desired_U_cm(), g.surface_depth)); // set target to 5 cm below surface level
        } else if (sub.ap.at_bottom) {
            // 在海底附近，把目标深度抬高一点，避免持续向下压底。
            position_control->set_pos_desired_U_cm(MAX(position_control->get_pos_estimate_U_m() * 100.0f + 10.0f, position_control->get_pos_desired_U_cm())); // set target to 10 cm above bottom
        }
    }

    // 用“目标垂直速度”积分更新目标深度，再由深度控制器闭环求出油门输出。
    position_control->D_set_pos_target_from_climb_rate_cms(target_climb_rate_cms);
    position_control->D_update_controller();
}
