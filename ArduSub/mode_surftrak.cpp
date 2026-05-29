// mode_surftrak.cpp - 海底距离跟踪模式（Surface Tracking）
// 基于 ALT_HOLD 的变种，使用测距仪（声纳）保持与海底的固定距离
//
// 工作原理：
//   1. 初始状态为"重置"（rangefinder_target_cm < 0）
//   2. 当满足以下条件时退出重置状态：
//      - 测距仪读数有效（健康、在量程内等）
//      - 潜器深度超过 SURFTRAK_DEPTH 参数
//   3. 正常工作时，将地形偏移目标设为当前测距仪读数，由 AC_PosControl 控制深度
//
// 设计原则：
//   - 测距仪故障时不重置目标（避免因故障导致目标跳变）
//   - 飞手手动控制深度时，同步调整测距仪目标（减少"弹回"效应）
//   - 测距仪响应比气压计慢，深度变化由气压计实时跟踪，测距仪用于微调

#include "Sub.h"

// INVALID_TARGET 表示测距仪目标尚未建立
#define INVALID_TARGET (-1)
#define HAS_VALID_TARGET (rangefinder_target_cm > 0)

// 构造函数：初始化测距仪目标为无效值
ModeSurftrak::ModeSurftrak() :
        rangefinder_target_cm(INVALID_TARGET),
        pilot_in_control(false),
        pilot_control_start_z_cm(0)
{ }

bool ModeSurftrak::init(bool ignore_checks)
{
    // 先调用 ALT_HOLD 初始化（需要深度传感器）
    if (!ModeAlthold::init(ignore_checks)) {
        return false;
    }

    reset(); // 清除旧的测距仪目标

    // 提示用户当前状态
    if (!sub.rangefinder_alt_ok()) {
        sub.gcs().send_text(MAV_SEVERITY_INFO, "waiting for a rangefinder reading");
#if AP_RANGEFINDER_ENABLED
    } else if (position_control->get_pos_estimate_U_m() * 100.0f >= sub.g.surftrak_depth) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "descend below %f meters to hold range", sub.g.surftrak_depth * 0.01f);
#endif
    }

    return true;
}

void ModeSurftrak::run()
{
    run_pre(); // 姿态控制（继承自 ALT_HOLD）

    if (!motors.armed()) {
        // 未解锁时清除测距仪目标，防止重新解锁后使用过期目标
        reset();
    } else {
        control_range(); // 海底距离控制
    }

    run_post(); // 输出（继承自 ALT_HOLD）
}

/*
 * set_rangefinder_target_cm - 设置测距仪目标距离（可由脚本调用）
 * 返回 true 表示设置成功
 */
bool ModeSurftrak::set_rangefinder_target_cm(float target_cm)
{
    bool success = false;

#if AP_RANGEFINDER_ENABLED
    // 安全检查：模式、深度、量程范围
    if (sub.control_mode != Number::SURFTRAK) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "wrong mode, rangefinder target not set");
    } else if (position_control->get_pos_estimate_U_m() * 100.0f >= sub.g.surftrak_depth) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "descend below %f meters to set rangefinder target", sub.g.surftrak_depth * 0.01f);
    } else if (target_cm < sub.rangefinder_state.min*100) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "rangefinder target below minimum, ignored");
    } else if (target_cm > sub.rangefinder_state.max*100) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "rangefinder target above maximum, ignored");
    } else {
        success = true;
    }

    if (success) {
        rangefinder_target_cm = target_cm;
        sub.gcs().send_text(MAV_SEVERITY_INFO, "rangefinder target is %.2f meters", rangefinder_target_cm * 0.01f);

        // Initialize the terrain offset
        auto terrain_offset_cm = position_control->get_pos_estimate_U_m() * 100.0f - rangefinder_target_cm;
        sub.pos_control.init_pos_terrain_U_cm(terrain_offset_cm);

    } else {
        reset();
    }
#endif

    return success;
}

void ModeSurftrak::reset()
{
    rangefinder_target_cm = INVALID_TARGET;

    // Reset the terrain offset
    sub.pos_control.init_pos_terrain_U_cm(0);
}

/*
 * Main controller, call at 100hz+
 */
void ModeSurftrak::control_range() {
    float target_climb_rate_cms = sub.get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate_cms = constrain_float(target_climb_rate_cms, -sub.get_pilot_speed_dn(), g.pilot_speed_up);

    // Desired_climb_rate returns 0 when within the deadzone
    if (fabsf(target_climb_rate_cms) < 0.05f)  {
        if (pilot_in_control) {
            // Pilot has released control; apply the delta to the rangefinder target
            set_rangefinder_target_cm(rangefinder_target_cm + position_control->get_pos_estimate_U_m() * 100.0f - pilot_control_start_z_cm);
            pilot_in_control = false;
        }
        if (sub.ap.at_surface) {
            // Set target depth to 5 cm below SURFACE_DEPTH and reset
            position_control->set_pos_desired_U_cm(MIN(position_control->get_pos_desired_U_cm(), g.surface_depth - 5.0f));
            reset();
        } else if (sub.ap.at_bottom) {
            // Set target depth to 10 cm above bottom and reset
            position_control->set_pos_desired_U_cm(MAX(position_control->get_pos_estimate_U_m() * 100.0f + 10.0f, position_control->get_pos_desired_U_cm()));
            reset();
        } else {
            // Typical operation
            update_surface_offset();
        }
    } else if (HAS_VALID_TARGET && !pilot_in_control) {
        // Pilot has taken control; note the current depth
        pilot_control_start_z_cm = position_control->get_pos_estimate_U_m() * 100.0f;
        pilot_in_control = true;
    }

    // Set the target altitude from the climb rate and the terrain offset
    position_control->D_set_pos_target_from_climb_rate_cms(target_climb_rate_cms);

    // Run the PID controllers
    position_control->D_update_controller();
}

/*
 * Update the AC_PosControl terrain offset if we have a good rangefinder reading
 */
void ModeSurftrak::update_surface_offset()
{
#if AP_RANGEFINDER_ENABLED
    if (sub.rangefinder_alt_ok()) {
        // Get the latest terrain offset
        float rangefinder_terrain_offset_cm = sub.rangefinder_state.rangefinder_terrain_offset_cm;

        // Handle the first reading or a reset
        if (!HAS_VALID_TARGET && sub.rangefinder_state.inertial_alt_cm < sub.g.surftrak_depth) {
            set_rangefinder_target_cm(sub.rangefinder_state.inertial_alt_cm - rangefinder_terrain_offset_cm);
        }

        if (HAS_VALID_TARGET) {
            // Will the new offset target cause the sub to ascend above SURFTRAK_DEPTH?
            float desired_z_cm = rangefinder_terrain_offset_cm + rangefinder_target_cm;
            if (desired_z_cm >= sub.g.surftrak_depth) {
                // Adjust the terrain offset to stay below SURFTRAK_DEPTH, this should avoid "at_surface" events
                rangefinder_terrain_offset_cm += sub.g.surftrak_depth - desired_z_cm;
            }

            // Set the offset target, AC_PosControl will do the rest
            sub.pos_control.set_pos_terrain_target_U_cm(rangefinder_terrain_offset_cm);
        }
    }
#endif  // AP_RANGEFINDER_ENABLED
}
