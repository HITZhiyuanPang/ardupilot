// surface_bottom_detector.cpp - 水面/海底接触检测
// 作者：Jacob Walser <jacob@bluerobotics.com>
//
// 使用深度传感器（优先）或速度估计来检测是否到达水面或海底：
//   - ap.at_surface：当深度小于 SURFACE_DEPTH 参数时置位
//   - ap.at_bottom：当推力触底限制触发且速度接近零时，经过 BOTTOM_DETECTOR_TRIGGER_SEC 秒后置位
// 每个主循环帧调用（MAIN_LOOP_RATE Hz）

#include "Sub.h"

// 检测计数器（需要连续若干帧满足条件才触发，避免瞬时误判）
static uint32_t bottom_detector_count = 0;
static uint32_t surface_detector_count = 0;
static float current_depth = 0;  // 当前深度（m，负值表示水面以下）

// update_surface_and_bottom_detector - 更新水面/海底检测状态
// 仅在解锁时运行
void Sub::update_surface_and_bottom_detector()
{
    if (!motors.armed()) { // 解锁后才进行检测
        set_surfaced(false);
        set_bottomed(false);
        return;
    }

    Vector3f velocity;
    UNUSED_RESULT(ahrs.get_velocity_NED(velocity));

    // 判断是否处于垂直静止状态（速度绝对值 < 0.05 m/s）
    bool vel_stationary = velocity.z > -0.05 && velocity.z < 0.05;

    if (ap.depth_sensor_present && sensor_health.depth) {
        // 有外部深度传感器：精确判断水面和海底
        current_depth = barometer.get_altitude(); // m（正值=水面以上，负值=水面以下）

        // 水面检测：添加 5cm 滞后防止频繁切换
        if (ap.at_surface) {
            set_surfaced(current_depth > g.surface_depth*0.01 - 0.05);
        } else {
            set_surfaced(current_depth > g.surface_depth*0.01);
        }

        // 海底检测：推力下限触发（电机已到最低推力）且速度接近零
        if (motors.limit.throttle_lower && vel_stationary) {
            if (bottom_detector_count < ((float)BOTTOM_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
                bottom_detector_count++;
            } else {
                set_bottomed(true);
            }
        } else {
            set_bottomed(false);
        }

    // 无外部深度传感器：仅靠垂直速度估计
    } else if (vel_stationary) {
        if (motors.limit.throttle_upper) {

            // 上行推力触限且静止：可能到达水面，开始计数
            if (surface_detector_count < ((float)SURFACE_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
                surface_detector_count++;
            } else {
                set_surfaced(true);
            }

        } else if (motors.limit.throttle_lower) {
            // bottom criteria met, increment counter and see if we've triggered
            if (bottom_detector_count < ((float)BOTTOM_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
                bottom_detector_count++;
            } else {
                set_bottomed(true);
            }

        } else { // we're not at the limits of throttle, so reset both detectors
            set_surfaced(false);
            set_bottomed(false);
        }

    } else { // we're moving up or down, so reset both detectors
        set_surfaced(false);
        set_bottomed(false);
    }
}

void Sub::set_surfaced(bool at_surface)
{


    if (ap.at_surface == at_surface) { // do nothing if state unchanged
        return;
    }

    ap.at_surface = at_surface;

    surface_detector_count = 0;

    if (ap.at_surface) {
        LOGGER_WRITE_EVENT(LogEvent::SURFACED);
    } else {
        LOGGER_WRITE_EVENT(LogEvent::NOT_SURFACED);
    }
}

void Sub::set_bottomed(bool at_bottom)
{

    if (ap.at_bottom == at_bottom) { // do nothing if state unchanged
        return;
    }

    ap.at_bottom = at_bottom;

    bottom_detector_count = 0;

    if (ap.at_bottom) {
        LOGGER_WRITE_EVENT(LogEvent::BOTTOMED);
    } else {
        LOGGER_WRITE_EVENT(LogEvent::NOT_BOTTOMED);
    }
}
