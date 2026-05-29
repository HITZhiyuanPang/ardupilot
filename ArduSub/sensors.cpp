// sensors.cpp - ArduSub 传感器读取和处理
// 包含：
//   - read_barometer：读取深度传感器（外部气压计）
//   - init_rangefinder：初始化测距仪（声纳/激光）
//   - read_rangefinder：读取测距仪并更新海底高度估计
//   - read_power_monitor：电源监控
//   - compass/optical flow 更新

#include "Sub.h"

// read_barometer - 读取气压计（水下用作深度传感器）
// 对于 ROV，气压计实际连接的是外部水压传感器
// 如果读数为正值（表示在水面以上），则触发重新校准
void Sub::read_barometer()
{
    barometer.update();
    // 正深度读数说明传感器未完全浸入水中，重新校准基准值
    // 水面以上即使几米也应该没有显著深度读数
    if(barometer.get_altitude() > 0) {
        barometer.update_calibration();
    }

    if (ap.depth_sensor_present) {
        sensor_health.depth = barometer.healthy(depth_sensor_idx);
    }
}

// init_rangefinder - 初始化测距仪（向下朝向，用于海底距离测量）
void Sub::init_rangefinder()
{
#if AP_RANGEFINDER_ENABLED
    rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
    // 初始化为向下朝向（ROTATION_PITCH_270 = 朝向海底）
    rangefinder.init(ROTATION_PITCH_270);
    rangefinder_state.alt_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
    rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);
#endif
}

// read_rangefinder - 读取测距仪数据并提供给导航库
// 更新 rangefinder_state 结构体，包含：健康状态、高度、滤波值、地形偏移
void Sub::read_rangefinder()
{
#if AP_RANGEFINDER_ENABLED
    rangefinder.update();

    // 信号质量范围 0（最差）到 100（最佳），-1 表示不支持
    int8_t signal_quality_pct = rangefinder.signal_quality_pct_orient(ROTATION_PITCH_270);

    // 判断测距仪是否健康：状态 Good + 连续有效读数超过阈值 + 信号质量足够
    rangefinder_state.alt_healthy =
            (rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) &&
            (rangefinder.range_valid_count_orient(ROTATION_PITCH_270) >= RANGEFINDER_HEALTH_MAX) &&
            (signal_quality_pct == -1 || signal_quality_pct >= g.rangefinder_signal_min);

    float temp_alt_m = rangefinder.distance_orient(ROTATION_PITCH_270);

#if RANGEFINDER_TILT_CORRECTION
    // 当潜器倾斜时补偿测距仪读数（将斜距转换为垂直距离）
    temp_alt_m = temp_alt_m * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
#endif

    rangefinder_state.alt = temp_alt_m;
    rangefinder_state.inertial_alt_cm = pos_control.get_pos_estimate_U_m() * 100.0f;
    rangefinder_state.min = rangefinder.min_distance_orient(ROTATION_PITCH_270);
    rangefinder_state.max = rangefinder.max_distance_orient(ROTATION_PITCH_270);

    // 计算 rangefinder_terrain_offset_cm（EKF 原点到海底的高度偏移）
    if (rangefinder_state.alt_healthy) {
        uint32_t now = AP_HAL::millis();
        if (now - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS) {
            // 超过超时时间未使用，重置滤波器避免陈旧数据影响
            rangefinder_state.alt_filt.reset(rangefinder_state.alt);
        } else {
            // 低通滤波，减少测距仪噪声对导航的影响
            rangefinder_state.alt_filt.apply(rangefinder_state.alt, 0.05f);
        }
        rangefinder_state.last_healthy_ms = now;
        // 地形偏移 = EKF 估计高度 - 测距仪滤波高度
        rangefinder_state.rangefinder_terrain_offset_cm =
            sub.rangefinder_state.inertial_alt_cm - (sub.rangefinder_state.alt_filt.get() * 100);
    }

    // 将测距仪数据提供给航点导航库和圆形导航库，用于地形跟随
    wp_nav.set_rangefinder_terrain_U_cm(
            rangefinder_state.enabled,
            rangefinder_state.alt_healthy,
            rangefinder_state.rangefinder_terrain_offset_cm);
    circle_nav.set_rangefinder_terrain_U_cm(
            rangefinder_state.enabled && wp_nav.rangefinder_used(),
            rangefinder_state.alt_healthy,
            rangefinder_state.rangefinder_terrain_offset_cm);
#endif  // AP_RANGEFINDER_ENABLED
}

// return true if rangefinder_alt can be used
bool Sub::rangefinder_alt_ok() const
{
    uint32_t now = AP_HAL::millis();
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy && now - rangefinder_state.last_healthy_ms < RANGEFINDER_TIMEOUT_MS);
}
