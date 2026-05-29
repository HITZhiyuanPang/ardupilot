#include "GCS_Sub.h"

#include "Sub.h"

// 更新并上报 ArduSub 的传感器/控制器状态位（present/enabled/health）。
// 这些状态位会编码到 MAVLink SYS_STATUS 中，用于地面站显示与健康判断。
void GCS_Sub::update_vehicle_sensor_status_flags()
{
    // 模式无关的基础控制能力：角速度控制、姿态稳定、偏航控制。
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_enabled |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_health |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
        MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;

    // 仅在具备位置/深度闭环的模式下，声明 Z 与 XY 控制“已启用且健康”。
    switch (sub.control_mode) {
    case Mode::Number::ALT_HOLD:
    case Mode::Number::AUTO:
    case Mode::Number::GUIDED:
    case Mode::Number::CIRCLE:
    case Mode::Number::SURFACE:
    case Mode::Number::POSHOLD:
        // 深度/高度闭环（Z轴）
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        // 水平位置闭环（XY平面）
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    default:
        // 其余模式不强制声明位置闭环能力。
        break;
    }

    // 覆盖父类对 ABSOLUTE_PRESSURE 的处理：
    // 在 Sub 中仅认可“水压深度传感器”，不使用通用气压语义。
    control_sensors_present &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    control_sensors_enabled &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    if (sub.ap.depth_sensor_present) {
        // 只要深度传感器存在，即认为该能力 present/enabled。
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        if (sub.sensor_health.depth) {
            // 仅在深度传感器健康时置 health。
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        }
    }

#if AP_TERRAIN_AVAILABLE
    // 地形子系统状态映射到 TERRAIN 状态位。
    switch (sub.terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // 待办：当 Sub 真正使用 terrain 后，再恢复不健康状态的细分上报。
        //control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        //control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        //break;
    case AP_Terrain::TerrainStatusOK:
        // 当前实现下，Unhealthy 会落入此分支，按可用且健康处理。
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

#if AP_RANGEFINDER_ENABLED
    // 测距仪（通常为向下测距）状态映射到 LASER_POSITION 状态位。
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (sub.rangefinder_state.enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        // 仅当存在对应朝向（ROTATION_PITCH_270，向下）有效数据时标记健康。
        if (rangefinder && rangefinder->has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif
}

#if AP_LTM_TELEM_ENABLED
// 空实现：避免在 Sub 构建中链接 LTM 遥测模块。
void AP_LTM_Telem::init() {};
#endif
#if AP_DEVO_TELEM_ENABLED
// 空实现：避免在 Sub 构建中链接 Devo 遥测模块。
void AP_DEVO_Telem::init() {};
#endif
