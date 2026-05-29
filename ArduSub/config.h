// config.h - ArduSub 编译时硬件和功能配置
// 所有可覆盖的默认值均使用 #ifndef 保护，可在 APM_Config.h 中针对特定硬件覆盖

#pragma once

#include "defines.h"

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef CONFIG_HAL_BOARD
#error CONFIG_HAL_BOARD must be defined to build ArduSub
#endif

// 主循环运行频率（Hz）：所有平台统一 400Hz
# define MAIN_LOOP_RATE    400

// 水面深度阈值：传感器读数高于此值（单位 m，负数表示水面以下）即判定为在水面
// 默认 -0.10m（水面以下 10cm）
#ifndef SURFACE_DEPTH_DEFAULT
# define SURFACE_DEPTH_DEFAULT -10.0f // pressure sensor reading 10cm depth means craft is considered surfaced
#endif

//////////////////////////////////////////////////////////////////////////////
// PWM 控制
// 默认 RC 输出频率（Hz）
#ifndef RC_SPEED_DEFAULT
#   define RC_SPEED_DEFAULT 200
#endif

//////////////////////////////////////////////////////////////////////////////
// 圆形航迹导航参数
//

#ifndef CIRCLE_NAV_ENABLED
# define CIRCLE_NAV_ENABLED 1   // 启用圆形航迹导航功能
#endif

//////////////////////////////////////////////////////////////////////////////
// RC 输入支持
//

#ifndef AP_SUB_RC_ENABLED
# define AP_SUB_RC_ENABLED 1    // 启用遥控器输入（可用于禁用以节省 flash）
#endif
#ifndef RCMAP_ENABLED
# define RCMAP_ENABLED AP_SUB_RC_ENABLED  // RC 通道映射与 RC 同步启停
#endif

//////////////////////////////////////////////////////////////////////////////
// 油门故障保护
// 遥控信号丢失时的油门 PWM 阈值，低于此值触发故障保护
#ifndef FS_THR_VALUE_DEFAULT
 # define FS_THR_VALUE_DEFAULT             975
#endif


//////////////////////////////////////////////////////////////////////////////
// 测距仪（Rangefinder）配置
//

// 连续有效读数次数阈值，超过此数才认为测距仪健康
#ifndef RANGEFINDER_HEALTH_MAX
# define RANGEFINDER_HEALTH_MAX 3          // number of good reads that indicates a healthy rangefinder
#endif

// 测距仪高度超时（ms）：超过此时间无有效读数则重置滤波器
#ifndef RANGEFINDER_TIMEOUT_MS
# define RANGEFINDER_TIMEOUT_MS  1000      // desired rangefinder alt will reset to current rangefinder alt after this many milliseconds without a good rangefinder alt
#endif

// 提供给航点导航库的测距仪高度低通滤波截止频率（Hz）
#ifndef RANGEFINDER_WPNAV_FILT_HZ
# define RANGEFINDER_WPNAV_FILT_HZ   0.25f // filter frequency for rangefinder altitude provided to waypoint navigation class
#endif

// 是否对测距仪数据进行倾斜补偿（0=禁用，适合用于EKF数据融合）
#ifndef RANGEFINDER_TILT_CORRECTION        // by disable tilt correction for use of range finder data by EKF
# define RANGEFINDER_TILT_CORRECTION 0
#endif

// 信号质量低于此百分比的测距仪读数将被忽略
#ifndef RANGEFINDER_SIGNAL_MIN_DEFAULT
# define RANGEFINDER_SIGNAL_MIN_DEFAULT 90 // rangefinder readings with signal quality below this value are ignored
#endif

// SurfTrak 模式的最大深度限制（m，负值）：超过此深度不再进行海底跟踪
#ifndef SURFTRAK_DEPTH_DEFAULT
# define SURFTRAK_DEPTH_DEFAULT -50.0f     // surftrak will try to keep the sub below this depth
#endif

// 障碍物规避功能（依赖 Proximity 近距传感器和 Fence 电子围栏）
#ifndef AVOIDANCE_ENABLED
# define AVOIDANCE_ENABLED 0              // 默认禁用（嵌入式资源有限）
#endif

#if AVOIDANCE_ENABLED // Avoidance Library relies on Fence
# define FENCE_ENABLED 1                  // 开启规避时自动启用电子围栏
#endif

// MAVLink 系统 ID
#ifndef MAV_SYSTEM_ID
# define MAV_SYSTEM_ID          1
#endif

//////////////////////////////////////////////////////////////////////////////
// 外部导航计算机（Nav-Guided）：允许外部计算机通过 MAVLink 发送导航命令
#ifndef NAV_GUIDED
# define NAV_GUIDED    1
#endif

//////////////////////////////////////////////////////////////////////////////
// 飞行模式默认参数
//

// Acro 模式：滚转/俯仰角速率 P 增益
#ifndef ACRO_RP_P
# define ACRO_RP_P                 4.5f
#endif

// Acro 模式：偏航角速率 P 增益
#ifndef ACRO_YAW_P
# define ACRO_YAW_P                3.375f
#endif

// Acro 训练模式允许的最大倾斜角（centi-degrees）
#ifndef ACRO_LEVEL_MAX_ANGLE
# define ACRO_LEVEL_MAX_ANGLE      3000
#endif

// Acro 训练模式中自动恢复水平的强度（滚转）
#ifndef ACRO_BALANCE_ROLL
#define ACRO_BALANCE_ROLL          1.0f
#endif

// Acro 训练模式中自动恢复水平的强度（俯仰）
#ifndef ACRO_BALANCE_PITCH
#define ACRO_BALANCE_PITCH         1.0f
#endif

// Acro 摇杆指数曲线（0=线性，1=全指数；使低速操控更精细）
#ifndef ACRO_EXPO_DEFAULT
#define ACRO_EXPO_DEFAULT          0.3f
#endif

// AUTO Mode
#ifndef WP_YAW_BEHAVIOR_DEFAULT
# define WP_YAW_BEHAVIOR_DEFAULT   WP_YAW_BEHAVIOR_CORRECT_XTRACK
#endif

#ifndef AUTO_YAW_SLEW_RATE
# define AUTO_YAW_SLEW_RATE    60              // degrees/sec
#endif

#ifndef YAW_LOOK_AHEAD_MIN_SPEED
# define YAW_LOOK_AHEAD_MIN_SPEED  100             // minimum ground speed in cm/s required before vehicle is aimed at ground course
#endif

//////////////////////////////////////////////////////////////////////////////
// Stabilize Rate Control
//
#ifndef ROLL_PITCH_INPUT_MAX
# define ROLL_PITCH_INPUT_MAX      4500            // roll, pitch input range
#endif
#ifndef DEFAULT_ANGLE_MAX
# define DEFAULT_ANGLE_MAX         4500            // ANGLE_MAX parameters default value
#endif

//////////////////////////////////////////////////////////////////////////////
// Loiter position control gains
//
#ifndef POS_XY_P
# define POS_XY_P               1.0f
#endif

//////////////////////////////////////////////////////////////////////////////
// PosHold parameter defaults
//
#ifndef POSHOLD_ENABLED
# define POSHOLD_ENABLED               1 // PosHold flight mode enabled by default
#endif

//////////////////////////////////////////////////////////////////////////////
// Throttle control gains
//

#ifndef THR_DZ_DEFAULT
# define THR_DZ_DEFAULT         100             // the deadzone above and below mid throttle while in althold or loiter
#endif

// default maximum velocities and acceleration the pilot may request
#ifndef PILOT_VELZ_MAX
# define PILOT_VELZ_MAX    500     // maximum vertical velocity in cm/s
#endif
#ifndef PILOT_SPEED_DEFAULT
# define PILOT_SPEED_DEFAULT 200 // maximum horizontal velocity in cm/s while under pilot control
#endif
#ifndef PILOT_ACCEL_Z_DEFAULT
# define PILOT_ACCEL_Z_DEFAULT 100 // vertical acceleration in cm/s/s while altitude is under pilot control
#endif

#ifndef AUTO_DISARMING_DELAY
# define AUTO_DISARMING_DELAY  0
#endif

#ifndef NEUTRAL_THROTTLE
# define NEUTRAL_THROTTLE (0.5f)   // Throttle output for "no vertical thrust"
#endif

//////////////////////////////////////////////////////////////////////////////
// Logging control
//

// Default logging bitmask
#ifndef DEFAULT_LOG_BITMASK
# define DEFAULT_LOG_BITMASK \
    MASK_LOG_ATTITUDE_MED | \
    MASK_LOG_GPS | \
    MASK_LOG_PM | \
    MASK_LOG_CTUN | \
    MASK_LOG_NTUN | \
    MASK_LOG_RCIN | \
    MASK_LOG_IMU | \
    MASK_LOG_CMD | \
    MASK_LOG_CURRENT | \
    MASK_LOG_RCOUT | \
    MASK_LOG_OPTFLOW | \
    MASK_LOG_COMPASS | \
    MASK_LOG_CAMERA | \
    MASK_LOG_MOTBATT
#endif

//Default flight modes
#ifndef FLIGHT_MODE_1
# define FLIGHT_MODE_1 Mode::Number::MANUAL
#endif
#ifndef FLIGHT_MODE_2
# define FLIGHT_MODE_2 Mode::Number::MANUAL
#endif
#ifndef FLIGHT_MODE_3
# define FLIGHT_MODE_3 Mode::Number::STABILIZE
#endif
#ifndef FLIGHT_MODE_4
# define FLIGHT_MODE_4 Mode::Number::STABILIZE
#endif
#ifndef FLIGHT_MODE_5
# define FLIGHT_MODE_5 Mode::Number::SURFACE
#endif
#ifndef FLIGHT_MODE_6
# define FLIGHT_MODE_6 Mode::Number::SURFACE
#endif

