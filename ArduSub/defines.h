// defines.h - ArduSub 全局常量与枚举定义
// 包含：底部/水面检测阈值、自动驾驶偏航模式、日志掩码、故障保护(Failsafe)动作定义等

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// 底部接触检测触发时间（秒）：速度静止且油门下限持续此时间后判定触底
#define BOTTOM_DETECTOR_TRIGGER_SEC 1.0
// 水面检测触发时间（秒）：速度静止且深度传感器读数 > surface_depth 持续此时间后判定已浮出
#define SURFACE_DETECTOR_TRIGGER_SEC 1.0

// 自动上浮状态机枚举（用于 AUTO 模式下的浮出水面任务）
enum AutoSurfaceState {
    AUTO_SURFACE_STATE_GO_TO_LOCATION,   // 先导航至目标水平位置
    AUTO_SURFACE_STATE_ASCEND            // 到达水平位置后垂直上升
};

// 自动驾驶偏航（Yaw）控制模式枚举
enum autopilot_yaw_mode {
    AUTO_YAW_HOLD =              0,  // 飞手控制航向（不自动）
    AUTO_YAW_LOOK_AT_NEXT_WP =   1,  // 朝向下一个航点（飞手不可干预）
    AUTO_YAW_ROI =               2,  // 朝向感兴趣区域 roi_WP（飞手不可干预）
    AUTO_YAW_LOOK_AT_HEADING =   3,  // 朝向固定角度（飞手不可干预）
    AUTO_YAW_LOOK_AHEAD =        4,  // 朝向运动方向
    AUTO_YAW_RESETTOARMEDYAW =   5,  // 恢复到解锁时的航向
    AUTO_YAW_CORRECT_XTRACK =    6,  // 修正横向偏差（航线跟踪）
    AUTO_YAW_RATE =              7   // 按期望偏航角速率控制
};

// Acro 训练模式类型（限制飞手翻滚角度防止倒置）
#define ACRO_TRAINER_DISABLED   0    // 无训练辅助
#define ACRO_TRAINER_LEVELING   1    // 缓慢恢复水平
#define ACRO_TRAINER_LIMITED    2    // 限制最大角度

// 任务中偏航行为（WP_YAW_BEHAVIOR 参数）
#define WP_YAW_BEHAVIOR_NONE                          0   // 任务和RTL中永不自动控制偏航
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP               1   // 朝向下一航点或Home（RTL时）
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL    2   // 朝向下一航点，RTL时保持最后航向
#define WP_YAW_BEHAVIOR_LOOK_AHEAD                    3   // 向运动方向看（直升机适用）
#define WP_YAW_BEHAVIOR_CORRECT_XTRACK                4   // 朝向中间轨迹目标（航线跟踪）



// 日志消息 ID 枚举（每个 ID 对应一类日志包格式）
// 注意：此处最多只有32条可用于载具
enum LoggingParameters {
    LOG_CONTROL_TUNING_MSG,   // 控制调参数据包
    LOG_DATA_INT16_MSG,       // int16 用户数据包
    LOG_DATA_UINT16_MSG,      // uint16 用户数据包
    LOG_DATA_INT32_MSG,       // int32 用户数据包
    LOG_DATA_UINT32_MSG,      // uint32 用户数据包
    LOG_DATA_FLOAT_MSG,       // float 用户数据包
    LOG_GUIDEDTARGET_MSG      // Guided 模式目标数据包
};

// 日志掩码位定义（LOG_BITMASK 参数的各个位）
#define MASK_LOG_ATTITUDE_FAST          (1<<0)   // 高速姿态日志
#define MASK_LOG_ATTITUDE_MED           (1<<1)   // 中速姿态日志
#define MASK_LOG_GPS                    (1<<2)   // GPS 日志
#define MASK_LOG_PM                     (1<<3)   // 性能监控
#define MASK_LOG_CTUN                   (1<<4)   // 控制调参日志
#define MASK_LOG_NTUN                   (1<<5)   // 导航调参日志
#define MASK_LOG_RCIN                   (1<<6)   // RC 输入日志
#define MASK_LOG_IMU                    (1<<7)   // IMU 数据日志
#define MASK_LOG_CMD                    (1<<8)   // 任务命令日志
#define MASK_LOG_CURRENT                (1<<9)   // 电流日志
#define MASK_LOG_RCOUT                  (1<<10)  // RC 输出日志
#define MASK_LOG_OPTFLOW                (1<<11)  // 光流日志
#define MASK_LOG_PID                    (1<<12)  // PID 日志
#define MASK_LOG_COMPASS                (1<<13)  // 罗盘日志
#define MASK_LOG_CAMERA                 (1<<15)  // 相机日志
#define MASK_LOG_MOTBATT                (1UL<<17) // 电机电池日志
#define MASK_LOG_IMU_FAST               (1UL<<18) // 高速 IMU 日志
#define MASK_LOG_IMU_RAW                (1UL<<19) // 原始 IMU 日志
#define MASK_LOG_ANY                    0xFFFF   // 所有日志

// GCS 心跳超时故障保护
#ifndef FS_GCS
# define FS_GCS                        0         // 默认不启用
#endif
#ifndef FS_GCS_TIMEOUT_S
# define FS_GCS_TIMEOUT_S             5.0    // GCS 心跳超时秒数（5秒无心跳则触发）
#endif

// 地形数据缺失故障保护超时（毫秒）
#ifndef FS_TERRAIN_TIMEOUT_MS
#define FS_TERRAIN_TIMEOUT_MS          1000     // 1 second of unhealthy rangefinder and/or missing terrain data will trigger failsafe
#endif

//////////////////////////////////////////////////////////////////////////////
//  EKF Failsafe
// EKF failsafe definitions (FS_EKF_ENABLE parameter)
#define FS_EKF_ACTION_DISABLED                0       // Disabled
#define FS_EKF_ACTION_WARN_ONLY               1       // Send warning to gcs
#define FS_EKF_ACTION_DISARM                  2       // Disarm


#ifndef FS_EKF_ACTION_DEFAULT
# define FS_EKF_ACTION_DEFAULT         FS_EKF_ACTION_DISABLED  // EKF failsafe
#endif

#ifndef FS_EKF_THRESHOLD_DEFAULT
# define FS_EKF_THRESHOLD_DEFAULT      0.8f    // EKF failsafe's default compass and velocity variance threshold above which the EKF failsafe will be triggered
#endif

// 地面站(GCS)故障保护动作（FS_GCS_ENABLE 参数）
#define FS_GCS_DISABLED     0 // 禁用
#define FS_GCS_WARN_ONLY    1 // 仅向 GCS 发警告（多 GCS 链路时有用）
#define FS_GCS_DISARM       2 // 加锁（停止推进器）
#define FS_GCS_HOLD         3 // 切换到深度保持或位置保持模式
#define FS_GCS_SURFACE      4 // 切换到自动上浮模式

// 漏水故障保护动作（FS_LEAK_ENABLE 参数）
#define FS_LEAK_DISABLED    0 // 禁用
#define FS_LEAK_WARN_ONLY   1 // 仅发警告
#define FS_LEAK_SURFACE     2 // 切换到自动上浮模式

// 内部气压故障保护（FS_PRESS_MAX / FS_PRESS_ENABLE 参数）
#define FS_PRESS_MAX_DEFAULT    105000 // 触发故障保护的最大内部气压（Pa），超过 105000Pa 说明内部压力异常
#define FS_PRESS_DISABLED       0
#define FS_PRESS_WARN_ONLY      1

// 内部温度故障保护（FS_TEMP_MAX / FS_TEMP_ENABLE 参数）
#define FS_TEMP_MAX_DEFAULT     62  // 触发故障保护的最高内部温度（°C）
#define FS_TEMP_DISABLED        0
#define FS_TEMP_WARN_ONLY       1

// 撞击检测故障保护
#define FS_CRASH_DISABLED  0
#define FS_CRASH_WARN_ONLY 1
#define FS_CRASH_DISARM    2  // 加锁

// AUTO 模式下地形故障保护动作
#define FS_TERRAIN_DISARM       0 // 加锁
#define FS_TERRAIN_HOLD         1 // 保持当前位置
#define FS_TERRAIN_SURFACE      2 // 自动上浮

// 飞手输入超时故障保护动作
#define FS_PILOT_INPUT_DISABLED    0
#define FS_PILOT_INPUT_WARN_ONLY   1
#define FS_PILOT_INPUT_DISARM      2 // 加锁

// 解锁时油门位置要求（THR_ARM_POS 参数）
#define WITHIN_THR_TRIM 1  // 要求油门在中立点附近才可解锁

// 遥控信号丢失故障保护（FS_THR 参数）
#define FS_THR_DISABLED                            0
#define FS_THR_WARN                                1 // 仅发警告
#define FS_THR_SURFACE                             2 // 自动上浮

// 地形数据恢复超时：在触发地形故障保护前尝试恢复有效测距仪数据的时间
#define FS_TERRAIN_RECOVER_TIMEOUT_MS 10000

// MAVLink SET_POSITION_TARGET 消息忽略掩码（用于 Guided 模式的外部指令）
#define MAVLINK_SET_POS_TYPE_MASK_Z_IGNORE        (1<<2)   // 忽略 Z 轴位置
#define MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE      ((1<<0) | (1<<1) | (1<<2))   // 忽略全部位置分量
#define MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE      ((1<<3) | (1<<4) | (1<<5))   // 忽略全部速度分量
#define MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE      ((1<<6) | (1<<7) | (1<<8))   // 忽略全部加速度分量
#define MAVLINK_SET_POS_TYPE_MASK_FORCE           (1<<9)   // 使用推力代替油门
#define MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE      (1<<10)  // 忽略偏航角
#define MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE (1<<11)  // 忽略偏航角速率

