#pragma once

// GCS_Sub.h - ArduSub 地面控制站接口类
// GCS_Sub 继承自 GCS 基类，为 Sub 车辆提供：
//   - 传感器状态标志更新（用于 MAVLink SYS_STATUS 消息）
//   - 飞行模式编号转换（custom_mode）
//   - 机架类型报告（frame_type）
//   - MAVLink 后端通道管理

#include <GCS_MAVLink/GCS.h>
#include "GCS_MAVLink_Sub.h"

class GCS_Sub : public GCS
{
    friend class Sub; // 允许 Sub 类访问 _chan 参数声明

public:

    // GCS_MAVLINK_CHAN_METHOD_DEFINITIONS 宏展开为一对方法，用于获取特定链路的 MAVLink 后端指针
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Sub);

    void update_vehicle_sensor_status_flags() override;  // 更新 MAVLink SYS_STATUS 传感器状态标志

    uint32_t custom_mode() const override;               // 返回当前飞行模式编号（用于 HEARTBEAT）
    MAV_TYPE frame_type() const override;                // 返回机架类型（MAV_TYPE_SUBMARINE）

    bool vehicle_initialised() const override;           // 车辆是否完成初始化

protected:

    // 主循环中发送 MAVLink 消息前必须保留的最小时间（微秒）
    // Sub 优先保证主飞控循环，通信排在其后
    uint16_t min_loop_time_remaining_for_message_send_us() const override {
        return 250;
    }

    // 为每个串口链路创建 Sub 专用的 MAVLink 后端
    GCS_MAVLINK_Sub *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override {
        return NEW_NOTHROW GCS_MAVLINK_Sub(uart);
    }

};
