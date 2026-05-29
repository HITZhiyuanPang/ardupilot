#pragma once

// GCS_MAVLink_Sub.h - ArduSub MAVLink 通信处理类
// 继承自 GCS_MAVLINK 基类，重写 Sub 特有的消息处理：
//   - handle_manual_control_axes：处理摇杆输入（主要输入方式）
//   - handle_command_int_packet：处理 MAVLink 命令（电机测试、ROI、校准等）
//   - send_scaled_pressure3：重用压力3通道发送水温数据
//   - global_position_int_alt：深度（负值）作为高度
//   - capabilities：声明 Sub 支持的 MAVLink 能力位图

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Sub : public GCS_MAVLINK {

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    MAV_RESULT handle_flight_termination(const mavlink_command_int_t &packet) override;  // 飞行终止命令

    MAV_RESULT handle_command_do_set_roi(const Location &roi_loc) override;              // 设置感兴趣点（ROI）
    MAV_RESULT _handle_command_preflight_calibration_baro(const mavlink_message_t &msg) override;  // 气压计校准
    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;  // 飞前校准

    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;  // 通用命令分发
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);   // 重定位命令
    void handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow) override;  // 摇杆输入处理

    // 重写以发送水温（利用 scaled_pressure3 通道传输温度传感器数据）
    void send_scaled_pressure3() override;

    int32_t global_position_int_alt() const override;           // 深度作为高度（cm，负=水下）
    int32_t global_position_int_relative_alt() const override;  // 相对高度

    void send_banner() override;                                 // 启动横幅（固件信息）

    void send_nav_controller_output() const override;           // 导航控制器输出（目标与实际对比）
    bool get_target_location(Location &loc) const override;     // 获取当前导航目标位置
    void send_pid_tuning() override;                            // 发送 PID 调整数据

    uint64_t capabilities() const override;                     // MAVLink 能力位图

    // 发送第 index 个可用飞行模式（index 从 1 开始），返回总模式数
    uint8_t send_available_mode(uint8_t index) const override;

private:

    void handle_message(const mavlink_message_t &msg) override;          // 消息路由分发
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override; // 处理引导模式请求
    bool try_send_message(enum ap_message id) override;                  // 尝试发送指定类型消息

    bool send_info(void);                                                // 发送 Sub 特有信息

    uint8_t base_mode() const override;                                  // MAVLink base_mode 字段
    MAV_STATE vehicle_system_status() const override;                    // 系统状态

    int16_t vfr_hud_throttle() const override;                          // VFR HUD 油门百分比
    float vfr_hud_alt() const override;                                  // VFR HUD 高度

    MAV_RESULT handle_MAV_CMD_CONDITION_YAW(const mavlink_command_int_t &packet);       // 偏航条件命令
    MAV_RESULT handle_MAV_CMD_MISSION_START(const mavlink_command_int_t &packet);       // 任务启动命令
    MAV_RESULT handle_MAV_CMD_DO_CHANGE_SPEED(const mavlink_command_int_t &packet);     // 改变速度命令
    MAV_RESULT handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet);       // 电机测试命令
    MAV_RESULT handle_MAV_CMD_NAV_LOITER_UNLIM(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_NAV_LAND(const mavlink_command_int_t &packet);

#if AP_RANGEFINDER_ENABLED
    // send WATER_DEPTH - metres and temperature
    void send_water_depth();
    // state variable for the last rangefinder we sent a WATER_DEPTH
    // message for.  We cycle through the rangefinder backends to
    // limit the amount of telemetry bandwidth we consume.
    uint8_t last_WATER_DEPTH_index;
#endif // AP_RANGEFINDER_ENABLED

#if HAL_HIGH_LATENCY2_ENABLED
    int16_t high_latency_target_altitude() const override;
    uint8_t high_latency_tgt_heading() const override;
    uint16_t high_latency_tgt_dist() const override;
    uint8_t high_latency_tgt_airspeed() const override;
#endif // HAL_HIGH_LATENCY2_ENABLED
};
