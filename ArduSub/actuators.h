// actuators.h - ArduSub 辅助执行器类（夹爪、机械臂等外设 PWM 控制）
// 最多支持 ACTUATOR_CHANNELS(6) 路独立执行器
// 每路执行器用 [0.0, 1.0] 的归一化值控制，运行时映射到实际 PWM 范围

#pragma once

#define ACTUATOR_CHANNELS 6         // 最大执行器通道数
#define ACTUATOR_DEFAULT_INCREMENT 0.1f // 默认步进增量（每次按钮触发改变 10%）

class Actuators
{
public:
    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    Actuators();
    void initialize_actuators();        // 初始化所有通道，以 trim 位置为起始值
    void update_actuators();            // 将内部值输出为 PWM
    void increase_actuator(uint8_t actuator_num);      // 增大执行器值（步进）
    void decrease_actuator(uint8_t actuator_num);      // 减小执行器值（步进）
    void min_actuator(uint8_t actuator_num);           // 设置到最小位置
    void max_actuator(uint8_t actuator_num);           // 设置到最大位置
    void min_toggle_actuator(uint8_t actuator_num);    // 最小/中立 切换
    void max_toggle_actuator(uint8_t actuator_num);    // 最大/中立 切换
    void center_actuator(uint8_t actuator_num);        // 设置到中立位置 (0.5)

protected:
    AP_Float actuator_increment_step[ACTUATOR_CHANNELS];    // 各通道步进量参数（来自地面站）
    float aux_actuator_change_speed[ACTUATOR_CHANNELS];     // 预留：连续变化速率
    float aux_actuator_value[ACTUATOR_CHANNELS];            // 各通道当前归一化值 [0, 1]
public:
   
};
