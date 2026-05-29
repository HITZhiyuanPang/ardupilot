// actuators.cpp - ArduSub 辅助执行器（夹爪、机械臂等外设）控制
// 管理最多 ACTUATOR_CHANNELS(6) 路辅助舵机/执行器的 PWM 输出
// 每路执行器的当前值用 [0.0, 1.0] 的归一化浮点数表示

#include "Sub.h"
#include "actuators.h"


// 执行器参数表 —— 每路执行器有独立的 PWM 步进增量参数
// 参数名称格式：ACT_N_INC（N = 1..6），单位 us，用于每次按钮触发时的步进量
const AP_Param::GroupInfo Actuators::var_info[] = {

    // @Param: 1_INC
    // @DisplayName: Increment step for actuator 1
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("1_INC", 1, Actuators, actuator_increment_step[0], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 2_INC
    // @DisplayName: Increment step for actuator 2
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("2_INC", 2, Actuators, actuator_increment_step[1], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 3_INC
    // @DisplayName: Increment step for actuator 3
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("3_INC", 3, Actuators, actuator_increment_step[2], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 4_INC
    // @DisplayName: Increment step for actuator 4
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("4_INC", 4, Actuators, actuator_increment_step[3], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 5_INC
    // @DisplayName: Increment step for actuator 5
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("5_INC", 5, Actuators, actuator_increment_step[4], ACTUATOR_DEFAULT_INCREMENT),

    // @Param: 6_INC
    // @DisplayName: Increment step for actuator 6
    // @Description:  Initial increment step for changing the actuator's PWM
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("6_INC", 6, Actuators, actuator_increment_step[5], ACTUATOR_DEFAULT_INCREMENT),

    AP_GROUPEND
};

// 构造函数 —— 从参数系统加载默认值
Actuators::Actuators() {
    AP_Param::setup_object_defaults(this, var_info);
}

// initialize_actuators - 初始化所有执行器通道
// 读取每路舵机的 trim（中立点）PWM，换算成 [0,1] 归一化值作为初始位置
// 然后立即下发一次输出，确保执行器从中立位置启动
void Actuators::initialize_actuators() {
    const SRV_Channel::Function first_aux = SRV_Channel::Function::k_actuator1;
    for (int i = 0; i < ACTUATOR_CHANNELS; i++) {
        uint8_t channel_number;
        // 如果该执行器功能未分配到任何通道，则跳过
        if (!SRV_Channels::find_channel((SRV_Channel::Function)(first_aux + i), channel_number)) {
            continue;
        }
        SRV_Channel* chan = SRV_Channels::srv_channel(channel_number);
        uint16_t servo_min = chan->get_output_min();
        uint16_t servo_max = chan->get_output_max();
        uint16_t servo_range = servo_max - servo_min;

        uint16_t servo_trim = chan->get_trim();
        // 将 trim PWM 映射到 [0, 1]，作为执行器当前值
        aux_actuator_value[i] = (servo_trim - servo_min) / static_cast<float>(servo_range);
    }
    update_actuators();
}

// update_actuators - 将内部归一化值 [0,1] 转换为实际 PWM 并写入舵机通道
// 每个主循环周期调用，以保持输出同步
void Actuators::update_actuators() {
    const SRV_Channel::Function first_aux = SRV_Channel::Function::k_actuator1;
    for (int i = 0; i < ACTUATOR_CHANNELS; i++) {
        uint8_t channel_number;
        // 未找到对应通道则跳过
        if (!SRV_Channels::find_channel((SRV_Channel::Function)(first_aux + i), channel_number)) {
            continue;
        }
        SRV_Channel* chan = SRV_Channels::srv_channel(channel_number);
        uint16_t servo_min = chan->get_output_min();
        uint16_t servo_max = chan->get_output_max();
        uint16_t servo_range = servo_max - servo_min;
        // 将归一化值线性映射到 [servo_min, servo_max] 的 PWM 输出
        chan->set_output_pwm(servo_min + servo_range * aux_actuator_value[i]);
    }
}

// increase_actuator - 按步进量增大指定执行器的值（上限 1.0）
void Actuators::increase_actuator(uint8_t actuator_num) {
    if (actuator_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[actuator_num] = constrain_float(aux_actuator_value[actuator_num] + actuator_increment_step[actuator_num], 0.0f, 1.0f);
}

// decrease_actuator - 按步进量减小指定执行器的值（下限 0.0）
void Actuators::decrease_actuator(uint8_t actuator_num) {
    if (actuator_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[actuator_num] = constrain_float(aux_actuator_value[actuator_num] - actuator_increment_step[actuator_num], 0.0f, 1.0f);
}

// min_actuator - 将指定执行器设置到最小位置（0.0 → servo_min PWM）
void Actuators::min_actuator(uint8_t actuator_num) {
    if (actuator_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[actuator_num] = 0;
}

// max_actuator - 将指定执行器设置到最大位置（1.0 → servo_max PWM）
void Actuators::max_actuator(uint8_t actuator_num) {
    if (actuator_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[actuator_num] = 1;
}

// min_toggle_actuator - 最小切换：若当前值 < 0.4 则设为中立 0.5，否则设为最小 0.0
void Actuators::min_toggle_actuator(uint8_t actuator_num) {
    if (actuator_num >= ACTUATOR_CHANNELS) {
        return;
    }
    if (aux_actuator_value[actuator_num] < 0.4) {
        aux_actuator_value[actuator_num] = 0.5;
    } else {
        aux_actuator_value[actuator_num] = 0;
    }
}

// max_toggle_actuator - 最大切换：若当前值 >= 0.6 则设为中立 0.5，否则设为最大 1.0
void Actuators::max_toggle_actuator(uint8_t actuator_num) {
    if (actuator_num >= ACTUATOR_CHANNELS) {
        return;
    }
    if (aux_actuator_value[actuator_num] >= 0.6) {
        aux_actuator_value[actuator_num] = 0.5;
    } else {
        aux_actuator_value[actuator_num] = 1;
    }
}

// center_actuator - 将执行器设置到中立位置（0.5 → servo_trim 附近）
void Actuators::center_actuator(uint8_t actuator_num) {
    if (actuator_num >= ACTUATOR_CHANNELS) {
        return;
    }
    aux_actuator_value[actuator_num] = 0.5;
}
