// radio.cpp - RC 输入/输出初始化和读取
// ArduSub 主要通过 MAVLink MANUAL_CONTROL 覆盖 RC 通道
// 如果编译了 AP_SUB_RC_ENABLED，也支持传统 RC 接收机
// 包含 6 个控制通道：滚转、俯仰、油门、偏航、前进、横移

#include "Sub.h"

// init_rc_in - 初始化遥控输入通道
// 绑定通道指针，设置量程、死区，并将初始值设为中立点（1500）
void Sub::init_rc_in()
{
    // 绑定 6 个控制通道到全局指针
    channel_roll     = &rc().get_roll_channel();
    channel_pitch    = &rc().get_pitch_channel();
    channel_throttle = &rc().get_throttle_channel();
    channel_yaw      = &rc().get_yaw_channel();
    channel_forward  = &rc().get_forward_channel();
    channel_lateral  = &rc().get_lateral_channel();

    // 设置通道量程（set_angle 表示对称 ±N，set_range 表示单极性 0~N）
    channel_roll->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_pitch->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_yaw->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_throttle->set_range(1000);   // 油门 0~1000
    channel_forward->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_lateral->set_angle(ROLL_PITCH_INPUT_MAX);

    // 设置默认死区（偏航稍大，因为偏航控制更敏感）
    channel_roll->set_default_dead_zone(30);
    channel_pitch->set_default_dead_zone(30);
    channel_throttle->set_default_dead_zone(30);
    channel_yaw->set_default_dead_zone(40);
    channel_forward->set_default_dead_zone(30);
    channel_lateral->set_default_dead_zone(30);

    // 将所有控制通道初始化为中立值 1500（避免开机时误动作）
    uint32_t tnow = AP_HAL::millis();
    channel_roll->set_override(1500, tnow);
    channel_pitch->set_override(1500, tnow);
    channel_yaw->set_override(1500, tnow);
    channel_throttle->set_override(1500, tnow);
    channel_forward->set_override(1500, tnow);
    channel_lateral->set_override(1500, tnow);

#if HAL_MOUNT_ENABLED
    // 初始化相机云台 RC 输入为中立值
    RC_Channel *cam_pan_chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOUNT1_YAW);
    if (cam_pan_chan != nullptr) {
        cam_pan_chan->set_override(1500, tnow);
    }
    RC_Channel *cam_tilt_chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOUNT1_PITCH);
    if (cam_tilt_chan != nullptr) {
        cam_tilt_chan->set_override(1500, tnow);
    }
#endif  // HAL_MOUNT_ENABLED
}

// init_rc_out - 初始化推进器输出
// 设置 ESC 刷新率、电机框架类型，并在校准通过后使能输出
void Sub::init_rc_out()
{
    motors.set_update_rate(g.rc_speed);   // 设置 PWM 刷新率（默认 490Hz）
    // 根据 FRAME_CONFIG 参数初始化电机框架（如 BlueROV2 Heavy 6DOF）
    motors.init((AP_Motors::motor_frame_class)g.frame_configuration.get(), AP_Motors::motor_frame_type::MOTOR_FRAME_TYPE_PLUS);
    motors.convert_pwm_min_max_param(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
    motors.update_throttle_range();

    // 如果 RC 校准通过，使能电机输出（防止未校准时输出错误 PWM）
    if (arming.rc_calibration_checks(true)) {
        enable_motor_output();
    }

    // 刷新辅助通道到功能的映射
    SRV_Channels::update_aux_servo_function();
}
#if AP_SUB_RC_ENABLED
void Sub::read_radio()
{ 
    const uint32_t tnow_ms = AP_HAL::millis();

    if (rc().read_input()) {
        //got valid input
        last_radio_update_ms = tnow_ms;
        failsafe.last_pilot_input_ms = tnow_ms;
        set_throttle_and_failsafe(channel_throttle->get_radio_in());
        return;
    }
    
   // No radio input this time
    if (failsafe.radio) {
        // already in failsafe! no further action
        return;
    }

    // trigger failsafe if no update from the RC Radio for RC_FS_TIMEOUT seconds
    const uint32_t elapsed_ms = tnow_ms - last_radio_update_ms;
    if (elapsed_ms < rc().get_fs_timeout_ms()) {
        // not timed out yet
        return;
    }
    if (!g.failsafe_throttle) {
        // throttle failsafe not enabled
        return;
    }
    if (!rc().has_ever_seen_rc_input() && !sub.motors.armed()) {
        // we only failsafe if we are armed OR we have ever seen an RC receiver
        return;
    }
    
    // Log an error and enter failsafe.
    LOGGER_WRITE_ERROR(LogErrorSubsystem::RADIO, LogErrorCode::RADIO_LATE_FRAME);
    set_failsafe_radio(true);
}

#define FS_COUNTER 3        // radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value
void Sub::set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    // if failsafe not enabled pass through throttle, clear RC failsafe if it exists, and exit
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        set_failsafe_radio(false);
        return;
    }

    //check for low throttle value
    if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (failsafe.radio || !(rc().has_ever_seen_rc_input() || sub.motors.armed())) {
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are received
        failsafe.radio_counter++;
        if( failsafe.radio_counter >= FS_COUNTER ) {
            failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe_radio(true);
        }
    }else{
        // we have a good throttle so reduce failsafe counter
        failsafe.radio_counter--;
        if( failsafe.radio_counter <= 0 ) {
            failsafe.radio_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (failsafe.radio) {
                set_failsafe_radio(false);
            }
        }
        // pass through throttle
    }
}
#endif
