// motors.cpp - ArduSub 推进器输出管理
// 包含：
//   - enable_motor_output：将所有推进器输出设为最低值（解锁初始化用）
//   - motors_output：主循环每帧调用，将控制指令转换为 PWM 并下发
//   - init_motor_test：新式电机测试初始化（安全检查）
//   - verify_motor_test：电机测试超时验证

#include "Sub.h"

// enable_motor_output - 使能推进器并输出最低推力值（防止启动时跳变）
void Sub::enable_motor_output()
{
    motors.output_min();
}

// motors_output - 计算并输出推进器 PWM
// 每个主循环周期（400Hz）调用
// 电机测试模式和电机方向检测模式有特殊处理
void Sub::motors_output()
{
    // 电机方向检测模式：由检测逻辑直接控制推进器，跳过正常输出
    if (control_mode == Mode::Number::MOTOR_DETECT){
        return;
    }
    // 检查是否正在执行电机测试
    if (ap.motor_test) {
        verify_motor_test();
    } else {
        // 正常输出流程：
        motors.set_interlock(true);         // 允许推进器输出
        auto &srv = AP::srv();
        srv.cork();                          // 批量缓冲，减少 SPI/I2C 通信次数
        SRV_Channels::calc_pwm();           // 计算各通道 PWM
        SRV_Channels::output_ch_all();      // 输出所有通道（含辅助舵机）
        motors.output();                    // 输出推进器 PWM
        srv.push();                         // 统一推送
    }
}

// init_motor_test - 初始化电机测试
// 执行安全检查：冷却期、安全开关、已解锁
// 返回 true 表示测试已开始
bool Sub::init_motor_test()
{
    uint32_t tnow = AP_HAL::millis();

    // 上次测试失败后需要 10 秒冷却期，防止频繁重试损坏推进器
    if (tnow < last_do_motor_test_fail_ms + 10000 && last_do_motor_test_fail_ms > 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "10 second cooldown required after motor test");
        return false;
    }

    // 检查硬件安全开关是否已关闭（必须先拨动安全开关才能测试）
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Disarm hardware safety switch before testing motors.");
        return false;
    }

    // 必须先解锁才能测试电机
    if (!motors.armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Arm motors before testing motors.");
        return false;
    }

    enable_motor_output(); // 先将所有电机输出设为零
    ap.motor_test = true;

    return true;
}

// verify_motor_test - 电机测试超时验证
// 要求以至少 2Hz 的频率收到 MAV_CMD_DO_SET_MOTOR 命令
// 超时则自动停止测试并加锁
bool Sub::verify_motor_test()
{
    bool pass = true;

    // 如果 500ms 内没有收到新的电机测试命令则超时
    if (AP_HAL::millis() > last_do_motor_test_ms + 500) {
        gcs().send_text(MAV_SEVERITY_INFO, "Motor test timed out!");
        pass = false;
    }

    if (!pass) {
        ap.motor_test = false;
        AP::arming().disarm(AP_Arming::Method::MOTORTEST);
        last_do_motor_test_fail_ms = AP_HAL::millis();
        return false;
    }

    return true;
}

bool Sub::handle_do_motor_test(mavlink_command_int_t command) {
    last_do_motor_test_ms = AP_HAL::millis();

    // If we are not already testing motors, initialize test
    static uint32_t tLastInitializationFailed = 0;
    if(!ap.motor_test) {
        // Do not allow initializations attempt under 2 seconds
        // If one fails, we need to give the user time to fix the issue
        // instead of spamming error messages
        if (AP_HAL::millis() > (tLastInitializationFailed + 2000)) {
            if (!init_motor_test()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "motor test initialization failed!");
                tLastInitializationFailed = AP_HAL::millis();
                return false; // init fail
            }
        } else {
            return false;
        }
    }

    float motor_number = command.param1;
    float throttle_type = command.param2;
    float throttle = command.param3;
    // float timeout_s = command.param4; // not used
    // float motor_count = command.param5; // not used
    const uint32_t test_type = command.y;

    if (test_type != MOTOR_TEST_ORDER_BOARD) {
        gcs().send_text(MAV_SEVERITY_WARNING, "bad test type %0.2f", (double)test_type);
        return false; // test type not supported here
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PILOT)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "bad throttle type %0.2f", (double)throttle_type);

        return false; // throttle type not supported here
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PWM)) {
        return motors.output_test_num(motor_number, throttle); // true if motor output is set
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PERCENT)) {
        throttle = constrain_float(throttle, 0.0f, 100.0f);
        throttle = channel_throttle->get_radio_min() + throttle * 0.01f * (channel_throttle->get_radio_max() - channel_throttle->get_radio_min());
        return motors.output_test_num(motor_number, throttle); // true if motor output is set
    }

    return false;
}


// translate wpnav roll/pitch outputs to lateral/forward
void Sub::translate_wpnav_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    int32_t lateral = wp_nav.get_roll();
    int32_t forward = -wp_nav.get_pitch(); // output is reversed

    // constrain target forward/lateral values
    // The outputs of wp_nav.get_roll and get_pitch should already be constrained to these values
    const float angle_max_cd = attitude_control.lean_angle_max_cd();
    lateral = constrain_int16(lateral, -angle_max_cd, angle_max_cd);
    forward = constrain_int16(forward, -angle_max_cd, angle_max_cd);

    // Normalize
    lateral_out = (float)lateral/(float)angle_max_cd;
    forward_out = (float)forward/(float)angle_max_cd;
}

// translate wpnav roll/pitch outputs to lateral/forward
void Sub::translate_circle_nav_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    int32_t lateral = circle_nav.get_roll_cd();
    int32_t forward = -circle_nav.get_pitch_cd(); // output is reversed

    // constrain target forward/lateral values
    const float angle_max_cd = attitude_control.lean_angle_max_cd();
    lateral = constrain_int16(lateral, -angle_max_cd, angle_max_cd);
    forward = constrain_int16(forward, -angle_max_cd, angle_max_cd);

    // Normalize
    lateral_out = (float)lateral/(float)angle_max_cd;
    forward_out = (float)forward/(float)angle_max_cd;
}

// translate pos_control roll/pitch outputs to lateral/forward
void Sub::translate_pos_control_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    int32_t lateral = pos_control.get_roll_cd();
    int32_t forward = -pos_control.get_pitch_cd(); // output is reversed

    // constrain target forward/lateral values
    const float angle_max_cd = attitude_control.lean_angle_max_cd();
    lateral = constrain_int16(lateral, -angle_max_cd, angle_max_cd);
    forward = constrain_int16(forward, -angle_max_cd, angle_max_cd);

    // Normalize
    lateral_out = (float)lateral/(float)angle_max_cd;
    forward_out = (float)forward/(float)angle_max_cd;
}
