// AP_Arming_Sub.cpp - ArduSub 解锁/加锁逻辑
// 潜航器解锁的关键差异：
//   1. 必须预先配置"加锁"或"急停"功能（防止水下无法加锁）
//   2. 解锁时可选要求油门处于中立位置（THR_ARM_POS 参数）
//   3. 解锁后初始化推进器输出并启用主循环看门狗

#include "AP_Arming_Sub.h"
#include "Sub.h"

// rc_calibration_checks - 检查滚转/俯仰/油门/偏航通道是否已校准
bool AP_Arming_Sub::rc_calibration_checks(bool display_failure)
{
    const RC_Channel *channels[] = {
        sub.channel_roll,
        sub.channel_pitch,
        sub.channel_throttle,
        sub.channel_yaw
    };
    return rc_checks_copter_sub(display_failure, channels);
}

// has_disarm_function - 检查是否配置了加锁功能
// 安全要求：水下没有加锁方法则无法安全终止推进器，因此不允许解锁
bool AP_Arming_Sub::has_disarm_function() const {
    if (sub.jsbutton_function_is_assigned(JSButton::k_arm_toggle) ||
        sub.jsbutton_function_is_assigned(JSButton::k_disarm)) {
        return true;
    }
    // 检查是否通过 AUX 功能配置了急停/加锁
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_ESTOP) || 
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::DISARM) || 
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::ARMDISARM) || 
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::ARM_EMERGENCY_STOP)) {
        return true;
    }  
    return false;
}

// pre_arm_checks - 解锁前检查（Sub 特有：必须配置加锁功能）
bool AP_Arming_Sub::pre_arm_checks(bool display_failure)
{
    if (armed) {
        return true;
    }
    // 没有加锁按钮/功能时拒绝解锁（水下安全关键检查）
    if (!has_disarm_function()) {
        check_failed(display_failure, "Must assign a disarm or arm_toggle button or disarm aux function");
        return false;
    }

    return AP_Arming::pre_arm_checks(display_failure);
}

// ins_checks - IMU 检查（额外验证 AHRS 姿态估计是否就绪）
bool AP_Arming_Sub::ins_checks(bool display_failure)
{
    // call parent class checks
    if (!AP_Arming::ins_checks(display_failure)) {
        return false;
    }

    // additional sub-specific checks
    if (check_enabled(Check::INS)) {
        char failure_msg[50] = {};
        if (!AP::ahrs().pre_arm_check(false, failure_msg, sizeof(failure_msg))) {
            check_failed(Check::INS, display_failure, "AHRS: %s", failure_msg);
            return false;
        }
    }

    return true;
}

// arm - 解锁推进器
// 流程：检查油门中立 → 调用父类 arm → 禁用 CPU 看门狗 → 初始化输出 → 通知 GCS
bool AP_Arming_Sub::arm(AP_Arming::Method method, bool do_arming_checks)
{
    static bool in_arm_motors = false;

    // 防止重入（arm() 不能被递归调用）
    if (in_arm_motors) {
        return false;
    }

    in_arm_motors = true;

    // 如果参数要求油门在中立位置附近才可解锁（水下防意外启动）
    if (check_enabled(Check::RC) &&
     rc().option_is_enabled(RC_Channels::Option::ARMING_CHECK_THROTTLE) &&
     (sub.g.thr_arming_position == WITHIN_THR_TRIM)) {
        const char *rc_item = "Throttle";
        // 检查油门是否在 trim ± dead_zone 范围内（即居中）
        if (!sub.channel_throttle->in_trim_dz()) {
           check_failed(Check::RC, true, "%s not centered/close to trim", rc_item);
           AP_Notify::events.arming_failed = true;
           in_arm_motors = false;
           return false;
        }
    }

    if (!AP_Arming::arm(method, do_arming_checks)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

#if HAL_LOGGING_ENABLED
    // let logger know that we're armed (it may open logs e.g.)
    AP::logger().set_vehicle_armed(true);
#endif

    // 解锁期间初始化会消耗时间，暂时禁用 CPU 死循环故障保护
    sub.mainloop_failsafe_disable();

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call notify update a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        AP::notify().update();
    }

    send_arm_disarm_statustext("Arming motors");

    AP_AHRS &ahrs = AP::ahrs();

    sub.initial_armed_bearing = ahrs.yaw_sensor;

    if (!ahrs.home_is_set()) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)

        // ROV 始终使用绝对高度（不重置 EKF 高度基准），注释中保留原来的代码供参考
        // ahrs.resetHeightDatum();
        // AP::logger().Write_Event(LogEvent::EKF_ALT_RESET);
    } else if (!ahrs.home_is_locked()) {
        // Home 已设置但未锁定时，重置为当前位置（水面位置）
        if (!sub.set_home_to_current_location(false)) {
            // ignore this failure
        }
    }

    // 通知 HAL 层已软件解锁（使能推进器 PWM 输出）
    hal.util->set_soft_armed(true);

    // 初始化推进器输出为零（防止突然启动）
    sub.enable_motor_output();

    // 正式标记推进器为解锁状态
    sub.motors.armed(true);

#if HAL_LOGGING_ENABLED
    // 记录当前飞行模式（解锁时可能已切换模式）
    AP::logger().Write_Mode((uint8_t)sub.control_mode, sub.control_mode_reason);
#endif

    // 解锁完成，重新启用主循环 CPU 看门狗
    sub.mainloop_failsafe_enable();

    // 忽略本次循环的性能数据（解锁初始化耗时不计入正常性能统计）
    AP::scheduler().perf_info.ignore_this_loop();

    in_arm_motors = false;

    // 如果没有 EKF 原点则无法使用地磁模型（WMM），罗盘精度会下降
    Location origin_loc;
    if (!ahrs.get_origin(origin_loc)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Compass performance degraded");
        if (check_enabled(Check::PARAMETERS)) {
            check_failed(Check::PARAMETERS, true, "No world position, check AHRS_ORIGIN_* parameters");
            return false;
        }
    }
    // return success
    return true;
}

// disarm - 加锁推进器（停止所有推进器输出）
// 流程：调用父类 disarm → 保存罗盘偏移 → 停止推进器输出 → 通知 GCS
bool AP_Arming_Sub::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    // 已经加锁则直接返回
    if (!sub.motors.armed()) {
        return false;
    }

    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }

    send_arm_disarm_statustext("Disarming motors");

    auto &ahrs = AP::ahrs();

    // 如果罗盘使用 EKF 学习偏移，加锁时保存学习到的偏移值（下次上电直接使用）
    if (ahrs.use_compass() && AP::compass().get_learn_type() == Compass::LearnType::COPY_FROM_EKF) {
        for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            Vector3f magOffsets;
            if (ahrs.getMagOffsets(i, magOffsets)) {
                AP::compass().set_and_save_offsets(i, magOffsets);
            }
        }
    }

    // 命令推进器停止（切断所有电机输出）
    sub.motors.armed(false);

    // 加锁后重置任务（下次解锁从第一条命令开始）
    sub.mission.reset();

#if HAL_LOGGING_ENABLED
    AP::logger().set_vehicle_armed(false);
#endif

    // 通知 HAL 层已软件加锁
    hal.util->set_soft_armed(false);

    // 清除遥控器/摇杆输入保持状态（防止遗留的 hold 指令影响下次解锁）
    sub.clear_input_hold();

    return true;
}
