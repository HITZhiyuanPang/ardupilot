// AP_Arming_Sub.h - ArduSub 解锁检查类
// 继承自通用 AP_Arming，添加潜航器特有的解锁前检查和解锁/加锁流程
// 关键检查：必须配置解锁/加锁按钮或 AUX 功能，才允许解锁（水下安全要求）

#pragma once

#include <AP_Arming/AP_Arming.h>

class AP_Arming_Sub : public AP_Arming {
public:

    AP_Arming_Sub() : AP_Arming() { }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Arming_Sub);

    // RC 校准检查（验证摇杆通道是否已校准）
    bool rc_calibration_checks(bool display_failure) override;
    // 解锁前综合检查（含「必须有解锁按钮」的 Sub 特有检查）
    bool pre_arm_checks(bool display_failure) override;
    // 检查是否已配置加锁/急停功能按钮或 AUX 功能
    bool has_disarm_function() const;

    // 加锁（断开推进器输出，并记录日志）
    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;
    // 解锁（初始化推进器、记录日志、启用主循环看门狗）
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

protected:
    // IMU/AHRS 检查（额外验证 AHRS pre-arm 状态）
    bool ins_checks(bool display_failure) override;
};
