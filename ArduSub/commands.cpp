// commands.cpp - ArduSub 家庭位置（Home）管理
// 包含：
//   - update_home_from_EKF：EKF 初始化后自动设置 Home 点
//   - set_home_to_current_location：将当前 GPS 位置设为 Home（水面海拔）
//   - set_home：设置指定位置为 Home

#include "Sub.h"

// update_home_from_EKF - 当 Home 未设置时，从 EKF 估计位置设置 Home
void Sub::update_home_from_EKF()
{
    // Home 已设置则立即返回
    if (ahrs.home_is_set()) {
        return;
    }
    if (!set_home_to_current_location(false)) {
        // 忽略失败（EKF 可能尚未初始化）
    }
}

// set_home_to_current_location - 将当前 GPS 位置设为 Home
// 重要：Home 总是设在水面高度（深度=0），而不是当前深度
// 这样无论在高山湖泊还是海面，相对高度都是从水面算起
bool Sub::set_home_to_current_location(bool lock)
{
    Location temp_loc;
    if (ahrs.get_location(temp_loc)) {

        // 将位置修正到水面高度（向上偏移当前气压计高度的相反数）
        // 这允许在水下解锁/加锁后重新解锁时 Home 仍然在水面
        temp_loc.offset_up_m(-barometer.get_altitude());
        return set_home(temp_loc, lock);
    }
    return false;
}

// set_home - 设置 AHRS Home 位置（RTL 的返回目标）
// 返回 true 表示设置成功
bool Sub::set_home(const Location& loc, bool lock)
{
    // 检查 EKF 原点是否已设置（EKF 需要先有原点才能设 Home）
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }

    // 设置 AHRS Home（用于 RTL 返回目标）
    if (!ahrs.set_home(loc)) {
        return false;
    }

    // 是否锁定 Home（锁定后不再自动更新）
    if (lock) {
        ahrs.lock_home();
    }

    return true;
}
