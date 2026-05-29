// inertia.cpp - 惯性导航数据更新
// 从 AHRS（姿态航向参考系统）/EKF 读取位置、高度、速度估计
// 并更新 current_loc 和 climb_rate 成员变量

#include "Sub.h"

// read_inertia - 从 EKF/AHRS 读取惯性导航数据
// 更新 current_loc（经纬度+高度）和 climb_rate（爬升速率）
void Sub::read_inertia()
{
    // 更新位置控制器的 EKF 估计值（内部调用 EKF 获取最新数据）
    sub.pos_control.update_estimates();

    // 从 AHRS 获取当前位置（失败时 AHRS 提供最优猜测）
    Location loc;
    UNUSED_RESULT(ahrs.get_location(loc));
    current_loc.lat = loc.lat;
    current_loc.lng = loc.lng;

    // 没有高度估计则不更新高度
    if (!AP::ahrs().has_status(AP_AHRS::Status::VERT_POS)) {
        return;
    }

    // 高度采用 EKF 位置控制器的 U 轴估计值（cm，绝对坐标，水面=0）
    current_loc.alt = pos_control.get_pos_estimate_U_m() * 100.0f;

    // 爬升速率（cm/s，正值=上升，负值=下降）
    climb_rate = pos_control.get_vel_estimate_U_ms() * 100.0f;
}
