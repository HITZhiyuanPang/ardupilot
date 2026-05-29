// turn_counter.cpp - 旋转圈数计数器（防止脐带电缆缠绕）
// 作者：Rustom Jehangir <rusty@bluerobotics.com>
//
// 将偏航角平面分为 4 个象限，通过检测象限切换方向来累计旋转圈数
//   quarter_turn_count > 0：右转（顺时针）
//   quarter_turn_count < 0：左转（逆时针）
// 每转一圈 quarter_turn_count 变化 ±4

#include "Sub.h"

// update_turn_counter - 每帧更新旋转计数
// 偏航角划分为 4 个象限（0~90°、90~180°、-180~-90°、-90~0°）
void Sub::update_turn_counter()
{
    // 确定当前象限
    // 状态 0: 0~90°，1: 90~180°，2: -180~-90°，3: -90~0°
    uint8_t turn_state;
    if (ahrs.get_yaw_rad() >= 0.0f && ahrs.get_yaw_rad() < radians(90)) {
        turn_state = 0;
    } else if (ahrs.get_yaw_rad() >= radians(90)) {
        turn_state = 1;
    } else if (ahrs.get_yaw_rad() < -radians(90)) {
        turn_state = 2;
    } else {
        turn_state = 3;
    }

    // 象限按序号递增 → 右转（顺时针）；递减 → 左转（逆时针）
    if (turn_state == (last_turn_state + 1) % 4) {
        quarter_turn_count++;
    } else if (turn_state == (uint8_t)(last_turn_state - 1) % 4) {
        quarter_turn_count--;
    }
    last_turn_state = turn_state;
}
