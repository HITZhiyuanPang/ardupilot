#pragma once

// script_button.h - Lua 脚本用摇杆按钮对象
// 提供两种使用摇杆按钮的方式：
//   1. is_pressed()：查询按钮当前是否按下（基于最新一帧 MANUAL_CONTROL 消息）
//   2. get_and_clear_count()：返回自上次调用以来按钮被按下的次数（适合统计点击）

#if AP_SCRIPTING_ENABLED

#include <AP_Common/AP_Common.h>

class ScriptButton {
public:
    ScriptButton(): pressed(false), count(0) {}

    void press();      // 标记按钮按下（由 joystick.cpp 调用）

    void release();    // 标记按钮释放

    bool is_pressed() const WARN_IF_UNUSED;         // 查询当前是否按下

    uint8_t get_count() const WARN_IF_UNUSED;       // 查询按下次数（不清除）

    void clear_count();                              // 清除按下计数

    uint8_t get_and_clear_count();                  // 获取并清除按下次数（原子操作）

private:
    bool pressed;     // 当前按下状态
    uint8_t count;    // 按下次数计数器
};

#endif // AP_SCRIPTING_ENABLED
