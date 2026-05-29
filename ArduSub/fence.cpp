// fence.cpp - 地理围栖检查
// 集成 AC_Fence 库，当潜器越出设定的地理范围时触发警告或动作
// ArduSub 当前仅记录越围日志，暂未实现自动驾驶响应

#include "Sub.h"

// 将 AC_Fence 库集成到 ArduSub 主代码

#if AP_FENCE_ENABLED

// fence_checks_async - 围栖异步检查（定时器以 1kHz 调用，内部限速至 3Hz）
// 仅在解锁状态下执行
void Sub::fence_checks_async()
{
    const uint32_t now = AP_HAL::millis();
    // 未解锁时不检查围栖
    if (!motors.armed()) {
        return;
    }

    // 限制检查频率为 3Hz（节约 CPU）
    if (!AP_HAL::timeout_expired(fence_breaches.last_check_ms, now, 333U)) {
        return;
    }

    fence_breaches.last_check_ms = now;
    const uint8_t orig_breaches = fence.get_breaches();
    // 检查是否有新的围栖越界（new_breaches 是越界围栖类型的位图）
    const uint8_t new_breaches = fence.check();

    // 如果发生越界且用户配置了响应动作
    if (new_breaches) {
        if (fence.get_action() != AC_Fence::Action::REPORT_ONLY) {
            // TODO: 实现自动响应（当前仅记录日志）
            // 下方注释的代码是将来可能实现的 RTL/LAND 响应
        }

        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(fence_breaches.new_breaches));

    } else if (orig_breaches) {
        // 围栖状态已恢复清除
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }
}

#endif
