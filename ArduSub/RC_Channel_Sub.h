#pragma once

// RC_Channel_Sub.h - ArduSub RC 通道扩展类
// 扩展通用 RC_Channel 类，添加 Sub 特有的：
//   - 辅助功能初始化（aux function）
//   - 模式拨动开关处理（如果编译了 AP_SUB_RC_ENABLED）
// 对于 MAVLink 摇杆控制（无 RC 接收机），使用精简版空类

#include <RC_Channel/RC_Channel.h>
#include "config.h"

#if AP_SUB_RC_ENABLED
// 完整 RC 接收机模式：支持辅助功能和飞行模式切换开关
class RC_Channel_Sub : public RC_Channel
{

public:

protected:

    __INITFUNC__ void init_aux_function(AUX_FUNC ch_option, AuxSwitchPos) override;  // 初始化辅助功能通道
    bool do_aux_function(const AuxFuncTrigger &trigger) override;                     // 执行辅助功能动作
    
private:
    void mode_switch_changed(modeswitch_pos_t new_pos) override;  // 飞行模式拨动开关位置改变时调用
};

class RC_Channels_Sub : public RC_Channels
{
public:
    bool has_valid_input() const override;           // 是否有有效 RC 输入
    bool in_rc_failsafe() const override;            // 是否处于 RC 失联故障保护
    bool arming_check_throttle() const override;     // 是否需要检查油门解锁条件
    RC_Channel_Sub obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Sub *channel(const uint8_t chan) override {
        if (chan >= ARRAY_SIZE(obj_channels)) {
            return nullptr;
        }
        return &obj_channels[chan];
    }
    const RC_Channel_Sub *channel(const uint8_t chan) const override {
        if (chan >= ARRAY_SIZE(obj_channels)) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:

    int8_t flight_mode_channel_number() const override;  // 返回飞行模式切换通道编号

};

#else

// 无 RC 接收机模式：仅使用 MAVLink MANUAL_CONTROL 摇杆输入，RC 类为空壳
class RC_Channel_Sub : public RC_Channel
{

public:

protected:

private:

};

class RC_Channels_Sub : public RC_Channels
{
public:

    RC_Channel_Sub obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Sub *channel(const uint8_t chan) override {
        if (chan >= ARRAY_SIZE(obj_channels)) {
            return nullptr;
        }
        return &obj_channels[chan];
    }
    const RC_Channel_Sub *channel(const uint8_t chan) const override {
        if (chan >= ARRAY_SIZE(obj_channels)) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    // tell the gimbal code all is good with RC input:
    bool in_rc_failsafe() const override { return false; };
    bool arming_check_throttle() const override;

protected:

    // note that these callbacks are not presently used on Plane:
    int8_t flight_mode_channel_number() const override;

};
#endif


