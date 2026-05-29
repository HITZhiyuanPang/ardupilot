// UserCode.cpp - 用户自定义代码钩子
// 提供一组可选的用户代码回调函数，通过在 config.h 中定义 USERHOOK_* 宏来启用
// 适合用户在不修改核心代码的前提下添加自定义功能
//
// 各钩子调用频率：
//   userhook_init        - 启动时调用一次（初始化）
//   userhook_FastLoop    - 100Hz（快速循环）
//   userhook_50Hz        - 50Hz
//   userhook_MediumLoop  - 10Hz（中速循环）
//   userhook_SlowLoop    - 3.3Hz（慢速循环）
//   userhook_SuperSlowLoop - 1Hz（超慢循环）

#include "Sub.h"

#ifdef USERHOOK_INIT
void Sub::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Sub::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Sub::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Sub::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Sub::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Sub::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
