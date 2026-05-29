// terrain.cpp - 地形数据更新和日志
// 对水下 ROV 来说，“地形”对应海底地形
// AP_TERRAIN 库用于基于 GPS 的海底地形跟随（需要海图数据）

#include "Sub.h"

// terrain_update - 更新地形数据
// 如果有地形高度数据，将其提供给测距仪用于省电模式判断
void Sub::terrain_update()
{
#if AP_TERRAIN_AVAILABLE
    terrain.update();

    // 将地形高度提供给测距仪，以便在距离太远时进入省电模式
#if AP_RANGEFINDER_ENABLED
    float height;
    if (terrain.height_above_terrain(height, true)) {
        rangefinder.set_estimated_terrain_height(height);
    }
#endif
#endif
}

#if HAL_LOGGING_ENABLED
// terrain_logging - 记录地形数据（应以 1Hz 调用）
void Sub::terrain_logging()
{
#if AP_TERRAIN_AVAILABLE
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }
#endif
}
#endif  // HAL_LOGGING_ENABLED

