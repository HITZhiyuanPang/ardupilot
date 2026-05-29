// system.cpp - ArduSub 系统初始化和启动流程
// 包含：
//   - init_ardupilot：主初始化函数（传感器、RC、电机、GCS、调度器等）
//   - 启动自检（arming checks）
//   - 飞行模式管理辅助函数

#include "Sub.h"

// failsafe_check_static - 静态包装，供定时器中断调用（C 链接需要）
static void failsafe_check_static()
{
    sub.mainloop_failsafe_check();
}

// init_ardupilot - 主初始化函数
// 处理硬件初始化（可能是空中重启，也可能是地面冷启动）
void Sub::init_ardupilot()
{
    // 初始化通知系统（LED、蜂鸣器等）
    notify.init();

    // 初始化电池监控
    battery.init();

    barometer.init();

#if AP_FEATURE_BOARD_DETECT
    // 根据硬件板型设置外部气压计总线（水下 ROV 使用外部气压计作为深度传感器）
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
        AP_Param::set_default_by_name("BARO_EXT_BUS", 0);
        break;
    case AP_BoardConfig::PX4_BOARD_PIXHAWK:
        AP_Param::set_by_name("BARO_EXT_BUS", 1);
        break;
    default:
        AP_Param::set_default_by_name("BARO_EXT_BUS", 1);
        break;
    }
#elif CONFIG_HAL_BOARD != HAL_BOARD_LINUX
    // 非 Linux 板型默认使用外部总线 1
    AP_Param::set_default_by_name("BARO_EXT_BUS", 1);
#endif

#if AP_TEMPERATURE_SENSOR_ENABLED
    // 保持与旧版 Sub 温度传感器行为兼容，设置默认 I2C 总线
    AP_Param::set_default_by_name("TEMP1_BUS", barometer.external_bus());
#endif

    // 设置遥测串口
    gcs().setup_uarts();

    // 初始化 RC 通道（包括模式转换）
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM);
    rc().init();

    init_rc_in();               // 初始化 RC 输入通道（6个控制轴）
    init_rc_out();              // 初始化电机和 ESC 输出
    init_joystick();            // 初始化摇杆（设置飞行模式和增益）

#if AP_RELAY_ENABLED
    relay.init();
#endif

#if OSD_ENABLED
    osd.init();
#endif

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Do GPS init
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init();

    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if AP_AIRSPEED_ENABLED
    airspeed.set_log_bit(MASK_LOG_IMU);
#endif

#if AP_OPTICALFLOW_ENABLED
    // initialise optical flow sensor
    optflow.init(MASK_LOG_OPTFLOW);
#endif

#if HAL_MOUNT_ENABLED
    // initialise camera mount
    camera_mount.init();
    // This step is necessary so that the servo is properly initialized
    camera_mount.set_angle_target(0, 0, 0, false);
    // for some reason the call to set_angle_targets changes the mode to mavlink targeting!
    camera_mount.set_mode(MAV_MOUNT_MODE_RC_TARGETING);
#endif

#if AP_CAMERA_ENABLED
    // initialise camera
    camera.init();
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

    // Init baro and determine if we have external (depth) pressure sensor
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate(false);
    barometer.update();

    for (uint8_t i = 0; i < barometer.num_instances(); i++) {
        if (barometer.get_type(i) == AP_Baro::BARO_TYPE_WATER) {
            barometer.set_primary_baro(i);
            depth_sensor_idx = i;
            ap.depth_sensor_present = true;
            sensor_health.depth = barometer.healthy(depth_sensor_idx); // initialize health flag
            break; // Go with the first one we find
        }
    }

    if (!ap.depth_sensor_present) {
        // We only have onboard baro
        // No external underwater depth sensor detected
        barometer.set_primary_baro(0);
        ahrs.set_alt_measurement_noise(10.0f);  // Readings won't correspond with rest of INS
    } else {
        ahrs.set_alt_measurement_noise(0.1f);
    }

    leak_detector.init();

    last_pilot_heading_rad = ahrs.get_yaw_rad();

    // initialise rangefinder
#if AP_RANGEFINDER_ENABLED
    init_rangefinder();
#endif

    // initialise mission library
    mission.init();
#if HAL_LOGGING_ENABLED
    mission.set_log_start_mission_item_bit(MASK_LOG_CMD);
#endif

    // initialise AP_Logger library
#if HAL_LOGGING_ENABLED
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&sub, &Sub::Log_Write_Vehicle_Startup_Messages, void));
#endif

    startup_INS_ground();

    // enable CPU failsafe
    mainloop_failsafe_enable();

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // PARAMETER_CONVERSION - Added: Mar-2026
    if (g2.param_conversion_increment < 1) {
        update_actuators_from_jsbuttons();
        update_lights_from_rcin();
        g2.param_conversion_increment.set_and_save(1);
    }

    g2.actuators.initialize_actuators();

#if LEAKDETECTOR_MAX_INSTANCES > 0
    update_leak_pins();
#endif
#if AP_RELAY_ENABLED
    update_relay_pins();
#endif
    // flag that initialisation has completed
    ap.initialised = true;
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void Sub::startup_INS_ground()
{
    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::SUBMARINE);
    ahrs.set_fly_forward(false);

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());

    // reset ahrs including gyro bias
    ahrs.reset();
}

// calibrate gyros - returns true if successfully calibrated
// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Sub::position_ok()
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return (ekf_position_ok() || optflow_position_ok());
}

// ekf_position_ok - returns true if the ekf claims it's horizontal absolute position estimate is ok and home position is set
bool Sub::ekf_position_ok()
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // if disarmed we accept a predicted horizontal position
    if (!motors.armed()) {
        if (ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS)) {
            return true;
        }
        if (ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_ABS)) {
            return true;
        }
        return false;
    }

    // once armed we require a good absolute position and EKF must not be in const_pos_mode
    if (ahrs.has_status(AP_AHRS::Status::CONST_POS_MODE)) {
        return false;
    }
    return ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS);
}

// optflow_position_ok - returns true if optical flow based position estimate is ok
bool Sub::optflow_position_ok()
{
    // return immediately if EKF not used
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // return immediately if neither optflow nor visual odometry is enabled
    bool enabled = false;
#if AP_OPTICALFLOW_ENABLED
    if (optflow.enabled()) {
        enabled = true;
    }
#endif
#if HAL_VISUALODOM_ENABLED
    if (visual_odom.enabled()) {
        enabled = true;
    }
#endif
    if (!enabled) {
        return false;
    }

    // if disarmed we accept a predicted horizontal relative position
    if (!motors.armed()) {
        return ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_REL);
    }

    if (ahrs.has_status(AP_AHRS::Status::CONST_POS_MODE)) {
        return false;
    }

    return ahrs.has_status(AP_AHRS::Status::HORIZ_POS_REL);
}

#if HAL_LOGGING_ENABLED
/*
  should we log a message type now?
 */
bool Sub::should_log(uint32_t mask)
{
    ap.logging_started = logger.logging_started();
    return logger.should_log(mask);
}
#endif

#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <AP_Avoidance/AP_Avoidance.h>
#include <AP_ADSB/AP_ADSB.h>

// dummy method to avoid linking AFS
#if AP_ADVANCEDFAILSAFE_ENABLED
bool AP_AdvancedFailsafe::gcs_terminate(bool should_terminate, const char *reason) { return false; }
AP_AdvancedFailsafe *AP::advancedfailsafe() { return nullptr; }
#endif

#if AP_ADSB_AVOIDANCE_ENABLED
// dummy method to avoid linking AP_Avoidance
AP_Avoidance *AP::ap_avoidance() { return nullptr; }
#endif  // AP_ADSB_AVOIDANCE_ENABLED
