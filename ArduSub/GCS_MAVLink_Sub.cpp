#include "Sub.h"

#include "GCS_MAVLink_Sub.h"
#include <AP_RPM/AP_RPM_config.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

// GCS_MAVLink_Sub.cpp - ArduSub 与地面站(MAVLink)通信适配层
// 主要职责：
//  1) 下行：向地面站发送状态/导航/调参/水深等遥测
//  2) 上行：接收地面站控制指令并转发到 Sub 各控制模块
//  3) 命令分发：处理 COMMAND_INT、手动控制、位置/姿态目标等消息

MAV_TYPE GCS_Sub::frame_type() const
{
    // 声明机体类型为潜航器，供地面站识别与界面适配
    return MAV_TYPE_SUBMARINE;
}

// base_mode 是 MAVLink 通用模式位掩码，主要给“通用地面站”做兼容显示。
// 对 ArduSub 来说，真正有精确定义的是 custom_mode（即 ArduSub 自己的模式编号）。
uint8_t GCS_MAVLINK_Sub::base_mode() const
{
    // 组装 MAVLink 标准 base_mode 位掩码（兼容通用地面站）
    uint8_t _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (sub.control_mode) {
    case Mode::Number::AUTO:
    case Mode::Number::GUIDED:
    case Mode::Number::CIRCLE:
    case Mode::Number::POSHOLD:
        _base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    default:
        break;
    }

    // all modes except INITIALISING have some form of manual
    // override if stick mixing is enabled
    _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    if (sub.motors.armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return _base_mode;
}

uint32_t GCS_Sub::custom_mode() const
{
    // ArduSub 的真实模式语义通过 custom_mode 传递
    return (uint32_t)sub.control_mode;
}

// system_status 是地面站状态灯/状态字的重要来源。
// 这里把 Sub 内部状态压缩成 MAVLink 标准枚举，便于 QGC 或其他 GCS 统一显示。
MAV_STATE GCS_MAVLINK_Sub::vehicle_system_status() const
{
    // 向地面站报告系统状态优先级：failsafe > armed > boot > standby
    // set system as critical if any failsafe have triggered
    if (sub.any_failsafe_triggered())  {
        return MAV_STATE_CRITICAL;
    }

    if (sub.motors.armed()) {
        return MAV_STATE_ACTIVE;
    }
    if (!sub.ap.initialised) {
    	return MAV_STATE_BOOT;
    }

    return MAV_STATE_STANDBY;
}

void GCS_MAVLINK_Sub::send_banner()
{
    // 启动横幅 + 当前机架类型字符串
    GCS_MAVLINK::send_banner();
    send_text(MAV_SEVERITY_INFO, "Frame: %s", sub.motors.get_frame_string());
}

// 这个消息给地面站展示“导航控制器当前在追什么目标”。
// 其中姿态目标来自 attitude_control，航向/距离来自 wp_nav，垂向误差来自 pos_control。
void GCS_MAVLINK_Sub::send_nav_controller_output() const
{
    // 发送导航控制输出（目标姿态、航点方位/距离、垂向误差）
    const Vector3f &targets = sub.attitude_control.get_att_target_euler_cd();
    mavlink_msg_nav_controller_output_send(
        chan,
        targets.x * 1.0e-2f,
        targets.y * 1.0e-2f,
        targets.z * 1.0e-2f,
        sub.wp_nav.get_wp_bearing_to_destination_cd() * 1.0e-2f,
        MIN(sub.wp_nav.get_wp_distance_to_destination_cm() * 1.0e-2f, UINT16_MAX),
        sub.pos_control.get_pos_error_U_cm() * 1.0e-2f,
        0,
        0);
}

// returns a Location to which the vehicle is currently heading, or
// would head in an autonomous mode
bool GCS_MAVLINK_Sub::get_target_location(Location &loc) const
{
    // 仅在 wp_nav 激活时返回当前目标点
    return sub.wp_nav.is_active() && sub.wp_nav.get_wp_destination_loc(loc);
}

int16_t GCS_MAVLINK_Sub::vfr_hud_throttle() const
{
    // HUD 油门按百分比上报
    return (int16_t)(sub.motors.get_throttle() * 100);
}

float GCS_MAVLINK_Sub::vfr_hud_alt() const
{
    // HUD 高度使用海平面高度(MSL)
    return sub.get_alt_msl();
}

// 借用 scaled_pressure3 消息通道把温度传感器数据送出去。
// 如果没有温度数据，就退回基类的标准实现。
void GCS_MAVLINK_Sub::send_scaled_pressure3()
{
#if AP_TEMPERATURE_SENSOR_ENABLED
    float temperature;
    if (!sub.temperature_sensor.get_temperature(temperature)) {
        // Fall back to original behaviour
        GCS_MAVLINK::send_scaled_pressure3();
        return;
    }
    mavlink_msg_scaled_pressure3_send(
        chan,
        AP_HAL::millis(),
        0,
        0,
        temperature * 100,
        0); // TODO: use differential pressure temperature
#else
    // Fall back to standard behaviour
    GCS_MAVLINK::send_scaled_pressure3();
#endif
}

bool GCS_MAVLINK_Sub::send_info()
{
    // 以 NAMED_VALUE_FLOAT 打包发送 Sub 特有状态（云台/灯光/系绳等）
    // Just do this all at once, hopefully the hard-wire telemetry requirement means this is ok
    // Name is char[10]
    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("CamTilt",
                     1 - (SRV_Channels::get_output_norm(SRV_Channel::k_mount_tilt) / 2.0f + 0.5f));

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("CamPan",
                     1 - (SRV_Channels::get_output_norm(SRV_Channel::k_mount_pan) / 2.0f + 0.5f));

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("TetherTrn",
                     (float)sub.quarter_turn_count / 4);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("Lights1",
                     SRV_Channels::get_output_norm(SRV_Channel::k_lights1) / 2.0f + 0.5f);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("Lights2",
                     SRV_Channels::get_output_norm(SRV_Channel::k_lights2) / 2.0f + 0.5f);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("PilotGain", sub.gain);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("InputHold", sub.input_hold_engaged);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("RollPitch", sub.roll_pitch_flag);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("RFTarget", sub.mode_surftrak.get_rangefinder_target_cm() * 0.01f);

    return true;
}

#if AP_RANGEFINDER_ENABLED
// 水深消息不是所有测距仪都能发，必须是朝下安装的测距仪。
// 这里每个循环只发一条 WATER_DEPTH，避免链路被大消息持续占满。
void GCS_MAVLINK_Sub::send_water_depth()
{
    // 发送 WATER_DEPTH：仅支持“朝下”测距传感器（ROTATION_PITCH_270）
    if (!HAVE_PAYLOAD_SPACE(chan, WATER_DEPTH)) {
        return;
    }

    RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder == nullptr) {
        return;
    }

    // depth can only be measured by a downward-facing rangefinder:
    if (!rangefinder->has_orientation(ROTATION_PITCH_270)) {
        return;
    }

    // get position
    const AP_AHRS &ahrs = AP::ahrs();
    Location loc;
    IGNORE_RETURN(ahrs.get_location(loc));

    const auto num_sensors = rangefinder->num_sensors();
    for (uint8_t i=0; i<num_sensors; i++) {
        last_WATER_DEPTH_index += 1;
        if (last_WATER_DEPTH_index >= num_sensors) {
            last_WATER_DEPTH_index = 0;
        }

        const AP_RangeFinder_Backend *s = rangefinder->get_backend(last_WATER_DEPTH_index);
        if (s == nullptr || s->orientation() != ROTATION_PITCH_270 || !s->has_data()) {
            continue;
        }

        // get temperature
        float temp_C;
        if (!s->get_temp(temp_C)) {
            // TODO: check known water temperature sources (temp sensor, external baro)
            temp_C = 0.0f;
        }

        const bool sensor_healthy = (s->status() == RangeFinder::Status::Good);

        mavlink_msg_water_depth_send(
            chan,
            AP_HAL::millis(),   // time since system boot TODO: take time of measurement
            last_WATER_DEPTH_index, // rangefinder instance
            sensor_healthy,     // sensor healthy
            loc.lat,            // latitude of vehicle
            loc.lng,            // longitude of vehicle
            loc.alt * 0.01f,    // altitude of vehicle (MSL)
            ahrs.get_roll_rad(),    // roll in radians
            ahrs.get_pitch_rad(),   // pitch in radians
            ahrs.get_yaw_rad(),     // yaw in radians
            s->distance(),    // distance in meters
            temp_C);            // temperature in degC

        break;  // only send one WATER_DEPTH message per loop
    }
}
#endif  // AP_RANGEFINDER_ENABLED

// 向地面站发送 PID 调参数据。
// 哪些轴要发由 g.gcs_pid_mask 控制，避免无意义地占用带宽。
void GCS_MAVLINK_Sub::send_pid_tuning()
{
    // 根据 gcs_pid_mask 选择性发送 PID_TUNING（roll/pitch/yaw/accz）
    const Parameters &g = sub.g;
    AP_AHRS &ahrs = AP::ahrs();
    AC_AttitudeControl_Sub &attitude_control = sub.attitude_control;

    const Vector3f &gyro = ahrs.get_gyro();
    if (g.gcs_pid_mask & 1) {
        const AP_PIDInfo &pid_info = attitude_control.get_rate_roll_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ROLL,
                                    pid_info.target*0.01f,
                                    degrees(gyro.x),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f,
                                    pid_info.slew_rate,
                                    pid_info.Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 2) {
        const AP_PIDInfo &pid_info = attitude_control.get_rate_pitch_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_PITCH,
                                    pid_info.target*0.01f,
                                    degrees(gyro.y),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f,
                                    pid_info.slew_rate,
                                    pid_info.Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 4) {
        const AP_PIDInfo &pid_info = attitude_control.get_rate_yaw_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_YAW,
                                    pid_info.target*0.01f,
                                    degrees(gyro.z),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f,
                                    pid_info.slew_rate,
                                    pid_info.Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 8) {
        const AP_PIDInfo &pid_info = sub.pos_control.D_get_accel_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ACCZ,
                                    pid_info.target*0.01f,
                                    -(ahrs.get_accel_ef().z + GRAVITY_MSS),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f,
                                    pid_info.slew_rate,
                                    pid_info.Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
}

bool GCS_Sub::vehicle_initialised() const {
    // 供 GCS 查询系统初始化完成状态
    return sub.ap.initialised;
}

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_Sub::try_send_message(enum ap_message id)
{
    // 发送调度入口：Sub 自定义消息优先，其余回退到基类
    switch (id) {

    case MSG_NAMED_FLOAT:
        send_info();
        break;

    case MSG_WIND: // other vehicles do something custom with wind:
        return true;

#if AP_RANGEFINDER_ENABLED
    case MSG_WATER_DEPTH:
        CHECK_PAYLOAD_SIZE(WATER_DEPTH);
        send_water_depth();
        break;
#endif  // AP_RANGEFINDER_ENABLED

    default:
        return GCS_MAVLINK::try_send_message(id);
    }

    return true;
}

bool GCS_MAVLINK_Sub::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    // Guided 任务请求交由 Sub 核心处理
    return sub.do_guided(cmd);
}

// 预飞气压计标定：为避免运行中重置基准，要求潜航器处于未解锁状态。
MAV_RESULT GCS_MAVLINK_Sub::_handle_command_preflight_calibration_baro(const mavlink_message_t &msg)
{
    // 气压计校准要求未解锁，避免运行中重标定
    if (sub.motors.armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Disarm before calibration.");
        return MAV_RESULT_FAILED;
    }

    if (!sub.control_check_barometer()) {
        return MAV_RESULT_FAILED;
    }

    AP::baro().calibrate(true);
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Sub::_handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    if (packet.y == 1) {
        // compassmot calibration
        //result = sub.mavlink_compassmot(chan);
        gcs().send_text(MAV_SEVERITY_INFO, "#CompassMot calibration not supported");
        return MAV_RESULT_UNSUPPORTED;
    }

    return GCS_MAVLINK::_handle_command_preflight_calibration(packet, msg);
}

MAV_RESULT GCS_MAVLINK_Sub::handle_command_do_set_roi(const Location &roi_loc)
{
    // ROI 的本质是“摄像头/机体朝向某个感兴趣目标点”
    if (!roi_loc.check_latlng()) {
        return MAV_RESULT_FAILED;
    }
    sub.mode_auto.set_auto_yaw_roi(roi_loc);
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Sub::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
    // DO_REPOSITION：可选切到 GUIDED 并设置新目标点
    const bool change_modes = ((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) == MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;
    if (!sub.flightmode->in_guided_mode() && !change_modes) {
        return MAV_RESULT_DENIED;
    }

    // sanity check location
    if (!check_latlng(packet.x, packet.y)) {
        return MAV_RESULT_DENIED;
    }

    Location request_location;
    if (!location_from_command_t(packet, request_location)) {
        return MAV_RESULT_DENIED;
    }

    if (request_location.sanitize(sub.current_loc)) {
        // if the location wasn't already sane don't load it
        return MAV_RESULT_DENIED; // failed as the location is not valid
    }

    // 先尝试写入目标点，再切模式。
    // 这样可以避免“模式切过去了，但目标点无效”的半完成状态。
    if (!sub.mode_guided.guided_set_destination(request_location)) {
        return MAV_RESULT_FAILED;
    }

    if (!sub.flightmode->in_guided_mode()) {
        if (!sub.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        // the position won't have been loaded if we had to change the flight mode, so load it again
        if (!sub.mode_guided.guided_set_destination(request_location)) {
            return MAV_RESULT_FAILED;
        }
    }

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Sub::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    // COMMAND_INT 分发：将常用任务/航向/速度/电机测试命令映射到对应处理器
    switch(packet.command) {

    case MAV_CMD_CONDITION_YAW:
        return handle_MAV_CMD_CONDITION_YAW(packet);

    case MAV_CMD_DO_CHANGE_SPEED:
        return handle_MAV_CMD_DO_CHANGE_SPEED(packet);

    case MAV_CMD_DO_MOTOR_TEST:
        return handle_MAV_CMD_DO_MOTOR_TEST(packet);

    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);

    case MAV_CMD_MISSION_START:
        if (!is_zero(packet.param1) || !is_zero(packet.param2)) {
            // first-item/last item not supported
            return MAV_RESULT_DENIED;
        }
        return handle_MAV_CMD_MISSION_START(packet);

    case MAV_CMD_NAV_LOITER_UNLIM:
        return handle_MAV_CMD_NAV_LOITER_UNLIM(packet);

    case MAV_CMD_NAV_LAND:
        return handle_MAV_CMD_NAV_LAND(packet);

    }

    return GCS_MAVLINK::handle_command_int_packet(packet, msg);
}

MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_NAV_LOITER_UNLIM(const mavlink_command_int_t &packet)
{
    // Sub 中用 POSHOLD 对应“无限悬停”语义
        if (!sub.set_mode(Mode::Number::POSHOLD, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_NAV_LAND(const mavlink_command_int_t &packet)
{
    // Sub 的 LAND 语义映射到 SURFACE（上浮到水面）
        if (!sub.set_mode(Mode::Number::SURFACE, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_CONDITION_YAW(const mavlink_command_int_t &packet)
{
    // 在任务/引导场景中修改期望朝向。
    // param4=1 表示相对当前朝向偏移，param4=0 表示绝对航向角。
        // param1 : target angle [0-360]
        // param2 : speed during change [deg per second]
        // param3 : direction (-1:ccw, +1:cw)
        // param4 : relative offset (1) or absolute angle (0)
        if ((packet.param1 >= 0.0f)   &&
            (packet.param1 <= 360.0f) &&
            (is_zero(packet.param4) || is_equal(packet.param4,1.0f))) {
            sub.mode_auto.set_auto_yaw_look_at_heading(packet.param1, packet.param2, (int8_t)packet.param3, (uint8_t)packet.param4);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_DENIED;
}

MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_DO_CHANGE_SPEED(const mavlink_command_int_t &packet)
{
    // 仅接受正速度；AIRSPEED 在 Sub 中按地速兼容处理
    if (!is_positive(packet.param2)) {
        // Target speed must be larger than zero
        return MAV_RESULT_DENIED;
    }

    switch (SPEED_TYPE(packet.param1)) {
        case SPEED_TYPE_CLIMB_SPEED:
        case SPEED_TYPE_DESCENT_SPEED:
        case SPEED_TYPE_ENUM_END:
            break;

        case SPEED_TYPE_AIRSPEED: // Airspeed is treated as ground speed for GCS compatibility
        case SPEED_TYPE_GROUNDSPEED:
            sub.wp_nav.set_speed_NE_cms(packet.param2 * 100.0);
            return MAV_RESULT_ACCEPTED;
    }

    return MAV_RESULT_DENIED;
}

MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_MISSION_START(const mavlink_command_int_t &packet)
{
    // 已解锁时允许切 AUTO 并启动任务
        if (sub.motors.armed() && sub.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
}

MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet)
{
        // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
        // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
        // param3 : throttle (range depends upon param2)
        // param4 : timeout (in seconds)
        if (!sub.handle_do_motor_test(packet)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
}

// this is called on receipt of a MANUAL_CONTROL packet and is
// expected to call manual_override to override RC input on desired
// axes.
void GCS_MAVLINK_Sub::handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow)
{
    // 将 MANUAL_CONTROL 映射为 RC override 通道输入
        sub.transform_manual_control_to_rc_override(
            packet.x,
            packet.y,
            packet.z,
            packet.r,
            packet.buttons,
            packet.buttons2,
            packet.enabled_extensions,
            packet.s,
            packet.t,
            packet.aux1,
            packet.aux2,
            packet.aux3,
            packet.aux4,
            packet.aux5,
            packet.aux6
        );

        sub.failsafe.last_pilot_input_ms = AP_HAL::millis();
}

void GCS_MAVLINK_Sub::handle_message(const mavlink_message_t &msg)
{
    // 消息总入口：处理 Sub 关心的 MAVLink 消息，其余回退基类
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {     // MAV ID: 70
        if (!gcs().sysid_is_gcs(msg.sysid)) {
            break;    // Only accept control from our gcs
        }

        sub.failsafe.last_pilot_input_ms = AP_HAL::millis();
        // a RC override message is considered to be a 'heartbeat'
        // from the ground station for failsafe purposes
        // 也就是说：即使没收到独立 heartbeat，只要持续有 override，就认为地面站仍在线。
        
        handle_rc_channels_override(msg);
        break;
    }

    
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET: { // MAV ID: 82
        // decode packet
        mavlink_set_attitude_target_t packet;
        mavlink_msg_set_attitude_target_decode(&msg, &packet);

        // ensure type_mask specifies to use attitude
        // the thrust can be used from the altitude hold
        if (packet.type_mask & (1<<6)) {
            sub.set_attitude_target_no_gps = {AP_HAL::millis(), packet};
        }

        // ensure type_mask specifies to use attitude and thrust
        if ((packet.type_mask & ((1<<7)|(1<<6))) != 0) {
            break;
        }

        // 把 0~1 的 thrust 重新映射为“升沉速度”指令：
        // 0.5 表示保持深度，>0.5 上升，<0.5 下潜。
        packet.thrust = constrain_float(packet.thrust, 0.0f, 1.0f);
        float climb_rate_cms = 0.0f;
        if (is_equal(packet.thrust, 0.5f)) {
            climb_rate_cms = 0.0f;
        } else if (packet.thrust > 0.5f) {
            // climb at up to WP_SPD_UP
            climb_rate_cms = (packet.thrust - 0.5f) * 2.0f * sub.wp_nav.get_default_speed_up_cms();
        } else {
            // descend at up to WP_SPD_DN
            climb_rate_cms = (packet.thrust - 0.5f) * 2.0f * sub.wp_nav.get_default_speed_down_cms();
        }
        sub.mode_guided.guided_set_angle(Quaternion(packet.q[0],packet.q[1],packet.q[2],packet.q[3]), climb_rate_cms);
        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: {   // MAV ID: 84
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if ((sub.control_mode != Mode::Number::GUIDED) && !(sub.control_mode == Mode::Number::AUTO && sub.auto_mode == Auto_NavGuided)) {
            break;
        }

        // check for supported coordinate frames
        if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
                packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_FRD) {
            break;
        }

        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
        bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
        bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

        // prepare position: 按坐标系做单位换算、机体系旋转、offset 补偿
        // MAVLink 本地目标以米为单位，这里统一转换成 ArduPilot 内部常用的厘米。
        Vector3f pos_vector;
        if (!pos_ignore) {
            // convert to cm
            pos_vector = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
            // rotate from body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_FRD ||
                    packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                sub.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
            }
            // add body offset if necessary
            if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_FRD ||
                    packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                Vector3f pos_cm = (sub.pos_control.get_pos_estimate_NED_m() * 100.0f).tofloat();
                pos_cm.z = -pos_cm.z;
                pos_vector += pos_cm;
            }
        }

        // prepare velocity: 同样处理机体系到 NE 旋转
        Vector3f vel_vector;
        if (!vel_ignore) {
            // convert to cm
            vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
            // rotate from body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_FRD || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                sub.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
            }
        }

        // prepare yaw: 转换到 centi-degree，区分相对/绝对偏航
        float yaw_cd =  0.0f;
        bool yaw_relative = false;
        float yaw_rate_cds = 0.0f;
        if (!yaw_ignore) {
            yaw_cd = degrees(packet.yaw) * 100.0f;
            yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
        }
        if (!yaw_rate_ignore) {
            yaw_rate_cds = degrees(packet.yaw_rate) * 100.0f;
        }

        // send request: 仅支持 pos+vel、vel-only、pos-only（acc 需忽略）
        // 这里并没有处理“加速度目标”控制，收到这类组合会被忽略。
        if (!pos_ignore && !vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_destination_posvel(pos_vector, vel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_velocity(vel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_destination(pos_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        }

        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: {  // MAV ID: 86
        // decode packet
        mavlink_set_position_target_global_int_t packet;
        mavlink_msg_set_position_target_global_int_decode(&msg, &packet);

        // exit if vehicle is not in Guided, Auto-Guided, or Depth Hold modes
        if ((sub.control_mode != Mode::Number::GUIDED)
            && !(sub.control_mode == Mode::Number::AUTO && sub.auto_mode == Auto_NavGuided)
            && !(sub.control_mode == Mode::Number::ALT_HOLD)) {
            break;
        }

        bool z_ignore        = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_Z_IGNORE;
        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         * bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
         * bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
         */

        if (!z_ignore && sub.control_mode == Mode::Number::ALT_HOLD) { // ALT_HOLD 下仅接管目标深度
            sub.pos_control.set_pos_desired_U_cm(packet.alt*100);
            break;
        }

        Vector3f pos_neu_cm;  // NEU 坐标（厘米）

        if (!pos_ignore) {
            // sanity check location
            if (!check_latlng(packet.lat_int, packet.lon_int)) {
                break;
            }
            Location::AltFrame frame;
            if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)packet.coordinate_frame, frame)) {
                // unknown coordinate frame
                break;
            }
            const Location loc{
                packet.lat_int,
                packet.lon_int,
                int32_t(packet.alt*100),
                frame,
            };
            if (!loc.get_vector_from_origin_NEU_cm(pos_neu_cm)) {
                break;
            }
        }

        if (!pos_ignore && !vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_destination_posvel(pos_neu_cm, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_destination(pos_neu_cm);
        }

        break;
    }

    // 远端漏水传感器支持：例如独立舱体通过 MAVLink SYS_STATUS 上报漏水状态。
    case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t packet;
        mavlink_msg_sys_status_decode(&msg, &packet);
        if ((msg.sysid == gcs().sysid_this_mav()) &&
            (packet.onboard_control_sensors_present & MAV_SYS_STATUS_EXTENSION_USED) &&
            (packet.onboard_control_sensors_enabled_extended & MAV_SYS_STATUS_SENSOR_LEAK) &&
            !(packet.onboard_control_sensors_health_extended & MAV_SYS_STATUS_SENSOR_LEAK)) {
            sub.leak_detector.set_detect();
        }
        break;
    }

    default:
        GCS_MAVLINK::handle_message(msg);
        break;
    }     // end switch
} // end handle mavlink

uint64_t GCS_MAVLINK_Sub::capabilities() const
{
    // 向地面站声明能力位：任务协议、位置/姿态目标、终止等
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
#if AP_TERRAIN_AVAILABLE
            (sub.terrain.enabled() ? MAV_PROTOCOL_CAPABILITY_TERRAIN : 0) |
#endif
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
            GCS_MAVLINK::capabilities()
        );
}

MAV_RESULT GCS_MAVLINK_Sub::handle_flight_termination(const mavlink_command_int_t &packet)
{
    // 飞行终止在 Sub 中等价为强制加锁
    if (packet.param1 > 0.5f) {
        sub.arming.disarm(AP_Arming::Method::TERMINATION);
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

int32_t GCS_MAVLINK_Sub::global_position_int_alt() const
{
    // GLOBAL_POSITION_INT 使用毫米，这里把米转换成 mm
    return static_cast<int32_t>(sub.get_alt_msl() * 1000.0f);
}

int32_t GCS_MAVLINK_Sub::global_position_int_relative_alt() const
{
    // 相对高度同样按毫米上报
    return static_cast<int32_t>(sub.get_alt_rel() * 1000.0f);
}

#if HAL_HIGH_LATENCY2_ENABLED
int16_t GCS_MAVLINK_Sub::high_latency_target_altitude() const
{
    AP_AHRS &ahrs = AP::ahrs();
    Location global_position_current;
    UNUSED_RESULT(ahrs.get_location(global_position_current));

    // 高延迟链路下的目标高度，单位为米
    if (sub.control_mode == Mode::Number::AUTO || sub.control_mode == Mode::Number::GUIDED) {
        return 0.01 * (global_position_current.alt + sub.pos_control.get_pos_error_U_cm());
    }
    return 0;
    
}

uint8_t GCS_MAVLINK_Sub::high_latency_tgt_heading() const
{
    // 高延迟链路下的目标航向，单位为 deg/2
    if (sub.control_mode == Mode::Number::AUTO || sub.control_mode == Mode::Number::GUIDED) {
        // need to convert -18000->18000 to 0->360/2
        return wrap_360_cd(sub.wp_nav.get_wp_bearing_to_destination_cd()) / 200;
    }
    return 0;      
}
    
uint16_t GCS_MAVLINK_Sub::high_latency_tgt_dist() const
{
    // 高延迟链路下的目标距离，单位为分米
    if (sub.control_mode == Mode::Number::AUTO || sub.control_mode == Mode::Number::GUIDED) {
        return MIN(sub.wp_nav.get_wp_distance_to_destination_cm() * 0.001, UINT16_MAX);
    }
    return 0;
}

uint8_t GCS_MAVLINK_Sub::high_latency_tgt_airspeed() const
{
    // 高延迟链路下的目标速度，单位为 m/s*5
    if (sub.control_mode == Mode::Number::AUTO || sub.control_mode == Mode::Number::GUIDED) {
        return MIN((sub.pos_control.get_vel_desired_NEU_cms().length()/100) * 5, UINT8_MAX);
    }
    return 0;
}
#endif // HAL_HIGH_LATENCY2_ENABLED

// Send the mode with the given index (not mode number!) return the total number of modes
// Index starts at 1
uint8_t GCS_MAVLINK_Sub::send_available_mode(uint8_t index) const
{
    // 上报支持的模式列表，供 GCS 动态构建模式选择界面
    const Mode* modes[] {
        &sub.mode_manual,
        &sub.mode_stabilize,
        &sub.mode_acro,
        &sub.mode_althold,
        &sub.mode_surftrak,
        &sub.mode_poshold,
        &sub.mode_auto,
        &sub.mode_guided,
        &sub.mode_circle,
        &sub.mode_surface,
        &sub.mode_motordetect,
    };

    const uint8_t mode_count = ARRAY_SIZE(modes);

    // Convert to zero indexed
    const uint8_t index_zero = index - 1;
    if (index_zero >= mode_count) {
        // Mode does not exist!?
        return mode_count;
    }

    // 让每个模式对象自己提供名称和编号，避免这里硬编码字符串映射表
    const char* name = modes[index_zero]->name();
    const uint8_t mode_number = (uint8_t)modes[index_zero]->number();

    mavlink_msg_available_modes_send(
        chan,
        mode_count,
        index,
        MAV_STANDARD_MODE::MAV_STANDARD_MODE_NON_STANDARD,
        mode_number,
        0, // MAV_MODE_PROPERTY bitmask
        name
    );

    return mode_count;
}
