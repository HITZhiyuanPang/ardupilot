// mode.h - ArduSub 飞行模式基类及枚举定义
// ArduSub 支持的飞行模式：
//   MANUAL(19)    - 直通模式：飞手输入直接驱动推进器，无任何稳定控制
//   STABILIZE(0)  - 稳定模式：自动稳定横滚/俯仰，飞手控制偏航和深度
//   ACRO(1)       - 特技模式：飞手控制角速率，无自动水平恢复
//   ALT_HOLD(2)   - 深度保持：自动维持当前深度，飞手控制水平方向
//   AUTO(3)       - 全自动任务：按航点执行预设任务
//   GUIDED(4)     - 引导模式：接受 GCS 实时坐标/速度指令
//   CIRCLE(7)     - 圆形航迹：自动绕圆飞行
//   SURFACE(9)    - 自动上浮：返回水面
//   POSHOLD(16)   - 位置保持：GPS 悬停，飞手可临时覆盖
//   MOTOR_DETECT(20) - 电机方向检测：自动识别推进器安装方向
//   SURFTRAK(21)  - 海底跟踪：保持与海底固定距离

#pragma once

#include "Sub.h"
class Parameters;
class ParametersG2;

class GCS_Sub;

// Guided modes
enum GuidedSubMode {
    Guided_WP,
    Guided_Velocity,
    Guided_PosVel,
    Guided_Angle,
};

// Auto modes
enum AutoSubMode {
    Auto_WP,
    Auto_CircleMoveToEdge,
    Auto_Circle,
    Auto_NavGuided,
    Auto_Loiter,
    Auto_TerrainRecover
};

// RTL states
enum RTLState {
    RTL_InitialClimb,
    RTL_ReturnHome,
    RTL_LoiterAtHome,
    RTL_FinalDescent,
    RTL_Land
};

class Mode
{

public:

    // Auto Pilot Modes enumeration
    // 飞行模式枚举：数字与 MAVLink 和地面站显示保持一致
    enum class Number : uint8_t {
        STABILIZE =     0,  // 稳定模式：手动横滚/俯仰角度 + 手动深度/油门
        ACRO =          1,  // 特技模式：手动机体坐标角速率 + 手动深度/油门
        ALT_HOLD =      2,  // 深度保持模式：手动角度 + 自动深度/油门
        AUTO =          3,  // 全自动航点任务
        GUIDED =        4,  // 引导模式：GCS 实时发送坐标或速度指令
        CIRCLE =        7,  // 圆形航迹模式：自动圆形飞行 + 自动油门
        SURFACE =       9,  // 自动上浮：保持水平位置不动，垂直返回水面
        POSHOLD =      16,  // 位置保持：GPS 悬停 + 手动覆盖 + 自动油门
        MANUAL =       19,  // 直通模式：无任何稳定，飞手完全控制
        MOTOR_DETECT = 20,  // 电机方向检测：自动检测推进器安装朝向
        SURFTRAK =     21   // 海底距离跟踪：保持与海底固定高度
        // 模式编号 30 保留给外部/Lua 脚本控制
    };

    // constructor
    Mode(void);

    // do not allow copying
    CLASS_NO_COPY(Mode);

    // 子类需要重写的核心虚函数
    virtual bool init(bool ignore_checks) { return true; } // 进入模式时的初始化
    virtual void run() = 0;                                // 每个控制周期调用（纯虚）
    virtual bool requires_GPS() const = 0;                 // 是否需要 GPS 定位
    virtual bool requires_altitude() const = 0;            // 是否需要高度估计（气压计/EKF）
    virtual bool allows_arming(bool from_gcs) const = 0;   // 是否允许在此模式下解锁
    virtual bool is_autopilot() const { return false; }    // 是否为全自动驾驶模式
    virtual bool in_guided_mode() const { return false; }  // 是否接受 GCS 引导指令

    // return a string for this flightmode
    virtual const char *name() const = 0;    // 模式名称（用于 GCS 显示）
    virtual const char *name4() const = 0;   // 4字符缩写

    // returns a unique number specific to this mode
    virtual Mode::Number number() const = 0;
  
    // pilot input processing
    void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out);


protected:

    // navigation support functions
    virtual void run_autopilot() {}

    // helper functions
    bool is_disarmed_or_landed() const;

    // functions to control landing
    // in modes that support landing
    void land_run_horizontal_control();
    void land_run_vertical_control(bool pause_descent = false);

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AP_AHRS &ahrs;
    AP_Motors6DOF &motors;
    RC_Channel *&channel_roll;
    RC_Channel *&channel_pitch;
    RC_Channel *&channel_throttle;
    RC_Channel *&channel_yaw;
    RC_Channel *&channel_forward;
    RC_Channel *&channel_lateral;
    AC_PosControl *position_control;
    AC_AttitudeControl_Sub *attitude_control;
    // TODO: channels
    float &G_Dt;

public:

    // pass-through functions to reduce code churn on conversion;
    // these are candidates for moving into the Mode base
    // class.
    bool set_mode(Mode::Number mode, ModeReason reason);
    GCS_Sub &gcs();

    // end pass-through functions
};

class ModeManual : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;
    virtual void run() override;
    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "Manual"; }
    const char *name4() const override { return "MANU"; }
    Mode::Number number() const override { return Mode::Number::MANUAL; }
};


class ModeAcro : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "Acro"; }
    const char *name4() const override { return "ACRO"; }
    Mode::Number number() const override { return Mode::Number::ACRO; }
};


class ModeStabilize : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "Stabilize"; }
    const char *name4() const override { return "STAB"; }
    Mode::Number number() const override { return Mode::Number::STABILIZE; }
};


class ModeAlthold : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }
    void control_depth();

protected:

    void run_pre();
    void run_post();

    const char *name() const override { return "Depth Hold"; }
    const char *name4() const override { return "ALTH"; }
    Mode::Number number() const override { return Mode::Number::ALT_HOLD; }
};


class ModeSurftrak : public ModeAlthold
{

public:
    // constructor
    ModeSurftrak();

    void run() override;

    bool init(bool ignore_checks) override;

    float get_rangefinder_target_cm() const WARN_IF_UNUSED { return rangefinder_target_cm; }
    bool set_rangefinder_target_cm(float target_cm);

protected:

    const char *name() const override { return "Surftrak"; }
    const char *name4() const override { return "STRK"; }
    Mode::Number number() const override { return Mode::Number::SURFTRAK; }

private:

    void reset();
    void control_range();
    void update_surface_offset();

    float rangefinder_target_cm;

    bool pilot_in_control;            // pilot is moving up/down
    float pilot_control_start_z_cm;   // alt when pilot took control
};

class ModeGuided : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }
    bool in_guided_mode() const override { return true; }
    bool guided_limit_check();
    void guided_limit_init_time_and_pos();
    void guided_set_angle(const Quaternion &q, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads);
    void guided_set_angle(const Quaternion&, float);
    void guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    bool guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity);
    bool guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    bool guided_set_destination(const Vector3f& destination);
    bool guided_set_destination(const Location&);
    bool guided_set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    void guided_set_velocity(const Vector3f& velocity);
    void guided_set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    void guided_set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle);
    float get_auto_heading();
    void guided_limit_clear();
    void set_auto_yaw_mode(autopilot_yaw_mode yaw_mode);

protected:

    const char *name() const override { return "Guided"; }
    const char *name4() const override { return "GUID"; }
    Mode::Number number() const override { return Mode::Number::GUIDED; }

    autopilot_yaw_mode get_default_auto_yaw_mode(bool rtl) const;

private:
    void guided_pos_control_run();
    void guided_vel_control_run();
    void guided_posvel_control_run();
    void guided_angle_control_run();
    void guided_takeoff_run();
    void guided_pos_control_start();
    void guided_vel_control_start();
    void guided_posvel_control_start();
    void guided_angle_control_start();
};



class ModeAuto : public ModeGuided
{

public:
    // inherit constructor
    using ModeGuided::ModeGuided;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }
    bool auto_loiter_start();
    void auto_wp_start(const Vector3f& destination);
    void auto_wp_start(const Location& dest_loc);
    void auto_circle_movetoedge_start(const Location &circle_center, float radius_m, bool ccw_turn);
    void auto_circle_start();
    void auto_nav_guided_start();
    void set_auto_yaw_roi(const Location &roi_location);
    void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, uint8_t relative_angle);
    void set_yaw_rate(float turn_rate_dps);
    bool auto_terrain_recover_start();

protected:

    const char *name() const override { return "Auto"; }
    const char *name4() const override { return "AUTO"; }
    Mode::Number number() const override { return Mode::Number::AUTO; }

private:
    void auto_wp_run();
    void auto_circle_run();
    void auto_nav_guided_run();
    void auto_loiter_run();
    void auto_terrain_recover_run();
};


class ModePoshold : public ModeAlthold
{

public:
    // inherit constructor
    using ModeAlthold::ModeAlthold;

    virtual void run() override;

    bool init(bool ignore_checks) override;

    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "Position Hold"; }
    const char *name4() const override { return "POSH"; }
    Mode::Number number() const override { return Mode::Number::POSHOLD; }

private:

    void control_horizontal();
};


class ModeCircle : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "Circle"; }
    const char *name4() const override { return "CIRC"; }
    Mode::Number number() const override { return Mode::Number::CIRCLE; }
};

class ModeSurface : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:
    const char *name() const override { return "Surface"; }
    const char *name4() const override { return "SURF"; }
    Mode::Number number() const override { return Mode::Number::SURFACE; }
    bool nobaro_mode;
};


class ModeMotordetect : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "Motor Detection"; }
    const char *name4() const override { return "DETE"; }
    Mode::Number number() const override { return Mode::Number::MOTOR_DETECT; }
};
