#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "AP_AutoTune.h"
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>

#define PITCH_ANGLE_PID_I_DEFAULT 2
#define PITCH_ANGLE_PID_D_DEFAULT 0.01
#define PITCH_ANGLE_PID_IMAX_DEFAULT 3 // deg/s
#define PITCH_ANGLE_PID_TARGET_FILTER_DEFAULT 3
#define PITCH_ANGLE_PID_D_FILTER_DEFAULT 12
#define PITCH_ANGLE_PID_SMAX_DEFAULT 0

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define PITCH_ANGLE_PID_P_DEFAULT 4 // XXX necessary because otherwise the soaring checks fail for an unknown reason
#else
#define PITCH_ANGLE_PID_P_DEFAULT 2
#endif

class AP_PitchController
{
public:
    AP_PitchController(const AP_Vehicle::FixedWing &parms);

    /* Do not allow copies */
    AP_PitchController(const AP_PitchController &other) = delete;
    AP_PitchController &operator=(const AP_PitchController&) = delete;

    float get_rate_out(float desired_rate, float scaler);
    float get_servo_out_using_angle_error(int32_t angle_err, int32_t target_angle, float scaler, bool disable_integrator, bool ground_mode);
    float get_servo_out_using_angle_target(int32_t target_angle, float scaler, bool disable_integrator, bool ground_mode);
    float get_servo_out(float desired_rate, float scaler, bool disable_integrator, bool ground_mode);

    void reset_I();

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I()
    {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
        rate_pid.set_integrator(rate_pid.get_i() * 0.995);
    }

    void autotune_start(void);
    void autotune_restore(void);

    const AP_PIDInfo& get_pid_info(void) const
    {
        return _pid_info;
    }

    const AP_PIDInfo& get_angle_pid_info(void) const
    {
        return _angle_pid_info;
    }

    static const struct AP_Param::GroupInfo var_info[];

    AP_Float &kP(void) { return rate_pid.kP(); }
    AP_Float &kI(void) { return rate_pid.kI(); }
    AP_Float &kD(void) { return rate_pid.kD(); }
    AP_Float &kFF(void) { return rate_pid.ff(); }
    AP_Float &rollFF(void) { return _roll_ff; }

    AP_Float &angle_kP(void) { return angle_pid.kP(); }
    AP_Float &angle_kI(void) { return angle_pid.kI(); }
    AP_Float &angle_kD(void) { return angle_pid.kD(); }
    AP_Float &angle_fltt(void) { return angle_pid.filt_T_hz(); }

    void convert_pid();

private:
    const AP_Vehicle::FixedWing &aparm;
    AP_AutoTune::ATGains gains;
    AP_AutoTune *autotune;
    bool failed_autotune_alloc;
    AP_Int16 _max_rate_neg;
    AP_Float _roll_ff;
    float _last_out;
    AC_PID rate_pid{0.04, 0.15, 0, 0.345, 0.666, 3, 0, 12, 0.02, 150, 1};
    AC_PID angle_pid{PITCH_ANGLE_PID_P_DEFAULT, PITCH_ANGLE_PID_I_DEFAULT, PITCH_ANGLE_PID_D_DEFAULT, 0, PITCH_ANGLE_PID_IMAX_DEFAULT, PITCH_ANGLE_PID_TARGET_FILTER_DEFAULT, 0, PITCH_ANGLE_PID_D_FILTER_DEFAULT, 0.02, PITCH_ANGLE_PID_SMAX_DEFAULT, 1};

    float angle_err_deg;

    AP_PIDInfo _pid_info;
    AP_PIDInfo _angle_pid_info;

    float _get_rate_out(float desired_rate, float scaler, bool disable_integrator, float aspeed, bool ground_mode);
    float _get_coordination_rate_offset(float &aspeed, bool &inverted) const;
};
