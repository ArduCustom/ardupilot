/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//	Initial Code by Jon Challinger
//  Modified by Paul Riseborough

#include <AP_HAL/AP_HAL.h>
#include "AP_PitchController.h"
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_PitchController::var_info[] = {

    // index 0 reserved for old TCONST

    // index 1 to 3 reserved for old PID values

    // @Param: _AGL_RMAX_UP
    // @DisplayName: Pitch up max rate
    // @Description: This sets the maximum nose up pitch rate that the attitude controller will demand (degrees/sec) in angle stabilized modes (all but MANUAL and ACRO). Setting it to zero disables the limit.
    // @Range: 0 100
    // @Units: deg/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_AGL_RMAX_UP",     4, AP_PitchController, gains.rmax_pos,   0.0f),

    // @Param: _AGL_RMAX_DN
    // @DisplayName: Pitch down max rate
    // @Description: This sets the maximum nose down pitch rate that the attitude controller will demand (degrees/sec) in angle stabilized modes (all but MANUAL and ACRO). Setting it to zero disables the limit.
    // @Range: 0 100
    // @Units: deg/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_AGL_RMAX_DN",     5, AP_PitchController, gains.rmax_neg,   0.0f),

    // @Param: _AGL_RLLCOMP
    // @DisplayName: Roll compensation
    // @Description: Gain added to pitch to keep aircraft from descending or ascending in turns. Increase in increments of 0.05 to reduce altitude loss. Decrease for altitude gain.
    // @Range: 0.7 1.5
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("_AGL_RLLCOMP",      6, AP_PitchController, _roll_ff,        1.0f),

    // index 7, 8 reserved for old IMAX, FF

    // @Param: _RATE_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain. Converts the difference between desired pitch rate and actual pitch rate into a control surface angle
    // @Range: 0.08 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: _RATE_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain. Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum. Constrains the maximum control surface angle that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain. Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.001 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 3.0
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(rate_pid, "_RATE_", 11, AP_PitchController, AC_PID),

    // @Param: _AGL_P
    // @DisplayName: Pitch axis angle controller P gain
    // @Description: Pitch axis angle controller P gain. Converts the difference between desired pitch angle and actual pitch angle into a pitch rate
    // @Range: 0.08 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: _AGL_I
    // @DisplayName: Pitch axis angle controller I gain
    // @Description: Pitch axis angle controller I gain. Corrects long-term difference in desired pitch angle vs actual pitch angle
    // @Range: 0.01 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: _AGL_IMAX
    // @DisplayName: Pitch axis angle controller I gain maximum
    // @Description: Pitch axis angle controller I gain maximum. Constrains the maximum pitch rate that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: _AGL_D
    // @DisplayName: Pitch axis angle controller D gain
    // @Description: Pitch axis angle controller D gain. Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.001 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: _AGL_FF
    // @DisplayName: Pitch axis angle controller feed forward
    // @Description: Pitch axis angle controller feed forward
    // @Range: 0 3.0
    // @Increment: 0.001
    // @User: Standard

    // @Param: _AGL_FLTT
    // @DisplayName: Pitch axis angle controller target frequency in Hz
    // @Description: Pitch axis angle controller target frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _AGL_FLTE
    // @DisplayName: Pitch axis angle controller error frequency in Hz
    // @Description: Pitch axis angle controller error frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _AGL_FLTD
    // @DisplayName: Pitch axis angle controller derivative frequency in Hz
    // @Description: Pitch axis angle controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _AGL_SMAX
    // @DisplayName: Requested pitch rate slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(angle_pid, "_AGL_", 12, AP_PitchController, AC_PID),

    AP_GROUPEND
};

AP_PitchController::AP_PitchController(const AP_Vehicle::FixedWing &parms)
    : aparm(parms)
{
    AP_Param::setup_object_defaults(this, var_info);
    rate_pid.set_slew_limit_scale(45);
}

/*
  AC_PID based rate controller
*/
float AP_PitchController::_get_rate_out(float desired_rate, float scaler, bool disable_integrator, float aspeed, bool ground_mode, float eas2tas, float rate_y)
{
    const float dt = AP::scheduler().get_loop_period_s();

    bool limit_I = fabsf(_last_out) >= 45;
    float old_I = rate_pid.get_i();

    rate_pid.set_dt(dt);

    bool underspeed = aspeed <= 0.5*float(aparm.airspeed_min);
    if (underspeed) {
        limit_I = true;
    }

    // the P and I elements are scaled by sq(scaler). To use an
    // unmodified AC_PID object we scale the inputs and calculate FF separately
    //
    // note that we run AC_PID in radians so that the normal scaling
    // range for IMAX in AC_PID applies (usually an IMAX value less than 1.0)
    rate_pid.update_all(radians(desired_rate) * scaler * scaler, rate_y * scaler * scaler, limit_I);

    if (underspeed) {
        // when underspeed we lock the integrator
        rate_pid.set_integrator(old_I);
    }

    // FF should be scaled by scaler/eas2tas, but since we have scaled
    // the AC_PID target above by scaler*scaler we need to instead
    // divide by scaler*eas2tas to get the right scaling
    const float ff = degrees(rate_pid.get_ff() / (scaler * eas2tas));

    if (disable_integrator) {
        rate_pid.reset_I();
    }

    // convert AC_PID info object to same scale as old controller
    _pid_info = rate_pid.get_pid_info();
    auto &pinfo = _pid_info;

    const float deg_scale = degrees(1);
    pinfo.FF = ff;
    pinfo.P *= deg_scale;
    pinfo.I *= deg_scale;
    pinfo.D *= deg_scale;

    // fix the logged target and actual values to not have the scalers applied
    pinfo.target = desired_rate;
    pinfo.actual = degrees(rate_y);

    // sum components
    float out = pinfo.FF + pinfo.P + pinfo.I + pinfo.D;
    if (ground_mode) {
        // when on ground suppress D and half P term to prevent oscillations
        out -= pinfo.D + 0.5*pinfo.P;
    }

    // remember the last output to trigger the I limit
    _last_out = out;

    if (autotune != nullptr && autotune->running && aspeed > aparm.airspeed_min) {
        // let autotune have a go at the values
        autotune->update(pinfo, scaler, angle_err_deg);
    }

    // output is scaled to notional centidegrees of deflection
    return constrain_float(out * 100, -4500, 4500);
}

/*
 Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are:
 1) demanded pitch rate in degrees/second
 2) control gain scaler = scaling_speed / aspeed
 3) boolean which is true when stabilise mode is active
 4) minimum FBW airspeed (metres/sec)
 5) maximum FBW airspeed (metres/sec)
*/
float AP_PitchController::get_rate_out(float desired_rate, float scaler)
{

    float eas2tas, rate_y, aspeed;
    bool have_aspeed;
    {
        AP_AHRS &_ahrs = AP::ahrs();
        WITH_SEMAPHORE(_ahrs.get_semaphore());
        eas2tas = _ahrs.get_EAS2TAS();
        rate_y = _ahrs.get_gyro().y;
        have_aspeed = AP::ahrs().airspeed_estimate(aspeed);
    }

    if (!have_aspeed) {
        // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aparm.airspeed_min) + float(aparm.airspeed_max));
    }

    return _get_rate_out(desired_rate, scaler, false, aspeed, false, eas2tas, rate_y);
}

/*
  get the rate offset in degrees/second needed for pitch in body frame
  to maintain height in a coordinated turn.

  Also returns the inverted flag and the estimated airspeed in m/s for
  use by the rest of the pitch controller
 */
float AP_PitchController::_get_coordination_rate_offset(const GSO_AHRS_Data &ahrs_data, bool &inverted) const
{
    // limit bank angle between +- 80 deg if right way up
    float roll = ahrs_data.roll;
    if (fabsf(roll) < radians(90))	{
        roll = constrain_float(roll, -radians(80), radians(80));
        inverted = false;
    } else {
        inverted = true;
        if (roll > 0.0f) {
            roll = constrain_float(roll, radians(100), radians(180));
        } else {
            roll = constrain_float(roll, -radians(180), -radians(100));
        }
    }

    float rate_offset;

    if (abs(ahrs_data.pitch_sensor) > 7000) {
        // don't do turn coordination handling when at very high pitch angles
        rate_offset = 0;
    } else {
        rate_offset = cosf(ahrs_data.pitch)*fabsf(ToDeg((GRAVITY_MSS / MAX((ahrs_data.aspeed * ahrs_data.eas2tas), MAX(aparm.airspeed_min, 1))) * tanf(roll) * sinf(roll))) * _roll_ff;
    }

    if (inverted) {
        rate_offset = -rate_offset;
    }

    return rate_offset;
}

// Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
// A positive demand is up
// Inputs are:
// 1) demanded pitch angle in centi-degrees
// 2) control gain scaler = scaling_speed / aspeed
// 3) boolean which is true when stabilise mode is active
// 4) minimum FBW airspeed (metres/sec)
// 5) maximum FBW airspeed (metres/sec)
//
float AP_PitchController::get_servo_out_using_angle_target(int32_t target_angle, float scaler, bool disable_integrator, bool ground_mode)
{
    const float dt = AP::scheduler().get_loop_period_s();
    angle_pid.set_dt(dt);

    GSO_AHRS_Data ahrs_data;
    _get_gso_ahrs_data(ahrs_data);

    const float target_angle_deg = target_angle * 0.01f;
    const float measured_angle_deg = ahrs_data.pitch_sensor * 0.01f;
    angle_err_deg = target_angle_deg - measured_angle_deg;

    if (angle_err_deg > 2.0f) {
        angle_pid.relax_integrator(0, 0.1f);
    }

    angle_pid.update_all(target_angle_deg, measured_angle_deg, false);

    if (disable_integrator) {
        angle_pid.reset_I();
    }

    _angle_pid_info = angle_pid.get_pid_info();
    auto &pinfo = _angle_pid_info;

    float desired_rate = pinfo.P + pinfo.I + pinfo.D;

    return _get_servo_out(desired_rate, scaler, disable_integrator, ground_mode, ahrs_data);
}

void AP_PitchController::_get_gso_ahrs_data(GSO_AHRS_Data &ahrs_data)
{

    bool have_aspeed;
    {
        AP_AHRS &_ahrs = AP::ahrs();
        WITH_SEMAPHORE(_ahrs.get_semaphore());
        ahrs_data.eas2tas = _ahrs.get_EAS2TAS();
        ahrs_data.rate_y = _ahrs.get_gyro().y;
        have_aspeed = AP::ahrs().airspeed_estimate(ahrs_data.aspeed);
        ahrs_data.roll_sensor = _ahrs.roll_sensor;
        ahrs_data.pitch_sensor = _ahrs.pitch_sensor;
        ahrs_data.roll = _ahrs.roll;
        ahrs_data.pitch = _ahrs.pitch;
    }

    if (!have_aspeed) {
        // If no airspeed available use average of min and max
        ahrs_data.aspeed = 0.5f*(float(aparm.airspeed_min) + float(aparm.airspeed_max));
    }

}

float AP_PitchController::get_servo_out_using_angle_error(int32_t angle_err, int32_t target_angle, float scaler, bool disable_integrator, bool ground_mode)
{
    const float dt = AP::scheduler().get_loop_period_s();
    angle_pid.set_dt(dt);

    angle_err_deg = angle_err * 0.01f;

    if (angle_err_deg > 2.0f) {
        angle_pid.relax_integrator(0, 0.1f);
    }

    angle_pid.update_error(angle_err_deg, false);

    if (disable_integrator) {
        angle_pid.reset_I();
    }

    _angle_pid_info = angle_pid.get_pid_info();
    auto &pinfo = _angle_pid_info;

    GSO_AHRS_Data ahrs_data;
    _get_gso_ahrs_data(ahrs_data);

    if (target_angle == 0) {
        pinfo.target = 0;
        pinfo.actual = 0;
    } else {
        const float actual_angle = ahrs_data.pitch_sensor;
        pinfo.target = target_angle * 0.01f;
        pinfo.actual = actual_angle * 0.01f;
    }

    float desired_rate = pinfo.P + pinfo.I + pinfo.D;

    return _get_servo_out(desired_rate, scaler, disable_integrator, ground_mode, ahrs_data);
}

float AP_PitchController::_get_servo_out(float desired_rate, float scaler, bool disable_integrator, bool ground_mode, const AP_PitchController::GSO_AHRS_Data &ahrs_data)
{
    // Calculate offset to pitch rate demand required to maintain pitch angle whilst banking
    // Calculate ideal turn rate from bank angle and airspeed assuming a level coordinated turn
    // Pitch rate offset is the component of turn rate about the pitch axis

    bool inverted;
    const float rate_offset = _get_coordination_rate_offset(ahrs_data, inverted);

    // limit the maximum pitch rate demand. Don't apply when inverted
    // as the rates will be tuned when upright, and it is common that
    // much higher rates are needed inverted
    if (!inverted) {
        desired_rate += rate_offset;
        if (gains.rmax_neg && desired_rate < -gains.rmax_neg) {
            desired_rate = -gains.rmax_neg;
        } else if (gains.rmax_pos && desired_rate > gains.rmax_pos) {
            desired_rate = gains.rmax_pos;
        }
    } else {
        // Make sure not to invert the turn coordination offset
        desired_rate = -desired_rate + rate_offset;
    }

    /*
      when we are past the users defined roll limit for the aircraft
      our priority should be to bring the aircraft back within the
      roll limit. Using elevator for pitch control at large roll
      angles is ineffective, and can be counter productive as it
      induces earth-frame yaw which can reduce the ability to roll. We
      linearly reduce pitch demanded rate when beyond the configured
      roll limit, reducing to zero at 90 degrees
    */
    float roll_wrapped = labs(ahrs_data.roll_sensor);

    if (roll_wrapped > 9000) {
        roll_wrapped = 18000 - roll_wrapped;
    }

    const float roll_limit_margin = MIN(aparm.roll_limit_cd + 500.0, 8500.0);
    if (roll_wrapped > roll_limit_margin && labs(ahrs_data.pitch_sensor) < 7000) {
        float roll_prop = (roll_wrapped - roll_limit_margin) / (float)(9000 - roll_limit_margin);
        desired_rate *= (1 - roll_prop);
    }

    return _get_rate_out(desired_rate, scaler, disable_integrator, ahrs_data.aspeed, ground_mode, ahrs_data.eas2tas, ahrs_data.rate_y);
}

void AP_PitchController::reset_I()
{
    _pid_info.I = 0;
    rate_pid.reset_I();
    _angle_pid_info.I = 0;
    angle_pid.reset_I();
}

/*
  convert from old to new PIDs
  this is a temporary conversion function during development
 */
void AP_PitchController::convert_pid()
{
    AP_Float &angle_kP = angle_pid.kP();
    if (angle_kP.configured()) {
        return;
    }

    float old_tconst;
    bool have_old = AP_Param::get_param_by_index(this, 0, AP_PARAM_FLOAT, &old_tconst);
    if (!have_old) {
        // tconst wasn't set
        return;
    }

    const float angle_kp = 1 / old_tconst;
    angle_pid.kP().set_and_save_ifchanged(angle_kp);
}

/*
  start an autotune
 */
void AP_PitchController::autotune_start(void)
{
    if (autotune == nullptr) {
        angle_i_backup = angle_pid.kI();
        angle_fltt_backup = angle_pid.filt_T_hz();
        gains.tau = tau();
        autotune = new AP_AutoTune(gains, &kP(), AP_AutoTune::AUTOTUNE_PITCH, aparm, rate_pid);
        if (autotune == nullptr) {
            if (!failed_autotune_alloc) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AutoTune: failed pitch allocation");
            }
            failed_autotune_alloc = true;
        }
        angle_pid.kI(0);
        angle_pid.kD().set_and_save_ifchanged(0);
        angle_pid.filt_T_hz(0);
    }
    if (autotune != nullptr) {
        autotune->start();
    }
}

/*
  restore autotune gains
 */
void AP_PitchController::autotune_restore(void)
{
    if (autotune != nullptr) {
        autotune->stop();
        angle_pid.kI(angle_i_backup);
        angle_pid.filt_T_hz(angle_fltt_backup);
    }
}
