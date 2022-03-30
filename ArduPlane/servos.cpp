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
/*
  main logic for servo control
 */

#include "Plane.h"
#include <utility>

/*****************************************
* Throttle slew limit
*****************************************/
void Plane::throttle_slew_limit(SRV_Channel::Aux_servo_function_t func)
{
#if HAL_QUADPLANE_ENABLED
    const bool do_throttle_slew = (control_mode->does_auto_throttle() || quadplane.in_assisted_flight() || quadplane.in_vtol_mode());
#else
    const bool do_throttle_slew = control_mode->does_auto_throttle();
#endif

    if (!do_throttle_slew) {
        // only do throttle slew limiting in modes where throttle control is automatic
        SRV_Channels::set_slew_rate(func, 0.0, 100, G_Dt);
        return;
    }

    uint8_t slewrate = aparm.throttle_slewrate;
    if (control_mode == &mode_auto) {
        if (auto_state.takeoff_complete == false && g.takeoff_throttle_slewrate != 0) {
            slewrate = g.takeoff_throttle_slewrate;
        } else if (landing.get_throttle_slewrate() != 0 && flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
            slewrate = landing.get_throttle_slewrate();
        }
    }
    if (g.takeoff_throttle_slewrate != 0 &&
        (flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF ||
         flight_stage == AP_Vehicle::FixedWing::FLIGHT_VTOL)) {
        // for VTOL we use takeoff slewrate, which helps with transition
        slewrate = g.takeoff_throttle_slewrate;
    }
#if HAL_QUADPLANE_ENABLED
    if (g.takeoff_throttle_slewrate != 0 && quadplane.in_transition()) {
        slewrate = g.takeoff_throttle_slewrate;
    }
#endif
    SRV_Channels::set_slew_rate(func, slewrate, 100, G_Dt);
}

/* We want to suppress the throttle if we think we are on the ground and in an autopilot controlled throttle mode.

   Disable throttle if following conditions are met:
   *       1 - We are in Circle mode (which we use for short term failsafe), or in FBW-B or higher
   *       AND
   *       2 - Our reported altitude is within 10 meters of the home altitude.
   *       3 - Our reported speed is under 5 meters per second.
   *       4 - We are not performing a takeoff in Auto mode or takeoff speed/accel not yet reached
   *       OR
   *       5 - Home location is not set
   *       OR
   *       6- Landing does not want to allow throttle
*/
bool Plane::suppress_throttle(void)
{

    if (arming.get_throttle_cut()) {
        return true;
    }

#if PARACHUTE == ENABLED
    if (control_mode->does_auto_throttle() && parachute.release_initiated()) {
        // throttle always suppressed in auto-throttle modes after parachute release initiated
        throttle_suppressed = true;
        return true;
    }
#endif

    if (landing.is_throttle_suppressed()) {
        return true;
    }

    if (!throttle_suppressed) {
        // we've previously met a condition for unsupressing the throttle
        return false;
    }
    if (!control_mode->does_auto_throttle()) {
        // the user controls the throttle
        throttle_suppressed = false;
        return false;
    }

    bool gps_movement = (gps.status() >= AP_GPS::GPS_OK_FIX_2D && gps.ground_speed() >= 5);

    if ((control_mode == &mode_auto &&
         auto_state.takeoff_complete == false) ||
        control_mode == &mode_takeoff) {

        uint32_t launch_duration_ms = ((int32_t)g.takeoff_throttle_delay)*100 + 2000;
        if (is_flying() &&
            millis() - started_flying_ms > MAX(launch_duration_ms, 5000U) && // been flying >5s in any mode
            adjusted_relative_altitude_cm() > 500 && // are >5m above AGL/home
            labs(ahrs.pitch_sensor) < 3000 && // not high pitch, which happens when held before launch
            gps_movement) { // definite gps movement
            // we're already flying, do not suppress the throttle. We can get
            // stuck in this condition if we reset a mission and cmd 1 is takeoff
            // but we're currently flying around below the takeoff altitude
            throttle_suppressed = false;
            return false;
        }
        if (auto_takeoff_check()) {
            // we're in auto takeoff
            throttle_suppressed = false;
            auto_state.baro_takeoff_alt = barometer.get_altitude();
            return false;
        }
        // keep throttle suppressed
        return true;
    }

    if (fabsf(relative_altitude) >= 10.0f) {
        // we're more than 10m from the home altitude
        throttle_suppressed = false;
        return false;
    }

    if (gps_movement) {
        // if we have an airspeed sensor, then check it too, and
        // require 5m/s. This prevents throttle up due to spiky GPS
        // groundspeed with bad GPS reception
#if AP_AIRSPEED_ENABLED
        if ((!ahrs.airspeed_sensor_enabled()) || airspeed.get_airspeed() >= 5) {
            // we're moving at more than 5 m/s
            throttle_suppressed = false;
            return false;
        }
#else
        // no airspeed sensor, so we trust that the GPS's movement is truthful
        throttle_suppressed = false;
        return false;
#endif
    }

#if HAL_QUADPLANE_ENABLED
    if (quadplane.is_flying()) {
        throttle_suppressed = false;
        return false;
    }
#endif

    // throttle remains suppressed
    return true;
}

float Plane::apply_throws_diff(float input, float diff) const
{
    const float mixing_diff_attn = (100 - MIN(abs(diff), 90)) * 0.01f;

    if ((input < 0 && diff > 0) || (input > 0 && diff < 0)) {
        return input * mixing_diff_attn;
    }

    return input;
}

/*
  mixer for elevon and vtail channels setup using designated servo
  function values. This mixer operates purely on scaled values,
  allowing the user to trim and limit individual servos using the
  SERVOn_* parameters
 */
void Plane::channel_function_mixer(SRV_Channel::Aux_servo_function_t func1_in, SRV_Channel::Aux_servo_function_t func2_in,
                                   SRV_Channel::Aux_servo_function_t func1_out, SRV_Channel::Aux_servo_function_t func2_out) const
{
    // the order is setup so that non-reversed servos go "up", and
    // func1 is the "left" channel. Users can adjust with channel
    // reversal as needed
    float in1 = SRV_Channels::get_output_scaled(func1_in);
    float in2 = SRV_Channels::get_output_scaled(func2_in);

    const int8_t mixing_offset = constrain_int16(g.mixing_offset, -g.mixing_gain * 80.0f, g.mixing_gain * 80.0f);

    in1 *= g.mixing_gain - mixing_offset * 0.01f;
    in2 *= g.mixing_gain + mixing_offset * 0.01f;

    float out1 = constrain_float(in2 - in1, -4500, 4500);
    float out2 = constrain_float(in2 + in1, -4500, 4500);

    out1 = apply_throws_diff(out1, g.mixing_diff);
    out2 = apply_throws_diff(out2, g.mixing_diff);

    SRV_Channels::set_output_scaled(func1_out, out1);
    SRV_Channels::set_output_scaled(func2_out, out2);
}


/*
  setup flaperon output channels
 */
void Plane::flaperon_update()
{
    /*
      flaperons are implemented as a mixer between aileron and a
      percentage of flaps. Flap input can come from a manual channel
      or from auto flaps.
     */
    float aileron_right = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    float aileron_left = -aileron_right;
    aileron_left = apply_throws_diff(aileron_left, g2.ailerons_diff);
    aileron_right = apply_throws_diff(aileron_right, g2.ailerons_diff);
    float flap_percent = SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto);
    float flaperon_left  = constrain_float(aileron_left - flap_percent * 45, -4500, 4500);
    float flaperon_right = constrain_float(aileron_right - flap_percent * 45, -4500, 4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_flaperon_left, flaperon_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_flaperon_right, flaperon_right);
}


/*
  setup differential spoiler output channels

  Differential spoilers are a type of elevon that is split on each
  wing to give yaw control, mixed from rudder
 */
void Plane::dspoiler_update(void)
{
    const int8_t bitmask = g2.crow_flap_options.get();
    const bool flying_wing       = (bitmask & CrowFlapOptions::FLYINGWING) != 0;
    const bool full_span_aileron = (bitmask & CrowFlapOptions::FULLSPAN) != 0;
    //progressive crow when option is set or RC switch is set to progressive
    const bool progressive_crow   = (bitmask & CrowFlapOptions::PROGRESSIVE_CROW) != 0  || crow_mode == CrowMode::PROGRESSIVE;

    // if flying wing use elevons else use ailerons
    float elevon_left;
    float elevon_right;
    if (flying_wing) {
        elevon_left = apply_throws_diff(SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left), g.mixing_diff);
        elevon_right = apply_throws_diff(SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right), g.mixing_diff);
    } else {
        const float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
        elevon_left = apply_throws_diff(-aileron, g2.ailerons_diff);
        elevon_right = apply_throws_diff(aileron, g2.ailerons_diff);
    }


    const float rudder_rate = g.dspoiler_rud_rate * 0.01f;
    const float rudder = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) * rudder_rate;
    float dspoiler_outer_left = elevon_left;
    float dspoiler_outer_right = elevon_right;

    float dspoiler_inner_left = 0;
    float dspoiler_inner_right = 0;

    // full span ailerons / elevons
    if (full_span_aileron) {
        dspoiler_inner_left = elevon_left;
        dspoiler_inner_right = elevon_right;
    }

    if (rudder > 0) {
        // apply rudder to right wing
        dspoiler_outer_right = constrain_float(dspoiler_outer_right + rudder, -4500, 4500);
        dspoiler_inner_right = constrain_float(dspoiler_inner_right - rudder, -4500, 4500);
    } else {
        // apply rudder to left wing
        dspoiler_outer_left = constrain_float(dspoiler_outer_left - rudder, -4500, 4500);
        dspoiler_inner_left = constrain_float(dspoiler_inner_left + rudder, -4500, 4500);
    }

    // limit flap throw used for aileron
    const int8_t aileron_matching = g2.crow_flap_aileron_matching.get();
    if (aileron_matching < 100) {
        // only do matching if it will make a difference
        const float aileron_matching_scaled = aileron_matching * 0.01;
        if (is_negative(dspoiler_inner_left)) {
            dspoiler_inner_left *= aileron_matching_scaled;
        }
        if (is_negative(dspoiler_inner_right)) {
            dspoiler_inner_right *= aileron_matching_scaled;
        }
    }

    int16_t weight_outer = g2.crow_flap_weight_outer.get();
    if (crow_mode == Plane::CrowMode::CROW_DISABLED) {   //override totally aileron crow if crow RC switch set to disabled
        weight_outer = 0;
    }
    const int16_t weight_inner = g2.crow_flap_weight_inner.get();
    if (weight_outer > 0 || weight_inner > 0) {
        /*
          apply crow flaps by apply the same split of the differential
          spoilers to both wings. Get flap percentage from k_flap_auto, which is set
          in set_servos_flaps() as the maximum of manual and auto flap control
         */
        const float flap_percent = SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto);

        if (is_positive(flap_percent)) {
            float inner_flap_scaled = flap_percent;
            float outer_flap_scaled = flap_percent;
            if (progressive_crow) {
                // apply 0 - full inner from 0 to 50% flap then add in outer above 50%
                inner_flap_scaled = constrain_float(inner_flap_scaled * 2, 0,100);
                outer_flap_scaled = constrain_float(outer_flap_scaled - 50, 0,50) * 2;
            }
            // scale flaps so when weights are 100 they give full up or down
            dspoiler_outer_left  = constrain_float(dspoiler_outer_left  - outer_flap_scaled * weight_outer * 0.45, -4500, 4500);
            dspoiler_inner_left  = constrain_float(dspoiler_inner_left  - inner_flap_scaled * weight_inner * 0.45, -4500, 4500);
            dspoiler_outer_right = constrain_float(dspoiler_outer_right - outer_flap_scaled * weight_outer * 0.45, -4500, 4500);
            dspoiler_inner_right = constrain_float(dspoiler_inner_right - inner_flap_scaled * weight_inner * 0.45, -4500, 4500);
        }
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_dspoilerLeft1, dspoiler_outer_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_dspoilerLeft2, dspoiler_inner_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_dspoilerRight1, dspoiler_outer_right);
    SRV_Channels::set_output_scaled(SRV_Channel::k_dspoilerRight2, dspoiler_inner_right);
}

/*
 set airbrakes based on reverse thrust and/or manual input RC channel
 */
void Plane::airbrake_update(void)
{
    // Calculate any manual airbrake input from RC channel option.
    float manual_airbrake_percent = 0;

    if (channel_airbrake != nullptr && !failsafe.rc_failsafe && failsafe.throttle_counter == 0) {
        manual_airbrake_percent = channel_airbrake->percent_input();
    }

    // Calculate auto airbrake from negative throttle.
    float throttle_min = aparm.throttle_min.get();
    float airbrake_pc = 0;

    float throttle_pc = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);

    if (throttle_min < 0) {
        if (landing.is_flaring()) {
            // Full airbrakes during the flare.
            airbrake_pc = 100;
        }
        else {
            // Determine fraction between zero and full negative throttle.
            airbrake_pc = constrain_float(-throttle_pc, 0, 100);
        }
    }

    // Manual overrides auto airbrake setting.
    if (airbrake_pc < manual_airbrake_percent) {
        airbrake_pc = manual_airbrake_percent;
    }

    // Output to airbrake servo types.
    SRV_Channels::set_output_scaled(SRV_Channel::k_airbrake, airbrake_pc);
}

/*
  setup servos for idle mode
  Idle mode is used during balloon launch to keep servos still, apart
  from occasional wiggle to prevent freezing up
 */
void Plane::set_servos_idle(void)
{
    int16_t servo_value;
    // move over full range for 2 seconds
    if (auto_state.idle_wiggle_stage != 0) {
        auto_state.idle_wiggle_stage += 2;
    }
    if (auto_state.idle_wiggle_stage == 0) {
        servo_value = 0;
    } else if (auto_state.idle_wiggle_stage < 50) {
        servo_value = auto_state.idle_wiggle_stage * (4500 / 50);
    } else if (auto_state.idle_wiggle_stage < 100) {
        servo_value = (100 - auto_state.idle_wiggle_stage) * (4500 / 50);
    } else if (auto_state.idle_wiggle_stage < 150) {
        servo_value = (100 - auto_state.idle_wiggle_stage) * (4500 / 50);
    } else if (auto_state.idle_wiggle_stage < 200) {
        servo_value = (auto_state.idle_wiggle_stage-200) * (4500 / 50);
    } else {
        auto_state.idle_wiggle_stage = 0;
        servo_value = 0;
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, servo_value);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, servo_value);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, servo_value);
    SRV_Channels::set_output_to_trim(SRV_Channel::k_throttle);

    SRV_Channels::output_ch_all();
}

/*
  pass through channels in manual mode
 */
void Plane::set_servos_manual_passthrough(void)
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder_in_expo(false));
    float throttle = get_throttle_input(true);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

#if HAL_QUADPLANE_ENABLED
    if (quadplane.available() && (quadplane.options & QuadPlane::OPTION_IDLE_GOV_MANUAL)) {
        // for quadplanes it can be useful to run the idle governor in MANUAL mode
        // as it prevents the VTOL motors from running
        int8_t min_throttle = aparm.throttle_min.get();

        // apply idle governor
        g2.ice_control.update_idle_governor(min_throttle);
        throttle = MAX(throttle, min_throttle);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
    }
#endif

    apply_throttle_dz();

    if (!g2.rc_channels.throttle_battery_compensation_is_disabled_in_manual_mode()) {
        // conpensate for battery voltage drop
        int8_t min_throttle = -100;
        int8_t max_throttle = 100;
        throttle_voltage_comp(min_throttle, max_throttle);
    }

    if (!g2.rc_channels.throttle_expo_is_disabled_in_manual_mode()) {
        apply_throttle_expo();
    }
}

/*
  Scale the throttle to conpensate for battery voltage drop
 */
void Plane::throttle_voltage_comp(int8_t &min_throttle, int8_t &max_throttle) const
{
    // return if not enabled, or setup incorrectly
    if (!is_positive(g2.fwd_thr_batt_voltage_min) || g2.fwd_thr_batt_voltage_min >= g2.fwd_thr_batt_voltage_max) {
        return;
    }

    float batt_voltage_resting_estimate = AP::battery().voltage_resting_estimate(g2.fwd_thr_batt_idx);
    // Return for a very low battery
    if (batt_voltage_resting_estimate < 0.25f * g2.fwd_thr_batt_voltage_min) {
        return;
    }

    // constrain read voltage to min and max params
    batt_voltage_resting_estimate = constrain_float(batt_voltage_resting_estimate,g2.fwd_thr_batt_voltage_min,g2.fwd_thr_batt_voltage_max);

    // don't apply compensation if the voltage is excessively low
    if (batt_voltage_resting_estimate < 1) {
        return;
    }

    // Scale the throttle up to compensate for voltage drop
    // Ratio = 1 when voltage = voltage max, ratio increases as voltage drops
    const float ratio = g2.fwd_thr_batt_voltage_max / batt_voltage_resting_estimate;

    // Scale the throttle limits to prevent subsequent clipping
    min_throttle = MAX((int8_t)(ratio * (float)min_throttle), -100);
    max_throttle = MIN((int8_t)(ratio * (float)max_throttle),  100);

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle,
                                        constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * ratio, -100, 100));
}

/*
  calculate any throttle limits based on the watt limiter
 */
void Plane::throttle_watt_limiter(int8_t &min_throttle, int8_t &max_throttle)
{
    uint32_t now = millis();
    if (battery.overpower_detected()) {
        // overpower detected, cut back on the throttle if we're maxing it out by calculating a limiter value
        // throttle limit will attack by 10% per second

        if (is_positive(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)) && // demanding too much positive thrust
            throttle_watt_limit_max < max_throttle - 25 &&
            now - throttle_watt_limit_timer_ms >= 1) {
            // always allow for 25% throttle available regardless of battery status
            throttle_watt_limit_timer_ms = now;
            throttle_watt_limit_max++;
        } else if (is_negative(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)) &&
                   min_throttle < 0 && // reverse thrust is available
                   throttle_watt_limit_min < -(min_throttle) - 25 &&
                   now - throttle_watt_limit_timer_ms >= 1) {
            // always allow for 25% throttle available regardless of battery status
            throttle_watt_limit_timer_ms = now;
            throttle_watt_limit_min++;
        }

    } else if (now - throttle_watt_limit_timer_ms >= 1000) {
        // it has been 1 second since last over-current, check if we can resume higher throttle.
        // this throttle release is needed to allow raising the max_throttle as the battery voltage drains down
        // throttle limit will release by 1% per second
        if (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) > throttle_watt_limit_max && // demanding max forward thrust
            throttle_watt_limit_max > 0) { // and we're currently limiting it
            throttle_watt_limit_timer_ms = now;
            throttle_watt_limit_max--;

        } else if (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) < throttle_watt_limit_min && // demanding max negative thrust
                   throttle_watt_limit_min > 0) { // and we're limiting it
            throttle_watt_limit_timer_ms = now;
            throttle_watt_limit_min--;
        }
    }

    max_throttle = constrain_int16(max_throttle, 0, max_throttle - throttle_watt_limit_max);
    if (min_throttle < 0) {
        min_throttle = constrain_int16(min_throttle, min_throttle + throttle_watt_limit_min, 0);
    }
}

void Plane::apply_throttle_dz(void)
{
    if (fabsf(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)) < g.throttle_dz) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
    }
}

void Plane::apply_throttle_expo(void)
{
    const float expo_param = control_mode->does_auto_throttle() ? g2.throttle_expo_auto : g2.throttle_expo_manual;
    const float expo = constrain_float(expo_param, 0, 100) * 0.01f;
    const float input_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * 0.01f;
    const int8_t sign_factor = signbit(input_throttle) ? -1 : 1;
    const float output_throttle = (1 - expo) * abs(input_throttle) + expo * sq(input_throttle);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle , sign_factor * output_throttle * 100);
}

void Plane::shift_elevator_output_pwm(int16_t elev_pwm_shift)
{
    SRV_Channels::shift_output_pwm(SRV_Channel::k_elevator, elev_pwm_shift);
    SRV_Channels::shift_output_pwm(SRV_Channel::k_elevon_left, elev_pwm_shift);
    SRV_Channels::shift_output_pwm(SRV_Channel::k_elevon_right, elev_pwm_shift);
    SRV_Channels::shift_output_pwm(SRV_Channel::k_vtail_left, elev_pwm_shift);
    SRV_Channels::shift_output_pwm(SRV_Channel::k_vtail_right, elev_pwm_shift);

    const int8_t bitmask = g2.crow_flap_options.get();
    const bool flying_wing       = (bitmask & CrowFlapOptions::FLYINGWING) != 0;
    const bool full_span_aileron = (bitmask & CrowFlapOptions::FULLSPAN) != 0;

    if (flying_wing) {

        SRV_Channels::shift_output_pwm(SRV_Channel::k_dspoilerLeft1, elev_pwm_shift);
        SRV_Channels::shift_output_pwm(SRV_Channel::k_dspoilerRight1, elev_pwm_shift);

        if (full_span_aileron) {
            SRV_Channels::shift_output_pwm(SRV_Channel::k_dspoilerLeft2, elev_pwm_shift);
            SRV_Channels::shift_output_pwm(SRV_Channel::k_dspoilerRight2, elev_pwm_shift);
        }

    }
}

void Plane::apply_throttle_to_elevator_mix(void)
{
    const float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    const int16_t elev_mix_pwm = lrintf(linear_interpolate(0, g.kff_throttle_above_trim_to_elevator, throttle, aparm.throttle_cruise, 100));
    shift_elevator_output_pwm(elev_mix_pwm);
}

void Plane::apply_flap_to_elevator_mix(void)
{
    const float flap_position = SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto);
    const uint16_t elev_mix_pwm = lrintf(flap_position * g.kff_flap_to_elevator * 0.01f);
    shift_elevator_output_pwm(elev_mix_pwm);
}

/*
  setup output channels all non-manual modes
 */
void Plane::set_servos_controlled(void)
{
    if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
        // allow landing to override servos if it would like to
        landing.override_servos();
    }

    // convert 0 to 100% (or -100 to +100) into PWM
    int8_t min_throttle = aparm.throttle_min.get();
    int8_t max_throttle = aparm.throttle_max.get();

    // apply idle governor
    g2.ice_control.update_idle_governor(min_throttle);

    if (min_throttle < 0 && !allow_reverse_thrust()) {
        // reverse thrust is available but inhibited.
        min_throttle = 0;
    }

    bool flight_stage_determines_max_throttle = false;
    if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF || 
        flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND
        ) {
        flight_stage_determines_max_throttle = true;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_transition()) {
        flight_stage_determines_max_throttle = true;
    }
#endif
    if (flight_stage_determines_max_throttle) {
        if (aparm.takeoff_throttle_max != 0) {
            max_throttle = aparm.takeoff_throttle_max;
        } else {
            max_throttle = aparm.throttle_max;
        }
    } else if (landing.is_flaring()) {
        min_throttle = 0;
    }

    // apply watt limiter
    throttle_watt_limiter(min_throttle, max_throttle);

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle,
                                    constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle), min_throttle, max_throttle));
    if (!hal.util->get_soft_armed()) {
        if (arming.arming_required() == AP_Arming::Required::YES_ZERO_PWM) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::ZERO_PWM);
        } else {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, 0.0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0.0);
        }
    } else if (suppress_throttle()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0); // default
        // throttle is suppressed (above) to zero in final flare in auto mode, but we allow instead thr_min if user prefers, eg turbines:
        if (landing.is_flaring() && landing.use_thr_min_during_flare() ) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, aparm.throttle_min.get());
        }
        if (g.throttle_suppress_manual) {
            // manual pass through of throttle while throttle is suppressed
            float throttle_input = get_throttle_input(true);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle_input);
            TECS_controller.set_throttle_demand(throttle_input);
        }
#if AP_SCRIPTING_ENABLED
    } else if (plane.nav_scripting.current_ms > 0 && nav_scripting.enabled) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.nav_scripting.throttle_pct);
#endif
    } else if (control_mode == &mode_stabilize ||
               control_mode == &mode_training ||
               control_mode == &mode_acro ||
               control_mode == &mode_fbwa ||
               control_mode == &mode_autotune) {
        // a manual throttle mode
        if (!rc().has_valid_input()) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        } else if (g.throttle_passthru_stabilize) {
            // manual pass through of throttle while in FBWA or
            // STABILIZE mode with THR_PASS_STAB set
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, get_throttle_input(true));
        } else {
            // get throttle, but adjust center to output TRIM_THROTTLE if flight option set
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, get_adjusted_throttle_input(true));
        }
    } else if (control_mode->is_guided_mode() &&
               guided_throttle_passthru) {
        // manual pass through of throttle while in GUIDED
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, get_throttle_input(true));
#if HAL_QUADPLANE_ENABLED
    } else if (quadplane.in_vtol_mode()) {
        float fwd_thr = 0;
        // if armed and not spooled down ask quadplane code for forward throttle
        if (quadplane.motors->armed() &&
            quadplane.motors->get_desired_spool_state() != AP_Motors::DesiredSpoolState::SHUT_DOWN) {

            fwd_thr = constrain_float(quadplane.forward_throttle_pct(), min_throttle, max_throttle);
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, fwd_thr);
#endif  // HAL_QUADPLANE_ENABLED
    }

    apply_throttle_dz();

    // let EKF know to start GSF yaw estimator before takeoff movement starts so that yaw angle is better estimated
    const float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (!is_flying() && arming.is_armed()) {
        // Check if rate of change of velocity along X axis exceeds 1-g which normally indicates a throw.
        // Tests with hand carriage of micro UAS indicates that a 1-g threshold does not false trigger prior
        // to the throw, but there is margin to increase this threshold if false triggering becomes problematic.
        const float accel_x_due_to_gravity = GRAVITY_MSS * ahrs.sin_pitch();
        const float accel_x_due_to_throw = ahrs.get_accel().x - accel_x_due_to_gravity;
        bool throw_detected = accel_x_due_to_throw > GRAVITY_MSS;
        bool throttle_up_detected = throttle > aparm.throttle_cruise;
        if (throw_detected || throttle_up_detected) {
            plane.ahrs.set_takeoff_expected(true);
        }
    }

    // conpensate for battery voltage drop
    throttle_voltage_comp(min_throttle, max_throttle);

    apply_throttle_expo();
}

/*
  setup flap outputs
 */
void Plane::set_servos_flaps(void)
{
    // Auto flap deployment
    float auto_flap_percent = 0;
    float manual_flap_percent = 0;

    // work out any manual flap input
    if (channel_flap != nullptr && rc().has_valid_input()) {
        manual_flap_percent = channel_flap->percent_input();
    }

    if (control_mode->does_auto_throttle()) {
        float flapSpeedSource = 0;
        if (ahrs.airspeed_sensor_enabled()) {
            flapSpeedSource = TECS_controller.get_target_airspeed();
        } else {
            flapSpeedSource = aparm.throttle_cruise;
        }
        auto_flap_percent = linear_interpolate(0, g.flap_deployed_percent, flapSpeedSource, g.flap_retracted_speed, g.flap_deployed_speed);

#if HAL_SOARING_ENABLED
        if (control_mode == &mode_thermal) {
            auto_flap_percent = g2.soaring_controller.get_thermalling_flap();
        }
#endif

        /*
          special flap levels for takeoff and landing. This works
          better than speed based flaps as it leads to less
          possibility of oscillation
         */
        switch (flight_stage) {
            case AP_Vehicle::FixedWing::FLIGHT_TAKEOFF:
            case AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND:
                if (g.takeoff_flap_percent != 0) {
                    auto_flap_percent = g.takeoff_flap_percent;
                }
                break;
            case AP_Vehicle::FixedWing::FLIGHT_NORMAL:
                if (g.takeoff_flap_percent != 0 && in_preLaunch_flight_stage()) {
                    // TODO: move this to a new FLIGHT_PRE_TAKEOFF stage
                    auto_flap_percent = g.takeoff_flap_percent;
                }
                break;
            case AP_Vehicle::FixedWing::FLIGHT_LAND:
                if (landing.get_flap_percent() != 0) {
                  auto_flap_percent = landing.get_flap_percent();
                }
                break;
            default:
                break;
        }
    }

    // manual flap input overrides auto flap input
    if (abs(manual_flap_percent) > auto_flap_percent) {
        auto_flap_percent = manual_flap_percent;
    }

    notify.set_plane_auto_flaps(lrintf(auto_flap_percent));

    SRV_Channels::set_output_scaled(SRV_Channel::k_flap_auto, auto_flap_percent);
    SRV_Channels::set_output_scaled(SRV_Channel::k_flap, manual_flap_percent);

    SRV_Channels::set_slew_rate(SRV_Channel::k_flap_auto, g.flap_slewrate, 100, G_Dt);
    SRV_Channels::set_slew_rate(SRV_Channel::k_flap, g.flap_slewrate, 100, G_Dt);

    // output to flaperons, if any
    flaperon_update();
}

#if LANDING_GEAR_ENABLED == ENABLED
/*
  setup landing gear state
 */
void Plane::set_landing_gear(void)
{
    if (control_mode == &mode_auto && hal.util->get_soft_armed() && is_flying() && gear.last_flight_stage != flight_stage) {
        switch (flight_stage) {
        case AP_Vehicle::FixedWing::FLIGHT_LAND:
            g2.landing_gear.deploy_for_landing();
            break;
        case AP_Vehicle::FixedWing::FLIGHT_NORMAL:
            g2.landing_gear.retract_after_takeoff();
            break;
        default:
            break;
        }
    }
    gear.last_flight_stage = flight_stage;
}
#endif // LANDING_GEAR_ENABLED


/*
  support for twin-engine planes
 */
void Plane::servos_twin_engine_mix(void)
{
    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    float rud_gain = float(plane.g2.rudd_dt_gain) * 0.01f;
    rudder_dt = rud_gain * SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) / SERVO_MAX;

#if ADVANCED_FAILSAFE == ENABLED
    if (afs.should_crash_vehicle()) {
        // when in AFS failsafe force rudder input for differential thrust to zero
        rudder_dt = 0;
    }
#endif

    float throttle_left, throttle_right;

    if (throttle < 0 && have_reverse_thrust() && allow_reverse_thrust()) {
        // doing reverse thrust
        throttle_left  = constrain_float(throttle + 50 * rudder_dt, -100, 0);
        throttle_right = constrain_float(throttle - 50 * rudder_dt, -100, 0);
    } else if (throttle <= 0) {
        throttle_left  = throttle_right = 0;
    } else {
        // doing forward thrust
        throttle_left  = constrain_float(throttle + 50 * rudder_dt, 0, 100);
        throttle_right = constrain_float(throttle - 50 * rudder_dt, 0, 100);
    }
    if (!hal.util->get_soft_armed()) {
        if (arming.arming_required() == AP_Arming::Required::YES_ZERO_PWM) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::ZERO_PWM);
        } else {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0);
        }
    } else {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, throttle_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, throttle_right);
        throttle_slew_limit(SRV_Channel::k_throttleLeft);
        throttle_slew_limit(SRV_Channel::k_throttleRight);
    }
}

/*
  Set throttle,attitude(in Attitude.cpp), and tilt servos for forced flare by RCx_OPTION switch for landing in FW mode
  For Fixed Wind modes with manual throttle control only. Forces tilts up and throttle to THR_MIN.
  Throttle stick must be in idle deadzone. This allows non-momentary switch to be used and quick bailouts
  for go-arounds. Also helps prevent propstrike after landing with switch release on ground.
*/
void Plane::force_flare(void)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_transition() && plane.arming.is_armed()) { //allows for ground checking of flare tilts
        return;
    }
    if (control_mode->is_vtol_mode()) {
        return;
    }
    /* to be active must be:
       -manual throttle mode
       -in an enabled flare mode (RC switch active)
       -at zero thrust: in throttle trim dz except for sprung throttle option where trim is at hover stick
    */
    if (!control_mode->does_auto_throttle() && flare_mode != FlareMode::FLARE_DISABLED && throttle_at_zero()) {
        int32_t tilt = -SERVO_MAX;  //this is tilts up for a normal tiltrotor if at zero thrust throttle stick      
        if (quadplane.tiltrotor.enabled() && (quadplane.tiltrotor.type == Tiltrotor::TILT_TYPE_BICOPTER)) {
            tilt = 0; // this is tilts up for a Bicopter
        }
        if (quadplane.tailsitter.enabled()) {
            tilt = SERVO_MAX; //this is tilts up for a tailsitter
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft, tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight, tilt);
        float throttle_min = MAX(aparm.throttle_min.get(),0); //allows ICE to run if used but accounts for reverse thrust setups
        if (arming.is_armed()) {  //prevent running motors if unarmed
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle_min);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, throttle_min);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, throttle_min);
        }
    }
#endif
}

/* Set the flight control servos based on the current calculated values

  This function operates by first building up output values for
  channels using set_servo() and set_radio_out(). Using
  set_radio_out() is for when a raw PWM value of output is given which
  does not depend on any output scaling. Using set_servo() is for when
  scaling and mixing will be needed.

  Finally servos_output() is called to push the final PWM values
  for output channels
*/
void Plane::set_servos(void)
{
    // start with output corked. the cork is released when we run
    // servos_output(), which is run from all code paths in this
    // function
    SRV_Channels::cork();

    // this is to allow the failsafe module to deliberately crash
    // the plane. Only used in extreme circumstances to meet the
    // OBC rules
#if ADVANCED_FAILSAFE == ENABLED
    if (afs.should_crash_vehicle()) {
        afs.terminate_vehicle();
        if (!afs.terminating_vehicle_via_landing()) {
            return;
        }
    }
#endif

    // do any transition updates for quadplane
#if HAL_QUADPLANE_ENABLED
    quadplane.update();
#endif

    if (control_mode == &mode_auto && auto_state.idle_mode) {
        // special handling for balloon launch
        set_servos_idle();
        servos_output();
        return;
    }

    /*
      see if we are doing ground steering.
     */
    if (!steering_control.ground_steering) {
        // we are not at an altitude for ground steering. Set the nose
        // wheel to the rudder just in case the barometer has drifted
        // a lot
        steering_control.steering = steering_control.rudder;
    } else if (!SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
        // we are within the ground steering altitude but don't have a
        // dedicated steering channel. Set the rudder to the ground
        // steering output
        steering_control.rudder = steering_control.steering;
    }

    // clear ground_steering to ensure manual control if the yaw stabilizer doesn't run
    steering_control.ground_steering = false;

    if (control_mode == &mode_training) {
        steering_control.rudder = rudder_in_expo(false);
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, steering_control.rudder);
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering_control.steering);

    if (control_mode == &mode_manual) {
        set_servos_manual_passthrough();
    } else {
        set_servos_controlled();
    }

    // setup flap outputs
    set_servos_flaps();

#if LANDING_GEAR_ENABLED == ENABLED
    // setup landing gear output
    set_landing_gear();
#endif

    // set airbrake outputs
    airbrake_update();

    // slew rate limit throttle
    throttle_slew_limit(SRV_Channel::k_throttle);

    if (!arming.is_armed()) {
        //Some ESCs get noisy (beep error msgs) if PWM == 0.
        //This little segment aims to avoid this.
        switch (arming.arming_required()) {
        case AP_Arming::Required::NO:
            //keep existing behavior: do nothing to radio_out
            //(don't disarm throttle channel even if AP_Arming class is)
            break;

        case AP_Arming::Required::YES_ZERO_PWM:
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttle, 0);
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, 0);
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, 0);
            break;

        case AP_Arming::Required::YES_MIN_PWM:
        default:
            int8_t min_throttle = MAX(aparm.throttle_min.get(),0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, min_throttle);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, min_throttle);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, min_throttle);
            break;
        }
    }

    uint8_t override_pct;
    if (g2.ice_control.throttle_override(override_pct)) {
        // the ICE controller wants to override the throttle for starting
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, override_pct);
    }

    // run output mixer and send values to the hal for output
    servos_output();
}

/*
    This sets servos to neutral if it is a control surface servo in auto mode
*/
void Plane::landing_neutral_control_surface_servos(void)
{
    if (!(landing.get_then_servos_neutral() > 0 &&
            control_mode == &mode_auto &&
            landing.get_disarm_delay() > 0 &&
            landing.is_complete() &&
            !arming.is_armed())) {
                return;
    }


    // after an auto land and auto disarm, set the servos to be neutral just
    // in case we're upside down or some crazy angle and straining the servos.
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS ; i++) {
            SRV_Channel *chan = SRV_Channels::srv_channel(i);
            if (chan == nullptr || !SRV_Channel::is_control_surface(chan->get_function())) {
                continue;
            }

            if (landing.get_then_servos_neutral() == 1) {
                SRV_Channels::set_output_scaled(chan->get_function(), 0);
            } else if (landing.get_then_servos_neutral() == 2) {
                SRV_Channels::set_output_limit(chan->get_function(), SRV_Channel::Limit::ZERO_PWM);
            }
    }
 
}

void Plane::channel_function_apply_diff(SRV_Channel::Aux_servo_function_t func, float diff) const
{
    const float input = SRV_Channels::get_output_scaled(func);
    SRV_Channels::set_output_scaled(func, apply_throws_diff(input, diff));
}

void Plane::set_aileron_outputs() const
{
    const float aileron_input = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);

    const float aileron_left_output = apply_throws_diff(-aileron_input, g2.ailerons_diff);
    const float aileron_right_output = apply_throws_diff(aileron_input, g2.ailerons_diff);

    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron_left, aileron_left_output);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron_right, aileron_right_output);
}

/*
  run configured output mixer. This takes calculated servo_out values
  for each channel and calculates PWM values, then pushes them to
  hal.rcout
 */
void Plane::servos_output(void)
{
    SRV_Channels::cork();

    // support twin-engine aircraft
    servos_twin_engine_mix();

    // run vtail and elevon mixers
    channel_function_mixer(SRV_Channel::k_aileron, SRV_Channel::k_elevator, SRV_Channel::k_elevon_left, SRV_Channel::k_elevon_right);
    channel_function_mixer(SRV_Channel::k_rudder,  SRV_Channel::k_elevator, SRV_Channel::k_vtail_right, SRV_Channel::k_vtail_left);

    // apply elevator differential throws
    channel_function_apply_diff(SRV_Channel::k_elevator, g2.elevator_diff);

    // set left/right aileron outputs
    set_aileron_outputs();

#if HAL_QUADPLANE_ENABLED
    // cope with tailsitters and bicopters
    quadplane.tailsitter.output();
    quadplane.tiltrotor.bicopter_output();
#endif

    // support forced flare option
    force_flare();

    // implement differential spoilers
    dspoiler_update();

    //  set control surface servos to neutral
    landing_neutral_control_surface_servos();

    apply_throttle_to_elevator_mix();
    apply_flap_to_elevator_mix();

    // support MANUAL_RCMASK
    if (g2.manual_rc_mask.get() != 0 && control_mode == &mode_manual) {
        SRV_Channels::copy_radio_in_out_mask(uint16_t(g2.manual_rc_mask.get()));
    }

    SRV_Channels::calc_pwm();

    SRV_Channels::output_ch_all();

    SRV_Channels::push();

    if (g2.servo_channels.auto_trim_mode() != SRV_Channels::SERVO_AUTO_TRIM_DISABLED || plane.auto_trim.run) {
        servos_auto_trim();
    }
}

void Plane::update_throttle_hover() {
    // update hover throttle at 100Hz
#if HAL_QUADPLANE_ENABLED
    quadplane.update_throttle_hover();
#endif
}

const Plane::ServoTrimSetEntry Plane::aileron_trim_set[] = {
    {SRV_Channel::k_aileron, TrimAdjustmentType::AdjustRoll, "Aileron trim saturation"},
    {SRV_Channel::k_aileron_left, TrimAdjustmentType::AdjustRollInverted, "Left aileron trim saturation"},
    {SRV_Channel::k_aileron_right, TrimAdjustmentType::AdjustRoll, "Right aileron trim saturation"},
    {SRV_Channel::k_flaperon_left, TrimAdjustmentType::AdjustRollInverted, "Left flaperon trim saturation"},
    {SRV_Channel::k_flaperon_right, TrimAdjustmentType::AdjustRoll, "Right flaperon trim saturation"},
};

const Plane::ServoTrimSetEntry Plane::elevator_trim_set[] = {
    {SRV_Channel::k_elevator, TrimAdjustmentType::AdjustPitch, "Elevator trim saturation"},
    {SRV_Channel::k_vtail_left, TrimAdjustmentType::AdjustPitch, "VTail left trim saturation"},
    {SRV_Channel::k_vtail_right, TrimAdjustmentType::AdjustPitch, "VTail right trim saturation"},
};

const Plane::ServoTrimSetEntry Plane::elevon_trim_set[] = {
    {SRV_Channel::k_elevon_left, TrimAdjustmentType::AdjustPitchAndRollInverted, "Left elevon trim saturation"},
    {SRV_Channel::k_elevon_right, TrimAdjustmentType::AdjustPitchAndRoll, "Right elevon trim saturation"},
};

const Plane::ServoTrimSetEntry Plane::dspoiler_outer_trim_set[] = {
    {SRV_Channel::k_dspoilerLeft1, TrimAdjustmentType::AdjustPitchAndRollInverted, "Left outer dspoil trim saturation"},
    {SRV_Channel::k_dspoilerRight1, TrimAdjustmentType::AdjustPitchAndRoll, "Right outer dspoil trim saturation"},
};

const Plane::ServoTrimSetEntry Plane::dspoiler_inner_trim_set[] = {
    {SRV_Channel::k_dspoilerLeft2, TrimAdjustmentType::AdjustPitchAndRollInverted, "Left inner dspoil trim saturation"},
    {SRV_Channel::k_dspoilerRight2, TrimAdjustmentType::AdjustPitchAndRoll, "Right inner dspoil trim saturation"},
};

bool Plane::servos_auto_trim_set(const Plane::ServoTrimSetEntry *const trim_set, uint trim_set_entries_count, float pitch_I, float roll_I, int &adjustment, bool *saturation_status, bool adjust_pitch_if_relevant)
{
    int adj_local = 0;
    uint adj_count = 0;
    bool used = false;
    const Plane::ServoTrimSetEntry *trim_set_entry = trim_set;

    while (trim_set_entries_count--) {
        float trim = 0;
        bool saturation;

        switch (trim_set_entry->adjustment_type) {
            case TrimAdjustmentType::AdjustPitch:
                trim = pitch_I;
                break;

            case TrimAdjustmentType::AdjustRoll:
                trim = roll_I;
                break;

            case TrimAdjustmentType::AdjustRollInverted:
                trim = -roll_I;
                break;

            case TrimAdjustmentType::AdjustPitchAndRoll:
                trim = pitch_I + roll_I;
                break;

            case TrimAdjustmentType::AdjustPitchAndRollInverted:
                trim = pitch_I - roll_I;
                break;
        }

        const auto status = g2.servo_channels.adjust_trim(trim_set_entry->function, trim, adj_local, saturation);

        if (status != SRV_Channels::TrimStatus::FunctionUnused) {
            adj_count += 1;
            used = true;

            #pragma GCC diagnostic ignored "-Wformat-security"
            if (!*saturation_status && saturation) GCS_SEND_TEXT(MAV_SEVERITY_INFO, trim_set_entry->channel_saturation_message);
            #pragma GCC diagnostic pop
            *saturation_status = saturation;
        }

        trim_set_entry += 1;
        saturation_status += 1;
    }

    if (!used) {
        return false;
    }

    adjustment += adj_local / adj_count;
    return true;
}

/*
  implement automatic persistent trim of control surfaces
 */
void Plane::servos_auto_trim(void)
{
    // only in auto modes and FBWA
    if (!control_mode->does_auto_throttle() && control_mode != &mode_fbwa) {
        return;
    }
    if (!hal.util->get_soft_armed()) {
        return;
    }
    if (!is_flying()) {
        return;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_assisted_flight() || quadplane.in_vtol_mode()) {
        // can't auto-trim with quadplane motors running
        return;
    }
#endif
    if (abs(nav_roll_cd) > 700 || abs(nav_pitch_cd) > 700) {
        // only when close to level
        return;
    }
    uint32_t now = AP_HAL::millis();
    if (now - auto_trim.last_trim_check < 500) {
        // check twice a second. We want slow trim update
        return;
    }
    if (ahrs.groundspeed() < 8 || smoothed_airspeed < 8) {
        // only when definitely moving
        return;
    }

    // adjust trim on channels by a small amount according to I value
    float roll_I = rollController.get_pid_info().I;
    float pitch_I = pitchController.get_pid_info().I;

    bool aileron_used = false, elevator_used = false, elevon_used = false, dspoiler_outer_used = false, dspoiler_inner_used = false;

    if (!SERVOS_TRIM_SET_STATUS(aileron).finished) aileron_used = SERVOS_TRIM_SET(aileron);
    if (!SERVOS_TRIM_SET_STATUS(elevator).finished) elevator_used = SERVOS_TRIM_SET(elevator);
    if (!SERVOS_TRIM_SET_STATUS(elevon).finished) elevon_used = SERVOS_TRIM_SET(elevon);

    if (!auto_trim.set_status.dspoiler_finished) {
        // cope with various dspoiler options
        const int8_t bitmask = g2.crow_flap_options.get();
        const bool flying_wing       = (bitmask & CrowFlapOptions::FLYINGWING) != 0;
        const bool full_span_aileron = (bitmask & CrowFlapOptions::FULLSPAN) != 0;

        if (!SERVOS_TRIM_SET_STATUS(dspoiler_outer).finished) dspoiler_outer_used = SERVOS_TRIM_SET_EXT(dspoiler_outer, flying_wing);
        if (!SERVOS_TRIM_SET_STATUS(dspoiler_inner).finished && full_span_aileron) dspoiler_inner_used = SERVOS_TRIM_SET_EXT(dspoiler_inner, flying_wing);
    }

    if (!aileron_used) SERVOS_TRIM_SET_STATUS(aileron).finished = true;
    if (!elevator_used) SERVOS_TRIM_SET_STATUS(elevator).finished = true;
    if (!elevon_used) SERVOS_TRIM_SET_STATUS(elevon).finished = true;
    if (!dspoiler_outer_used) SERVOS_TRIM_SET_STATUS(dspoiler_outer).finished = true;
    if (!dspoiler_inner_used) SERVOS_TRIM_SET_STATUS(dspoiler_inner).finished = true;
    if (SERVOS_TRIM_SET_STATUS(dspoiler_inner).finished && SERVOS_TRIM_SET_STATUS(dspoiler_outer).finished) auto_trim.set_status.dspoiler_finished = true;

    auto_trim.last_trim_check = now;

    if (!auto_trim.last_trim_save) {
        auto_trim.last_trim_save = now;
    } else if (now - auto_trim.last_trim_save > 10000) {

        if (!SERVOS_TRIM_SET_STATUS(aileron).finished && abs(SERVOS_TRIM_SET_STATUS(aileron).adjustment) < 4) {
            SERVOS_TRIM_SET_STATUS(aileron).finished = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ailerons trim finished");
        }
        SERVOS_TRIM_SET_STATUS(aileron).adjustment = 0;

        if (!SERVOS_TRIM_SET_STATUS(elevator).finished && abs(SERVOS_TRIM_SET_STATUS(elevator).adjustment) < 4) {
            SERVOS_TRIM_SET_STATUS(elevator).finished = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Elevator trim finished");
        }
        SERVOS_TRIM_SET_STATUS(elevator).adjustment = 0;

        if (!SERVOS_TRIM_SET_STATUS(elevon).finished && abs(SERVOS_TRIM_SET_STATUS(elevon).adjustment) < 4) {
            SERVOS_TRIM_SET_STATUS(elevon).finished = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Elevons trim finished");
        }
        SERVOS_TRIM_SET_STATUS(elevon).adjustment = 0;

        if (!SERVOS_TRIM_SET_STATUS(dspoiler_finished) && abs(SERVOS_TRIM_SET_STATUS(dspoiler_outer).adjustment) < 4 && abs(SERVOS_TRIM_SET_STATUS(dspoiler_inner).adjustment) < 4) {
            SERVOS_TRIM_SET_STATUS(dspoiler_inner).finished = true;
            SERVOS_TRIM_SET_STATUS(dspoiler_outer).finished = true;
            SERVOS_TRIM_SET_STATUS(dspoiler_finished) = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Dspoilers trim finished");
        }
        SERVOS_TRIM_SET_STATUS(dspoiler_inner).adjustment = 0;
        SERVOS_TRIM_SET_STATUS(dspoiler_outer).adjustment = 0;

        if (SERVOS_TRIM_SET_STATUS(aileron).finished && SERVOS_TRIM_SET_STATUS(elevator).finished && SERVOS_TRIM_SET_STATUS(elevon).finished && SERVOS_TRIM_SET_STATUS(dspoiler_finished)) {
            auto_trim.run = false;
            plane.g2.servo_channels.disable_autotrim_if_temporary_enabled();
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Servo auto trim finished");
        }

        g2.servo_channels.save_trim();
        auto_trim.last_trim_save = now;
    }

}
