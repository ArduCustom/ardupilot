#include "mode.h"
#include "Plane.h"

bool ModeRTL::_enter()
{
    plane.prev_WP_loc = plane.current_loc;
    plane.do_RTL(plane.get_RTL_altitude_cm());
    plane.rtl.done_climb = false;
    plane.rtl.triggered_by_rc_failsafe = plane.failsafe.rc_failsafe;
    plane.rtl.manual_alt_control = false;
    plane.auto_state.emergency_landing = false;
    plane.auto_state.reached_home_in_fs_ms = 0;
    plane.auto_state.reached_emergency_landing_no_return_altitude = false;
#if HAL_QUADPLANE_ENABLED
    plane.vtol_approach_s.approach_stage = Plane::Landing_ApproachStage::RTL;

    // treat RTL as QLAND if we are in guided wait takeoff state, to cope
    // with failsafes during GUIDED->AUTO takeoff sequence
    if (plane.quadplane.guided_wait_takeoff_on_mode_enter) {
       plane.set_mode(plane.mode_qland, ModeReason::QLAND_INSTEAD_OF_RTL);
       return true;
    }

    // make sure the local target altitude is the same as the nav target used for loiter nav
    // this allows us to do FBWB style stick control
    if (plane.g2.flight_options & FlightOptions::RTL_MANUAL_ALT_CONTROL && !plane.rtl.triggered_by_rc_failsafe) {
        plane.rtl.manual_alt_control = true;
        IGNORE_RETURN(plane.prev_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, plane.target_altitude.amsl_cm));
    }

    // do not check if we have reached the loiter target if switching from loiter this will trigger as the nav controller has not yet proceeded the new destination
    switch_QRTL(false);
#endif

    return true;
}

void ModeRTL::update()
{
    plane.calc_nav_roll();

    if (!plane.failsafe.rc_failsafe && !plane.rtl.triggered_by_rc_failsafe) {
        // not int FS

        if (plane.g2.flight_options & FlightOptions::RTL_MANUAL_ALT_CONTROL) {
            plane.rtl.manual_alt_control = true;
            plane.update_fbwb_speed_height();
            return;
        }

        if (plane.g2.flight_options & FlightOptions::RTL_CLIMB_FIRST_ONLY_IN_FS) {
            plane.rtl.done_climb = true;
            plane.calc_nav_pitch();
            plane.calc_throttle();
            return;
        }

    } else if (plane.rtl.manual_alt_control) {
        // was not in FS but now is and was already in RTL with manual control enabled
        plane.prev_WP_loc = plane.current_loc;
        plane.do_RTL(plane.get_RTL_altitude_cm());
        plane.rtl.manual_alt_control = false;
    }

    plane.calc_nav_pitch();
    plane.calc_throttle();

    bool alt_threshold_reached = false;
    if (plane.g2.flight_options & FlightOptions::CLIMB_BEFORE_TURN) {
        // Climb to RTL_ALT_MIN before turning. This overrides RTL_CLIMB_MIN.
        alt_threshold_reached = plane.current_loc.alt > plane.next_WP_loc.alt;
    } else if (plane.g2.rtl_climb_min > 0) {
        /*
           when RTL first starts limit bank angle to RTL_LVL_RLL_LMT
           until we have climbed by RTL_CLIMB_MIN meters
           */
        alt_threshold_reached = (plane.current_loc.alt - plane.prev_WP_loc.alt)*0.01 > plane.g2.rtl_climb_min;
    } else {
        return;
    }

    if (!plane.rtl.done_climb && alt_threshold_reached) {
        plane.prev_WP_loc = plane.current_loc;
        plane.setup_glide_slope();
        plane.rtl.done_climb = true;
    }
    if (!plane.rtl.done_climb) {
        // Constrain the roll limit as a failsafe, that way if something goes wrong the plane will
        // eventually turn back and go to RTL instead of going perfectly straight. This also leaves
        // some leeway for fighting wind.
        const int level_roll_limit_cd = MIN(plane.roll_limit_cd, plane.g.rtl_level_roll_limit*100);
        plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -level_roll_limit_cd, level_roll_limit_cd);
    }
}

void ModeRTL::navigate()
{
    const uint32_t now = AP_HAL::millis();

#if HAL_QUADPLANE_ENABLED
    if (plane.control_mode->mode_number() != QRTL) {
        // QRTL shares this navigate function with RTL

        if (plane.quadplane.available() && (plane.quadplane.rtl_mode == QuadPlane::RTL_MODE::VTOL_APPROACH_QRTL)) {
            // VTOL approach landing
            AP_Mission::Mission_Command cmd;
            cmd.content.location = plane.next_WP_loc;
            plane.verify_landing_vtol_approach(cmd);
            if (plane.vtol_approach_s.approach_stage == Plane::Landing_ApproachStage::VTOL_LANDING) {
                plane.set_mode(plane.mode_qrtl, ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL);
            }
            return;
        }

        if ((now - plane.last_mode_change_ms > 1000) && switch_QRTL()) {
            return;
        }
#endif

        if (plane.failsafe.rc_failsafe &&
            !(plane.mission.contains_item(MAV_CMD_DO_LAND_START) && (plane.g.rtl_autoland == RtlAutoland::RTL_THEN_DO_LAND_START || plane.g.rtl_autoland == RtlAutoland::RTL_IMMEDIATE_DO_LAND_START)) &&
            plane.g2.flight_options & FlightOptions::RTL_FAILSAFE_LAND_AFTER_2MIN && plane.reached_loiter_target()) {
            if (plane.auto_state.reached_home_in_fs_ms) {
                if (now - plane.auto_state.reached_home_in_fs_ms > 120000) {
                    plane.set_auto_thr_gliding(true);
                    if (!plane.auto_state.emergency_landing) {
                        plane.auto_state.emergency_landing = true;
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Emergency landing started");
                    }
                }
            } else {
                plane.auto_state.reached_home_in_fs_ms = now;
            }

            if (plane.auto_state.emergency_landing && plane.relative_altitude < 10) {
                plane.auto_state.reached_emergency_landing_no_return_altitude = true;
            }
        } else if (!plane.auto_state.reached_emergency_landing_no_return_altitude) {
            plane.set_auto_thr_gliding(false);
            plane.auto_state.emergency_landing = false;
            plane.auto_state.reached_home_in_fs_ms = 0;
        }

        if (plane.auto_state.reached_emergency_landing_no_return_altitude && !plane.is_flying()) {
            plane.disarm_if_autoland_complete();
        }

#if HAL_QUADPLANE_ENABLED
    }
#endif

    if (plane.g.rtl_autoland == RtlAutoland::RTL_THEN_DO_LAND_START &&
        !plane.auto_state.checked_for_autoland &&
        plane.reached_loiter_target() && 
        labs(plane.altitude_error_cm) < 1000) {
        // we've reached the RTL point, see if we have a landing sequence
        if (plane.mission.jump_to_landing_sequence()) {
            // switch from RTL -> AUTO
            plane.mission.set_force_resume(true);
            plane.set_mode(plane.mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND);
        }

        // prevent running the expensive jump_to_landing_sequence
        // on every loop
        plane.auto_state.checked_for_autoland = true;
    }
    else if (plane.g.rtl_autoland == RtlAutoland::RTL_IMMEDIATE_DO_LAND_START &&
        !plane.auto_state.checked_for_autoland) {
        // Go directly to the landing sequence
        if (plane.mission.jump_to_landing_sequence()) {
            // switch from RTL -> AUTO
            plane.mission.set_force_resume(true);
            plane.set_mode(plane.mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND);
        }

        // prevent running the expensive jump_to_landing_sequence
        // on every loop
        plane.auto_state.checked_for_autoland = true;
    }
    uint16_t radius = abs(plane.g.rtl_radius);
    if (radius > 0) {
        plane.loiter.direction = (plane.g.rtl_radius < 0) ? -1 : 1;
    }

    plane.update_loiter(radius);
}

#if HAL_QUADPLANE_ENABLED
// Switch to QRTL if enabled and within radius
bool ModeRTL::switch_QRTL(bool check_loiter_target)
{
    if (!plane.quadplane.available() || ((plane.quadplane.rtl_mode != QuadPlane::RTL_MODE::SWITCH_QRTL) && (plane.quadplane.rtl_mode != QuadPlane::RTL_MODE::QRTL_ALWAYS))) {  
        return false;
    }

   // if Q_RTL_MODE is QRTL always, then immediately switch to QRTL mode
   if (plane.quadplane.rtl_mode == QuadPlane::RTL_MODE::QRTL_ALWAYS) {
       plane.set_mode(plane.mode_qrtl, ModeReason::QRTL_INSTEAD_OF_RTL);
       return true;
   }

    uint16_t qrtl_radius = abs(plane.g.rtl_radius);
    if (qrtl_radius == 0) {
        qrtl_radius = abs(plane.aparm.loiter_radius);
    }

    if ( (check_loiter_target && plane.nav_controller->reached_loiter_target()) ||
         plane.current_loc.past_interval_finish_line(plane.prev_WP_loc, plane.next_WP_loc) ||
         plane.auto_state.wp_distance < MAX(qrtl_radius, plane.quadplane.stopping_distance())) {
        /*
          for a quadplane in RTL mode we switch to QRTL when we
          are within the maximum of the stopping distance and the
          RTL_RADIUS
         */
        plane.set_mode(plane.mode_qrtl, ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL);
        return true;
    }

    return false;
}

#endif  // HAL_QUADPLANE_ENABLED
