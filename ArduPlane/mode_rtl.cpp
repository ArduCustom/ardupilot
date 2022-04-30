#include "mode.h"
#include "Plane.h"

bool ModeRTL::_enter()
{
    int16_t radius = plane.g.rtl_radius != 0 ? plane.g.rtl_radius : plane.aparm.loiter_radius;
    plane.loiter.radius = abs(radius);
    plane.loiter.direction = radius < 0 ? -1 : 1;
    plane.loiter.navigate_last_ms = 0;
    plane.nav_controller->reset_reached_loiter_target();
    plane.prev_WP_loc = plane.current_loc;
    plane.do_RTL(plane.get_RTL_altitude_cm());
    plane.rtl.done_climb = false;
    plane.rtl.triggered_by_rc_failsafe = plane.failsafe.rc_failsafe;
    plane.rtl.manual_alt_control = false;
    plane.rtl.reached_home_altitude = false;
    plane.rtl.emergency_landing_status = Plane::FSEmergencyLandingStatus::INACTIVE;
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

void ModeRTL::_exit()
{
    plane.rtl.loitering = false;
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

    float altitude = plane.relative_altitude;
    if (!plane.terrain_disabled()) plane.terrain.height_above_terrain(altitude, true);

    if (plane.rtl.emergency_landing_status >= Plane::FSEmergencyLandingStatus::GLIDING &&
            (plane.g.fs_emergency_landing_land_upwind || (plane.g.fs_emergency_landing_leveling_altitude > -1 && altitude < plane.g.fs_emergency_landing_leveling_altitude.get()))) {
        plane.nav_roll_cd = 0;
        return;
    }

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
            plane.g.fs_emergency_landing_delay > -1) {
            
            float emergency_landing_gliding_altitude_m = plane.g.fs_emergency_landing_gliding_altitude;
            if (plane.g.fs_emergency_landing_leveling_altitude > -1 && plane.g.fs_emergency_landing_leveling_altitude > emergency_landing_gliding_altitude_m) {
                emergency_landing_gliding_altitude_m = plane.g.fs_emergency_landing_leveling_altitude;
            }

            switch (plane.rtl.emergency_landing_status) {
                case Plane::FSEmergencyLandingStatus::INACTIVE:
                    if (plane.reached_loiter_target() && plane.rtl.reached_home_altitude) {
                        plane.rtl.emergency_landing_tstamp_ms = now;
                        plane.rtl.emergency_landing_status = Plane::FSEmergencyLandingStatus::DELAY;
                    } else {
                        break;
                    }
                    FALLTHROUGH;

                case Plane::FSEmergencyLandingStatus::DELAY:
                    if (now - plane.rtl.emergency_landing_tstamp_ms > uint32_t(MAX(0, plane.g.fs_emergency_landing_delay)) * 1000) {
                        plane.next_WP_loc.set_alt_cm(emergency_landing_gliding_altitude_m * 100, Location::AltFrame::ABOVE_HOME);
                        plane.setup_terrain_target_alt(plane.next_WP_loc);
                        plane.set_target_altitude_location(plane.next_WP_loc);
                        plane.rtl.emergency_landing_status = Plane::FSEmergencyLandingStatus::SINKING_TO_GLIDE_ALTITUDE;
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Emergency landing started");
                    } else {
                        break;
                    }
                    FALLTHROUGH;

                case Plane::FSEmergencyLandingStatus::SINKING_TO_GLIDE_ALTITUDE: {
                    float altitude = plane.relative_altitude;

                    // if (altitude < emergency_landing_gliding_altitude_m + 2.0f && (!plane.g.fs_emergency_landing_land_upwind || wind_speed_mps <= 1.0f || (angle > target_angle - 2 && angle < target_angle + 2))) {
                    if (altitude < emergency_landing_gliding_altitude_m + 2.0f) {
                        plane.rtl.emergency_landing_tstamp_ms = now;
                        plane.rtl.emergency_landing_status = Plane::FSEmergencyLandingStatus::ALIGNMENT_INTO_WIND;
                    } else {
                        break;
                    }
                    FALLTHROUGH;
                }

                case Plane::FSEmergencyLandingStatus::ALIGNMENT_INTO_WIND:
                    if (plane.g.fs_emergency_landing_land_upwind) {
                        float yaw;
                        Vector3f v;
                        {
                            AP_AHRS &ahrs = AP::ahrs();
                            WITH_SEMAPHORE(ahrs.get_semaphore());
                            v = ahrs.wind_estimate();
                            yaw = ahrs.yaw;
                        }
                        float wind_angle = 0;
                        const float wind_speed_mps = v.length();
                        if (wind_speed_mps > 1.0f) {
                            wind_angle = ToDeg(wrap_2PI(atan2f(v.y, v.x) - yaw));
                        }

                        const float wind_target_angle = 180 + plane.loiter.direction * 2;

                        if (now - plane.rtl.emergency_landing_tstamp_ms > 120000 || wind_speed_mps <= 1.0f || (wind_angle > wind_target_angle - 2 && wind_angle < wind_target_angle + 2)) {
                            plane.rtl.emergency_landing_status = Plane::FSEmergencyLandingStatus::GLIDING;
                        } else {
                            break;
                        }
                    } else {
                        plane.rtl.emergency_landing_status = Plane::FSEmergencyLandingStatus::GLIDING;
                    }
                    FALLTHROUGH;


                case Plane::FSEmergencyLandingStatus::GLIDING: {
                    plane.set_auto_thr_gliding(true);
                    float altitude = plane.relative_altitude;
                    if (!plane.terrain_disabled()) plane.terrain.height_above_terrain(altitude, true);

                    if (altitude < 10) {
                        // below 10m don't go back to home altitude if FS ends
                        plane.rtl.emergency_landing_status = Plane::FSEmergencyLandingStatus::GLIDING_NO_RETURN;
                    }
                    FALLTHROUGH;
                }

                case Plane::FSEmergencyLandingStatus::GLIDING_NO_RETURN:
                    plane.disarm_if_autoland_complete();
            }

        } else if (plane.rtl.emergency_landing_status != Plane::FSEmergencyLandingStatus::INACTIVE && plane.rtl.emergency_landing_status != Plane::FSEmergencyLandingStatus::GLIDING_NO_RETURN) {
            // we just came out of FS and not reached no return altitude in emergency landing so reset loiter radius
            int16_t radius = plane.g.rtl_radius != 0 ? plane.g.rtl_radius : plane.aparm.loiter_radius;
            plane.loiter.radius = abs(radius);
            plane.loiter.direction = radius < 0 ? -1 : 1;
            plane.rtl.emergency_landing_status = Plane::FSEmergencyLandingStatus::INACTIVE;
        }

        if (((plane.g2.flight_options & FlightOptions::RTL_MANUAL_ALT_CONTROL) == 0 || plane.failsafe.rc_failsafe || plane.rtl.triggered_by_rc_failsafe) && plane.rtl.emergency_landing_status == Plane::FSEmergencyLandingStatus::INACTIVE && plane.reached_loiter_target()) {
            int32_t home_altitude_cm;
            if (plane.g.RTL_home_altitude > -1) {
                home_altitude_cm = plane.get_home_RTL_altitude_cm();
            } else {
                home_altitude_cm = plane.get_RTL_altitude_cm();
            }

            plane.next_WP_loc.set_alt_cm(home_altitude_cm, Location::AltFrame::ABSOLUTE);
            plane.setup_terrain_target_alt(plane.next_WP_loc);
            plane.set_target_altitude_location(plane.next_WP_loc);

            plane.rtl.done_climb = true;

            if (abs(home_altitude_cm - plane.current_loc.alt) < 500) {
                // less than 5m away from target home altitude
                plane.rtl.reached_home_altitude = true;
            }

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

    // manual loiter radius and direction control
    plane.update_loiter_radius_and_direction();

    if (plane.rc_failsafe()) {
        int16_t radius = plane.g.rtl_radius != 0 ? plane.g.rtl_radius : plane.aparm.loiter_radius;

        if (plane.rtl.emergency_landing_status >= Plane::FSEmergencyLandingStatus::SINKING_TO_GLIDE_ALTITUDE && plane.g.fs_emergency_landing_loiter_radius) {
            radius = plane.g.fs_emergency_landing_loiter_radius;
        }

        plane.loiter.radius = abs(radius);
        plane.loiter.direction = radius < 0 ? -1 : 1;
    }

    if (plane.reached_loiter_target()) {
        plane.rtl.loitering = true;
    }

    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(lrintf(plane.loiter.radius));
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
