#include "mode.h"
#include "Plane.h"

bool ModeCruise::_enter()
{
    ModeCourseHold::_enter();

#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();

    return true;
}

void ModeCruise::update()
{
    /*
      in CRUISE mode we use the navigation code to control
      roll when heading is locked. Heading becomes unlocked on
      any aileron or rudder input
    */
    if (!is_zero(plane.channel_roll->get_control_in()) || ((plane.g2.flight_options & FlightOptions::COURSE_HOLD_HEADING_CONTROL_WITH_YAW_STICK) == 0 && !is_zero(plane.channel_rudder->get_control_in()))) {
        plane.course_hold.locked_heading = false;
        plane.course_hold.lock_timer_ms = 0;
    }

    if (!plane.course_hold.locked_heading) {
        plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
        plane.update_load_factor();
    } else {
        plane.calc_nav_roll();
    }
    plane.update_fbwb_speed_height();
}
