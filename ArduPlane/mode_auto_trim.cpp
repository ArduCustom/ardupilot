#include "mode.h"
#include "Plane.h"

bool ModeAutoTrim::_enter()
{
    if (!ModeCourseHold::_enter()) {
        return false;
    }

    plane.servos_auto_trim_start();

    return true;
}

void ModeAutoTrim::_exit() {
    plane.servos_auto_trim_stop();
    ModeCourseHold::_enter();
}