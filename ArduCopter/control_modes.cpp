#include "Copter.h"

void Copter::read_control_switch()
{
    static bool switch_debouncer;
    uint8_t switchPosition = readSwitch();

    // If switchPosition = 255 this indicates that the mode control channel input was out of range
    // If we get this value we do not want to change modes.
    if(switchPosition == 255) return;

    if (!rc().has_valid_input()) {
        // ignore the mode switch channel if there is no valid RC input
        return;
    }

    if (oldSwitchPosition != switchPosition) {

        if (switch_debouncer == false) {
            // this ensures that mode switches only happen if the
            // switch changes for 2 reads. This prevents momentary
            // spikes in the mode control channel from causing a mode
            // switch
            switch_debouncer = true;
            return;
        }

        enum Mode::Number fm;
        if (switchPosition < 6) {
            fm = (enum Mode::Number)flight_modes[switchPosition].get();
        } else {
            fm = (enum Mode::Number)flight_modes2[switchPosition-6].get();
        }

        oldSwitchPosition = switchPosition;

        if (!copter.set_mode(fm, ModeReason::RC_COMMAND)) {
            return;
        }

        if (!rc().find_channel_for_option(RC_Channel_Copter::AUX_FUNC::SIMPLE_MODE) &&
            !rc().find_channel_for_option(RC_Channel_Copter::AUX_FUNC::SUPERSIMPLE_MODE)) {
            // if none of the Aux Switches are set to Simple or Super Simple Mode then
            // set Simple Mode using stored parameters from EEPROM
            if (BIT_IS_SET(copter.g.super_simple, switchPosition)) {
                copter.set_simple_mode(Copter::SimpleMode::SUPERSIMPLE);
            } else {
                copter.set_simple_mode(BIT_IS_SET(copter.g.simple_modes, switchPosition) ? Copter::SimpleMode::SIMPLE : Copter::SimpleMode::NONE);
            }
        }
    }

    switch_debouncer = false;

}

uint8_t Copter::readSwitch(void) const
{
    uint16_t pulsewidth = RC_Channels::get_radio_in(g.flight_mode_chan.get() - 1);
    if (pulsewidth <= 900 || pulsewidth >= 2200) return 255;            // This is an error condition

    if (g2.fltmode_ext == 0) {
        if (pulsewidth < 1231) return 0;
        if (pulsewidth < 1361) return 1;
        if (pulsewidth < 1491) return 2;
        if (pulsewidth < 1621) return 3;
        if (pulsewidth < 1750) return 4;              // Software Manual
        return 5;                                                           // Hardware Manual
    } else {
        if (pulsewidth < 1126) return 0;
        if (pulsewidth < 1201) return 1;
        if (pulsewidth < 1276) return 2;
        if (pulsewidth < 1351) return 3;
        if (pulsewidth < 1426) return 4;
        if (pulsewidth < 1501) return 5;
        if (pulsewidth < 1576) return 6;
        if (pulsewidth < 1651) return 7;
        if (pulsewidth < 1726) return 8;
        if (pulsewidth < 1801) return 9;
        if (pulsewidth < 1876) return 10;
        return 11;
    }
}

void Copter::reset_control_switch()
{
    oldSwitchPosition = 254;
    read_control_switch();
}