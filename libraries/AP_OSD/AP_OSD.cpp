/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * AP_OSD partially based on betaflight and inav osd.c implemention.
 * clarity.mcm font is taken from inav configurator.
 * Many thanks to their authors.
 */

#include "AP_OSD.h"

#if OSD_ENABLED || OSD_PARAM_ENABLED

#include "AP_OSD_MAX7456.h"
#ifdef WITH_SITL_OSD
#include "AP_OSD_SITL.h"
#endif
#include "AP_OSD_MSP.h"
#include "AP_OSD_MSP_DisplayPort.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <utility>
#include <AP_Notify/AP_Notify.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RSSI/AP_RSSI.h>
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>

// macro for easy use of var_info2
#define AP_SUBGROUPINFO2(element, name, idx, thisclazz, elclazz) { AP_PARAM_GROUP, idx, name, AP_VAROFFSET(thisclazz, element), { group_info : elclazz::var_info2 }, AP_PARAM_FLAG_NESTED_OFFSET }

const AP_Param::GroupInfo AP_OSD::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: OSD type
    // @Description: OSD type. TXONLY makes the OSD parameter selection available to other modules even if there is no native OSD support on the board, for instance CRSF.
    // @Values: 0:None,1:MAX7456,2:SITL,3:MSP,4:TXONLY
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_OSD, osd_type, 0, AP_PARAM_FLAG_ENABLE),

#if OSD_ENABLED
    // @Param: _CHAN
    // @DisplayName: Screen switch transmitter channel
    // @Description: This sets the channel used to switch different OSD screens.
    // @Values: 0:Disable,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    // @User: Standard
    AP_GROUPINFO("_CHAN", 2, AP_OSD, rc_channel, 0),

    // @Group: 1_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[0], "1_", 3, AP_OSD, AP_OSD_Screen),

    // @Group: 2_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[1], "2_", 4, AP_OSD, AP_OSD_Screen),

    // @Group: 3_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[2], "3_", 5, AP_OSD, AP_OSD_Screen),

    // @Group: 4_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[3], "4_", 6, AP_OSD, AP_OSD_Screen),

    // @Group: 5_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[4], "5_", 58, AP_OSD, AP_OSD_Screen),

    // @Param: _SW_METHOD
    // @DisplayName: Screen switch method
    // @Description: This sets the method used to switch different OSD screens.
    // @Values: 0: switch to next screen if channel value was changed, 1: select screen based on pwm ranges specified for each screen, 2: switch to next screen after low to high transition and every 1s while channel value is high
    // @User: Standard
    AP_GROUPINFO("_SW_METHOD", 7, AP_OSD, sw_method, AP_OSD::TOGGLE),

    // @Param: _OPTIONS
    // @DisplayName: OSD Options
    // @Description: This sets options that change the display
    // @Bitmask: 0:UseDecimalPack, 1:InvertedWindPointer, 2:InvertedAHRoll, 3:Convert feet to miles at 5280ft instead of 10000ft, 4:DisableCrosshair, 20:Prefix LQ with RF Mode, 21:One decimal attitude, 22:One decimal throttle, 23:Shorten Pluscode 
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 8, AP_OSD, options, OPTION_DECIMAL_PACK | OPTION_ONE_DECIMAL_ATTITUDE | OPTION_RF_MODE_ALONG_WITH_LQ),

    // @Param: _FONT
    // @DisplayName: OSD Font
    // @Description: This sets which OSD font to use. It is an integer from 0 to the number of fonts available
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_FONT", 9, AP_OSD, font_num, 0),

    // @Param: _V_OFFSET
    // @DisplayName: OSD vertical offset
    // @Description: Sets vertical offset of the osd inside image
    // @Range: 0 31
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_V_OFFSET", 10, AP_OSD, v_offset, 16),

    // @Param: _H_OFFSET
    // @DisplayName: OSD horizontal offset
    // @Description: Sets horizontal offset of the osd inside image
    // @Range: 0 63
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_H_OFFSET", 11, AP_OSD, h_offset, 32),

    // @Param: _W_RSSI
    // @DisplayName: RSSI warn level (in %)
    // @Description: Set level at which RSSI item will flash
    // @Range: 0 99
    // @User: Standard
    AP_GROUPINFO("_W_RSSI", 12, AP_OSD, warn_rssi, 30),

    // @Param: _W_NSAT
    // @DisplayName: NSAT warn level
    // @Description: Set level at which NSAT item will flash
    // @Range: 1 30
    // @User: Standard
    AP_GROUPINFO("_W_NSAT", 13, AP_OSD, warn_nsat, 9),

    // @Param: _UNITS
    // @DisplayName: Display Units
    // @Description: Sets the units to use in displaying items
    // @Values: 0:Metric,1:Imperial,2:SI,3:Aviation
    // @User: Standard
    AP_GROUPINFO("_UNITS", 15, AP_OSD, units, 0),

    // @Param: _MSG_TIME
    // @DisplayName: Message display duration in seconds
    // @Description: Sets message duration seconds
    // @Range: 1 20
    // @User: Standard
    AP_GROUPINFO("_MSG_TIME", 16, AP_OSD, msgtime_s, 10),

    // @Param: _ARM_SCR
    // @DisplayName: Arm screen
    // @Description: Screen to be shown on Arm event. Zero to disable the feature.
    // @Range: 0 4
    // @User: Standard
    AP_GROUPINFO("_ARM_SCR", 17, AP_OSD, arm_scr, 0),

    // @Param: _DSARM_SCR
    // @DisplayName: Disarm screen
    // @Description: Screen to be shown on disarm event. Zero to disable the feature.
    // @Range: 0 4
    // @User: Standard
    AP_GROUPINFO("_DSARM_SCR", 18, AP_OSD, disarm_scr, 0),

    // @Param: _FS_SCR
    // @DisplayName: Failsafe screen
    // @Description: Screen to be shown on failsafe event. Zero to disable the feature.
    // @Range: 0 4
    // @User: Standard
    AP_GROUPINFO("_FS_SCR", 19, AP_OSD, failsafe_scr, 0),

#if OSD_PARAM_ENABLED
    // @Param: _BTN_DELAY
    // @DisplayName: Button delay
    // @Description: Debounce time in ms for stick commanded parameter navigation.
    // @Range: 0 3000
    // @User: Advanced
    AP_GROUPINFO("_BTN_DELAY", 20, AP_OSD, button_delay_ms, 300),
#endif
#if AP_TERRAIN_AVAILABLE
    // @Param: _W_TERR
    // @DisplayName: Terrain warn level
    // @Description: Set level below which TER_HGT item will flash. -1 disables.
    // @Range: -1 3000
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("_W_TERR", 23, AP_OSD, warn_terr, -1),
#endif

    // @Param: _AH_PITCH_MAX
    // @DisplayName: Maximum pitch the artificial horizon can display
    // @Description: Maximum pitch the artificial horizon can display
    // @Range: 0 90
    // @User: Standard
    AP_GROUPINFO("_AH_PITCH_MAX", 59, AP_OSD, ah_pitch_max, 20),

    // @Param: _W_VERT_ACC
    // @DisplayName: Underspeed warn speed
    // @Description: Set speed under which ASPDx items will flash
    // @Range: 0 1000
    // @Units: gravities
    // @User: Standard
    AP_GROUPINFO("_W_VERT_ACC", 60, AP_OSD, warn_vert_acc, 0),

    // @Param: _W_ASPD_LOW
    // @DisplayName: Underspeed warn speed
    // @Description: Set speed under which ASPDx items will flash
    // @Range: 0 1000
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("_W_ASPD_LOW", 61, AP_OSD, warn_aspd_low, 0),

    // @Param: _W_ASPD_HIGH
    // @DisplayName: Overspeed warn speed
    // @Description: Set speed above which ASPDx items will flash
    // @Range: 0 1000
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("_W_ASPD_HIGH", 62, AP_OSD, warn_aspd_high, 0),

    // @Param: _EFF_UNIT
    // @DisplayName: Base unit for efficiency values
    // @Description: Base unit to be used for displaying distance and vertical speed efficiency values
    // @Values: 0:mAh,1:Wh
    // @User: Standard
    AP_GROUPINFO("_EFF_UNIT", 63, AP_OSD, efficiency_unit_base, AP_OSD::EFF_UNIT_BASE_MAH),

#if HAL_WITH_ESC_TELEM
    // @Param: _W_BLHRPM
    // @DisplayName: BLHRPM warn level
    // @Description: Set level above which BLHRPM item will flash
    // @Range: 0 500
    // @Units: kRPM
    // @User: Standard
    AP_GROUPINFO("_W_BLHRPM", 28, AP_OSD, warn_blh_high_rpm, 0.0f),

    // @Param: _W_BLHLRPM
    // @DisplayName: BLHRPM warn level
    // @Description: Set level under which BLHRPM item will flash with throttle > 5% and armed
    // @Range: 0 10000
    // @Units: RPM
    // @User: Standard
    AP_GROUPINFO("_W_BLHLRPM", 29, AP_OSD, warn_blh_low_rpm, 0.1f),

    // @Param: _W_BLHTEMP
    // @DisplayName: BLHTEMP warn level
    // @Description: Set level at which BLHTEMP item will flash
    // @Range: 0 200
    // @Units: degC
    // @User: Standard
    AP_GROUPINFO("_W_BLHTEMP", 30, AP_OSD, warn_blhtemp, 80.0f),
#endif

#endif //osd enabled

#if OSD_PARAM_ENABLED
    // @Group: 6_
    // @Path: AP_OSD_ParamScreen.cpp
    AP_SUBGROUPINFO(param_screen[0], "6_", 21, AP_OSD, AP_OSD_ParamScreen),

    // @Group: 7_
    // @Path: AP_OSD_ParamScreen.cpp
    AP_SUBGROUPINFO(param_screen[1], "7_", 22, AP_OSD, AP_OSD_ParamScreen),
#endif

#if OSD_ENABLED
    // additional tables to go beyond 63 limit
    AP_SUBGROUPINFO2(screen[0], "1_", 31, AP_OSD, AP_OSD_Screen),
    AP_SUBGROUPINFO2(screen[1], "2_", 32, AP_OSD, AP_OSD_Screen),
    AP_SUBGROUPINFO2(screen[2], "3_", 33, AP_OSD, AP_OSD_Screen),
    AP_SUBGROUPINFO2(screen[3], "4_", 34, AP_OSD, AP_OSD_Screen),
    AP_SUBGROUPINFO2(screen[4], "5_", 35, AP_OSD, AP_OSD_Screen),
#endif

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

// singleton instance
AP_OSD *AP_OSD::_singleton;

AP_OSD::AP_OSD()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_OSD must be singleton");
    }
    AP_Param::setup_object_defaults(this, var_info);
#if OSD_ENABLED
    // default first screen enabled
    screen[0].enabled = 1;
    previous_pwm_screen = -1;
#endif
#ifdef WITH_SITL_OSD
    osd_type.set_default(OSD_SITL);
#endif

#ifdef HAL_OSD_TYPE_DEFAULT
    osd_type.set_default(HAL_OSD_TYPE_DEFAULT);
#endif
    _singleton = this;
}

void AP_OSD::init()
{
    switch (osd_types(osd_type.get())) {
    case OSD_NONE:
    case OSD_TXONLY:
    default:
        break;

    case OSD_MAX7456: {
#ifdef HAL_WITH_SPI_OSD
        AP_HAL::OwnPtr<AP_HAL::Device> spi_dev = std::move(hal.spi->get_device("osd"));
        if (!spi_dev) {
            break;
        }
        backend = AP_OSD_MAX7456::probe(*this, std::move(spi_dev));
        if (backend == nullptr) {
            break;
        }
        hal.console->printf("Started MAX7456 OSD\n");
#endif
        break;
    }

#ifdef WITH_SITL_OSD
    case OSD_SITL: {
        backend = AP_OSD_SITL::probe(*this);
        if (backend == nullptr) {
            break;
        }
        hal.console->printf("Started SITL OSD\n");
        break;
    }
#endif
    case OSD_MSP: {
        backend = AP_OSD_MSP::probe(*this);
        if (backend == nullptr) {
            break;
        }
        hal.console->printf("Started MSP OSD\n");
        break;
    }
#if HAL_WITH_MSP_DISPLAYPORT
    case OSD_MSP_DISPLAYPORT: {
        backend = AP_OSD_MSP_DisplayPort::probe(*this);
        if (backend == nullptr) {
            break;
        }
        hal.console->printf("Started MSP DisplayPort OSD\n");
        break;
    }
#endif
    }
#if OSD_ENABLED
    if (backend != nullptr) {
        // populate the fonts lookup table
        backend->init_symbol_set(AP_OSD_AbstractScreen::symbols_lookup_table, AP_OSD_NUM_SYMBOLS);
        // create thread as higher priority than IO for all backends
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_OSD::osd_thread, void), "OSD", 1280, AP_HAL::Scheduler::PRIORITY_IO, 1);
    }
#endif
}

#if OSD_ENABLED
void AP_OSD::osd_thread()
{
    while (true) {
        hal.scheduler->delay(100);
        update_osd();
    }
}

void AP_OSD::update_osd()
{
    backend->clear();

    if (!_disable) {
        update_stats();
        update_current_screen();

        get_screen(current_screen).set_backend(backend);
        // skip drawing for MSP OSD backends to save some resources
        if (osd_types(osd_type.get()) != OSD_MSP) {
            get_screen(current_screen).draw();
        }
    }

    backend->flush();
}

//update maximums and totals
void AP_OSD::update_stats()
{
    // allow other threads to consume stats info
    WITH_SEMAPHORE(_sem);

    uint32_t now = AP_HAL::millis();
    if (!AP_Notify::flags.armed) {
        _stats.last_update_ms = now;
        return;
    }

    uint32_t delta_ms = now - _stats.last_update_ms;
    _stats.last_update_ms = now;
    uint32_t new_samples = _stats.samples + 1;

    AP_BattMonitor &battery = AP::battery();

    // maximum and average current
    float amps;
    if (battery.current_amps(amps)) {
        _stats.max_current_a = fmaxf(_stats.max_current_a, amps);
        _stats.avg_current_a = (_stats.avg_current_a * _stats.samples + amps) / new_samples;
    }

    // maximum and average power
    float power;
    if (battery.power_watts(power)) {
        _stats.max_power_w = fmaxf(_stats.max_power_w, power);
        _stats.avg_power_w = (_stats.avg_power_w * _stats.samples + power) / new_samples;
    }

    // minimum voltage
    float voltage = battery.voltage();
    if (voltage > 0) {
        _stats.min_voltage_v = fminf(_stats.min_voltage_v, voltage);
    }

    // minimum cell voltage
    float cell_voltage;
    const bool cell_voltage_available = battery.cell_avg_voltage(cell_voltage);
    if (cell_voltage_available && cell_voltage > 0) {
        _stats.min_cell_voltage_v = fminf(_stats.min_cell_voltage_v, cell_voltage);
    }

    // armed consumed mAh / Wh
    _stats.consumed_mah_available = battery.consumed_mah(_stats.consumed_mah);
    _stats.consumed_wh_available = battery.consumed_wh(_stats.consumed_wh);

    Vector2f ground_speed_vector;
    Vector3f wind_speed_vector;
    Location loc {};
    Location home_loc;
    bool home_is_set;
    bool have_airspeed_estimate;
    float alt;
    float air_speed_mps = 0.0f;
    {
        // minimize semaphore scope
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        ground_speed_vector = ahrs.groundspeed_vector();
        home_is_set = ahrs.get_location(loc) && ahrs.home_is_set();
        if (home_is_set) {
            home_loc = ahrs.get_home();
        }
        ahrs.get_relative_position_D_home(alt);
        have_airspeed_estimate = ahrs.airspeed_estimate(air_speed_mps);

        wind_speed_vector = ahrs.wind_estimate();
    }

    float wind_speed_mps = wind_speed_vector.length();
    float ground_speed_mps = ground_speed_vector.length();
    if (ground_speed_mps < 0.178) {
        ground_speed_mps = 0.0;
    }

    // total distance travelled
    float dist_ground_m = (ground_speed_mps * delta_ms)*0.001;
    _stats.last_ground_distance_m += dist_ground_m;

    // maximum ground and wind speed
    _stats.max_ground_speed_mps = fmaxf(_stats.max_ground_speed_mps, ground_speed_mps);
    _stats.max_wind_speed_mps = fmaxf(_stats.max_wind_speed_mps, wind_speed_mps);

    // average wind speed
    _stats.avg_wind_speed_mps = (_stats.avg_wind_speed_mps * _stats.samples + wind_speed_mps) / new_samples;

    // maximum distance
    if (home_is_set) {
        float distance = home_loc.get_distance(loc);
        _stats.max_dist_m = fmaxf(_stats.max_dist_m, distance);
    }

    // maximum altitude
    alt = -alt;
    _stats.max_alt_m = fmaxf(_stats.max_alt_m, alt);

    // minimum RSSI
    AP_RSSI *ap_rssi = AP_RSSI::get_singleton();
    if (ap_rssi) {
        _stats.min_rssi = fminf(_stats.min_rssi, ap_rssi->read_receiver_rssi());
    }

    // minimum RSSI dBm
    const int8_t rssi_dbm = AP::crsf()->get_link_status().rssi_dbm;
    if (rssi_dbm >= 0) {
        _stats.min_rssi_dbm = MAX(_stats.min_rssi_dbm, rssi_dbm);
    }

    // maximum TX power
    const int16_t tx_power = AP::crsf()->get_link_status().tx_power;
    _stats.max_tx_power = MAX(_stats.max_tx_power, tx_power);

    // max airspeed / total air distance travelled using either true or synthetic air speed
    if (have_airspeed_estimate) {
        _stats.max_air_speed_mps = fmaxf(_stats.max_air_speed_mps, air_speed_mps);

        // XXX might be interesting to take into account AoA for better precision 
        float air_dist_m = (air_speed_mps * delta_ms)*0.001;
        _stats.last_air_distance_m += air_dist_m;
    }

#if HAL_WITH_ESC_TELEM
    // max/avg esc temp
    AP_ESC_Telem& telem = AP::esc_telem();
    int16_t highest_temperature;
    _stats.esc_temperature_available = telem.get_highest_temperature(highest_temperature);
    if (_stats.esc_temperature_available) {
        highest_temperature /= 100;
        _stats.max_esc_temp = MAX(_stats.max_esc_temp, highest_temperature);
        _stats.avg_esc_temp = (int32_t(_stats.avg_esc_temp) * _stats.samples + highest_temperature) / new_samples;
    }
#endif

    _stats.samples = new_samples;
}

//Thanks to minimosd authors for the multiple osd screen idea
void AP_OSD::update_current_screen()
{
    // Switch on ARM/DISARM event
    if (AP_Notify::flags.armed) {
        if (!was_armed && arm_scr > 0 && arm_scr <= AP_OSD_NUM_DISPLAY_SCREENS && get_screen(arm_scr-1).enabled) {
            current_screen = arm_scr-1;
        }
        was_armed = true;
    } else if (was_armed) {
        if (disarm_scr > 0 && disarm_scr <= AP_OSD_NUM_DISPLAY_SCREENS && get_screen(disarm_scr-1).enabled) {
            current_screen = disarm_scr-1;
        }
        was_armed = false;
    }

    // Switch on failsafe event
    if (AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_battery) {
        if (!was_failsafe && failsafe_scr > 0 && failsafe_scr <= AP_OSD_NUM_DISPLAY_SCREENS && get_screen(failsafe_scr-1).enabled) {
            pre_fs_screen = current_screen;
            current_screen = failsafe_scr-1;
        }
        was_failsafe = true;
    } else if (was_failsafe) {
        if (get_screen(pre_fs_screen).enabled) {
            current_screen = pre_fs_screen;
        }
        was_failsafe = false;
    }

    if (rc_channel == 0) {
        return;
    }

    RC_Channel *channel = RC_Channels::rc_channel(rc_channel-1);
    if (channel == nullptr) {
        return;
    }

    int16_t channel_value = channel->get_radio_in();
    switch (sw_method) {
    //switch to next screen if channel value was changed
    default:
    case TOGGLE:
        if (previous_channel_value == 0) {
            //do not switch to the next screen just after initialization
            previous_channel_value = channel_value;
        }
        if (abs(channel_value-previous_channel_value) > 200) {
            if (switch_debouncer) {
                next_screen();
                previous_channel_value = channel_value;
            } else {
                switch_debouncer = true;
                return;
            }
        }
        break;
    //select screen based on pwm ranges specified
    case PWM_RANGE:
        for (int i=0; i<AP_OSD_NUM_SCREENS; i++) {
            if (get_screen(i).enabled && get_screen(i).channel_min <= channel_value && get_screen(i).channel_max > channel_value) {
                if (previous_pwm_screen == i) {
                    break;
                } else {
                current_screen = previous_pwm_screen = i;
                }
            }
        }
        break;
    //switch to next screen after low to high transition and every 1s while channel value is high
    case AUTO_SWITCH:
        if (channel_value > channel->get_radio_trim()) {
            if (switch_debouncer) {
                uint32_t now = AP_HAL::millis();
                if (now - last_switch_ms > 1000) {
                    next_screen();
                    last_switch_ms = now;
                }
            } else {
                switch_debouncer = true;
                return;
            }
        } else {
            last_switch_ms = 0;
        }
        break;
    }
    switch_debouncer = false;
}

//select next avaliable screen, do nothing if all screens disabled
void AP_OSD::next_screen()
{
    uint8_t t = current_screen;
    do {
        t = (t + 1)%AP_OSD_NUM_SCREENS;
    } while (t != current_screen && !get_screen(t).enabled);
    current_screen = t;
}

// set navigation information for display
void AP_OSD::set_nav_info(NavInfo &navinfo)
{
    // do this without a lock for now
    nav_info = navinfo;
}
#endif // OSD_ENABLED

// handle OSD parameter configuration
#if HAL_GCS_ENABLED
void AP_OSD::handle_msg(const mavlink_message_t &msg, const GCS_MAVLINK& link)
{
    bool found = false;

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_OSD_PARAM_CONFIG: {
        mavlink_osd_param_config_t packet;
        mavlink_msg_osd_param_config_decode(&msg, &packet);
#if OSD_PARAM_ENABLED
        for (uint8_t i = 0; i < AP_OSD_NUM_PARAM_SCREENS; i++) {
            if (packet.osd_screen == i + AP_OSD_NUM_DISPLAY_SCREENS + 1) {
                param_screen[i].handle_write_msg(packet, link);
                found = true;
            }
        }
#endif
        // send back an error
        if (!found) {
            mavlink_msg_osd_param_config_reply_send(link.get_chan(), packet.request_id, OSD_PARAM_INVALID_SCREEN);
        }
    }
        break;
    case MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG: {
        mavlink_osd_param_show_config_t packet;
        mavlink_msg_osd_param_show_config_decode(&msg, &packet);
#if OSD_PARAM_ENABLED
        for (uint8_t i = 0; i < AP_OSD_NUM_PARAM_SCREENS; i++) {
            if (packet.osd_screen == i + AP_OSD_NUM_DISPLAY_SCREENS + 1) {
                param_screen[i].handle_read_msg(packet, link);
                found = true;
            }
        }
#endif
        // send back an error
        if (!found) {
            mavlink_msg_osd_param_show_config_reply_send(link.get_chan(), packet.request_id, OSD_PARAM_INVALID_SCREEN,
                nullptr, OSD_PARAM_NONE, 0, 0, 0);
        }
    }
        break;
    default:
        break;
    }
}
#endif

AP_OSD *AP::osd() {
    return AP_OSD::get_singleton();
}

#endif // OSD_ENABLED || OSD_PARAM_ENABLED
