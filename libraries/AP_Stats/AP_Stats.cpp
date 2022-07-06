#include "AP_Stats.h"

#include <AP_Math/AP_Math.h>
#include <AP_RTC/AP_RTC.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_RSSI/AP_RSSI.h>
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>
#include <AP_Vehicle/AP_Vehicle.h>

const extern AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Stats::var_info[] = {

    // @Param: _BOOT_CNT
    // @DisplayName: Boot Count
    // @Description: Number of times board has been booted
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_BOOT_CNT",    0, AP_Stats, params.boot_count, 0),

    // @Param: _FLT_TIME
    // @DisplayName: Total flight time
    // @Description: Total flight time
    // @Units: s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_FLT_TIME",    1, AP_Stats, params.flying_time, 0),

    // @Param: _RUN_TIME
    // @DisplayName: Total run_time
    // @Description: Total time autopilot has run
    // @Units: s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_RUN_TIME",    2, AP_Stats, params.run_time, 0),

    // @Param: _RESET
    // @DisplayName: Statistics reset time
    // @Description: Seconds since January 1st 2016 (Unix epoch+1451606400) since statistics reset (set to 0 to reset statistics)
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_RESET",    3, AP_Stats, params.reset, 1),

    // @Param: _TRAVEL_GND
    // @DisplayName: Total ground distance traveled
    // @Description: Total ground distance traveled
    // @Units: m
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_TRAVEL_GND",    4, AP_Stats, params.flying_ground_traveled, 0),

    // @Param: _FLT_ENERGY
    // @DisplayName: Total consumed energy while flying
    // @Description: Total consumed energy while flying
    // @Units: Wh
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_FLT_ENERGY",  5, AP_Stats, params.flying_energy, 0),

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // @Param: _TRAVEL_AIR
    // @DisplayName: Total air distance traveled
    // @Description: Total air distance traveled
    // @Units: m
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_TRAVEL_AIR",    6, AP_Stats, params.flying_air_traveled, 0),
#endif

    // @Param: _GSPD_AVG
    // @DisplayName: Average ground speed
    // @Description: Average ground speed
    // @Units: m/s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_GSPD_AVG",    7, AP_Stats, params.avg_ground_speed_mps, 0),

    // @Param: _GSPD_MAX
    // @DisplayName: Maximum ground speed
    // @Description: Maximum ground speed
    // @Units: m/s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_GSPD_MAX",    8, AP_Stats, params.max_ground_speed_mps, 0),

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // @Param: _ASPD_AVG
    // @DisplayName: Average air speed
    // @Description: Average air speed
    // @Units: m/s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_ASPD_AVG",    9, AP_Stats, params.avg_air_speed_mps, 0),

    // @Param: _ASPD_MAX
    // @DisplayName: Maximum air speed
    // @Description: Maximum air speed
    // @Units: m/s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_ASPD_MAX",    10, AP_Stats, params.max_air_speed_mps, 0),
#endif

    // @Param: _WSPD_AVG
    // @DisplayName: Average wind speed
    // @Description: Average wind speed
    // @Units: m/s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_WSPD_AVG",    11, AP_Stats, params.avg_wind_speed_mps, 0),

    // @Param: _WSPD_MAX
    // @DisplayName: Maximum wind speed
    // @Description: Maximum wind speed
    // @Units: m/s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_WSPD_MAX",    12, AP_Stats, params.max_wind_speed_mps, 0),

    // @Param: _HOMEDST_MAX
    // @DisplayName: Maximum home distance
    // @Description: Maximum home distance
    // @Units: m
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_HOMEDST_MAX",  13, AP_Stats, params.max_home_distance_m, 0),

    // @Param: _HOMEALT_MAX
    // @DisplayName: Maximum relative altitude
    // @Description: Maximum home distance
    // @Units: m
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_HOMEALT_MAX",  14, AP_Stats, params.max_relative_altitude_m, 0),

    // @Param: _CURRENT_AVG
    // @DisplayName: Average current while flying
    // @Description: Average current while flying
    // @Units: A
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_CURRENT_AVG",  15, AP_Stats, params.avg_flying_current_a, 0),

    // @Param: _CURRENT_MAX
    // @DisplayName: Maximum current while flying
    // @Description: Maximum current while flying
    // @Units: A
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_CURRENT_MAX",  16, AP_Stats, params.max_flying_current_a, 0),

    // @Param: _POWER_AVG
    // @DisplayName: Average power while flying
    // @Description: Average power while flying
    // @Units: W
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_POWER_AVG",  17, AP_Stats, params.avg_flying_power_w, 0),

    // @Param: _POWER_MAX
    // @DisplayName: Maximum power while flying
    // @Description: Maximum power while flying
    // @Units: W
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_POWER_MAX",  18, AP_Stats, params.max_flying_power_w, 0),

    // @Param: _LOAD
    // @DisplayName: Set to 1 then set stat values then reboot or set back to 0
    // @Description: Set to 1 then set stat values then reboot or set back to 0
    // @User: Standard
    AP_GROUPINFO("_LOAD",    19, AP_Stats, params.load, 0),

    // @Param: _FLT_CNT
    // @DisplayName: Flight counter
    // @Description: Number of times the aircraft has been flying
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_FLT_CNT",    20, AP_Stats, params.flight_count, 0),

    // @Param: _HOMEDST_AVG
    // @DisplayName: Average home distance
    // @Description: Average home distance
    // @Units: m
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_HOMEDST_AVG",  21, AP_Stats, params.avg_home_distance_m, 0),

    // @Param: _FLT_TIME_MX
    // @DisplayName: Maximum flight time
    // @Description: Maximum flight time
    // @Units: s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_FLT_TIME_MX",   22, AP_Stats, params.flight_time_max, 0),

    AP_GROUPEND
};

AP_Stats *AP_Stats::_singleton;

// constructor
AP_Stats::AP_Stats(void)
{
    _singleton = this;
}

void AP_Stats::copy_variables_from_parameters()
{
    _total_boot_flying_time_s = params.flying_time;
    _total_boot_flying_ground_traveled_m = params.flying_ground_traveled;
    _total_boot_flying_air_traveled_m = params.flying_air_traveled;
    _total_boot_flying_energy_wh = params.flying_energy;
    _total_boot_avg_ground_speed_mps = params.avg_ground_speed_mps;
    _total_boot_max_ground_speed_mps = params.max_ground_speed_mps;
    _total_boot_avg_air_speed_mps = params.avg_air_speed_mps;
    _total_boot_max_air_speed_mps = params.max_air_speed_mps;
    _total_boot_avg_wind_speed_mps = params.avg_wind_speed_mps;
    _total_boot_max_wind_speed_mps = params.max_wind_speed_mps;
    _total_boot_avg_home_distance_m = params.avg_home_distance_m;
    _total_boot_max_home_distance_m = params.max_home_distance_m;
    _total_boot_max_relative_altitude_m = params.max_relative_altitude_m;
    _total_boot_avg_flying_current_a = params.avg_flying_current_a;
    _total_boot_max_flying_current_a = params.max_flying_current_a;
    _total_boot_avg_flying_power_w = params.avg_flying_power_w;
    _total_boot_max_flying_power_w = params.max_flying_power_w;
    _total_boot_run_time_s = params.run_time;
    _total_boot_flight_count = params.flight_count;
    _reset_tstamp_s = params.reset;
}

void AP_Stats::init()
{
    params.boot_count.set_and_save(params.boot_count+1);

    _boot_tstamp_ms = AP_HAL::millis();
    params.load.set_and_save_ifchanged(0);

    // initialise our variables from parameters:
    copy_variables_from_parameters();
}


void AP_Stats::flush()
{
    params.flight_time_max.set_and_save_ifchanged(MAX(uint32_t(params.flight_time_max), get_current_flight_time_s()));
    params.flying_time.set_and_save_ifchanged(get_total_flying_time_s());
    params.flying_ground_traveled.set_and_save_ifchanged(lrintf(get_total_flying_ground_traveled_m()));
    params.flying_air_traveled.set_and_save_ifchanged(lrintf(get_total_flying_air_traveled_m()));
    params.flying_energy.set_and_save_ifchanged(get_total_flying_energy_wh());
    params.avg_ground_speed_mps.set_and_save_ifchanged(get_total_avg_ground_speed_mps());
    params.max_ground_speed_mps.set_and_save_ifchanged(get_total_max_ground_speed_mps());
    params.avg_air_speed_mps.set_and_save_ifchanged(get_total_avg_air_speed_mps());
    params.max_air_speed_mps.set_and_save_ifchanged(get_total_max_air_speed_mps());
    params.avg_wind_speed_mps.set_and_save_ifchanged(get_total_avg_wind_speed_mps());
    params.max_wind_speed_mps.set_and_save_ifchanged(get_total_max_wind_speed_mps());
    params.avg_home_distance_m.set_and_save_ifchanged(get_total_avg_home_distance_m());
    params.max_home_distance_m.set_and_save_ifchanged(get_total_max_home_distance_m());
    params.max_relative_altitude_m.set_and_save_ifchanged(get_total_max_relative_altitude_m());
    params.avg_flying_current_a.set_and_save_ifchanged(get_total_avg_flying_current_a());
    params.max_flying_current_a.set_and_save_ifchanged(get_total_max_flying_current_a());
    params.avg_flying_power_w.set_and_save_ifchanged(get_total_avg_flying_power_w());
    params.max_flying_power_w.set_and_save_ifchanged(get_total_max_flying_power_w());
    params.flight_count.set_and_save_ifchanged(get_total_flight_count());
    params.run_time.set_and_save_ifchanged(get_total_run_time_s());
}

void AP_Stats::update_flying_time(uint32_t flying_update_delta)
{
    _current_flight_time_ms += flying_update_delta;
    _boot_flying_time_ms += flying_update_delta;
}

void AP_Stats::update_flying_distances_and_speeds(uint32_t flying_update_delta, uint32_t old_flying_sample_count, uint32_t new_flying_sample_count)
{
    Vector2f ground_speed_vector;
    Vector3f wind_speed_vector;
    Location loc {};
    Location home_loc;
    bool home_is_set;
    bool have_airspeed_estimate;
    float relative_altitude_m;
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
        ahrs.get_relative_position_D_home(relative_altitude_m);
        have_airspeed_estimate = ahrs.airspeed_estimate(air_speed_mps);

        wind_speed_vector = ahrs.wind_estimate();
    }
    relative_altitude_m = -relative_altitude_m; // NED to NEU

    float wind_speed_mps = wind_speed_vector.length();
    float ground_speed_mps = ground_speed_vector.length();

    // maximum ground speed
    _boot_max_ground_speed_mps = fmaxf(_boot_max_ground_speed_mps, ground_speed_mps);

    // only starts updating avg and max wind and air speed after a while so the wind speed estimation had time to settle
    if (has_been_flying_for_at_least_s(30)) {
        const uint32_t new_flying_for_some_time_sample_count = _flying_for_some_time_sample_count + 1;

        // maximum wind speed
        _boot_max_wind_speed_mps = fmaxf(_boot_max_wind_speed_mps, wind_speed_mps);

        // average wind speed
        _boot_avg_wind_speed_mps = (_boot_avg_wind_speed_mps * _flying_for_some_time_sample_count + wind_speed_mps) / new_flying_for_some_time_sample_count;

        _wind_speeds_are_available = true;
        _flying_for_some_time_sample_count = new_flying_for_some_time_sample_count;
    }

    // max airspeed / total air distance travelled using either true or synthetic air speed
    if (have_airspeed_estimate) {
        _boot_max_air_speed_mps = fmaxf(_boot_max_air_speed_mps, air_speed_mps);
    }

    // average and maximum distance from home
    if (home_is_set) {
        const uint32_t home_distance = lrintf(home_loc.get_distance(loc));
        _boot_max_home_distance_m = fmaxf(_boot_max_home_distance_m, home_distance);
        _boot_avg_home_distance_m = (_boot_avg_home_distance_m * old_flying_sample_count + home_distance) / new_flying_sample_count;
    }

    // maximum altitude
    _boot_max_relative_altitude_m = fmaxf(_boot_max_relative_altitude_m, lrintf(relative_altitude_m));
}

void AP_Stats::update_flying_travel(uint32_t flying_update_delta, uint32_t old_flying_sample_count, uint32_t new_flying_sample_count)
{
    Vector2f ground_speed_vector;
    bool have_airspeed_estimate;
    float air_speed_mps = 0.0f;
    {
        // minimize semaphore scope
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        ground_speed_vector = ahrs.groundspeed_vector();
        have_airspeed_estimate = ahrs.airspeed_estimate(air_speed_mps);
    }

    float ground_speed_mps = ground_speed_vector.length();

    // total distance travelled
    float dist_ground_m = (ground_speed_mps * flying_update_delta) * 0.001f;
    _boot_flying_ground_traveled_m += dist_ground_m;

    // max airspeed / total air distance travelled using either true or synthetic air speed
    if (have_airspeed_estimate) {
        _boot_max_air_speed_mps = fmaxf(_boot_max_air_speed_mps, air_speed_mps);

        // XXX might be interesting to take into account AoA for better precision 
        float air_distance_m = (air_speed_mps * flying_update_delta) * 0.001f;
        _boot_flying_air_traveled_m += air_distance_m;
    }
}

void AP_Stats::update_flying_current_and_power(uint32_t old_flying_sample_count, uint32_t new_flying_sample_count)
{
    AP_BattMonitor &battery = AP::battery();

    // maximum and average current
    float amps;
    if (battery.current_amps(amps)) {
        _boot_max_flying_current_a = fmaxf(_boot_max_flying_current_a, amps);
        _boot_avg_flying_current_a = (_boot_avg_flying_current_a * old_flying_sample_count + amps) / new_flying_sample_count;
    }

    // maximum and average power
    float power;
    if (battery.power_watts(power)) {
        _boot_max_flying_power_w = fmaxf(_boot_max_flying_power_w, power);
        _boot_avg_flying_power_w = (_boot_avg_flying_power_w * old_flying_sample_count + power) / new_flying_sample_count;
    }
}

void AP_Stats::update_battery(void)
{
    AP_BattMonitor &battery = AP::battery();

    // Energy
    float energy_wh = 0;
    _energy_is_available = battery.consumed_wh_without_losses(energy_wh);
    if (_energy_is_available && is_flying()) {
        _boot_flying_energy_wh += energy_wh - _prev_update_energy_wh;
    }
    _prev_update_energy_wh = energy_wh;

    // mAh
    float mah = 0;
    _mah_is_available = battery.consumed_mah(mah);
    if (_mah_is_available && is_flying()) {
        _boot_flying_mah += mah - _prev_update_mah;
    }
    _prev_update_mah = mah;

    // minimum voltage
    float voltage = battery.voltage();
    if (voltage > 0) {
        _boot_min_voltage_v = fminf(_boot_min_voltage_v, voltage);
    }

    // minimum cell voltage
    float cell_voltage;
    _cell_voltage_is_available = battery.cell_avg_voltage(cell_voltage);
    if (_cell_voltage_is_available && cell_voltage > 0) {
        _boot_min_cell_voltage_v = fminf(_boot_min_cell_voltage_v, cell_voltage);
    }
}

void AP_Stats::update_flying_rc(uint32_t old_flying_sample_count, uint32_t new_flying_sample_count)
{
    // minimum RSSI
    AP_RSSI *ap_rssi = AP_RSSI::get_singleton();
    if (ap_rssi) {
        _boot_min_rc_rssi = fminf(_boot_min_rc_rssi, ap_rssi->read_receiver_rssi());
    }

    // minimum RSSI dBm
    const int8_t rssi_dbm = AP::crsf()->get_link_status().rssi_dbm;
    if (rssi_dbm >= 0) {
        _boot_min_rc_rssi_dbm = MAX(_boot_min_rc_rssi_dbm, rssi_dbm);
    }

    // maximum TX power
    const int16_t tx_power_mw = AP::crsf()->get_link_status().tx_power;
    _boot_max_rc_tx_power_mw = MAX(_boot_max_rc_tx_power_mw, tx_power_mw);
}

#if HAL_WITH_ESC_TELEM
void AP_Stats::update_flying_esc(uint32_t old_flying_sample_count, uint32_t new_flying_sample_count)
{
    // max/avg esc temp
    AP_ESC_Telem& telem = AP::esc_telem();
    int16_t highest_temperature;
    _esc_temperatures_are_available = telem.get_highest_temperature(highest_temperature);
    if (_esc_temperatures_are_available) {
        highest_temperature /= 100;
        _boot_max_esc_temperature_degc = MAX(_boot_max_esc_temperature_degc, highest_temperature);
        _boot_avg_esc_temperature_degc = (int32_t(_boot_avg_esc_temperature_degc) * old_flying_sample_count + highest_temperature) / new_flying_sample_count;
    }
}
#endif


void AP_Stats::reset_params_if_requested(void)
{
    const uint32_t params_reset = params.reset;
    if (params_reset != _reset_tstamp_s || params_reset == 0) {

        params.boot_count.set_and_save_ifchanged(1);
        params.flying_time.set_and_save_ifchanged(0);
        params.flying_ground_traveled.set_and_save_ifchanged(0);
        params.flying_air_traveled.set_and_save_ifchanged(0);
        params.flying_energy.set_and_save_ifchanged(0);
        params.avg_ground_speed_mps.set_and_save_ifchanged(0);
        params.max_ground_speed_mps.set_and_save_ifchanged(0);
        params.avg_air_speed_mps.set_and_save_ifchanged(0);
        params.max_air_speed_mps.set_and_save_ifchanged(0);
        params.avg_wind_speed_mps.set_and_save_ifchanged(0);
        params.max_wind_speed_mps.set_and_save_ifchanged(0);
        params.avg_home_distance_m.set_and_save_ifchanged(0);
        params.max_home_distance_m.set_and_save_ifchanged(0);
        params.max_relative_altitude_m.set_and_save_ifchanged(0);
        params.avg_flying_current_a.set_and_save_ifchanged(0);
        params.max_flying_current_a.set_and_save_ifchanged(0);
        params.avg_flying_power_w.set_and_save_ifchanged(0);
        params.max_flying_power_w.set_and_save_ifchanged(0);
        params.flight_count.set_and_save_ifchanged(0);
        params.flight_time_max.set_and_save_ifchanged(0);
        params.run_time.set_and_save_ifchanged(0);
        params.load.set_and_save_ifchanged(0);
        _boot_tstamp_ms = AP_HAL::millis();

        if (!params_reset) {
            uint32_t system_clock = 0; // in seconds
            uint64_t rtc_clock_us;
            if (AP::rtc().get_utc_usec(rtc_clock_us)) {
                system_clock = rtc_clock_us / 1000000;
                // can't store Unix seconds in a 32-bit float.  Change the
                // time base to Jan 1st 2016:
                system_clock -= 1451606400;
            } else {
                system_clock = 1;
            }
            params.reset.set_and_save_ifchanged(system_clock);
        }

        copy_variables_from_parameters();
    }
}

void AP_Stats::update()
{
    const uint32_t now_ms = AP_HAL::millis();

    {
        WITH_SEMAPHORE(_sem);

        if (params.load) {
            _prev_load = true;
            return;
        } else if (_prev_load) {
            copy_variables_from_parameters();
            _prev_load = false;
        }

        if (is_flying()) {
            if (_last_update_flying_tstamp_ms) {
                uint32_t new_flying_sample_count = _flying_sample_count + 1;
                const uint32_t flying_update_delta = now_ms - _last_update_flying_tstamp_ms;

                update_flying_travel(flying_update_delta, _flying_sample_count, new_flying_sample_count);

                _flying_sample_count = new_flying_sample_count;
            }
            _last_update_flying_tstamp_ms = now_ms;

            if (_last_slow_update_flying_tstamp_ms) {
                if (now_ms - _last_slow_update_flying_tstamp_ms >= 100) {
                    uint32_t new_flying_slow_update_sample_count = _flying_slow_update_sample_count + 1;
                    const uint32_t flying_slow_update_delta = now_ms - _last_slow_update_flying_tstamp_ms;

                    update_flying_time(flying_slow_update_delta);
                    update_flying_distances_and_speeds(flying_slow_update_delta, _flying_slow_update_sample_count, new_flying_slow_update_sample_count);
                    update_flying_current_and_power(_flying_slow_update_sample_count, new_flying_slow_update_sample_count);

                    update_flying_rc(_flying_slow_update_sample_count, new_flying_slow_update_sample_count);

                    #if HAL_WITH_ESC_TELEM
                    update_flying_esc(_flying_slow_update_sample_count, new_flying_slow_update_sample_count);
                    #endif

                    _flying_slow_update_sample_count = new_flying_slow_update_sample_count;
                    _last_slow_update_flying_tstamp_ms = now_ms;
                }
            } else {
                _last_slow_update_flying_tstamp_ms = now_ms;
            }

        } else {
            _last_update_flying_tstamp_ms = 0;
        }

        if (_last_slow_update_tstamp_ms) {
            if (now_ms - _last_slow_update_tstamp_ms >= 100) {
                _last_slow_update_tstamp_ms = now_ms;
                update_battery();
            }
        } else {
            _last_slow_update_tstamp_ms = now_ms;
        }
    }

    if (_flush_tstamp_ms) {
        if (now_ms - _flush_tstamp_ms > flush_interval_ms) {
            flush();
            _flush_tstamp_ms = now_ms;
        }
    } else {
        _flush_tstamp_ms = now_ms;
    }

    reset_params_if_requested();

}

void AP_Stats::set_flying(const bool status)
{
    WITH_SEMAPHORE(_sem);
    if (status) {
        if (!_flying_start_tstamp_ms) {
            _boot_flight_count += 1;
            _flying_start_tstamp_ms = AP_HAL::millis();
        }
    } else {
        _flying_start_tstamp_ms = 0;
        _last_update_flying_tstamp_ms = 0;
        _last_slow_update_flying_tstamp_ms = 0;
        _current_flight_time_ms = 0;
    }
}

float AP_Stats::calc_total_flying_time_related_avg(float total_boot_value, float boot_value)
{
    const uint32_t boot_flying_time_s = get_boot_flying_time_s();
    const uint32_t total_flying_time_s = _total_boot_flying_time_s + boot_flying_time_s;
    if (!total_flying_time_s) return 0;
    return (total_boot_value * _total_boot_flying_time_s + boot_value * boot_flying_time_s) / total_flying_time_s;
}

bool AP_Stats::has_been_flying_for_at_least_s(uint32_t time_s)
{
    return is_flying() && AP_HAL::millis() - _flying_start_tstamp_ms >= time_s * 1000;
}

uint32_t AP_Stats::get_current_flight_time_s(void)
{
    return _current_flight_time_ms / 1000;
}

uint32_t AP_Stats::get_boot_flying_time_s(void)
{
    return _boot_flying_time_ms / 1000;
}

uint32_t AP_Stats::get_total_flying_time_s(void)
{
    return _total_boot_flying_time_s + get_boot_flying_time_s();
}

uint32_t AP_Stats::get_boot_run_time_s(void)
{
    uint32_t now = AP_HAL::millis();
    return (now - _boot_tstamp_ms) / 1000;
}

uint32_t AP_Stats::get_total_run_time_s(void)
{
    return _total_boot_run_time_s + get_boot_run_time_s();
}

float AP_Stats::get_total_flying_ground_traveled_m(void)
{
    return _total_boot_flying_ground_traveled_m + get_boot_flying_ground_traveled_m();
}

float AP_Stats::get_total_flying_air_traveled_m(void)
{
    return _total_boot_flying_air_traveled_m + get_boot_flying_air_traveled_m();
}

float AP_Stats::get_total_flying_energy_wh(void)
{
    return _total_boot_flying_energy_wh + get_boot_flying_energy_wh();
}

float AP_Stats::get_boot_avg_ground_speed_mps(void)
{
    const auto flight_time_s = get_boot_flying_time_s();
    if (!flight_time_s) return 0;
    return _boot_flying_ground_traveled_m / flight_time_s;
}

float AP_Stats::get_total_avg_ground_speed_mps(void)
{
    return calc_total_flying_time_related_avg(_total_boot_avg_ground_speed_mps, get_boot_avg_ground_speed_mps());
}

float AP_Stats::get_total_max_ground_speed_mps(void)
{
    return MAX(_total_boot_max_ground_speed_mps, get_boot_max_ground_speed_mps());
}

float AP_Stats::get_boot_avg_air_speed_mps(void)
{
    const auto flight_time_s = get_boot_flying_time_s();
    if (!flight_time_s) return 0;
    return _boot_flying_air_traveled_m / flight_time_s;
}

float AP_Stats::get_total_avg_air_speed_mps(void)
{
    return calc_total_flying_time_related_avg(_total_boot_avg_air_speed_mps, get_boot_avg_air_speed_mps());
}

float AP_Stats::get_total_max_air_speed_mps(void)
{
    return MAX(_total_boot_max_air_speed_mps, get_boot_max_air_speed_mps());
}

float AP_Stats::get_total_avg_wind_speed_mps(void)
{
    return calc_total_flying_time_related_avg(_total_boot_avg_wind_speed_mps, get_boot_avg_wind_speed_mps());
}

float AP_Stats::get_total_max_wind_speed_mps(void)
{
    return MAX(_total_boot_max_wind_speed_mps, get_boot_max_wind_speed_mps());
}

uint32_t AP_Stats::get_total_avg_home_distance_m(void)
{
    return calc_total_flying_time_related_avg(_total_boot_avg_home_distance_m, get_boot_avg_home_distance_m());
}

uint32_t AP_Stats::get_total_max_home_distance_m(void)
{
    return MAX(_total_boot_max_home_distance_m, get_boot_max_home_distance_m());
}

uint32_t AP_Stats::get_total_max_relative_altitude_m(void)
{
    return MAX(_total_boot_max_relative_altitude_m, get_boot_max_relative_altitude_m());
}

float AP_Stats::get_total_avg_flying_current_a(void)
{
    return calc_total_flying_time_related_avg(_total_boot_avg_flying_current_a, get_boot_avg_flying_current_a());
}

float AP_Stats::get_total_max_flying_current_a(void)
{
    return MAX(_total_boot_max_flying_current_a, get_boot_max_flying_current_a());
}

float AP_Stats::get_total_avg_flying_power_w(void)
{
    return calc_total_flying_time_related_avg(_total_boot_avg_flying_power_w, get_boot_avg_flying_power_w());
}

float AP_Stats::get_total_max_flying_power_w(void)
{
    return MAX(_total_boot_max_flying_power_w, get_boot_max_flying_power_w());
}

uint32_t AP_Stats::get_total_flight_count(void)
{
    return _total_boot_flight_count + _boot_flight_count;
}

AP_Stats *AP::stats(void)
{
    return AP_Stats::get_singleton();
}
