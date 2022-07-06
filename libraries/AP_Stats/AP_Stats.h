#pragma once

// AP_Stats is used to collect and put to permanent storage data about
// the vehicle's autopilot

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class AP_Stats
{
public:
    // constructor
    AP_Stats();
    
    void init();

    // copy state into underlying parameters:
    void flush();

    // periodic update function (e.g. put our values to permanent storage):
    // call at least 1Hz
    void update();

    void set_flying(bool b);

    HAL_Semaphore &get_semaphore(void) { return _sem; }

    uint32_t get_current_flight_time_s(void);
    uint32_t get_boot_flying_time_s(void);
    uint32_t get_total_flying_time_s(void);
    uint32_t get_boot_run_time_s(void);
    uint32_t get_total_run_time_s(void);
    float    get_total_flying_ground_traveled_m(void);
    float    get_total_flying_air_traveled_m(void);
    float    get_total_flying_energy_wh(void);
    float    get_boot_avg_ground_speed_mps(void);
    float    get_total_avg_ground_speed_mps(void);
    float    get_total_max_ground_speed_mps(void);
    float    get_boot_avg_air_speed_mps(void);
    float    get_total_avg_air_speed_mps(void);
    float    get_total_max_air_speed_mps(void);
    float    get_total_avg_wind_speed_mps(void);
    float    get_total_max_wind_speed_mps(void);
    uint32_t get_total_avg_home_distance_m(void);
    uint32_t get_total_max_home_distance_m(void);
    uint32_t get_total_max_relative_altitude_m(void);
    float    get_total_avg_flying_current_a(void);
    float    get_total_max_flying_current_a(void);
    float    get_total_avg_flying_power_w(void);
    float    get_total_max_flying_power_w(void);
    uint32_t get_total_flight_count(void);

    float get_boot_flying_ground_traveled_m(void) { return _boot_flying_ground_traveled_m; }
    float get_boot_flying_air_traveled_m(void) { return _boot_flying_air_traveled_m; }
    float get_boot_flying_energy_wh(void) { return _boot_flying_energy_wh; }
    uint32_t get_boot_flying_mah(void) { return lrintf(_boot_flying_mah); }
    float get_boot_max_ground_speed_mps(void) { return _boot_max_ground_speed_mps; }
    float get_boot_max_air_speed_mps(void) { return _boot_max_air_speed_mps; }
    float get_boot_avg_wind_speed_mps(void) { return _boot_avg_wind_speed_mps; }
    float get_boot_max_wind_speed_mps(void) { return _boot_max_wind_speed_mps; }
    uint32_t get_boot_avg_home_distance_m(void) { return _boot_avg_home_distance_m; }
    uint32_t get_boot_max_home_distance_m(void) { return _boot_max_home_distance_m; }
    uint32_t get_boot_max_relative_altitude_m(void) { return _boot_max_relative_altitude_m; }
    float get_boot_min_rc_rssi(void) { return _boot_min_rc_rssi; }
    uint8_t get_boot_min_rc_rssi_dbm(void) { return _boot_min_rc_rssi_dbm; }
    uint16_t get_boot_max_rc_tx_power_mw(void) { return _boot_max_rc_tx_power_mw; }
    float get_boot_avg_flying_current_a(void) { return _boot_avg_flying_current_a; }
    float get_boot_max_flying_current_a(void) { return _boot_max_flying_current_a; }
    float get_boot_avg_flying_power_w(void) { return _boot_avg_flying_power_w; }
    float get_boot_max_flying_power_w(void) { return _boot_max_flying_power_w; }
    float get_boot_min_voltage_v(void) { return _boot_min_voltage_v; }
    float get_boot_min_cell_voltage_v(void) { return _boot_min_cell_voltage_v; }
    int16_t get_boot_avg_esc_temperature_degc(void) { return _boot_avg_esc_temperature_degc; }
    int16_t get_boot_max_esc_temperature_degc(void) { return _boot_max_esc_temperature_degc; }
    uint32_t get_boot_flight_count(void) { return _boot_flight_count; }


    bool available(void) { return _flying_sample_count > 0; }
    bool energy_is_available(void) const { return _energy_is_available; }
    bool mah_is_available(void) const { return _mah_is_available; }
    bool cell_voltage_is_available(void) const  { return _cell_voltage_is_available; }
    bool air_speeds_are_available(void) const  { return _air_speeds_are_available; }
    bool esc_temperatures_are_available(void) const  { return _esc_temperatures_are_available; }
    bool flying_time_s_is_available(void) { return get_boot_flying_time_s() > 0; }
    bool avg_ground_speed_is_available(void) { return flying_time_s_is_available(); }
    bool avg_air_speed_is_available(void) { return flying_time_s_is_available(); }
    bool wind_speeds_are_available(void) { return _flying_for_some_time_sample_count > 0; }
    bool min_rc_rssi_dbm_is_available(void) { return _boot_min_rc_rssi_dbm >= 0; }
    bool max_rc_tx_power_is_available(void) { return _boot_max_rc_tx_power_mw > 0; }

    // get singleton
    static AP_Stats *get_singleton(void) {
        return _singleton;
    }
    
    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_Stats *_singleton;
    
    struct {
        AP_Int16 boot_count;
        AP_Int32 flying_time;
        AP_Int32 flying_ground_traveled;
        AP_Int32 flying_air_traveled;
        AP_Float flying_energy;
        AP_Int32 flight_count;
        AP_Int32 flight_time_max;

        AP_Float avg_ground_speed_mps;
        AP_Float max_ground_speed_mps;
        AP_Float avg_air_speed_mps;
        AP_Float max_air_speed_mps;
        AP_Float avg_wind_speed_mps;
        AP_Float max_wind_speed_mps;
        AP_Int32 avg_home_distance_m;
        AP_Int32 max_home_distance_m;
        AP_Int32 max_relative_altitude_m;
        AP_Float avg_flying_current_a;
        AP_Float max_flying_current_a;
        AP_Float avg_flying_power_w;
        AP_Float max_flying_power_w;

        AP_Int32 run_time;
        AP_Int32 reset;
        AP_Int8  load;
    } params;

    void copy_variables_from_parameters();

    uint32_t _flush_tstamp_ms = 0;
    static constexpr uint16_t flush_interval_ms = 30000;

    uint32_t _boot_tstamp_ms = 0;
    uint32_t _flying_start_tstamp_ms = 0;
    uint32_t _last_slow_update_tstamp_ms = 0;
    uint32_t _last_update_flying_tstamp_ms = 0;
    uint32_t _last_slow_update_flying_tstamp_ms = 0;

    uint32_t _flying_sample_count = 0;
    uint32_t _flying_slow_update_sample_count = 0;
    uint32_t _flying_for_some_time_sample_count = 0;

    float    _prev_update_energy_wh = 0;
    float    _prev_update_mah = 0;

    bool     _energy_is_available = false;
    bool     _mah_is_available = false;
    bool     _current_is_available = false;
    bool     _power_is_available = false;
    bool     _cell_voltage_is_available = false;
    bool     _wind_speeds_are_available = false;
    bool     _air_speeds_are_available = false;
    bool     _esc_temperatures_are_available = false;

    uint32_t _current_flight_time_ms = 0;

    uint32_t _boot_flying_time_ms = 0;
    float    _boot_flying_ground_traveled_m = 0;
    float    _boot_flying_air_traveled_m = 0;
    float    _boot_flying_energy_wh = 0;
    float    _boot_flying_mah = 0;
    float    _boot_max_ground_speed_mps = 0;
    float    _boot_max_air_speed_mps = 0;
    float    _boot_avg_wind_speed_mps = 0;
    float    _boot_max_wind_speed_mps = 0;
    float    _boot_avg_home_distance_m = 0;
    uint32_t _boot_max_home_distance_m = 0;
    uint32_t _boot_max_relative_altitude_m = 0;
    float    _boot_min_rc_rssi = FLT_MAX;
    int8_t   _boot_min_rc_rssi_dbm = -1;
    int16_t  _boot_max_rc_tx_power_mw = 0;
    float    _boot_max_flying_current_a = 0;
    float    _boot_avg_flying_current_a = 0;
    float    _boot_max_flying_power_w = 0;
    float    _boot_avg_flying_power_w = 0;
    float    _boot_min_voltage_v = FLT_MAX;
    float    _boot_min_cell_voltage_v = FLT_MAX;
    int16_t  _boot_avg_esc_temperature_degc = 0;
    int16_t  _boot_max_esc_temperature_degc = INT16_MIN;
    uint32_t _boot_flight_count = 0;

    uint32_t _total_boot_flying_time_s = 0;             // seconds spent flying
    uint32_t _total_boot_flying_ground_traveled_m = 0;  // ground distance in meter traveled while flying
    uint32_t _total_boot_flying_air_traveled_m = 0;     // air distance in meter traveled while flying
    float    _total_boot_flying_energy_wh = 0;          // consumed energy in Wh while flying
    float    _total_boot_avg_ground_speed_mps = 0;
    float    _total_boot_max_ground_speed_mps = 0;
    float    _total_boot_avg_air_speed_mps = 0;
    float    _total_boot_max_air_speed_mps = 0;
    float    _total_boot_avg_wind_speed_mps = 0;
    float    _total_boot_max_wind_speed_mps = 0;
    uint32_t _total_boot_avg_home_distance_m = 0;
    uint32_t _total_boot_max_home_distance_m = 0;
    uint32_t _total_boot_max_relative_altitude_m = 0;
    float    _total_boot_avg_flying_current_a = 0;
    float    _total_boot_max_flying_current_a = 0;
    float    _total_boot_avg_flying_power_w = 0;
    float    _total_boot_max_flying_power_w = 0;
    uint32_t _total_boot_flight_count = 0;
    uint32_t _total_boot_run_time_s = 0;                // total stored wallclock time spent running ArduPilot (seconds) when booted up
    uint32_t _reset_tstamp_s = 0;                       // last time AP_Stats parameters were reset (in seconds since AP_Stats Jan 1st 2016)

    bool     _prev_load = false;

    HAL_Semaphore _sem;

    bool is_flying(void) const { return _flying_start_tstamp_ms != 0; }

    void reset_params_if_requested(void);
    void update_flying_time(uint32_t flying_update_delta);
    void update_flying_travel(uint32_t flying_update_delta, uint32_t old_flying_sample_count, uint32_t new_flying_sample_count);
    void update_flying_distances_and_speeds(uint32_t flying_update_delta, uint32_t old_flying_sample_count, uint32_t new_flying_sample_count);
    void update_flying_current_and_power(uint32_t old_flying_sample_count, uint32_t new_flying_sample_count);
    void update_battery(void);
    void update_flying_rc(uint32_t old_flying_sample_count, uint32_t new_flying_sample_count);
    void update_flying_esc(uint32_t old_flying_sample_count, uint32_t new_flying_sample_count);
    bool has_been_flying_for_at_least_s(uint32_t time_s);
    float calc_total_flying_time_related_avg(float total_boot_value, float boot_value);
};

namespace AP {
    AP_Stats *stats();
};
