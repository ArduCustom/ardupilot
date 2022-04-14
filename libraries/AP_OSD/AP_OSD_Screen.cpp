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
/*
  parameter settings for one screen
 */
#include "AP_OSD.h"
#include "AP_OSD_Backend.h"

#if OSD_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_RSSI/AP_RSSI.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Stats/AP_Stats.h>
#include <AP_Common/Location.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_MSP/msp.h>
#include <AP_OLC/AP_OLC.h>
#include <AP_VideoTX/AP_VideoTX.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>
#if APM_BUILD_TYPE(APM_BUILD_Rover)
#include <AP_WindVane/AP_WindVane.h>
#endif
#include <AP_Filesystem/AP_Filesystem.h>

#include <ctype.h>
#include <GCS_MAVLink/GCS.h>
#include <AC_Fence/AC_Fence.h>

const AP_Param::GroupInfo AP_OSD_Screen::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable screen
    // @Description: Enable this screen
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_OSD_Screen, enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: CHAN_MIN
    // @DisplayName: Transmitter switch screen minimum pwm
    // @Description: This sets the PWM lower limit for this screen
    // @Range: 900 2100
    // @User: Standard
    AP_GROUPINFO("CHAN_MIN", 2, AP_OSD_Screen, channel_min, 900),

    // @Param: CHAN_MAX
    // @DisplayName: Transmitter switch screen maximum pwm
    // @Description: This sets the PWM upper limit for this screen
    // @Range: 900 2100
    // @User: Standard
    AP_GROUPINFO("CHAN_MAX", 3, AP_OSD_Screen, channel_max, 2100),

    // @Param: ALTITUDE_EN
    // @DisplayName: ALTITUDE_EN
    // @Description: Enables display of altitude AGL
    // @Values: 0:Disabled,1:Enabled

    // @Param: ALTITUDE_X
    // @DisplayName: ALTITUDE_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ALTITUDE_Y
    // @DisplayName: ALTITUDE_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(altitude, "ALTITUDE", 4, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: BAT_VOLT_EN
    // @DisplayName: BATVOLT_EN
    // @Description: Displays main battery voltage
    // @Values: 0:Disabled,1:Enabled

    // @Param: BAT_VOLT_X
    // @DisplayName: BATVOLT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BAT_VOLT_Y
    // @DisplayName: BATVOLT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(bat_volt, "BAT_VOLT", 5, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: RSSI_EN
    // @DisplayName: RSSI_EN
    // @Description: Displays RC signal strength
    // @Values: 0:Disabled,1:Enabled

    // @Param: RSSI_X
    // @DisplayName: RSSI_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: RSSI_Y
    // @DisplayName: RSSI_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(rssi, "RSSI", 6, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: CURRENT_EN
    // @DisplayName: CURRENT_EN
    // @Description: Displays main battery current
    // @Values: 0:Disabled,1:Enabled

    // @Param: CURRENT_X
    // @DisplayName: CURRENT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CURRENT_Y
    // @DisplayName: CURRENT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(current, "CURRENT", 7, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: BATUSED_EN
    // @DisplayName: BATUSED_EN
    // @Description: Displays primary battery mAh consumed
    // @Values: 0:Disabled,1:Enabled

    // @Param: BATUSED_X
    // @DisplayName: BATUSED_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BATUSED_Y
    // @DisplayName: BATUSED_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(batused, "BATUSED", 8, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: SATS_EN
    // @DisplayName: SATS_EN
    // @Description: Displays number of acquired satellites
    // @Values: 0:Disabled,1:Enabled

    // @Param: SATS_X
    // @DisplayName: SATS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: SATS_Y
    // @DisplayName: SATS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(sats, "SATS", 9, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: FLTMODE_EN
    // @DisplayName: FLTMODE_EN
    // @Description: Displays flight mode
    // @Values: 0:Disabled,1:Enabled

    // @Param: FLTMODE_X
    // @DisplayName: FLTMODE_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: FLTMODE_Y
    // @DisplayName: FLTMODE_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(fltmode, "FLTMODE", 10, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: MESSAGE_EN
    // @DisplayName: MESSAGE_EN
    // @Description: Displays Mavlink messages
    // @Values: 0:Disabled,1:Enabled

    // @Param: MESSAGE_X
    // @DisplayName: MESSAGE_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: MESSAGE_Y
    // @DisplayName: MESSAGE_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(message, "MESSAGE", 11, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: GSPEED_EN
    // @DisplayName: GSPEED_EN
    // @Description: Displays GPS ground speed
    // @Values: 0:Disabled,1:Enabled

    // @Param: GSPEED_X
    // @DisplayName: GSPEED_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: GSPEED_Y
    // @DisplayName: GSPEED_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(gspeed, "GSPEED", 12, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: HORIZON_EN
    // @DisplayName: HORIZON_EN
    // @Description: Displays artificial horizon
    // @Values: 0:Disabled,1:Enabled

    // @Param: HORIZON_X
    // @DisplayName: HORIZON_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: HORIZON_Y
    // @DisplayName: HORIZON_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(horizon, "HORIZON", 13, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: HOME_EN
    // @DisplayName: HOME_EN
    // @Description: Displays distance and relative direction to HOME
    // @Values: 0:Disabled,1:Enabled

    // @Param: HOME_X
    // @DisplayName: HOME_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: HOME_Y
    // @DisplayName: HOME_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(home, "HOME", 14, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: HEADING_EN
    // @DisplayName: HEADING_EN
    // @Description: Displays heading
    // @Values: 0:Disabled,1:Enabled

    // @Param: HEADING_X
    // @DisplayName: HEADING_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: HEADING_Y
    // @DisplayName: HEADING_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(heading, "HEADING", 15, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: THR_OUT_EN
    // @DisplayName: THR_OUT_EN
    // @Description: Displays actual throttle percentage being sent to motor(s)
    // @Values: 0:Disabled,1:Enabled

    // @Param: THR_OUT_X
    // @DisplayName: THR_OUT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: THR_OUT_Y
    // @DisplayName: THR_OUT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(throttle_output, "THR_OUT", 16, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: COMPASS_EN
    // @DisplayName: COMPASS_EN
    // @Description: Enables display of compass rose
    // @Values: 0:Disabled,1:Enabled

    // @Param: COMPASS_X
    // @DisplayName: COMPASS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: COMPASS_Y
    // @DisplayName: COMPASS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(compass, "COMPASS", 17, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: WIND_EN
    // @DisplayName: WIND_EN
    // @Description: Displays wind speed and relative direction, on Rover this is the apparent wind speed and direction from the windvane, if fitted
    // @Values: 0:Disabled,1:Enabled

    // @Param: WIND_X
    // @DisplayName: WIND_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: WIND_Y
    // @DisplayName: WIND_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(wind, "WIND", 18, AP_OSD_Screen, AP_OSD_Setting),


#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // @Param: ASPEED_EN
    // @DisplayName: ASPEED_EN
    // @Description: Displays airspeed value being used by TECS (fused value)
    // @Values: 0:Disabled,1:Enabled

    // @Param: ASPEED_X
    // @DisplayName: ASPEED_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ASPEED_Y
    // @DisplayName: ASPEED_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(aspeed, "ASPEED", 19, AP_OSD_Screen, AP_OSD_Setting),
#endif

    // @Param: VSPEED_EN
    // @DisplayName: VSPEED_EN
    // @Description: Displays climb rate
    // @Values: 0:Disabled,1:Enabled

    // @Param: VSPEED_X
    // @DisplayName: VSPEED_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: VSPEED_Y
    // @DisplayName: VSPEED_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(vspeed, "VSPEED", 20, AP_OSD_Screen, AP_OSD_Setting),

#if HAL_WITH_ESC_TELEM
    // @Param: ESCTEMP_EN
    // @DisplayName: ESCTEMP_EN
    // @Description: Displays the temperature of the ESC with highest temperature
    // @Values: 0:Disabled,1:Enabled

    // @Param: ESCTEMP_X
    // @DisplayName: ESCTEMP_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ESCTEMP_Y
    // @DisplayName: ESCTEMP_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(highest_esc_temp, "ESCTEMP", 21, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ESCARPM_EN
    // @DisplayName: ESCARPM_EN
    // @Description: Displays the average motor RPM as reported by the ESCs
    // @Values: 0:Disabled,1:Enabled

    // @Param: ESCARPM_X
    // @DisplayName: ESCARPM_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ESCARPM_Y
    // @DisplayName: ESCARPM_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(avg_esc_rpm, "ESCARPM", 22, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ESCHAMPS_EN
    // @DisplayName: ESCHAMPS_EN
    // @Description: Display the current going through the ESC with highest measured current
    // @Values: 0:Disabled,1:Enabled

    // @Param: ESCHAMPS_X
    // @DisplayName: ESCHAMPS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ESCHAMPS_Y
    // @DisplayName: ESCHAMPS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(highest_esc_amps, "ESCHAMPS", 23, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ESCAAMPS_EN
    // @DisplayName: ESCAAMPS_EN
    // @Description: Display the average current going through the ESCs
    // @Values: 0:Disabled,1:Enabled

    // @Param: ESCAAMPS_X
    // @DisplayName: ESCAAMPS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ESCAAMPS_Y
    // @DisplayName: ESCAAMPS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(avg_esc_amps, "ESCAAMPS", 61, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ESCTAMPS_EN
    // @DisplayName: ESCTAMPS_EN
    // @Description: Display the total current going through the ESCs
    // @Values: 0:Disabled,1:Enabled

    // @Param: ESCTAMPS_X
    // @DisplayName: ESCTAMPS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ESCTAMPS_Y
    // @DisplayName: ESCTAMPS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(total_esc_amps, "ESCTAMPS", 62, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ESCHRPM_EN
    // @DisplayName: ESCHRPM_EN
    // @Description: Displays the highest motor RPM as reported by the ESCs
    // @Values: 0:Disabled,1:Enabled

    // @Param: ESCHRPM_X
    // @DisplayName: ESCHRPM_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ESCHRPM_Y
    // @DisplayName: ESCHRPM_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(highest_esc_rpm, "ESCHRPM", 63, AP_OSD_Screen, AP_OSD_Setting),
#endif
    // @Param: GPSLAT_EN
    // @DisplayName: GPSLAT_EN
    // @Description: Displays GPS latitude
    // @Values: 0:Disabled,1:Enabled

    // @Param: GPSLAT_X
    // @DisplayName: GPSLAT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: GPSLAT_Y
    // @DisplayName: GPSLAT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(gps_latitude, "GPSLAT", 24, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: GPSLONG_EN
    // @DisplayName: GPSLONG_EN
    // @Description: Displays GPS longitude
    // @Values: 0:Disabled,1:Enabled

    // @Param: GPSLONG_X
    // @DisplayName: GPSLONG_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: GPSLONG_Y
    // @DisplayName: GPSLONG_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(gps_longitude, "GPSLONG", 25, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ROLL_EN
    // @DisplayName: ROLL_EN
    // @Description: Displays degrees of roll from level
    // @Values: 0:Disabled,1:Enabled

    // @Param: ROLL_X
    // @DisplayName: ROLL_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ROLL_Y
    // @DisplayName: ROLL_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(roll_angle, "ROLL", 26, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: PITCH_EN
    // @DisplayName: PITCH_EN
    // @Description: Displays degrees of pitch from level
    // @Values: 0:Disabled,1:Enabled

    // @Param: PITCH_X
    // @DisplayName: PITCH_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: PITCH_Y
    // @DisplayName: PITCH_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(pitch_angle, "PITCH", 27, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: TEMP_EN
    // @DisplayName: TEMP_EN
    // @Description: Displays temperature reported by primary barometer
    // @Values: 0:Disabled,1:Enabled

    // @Param: TEMP_X
    // @DisplayName: TEMP_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: TEMP_Y
    // @DisplayName: TEMP_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(temp, "TEMP", 28, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: HDOP_EN
    // @DisplayName: HDOP_EN
    // @Description: Displays Horizontal Dilution Of Position
    // @Values: 0:Disabled,1:Enabled

    // @Param: HDOP_X
    // @DisplayName: HDOP_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: HDOP_Y
    // @DisplayName: HDOP_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(hdop, "HDOP", 29, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: WAYPOINT_EN
    // @DisplayName: WAYPOINT_EN
    // @Description: Displays bearing and distance to next waypoint
    // @Values: 0:Disabled,1:Enabled

    // @Param: WAYPOINT_X
    // @DisplayName: WAYPOINT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: WAYPOINT_Y
    // @DisplayName: WAYPOINT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(waypoint, "WAYPOINT", 30, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: XTRACK_EN
    // @DisplayName: XTRACK_EN
    // @Description: Displays crosstrack error
    // @Values: 0:Disabled,1:Enabled

    // @Param: XTRACK_X
    // @DisplayName: XTRACK_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: XTRACK_Y
    // @DisplayName: XTRACK_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(xtrack_error, "XTRACK", 31, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: GNDTRVL_EN
    // @DisplayName: GNDTRVL_EN
    // @Description: Displays ground distance traveled
    // @Values: 0:Disabled,1:Enabled

    // @Param: GNDTRVL_X
    // @DisplayName: GNDTRVL_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: GNDTRVL_Y
    // @DisplayName: GNDTRVL_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(traveled_ground_distance, "GNDTRVL", 32, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: STATS_EN
    // @DisplayName: STATS_EN
    // @Description: Displays flight stats
    // @Values: 0:Disabled,1:Enabled

    // @Param: STATS_X
    // @DisplayName: STATS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: STATS_Y
    // @DisplayName: STATS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(stats, "STATS", 33, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: FLTIME_EN
    // @DisplayName: FLTIME_EN
    // @Description: Displays total flight time
    // @Values: 0:Disabled,1:Enabled

    // @Param: FLTIME_X
    // @DisplayName: FLTIME_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: FLTIME_Y
    // @DisplayName: FLTIME_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(flightime, "FLTIME", 34, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: CLIMBEFF_EN
    // @DisplayName: CLIMBEFF_EN
    // @Description: Displays climb efficiency (climb rate/current)
    // @Values: 0:Disabled,1:Enabled

    // @Param: CLIMBEFF_X
    // @DisplayName: CLIMBEFF_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CLIMBEFF_Y
    // @DisplayName: CLIMBEFF_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(climbeff, "CLIMBEFF", 35, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: EFFG_EN
    // @DisplayName: EFFG_EN
    // @Description: Displays ground flight efficiency (mAh or Wh / km or mi)
    // @Values: 0:Disabled,1:Enabled

    // @Param: EFFG_X
    // @DisplayName: EFFG_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: EFFG_Y
    // @DisplayName: EFFG_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(eff_ground, "EFFG", 36, AP_OSD_Screen, AP_OSD_Setting),

#if BARO_MAX_INSTANCES > 1
    // @Param: BTEMP_EN
    // @DisplayName: BTEMP_EN
    // @Description: Displays temperature reported by secondary barometer
    // @Values: 0:Disabled,1:Enabled

    // @Param: BTEMP_X
    // @DisplayName: BTEMP_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BTEMP_Y
    // @DisplayName: BTEMP_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(btemp, "BTEMP", 37, AP_OSD_Screen, AP_OSD_Setting),
#endif

    // @Param: ATEMP_EN
    // @DisplayName: ATEMP_EN
    // @Description: Displays temperature reported by primary airspeed sensor
    // @Values: 0:Disabled,1:Enabled

    // @Param: ATEMP_X
    // @DisplayName: ATEMP_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ATEMP_Y
    // @DisplayName: ATEMP_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(atemp, "ATEMP", 38, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: BAT2_VLT_EN
    // @DisplayName: BAT2VLT_EN
    // @Description: Displays battery2 voltage
    // @Values: 0:Disabled,1:Enabled

    // @Param: BAT2_VLT_X
    // @DisplayName: BAT2VLT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BAT2_VLT_Y
    // @DisplayName: BAT2VLT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(bat2_vlt, "BAT2_VLT", 39, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: BAT2USED_EN
    // @DisplayName: BAT2USED_EN
    // @Description: Displays secondary battery mAh consumed
    // @Values: 0:Disabled,1:Enabled

    // @Param: BAT2USED_X
    // @DisplayName: BAT2USED_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BAT2USED_Y
    // @DisplayName: BAT2USED_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(bat2used, "BAT2USED", 40, AP_OSD_Screen, AP_OSD_Setting),

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // @Param: ASPD2_EN
    // @DisplayName: ASPD2_EN
    // @Description: Displays airspeed reported directly from secondary airspeed sensor
    // @Values: 0:Disabled,1:Enabled

    // @Param: ASPD2_X
    // @DisplayName: ASPD2_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ASPD2_Y
    // @DisplayName: ASPD2_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(aspd2, "ASPD2", 41, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ASPD1_EN
    // @DisplayName: ASPD1_EN
    // @Description: Displays airspeed reported directly from primary airspeed sensor
    // @Values: 0:Disabled,1:Enabled

    // @Param: ASPD1_X
    // @DisplayName: ASPD1_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ASPD1_Y
    // @DisplayName: ASPD1_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(aspd1, "ASPD1", 42, AP_OSD_Screen, AP_OSD_Setting),
#endif

    // @Param: CLK_EN
    // @DisplayName: CLK_EN
    // @Description: Displays a clock panel based on AP_RTC local time
    // @Values: 0:Disabled,1:Enabled

    // @Param: CLK_X
    // @DisplayName: CLK_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CLK_Y
    // @DisplayName: CLK_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(clk, "CLK", 43, AP_OSD_Screen, AP_OSD_Setting),

#if HAL_OSD_SIDEBAR_ENABLE || HAL_MSP_ENABLED
    // @Param: SIDEBARS_EN
    // @DisplayName: SIDEBARS_EN
    // @Description: Displays artificial horizon side bars
    // @Values: 0:Disabled,1:Enabled

    // @Param: SIDEBARS_X
    // @DisplayName: SIDEBARS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: SIDEBARS_Y
    // @DisplayName: SIDEBARS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(sidebars, "SIDEBARS", 44, AP_OSD_Screen, AP_OSD_Setting),
#endif

#if HAL_MSP_ENABLED
    // @Param: CRSSHAIR_EN
    // @DisplayName: CRSSHAIR_EN
    // @Description: Displays artificial horizon crosshair (MSP OSD only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: CRSSHAIR_X
    // @DisplayName: CRSSHAIR_X
    // @Description: Horizontal position on screen (MSP OSD only)
    // @Range: 0 29

    // @Param: CRSSHAIR_Y
    // @DisplayName: CRSSHAIR_Y
    // @Description: Vertical position on screen (MSP OSD only)
    // @Range: 0 15
    AP_SUBGROUPINFO(crosshair, "CRSSHAIR", 45, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: HOMEDIST_EN
    // @DisplayName: HOMEDIST_EN
    // @Description: Displays distance from HOME (MSP OSD only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: HOMEDIST_X
    // @DisplayName: HOMEDIST_X
    // @Description: Horizontal position on screen (MSP OSD only)
    // @Range: 0 29

    // @Param: HOMEDIST_Y
    // @DisplayName: HOMEDIST_Y
    // @Description: Vertical position on screen (MSP OSD only)
    // @Range: 0 15
    AP_SUBGROUPINFO(home_dist, "HOMEDIST", 46, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: HOMEDIR_EN
    // @DisplayName: HOMEDIR_EN
    // @Description: Displays relative direction to HOME (MSP OSD only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: HOMEDIR_X
    // @DisplayName: HOMEDIR_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: HOMEDIR_Y
    // @DisplayName: HOMEDIR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(home_dir, "HOMEDIR", 47, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: CELLVOLT_EN
    // @DisplayName: CELL_VOLT_EN
    // @Description: Displays average cell voltage (MSP OSD only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: CELLVOLT_X
    // @DisplayName: CELL_VOLT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CELLVOLT_Y
    // @DisplayName: CELL_VOLT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(cell_volt, "CELLVOLT", 49, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: BATTBAR_EN
    // @DisplayName: BATT_BAR_EN
    // @Description: Displays battery usage bar (MSP OSD only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: BATTBAR_X
    // @DisplayName: BATT_BAR_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BATTBAR_Y
    // @DisplayName: BATT_BAR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(batt_bar, "BATTBAR", 50, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ARMING_EN
    // @DisplayName: ARMING_EN
    // @Description: Displays arming status (MSP OSD only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: ARMING_X
    // @DisplayName: ARMING_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ARMING_Y
    // @DisplayName: ARMING_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(arming, "ARMING", 51, AP_OSD_Screen, AP_OSD_Setting),
#endif //HAL_MSP_ENABLED

    // @Param: POWER_EN
    // @DisplayName: POWER_EN
    // @Description: Displays total power draw from all batteries
    // @Values: 0:Disabled,1:Enabled

    // @Param: POWER_X
    // @DisplayName: POWER_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: POWER_Y
    // @DisplayName: POWER_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(power, "POWER", 48, AP_OSD_Screen, AP_OSD_Setting),

#if HAL_PLUSCODE_ENABLE
    // @Param: PLUSCODE_EN
    // @DisplayName: PLUSCODE_EN
    // @Description: Displays pluscode (OLC) element
    // @Values: 0:Disabled,1:Enabled

    // @Param: PLUSCODE_X
    // @DisplayName: PLUSCODE_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: PLUSCODE_Y
    // @DisplayName: PLUSCODE_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(pluscode, "PLUSCODE", 52, AP_OSD_Screen, AP_OSD_Setting),
#endif

#if HAVE_FILESYSTEM_SUPPORT
    // @Param: CALLSIGN_EN
    // @DisplayName: CALLSIGN_EN
    // @Description: Displays callsign from callsign.txt on microSD card
    // @Values: 0:Disabled,1:Enabled

    // @Param: CALLSIGN_X
    // @DisplayName: CALLSIGN_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CALLSIGN_Y
    // @DisplayName: CALLSIGN_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(callsign, "CALLSIGN", 53, AP_OSD_Screen, AP_OSD_Setting),
#endif

    // @Param: CURRENT2_EN
    // @DisplayName: CURRENT2_EN
    // @Description: Displays 2nd battery current
    // @Values: 0:Disabled,1:Enabled

    // @Param: CURRENT2_X
    // @DisplayName: CURRENT2_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CURRENT2_Y
    // @DisplayName: CURRENT2_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(current2, "CURRENT2", 54, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: VTX_PWR_EN
    // @DisplayName: VTX_PWR_EN
    // @Description: Displays VTX Power
    // @Values: 0:Disabled,1:Enabled

    // @Param: VTX_PWR_X
    // @DisplayName: VTX_PWR_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: VTX_PWR_Y
    // @DisplayName: VTX_PWR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(vtx_power, "VTX_PWR", 55, AP_OSD_Screen, AP_OSD_Setting),

#if AP_TERRAIN_AVAILABLE
    // @Param: TER_HGT_EN
    // @DisplayName: TER_HGT_EN
    // @Description: Displays Height above terrain
    // @Values: 0:Disabled,1:Enabled

    // @Param: TER_HGT_X
    // @DisplayName: TER_HGT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: TER_HGT_Y
    // @DisplayName: TER_HGT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(hgt_abvterr, "TER_HGT", 56, AP_OSD_Screen, AP_OSD_Setting),
#endif

    // @Param: AVGCELLV_EN
    // @DisplayName: AVGCELLV_EN
    // @Description: Displays average cell voltage. WARNING: this can be inaccurate if the cell count is not detected properly. If the cell count detection voltage is not right or the battery is far from fully charged the detected cell count might not be accurate. Use BATT_CELL_COUNT to force the number of cells.
    // @Values: 0:Disabled,1:Enabled

    // @Param: AVGCELLV_X
    // @DisplayName: AVGCELLV_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: AVGCELLV_Y
    // @DisplayName: AVGCELLV_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(avgcellvolt, "AVGCELLV", 57, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: RESTVOLT_EN
    // @DisplayName: RESTVOLT_EN
    // @Description: Displays main battery resting voltage
    // @Values: 0:Disabled,1:Enabled

    // @Param: RESTVOLT_X
    // @DisplayName: RESTVOLT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: RESTVOLT_Y
    // @DisplayName: RESTVOLT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(restvolt, "RESTVOLT", 58, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: FENCE_EN
    // @DisplayName: FENCE_EN
    // @Description: Displays indication of fence enable and breach
    // @Values: 0:Disabled,1:Enabled

    // @Param: FENCE_X
    // @DisplayName: FENCE_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: FENCE_Y
    // @DisplayName: FENCE_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(fence, "FENCE", 59, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: RNGF_EN
    // @DisplayName: RNGF_EN
    // @Description: Displays a rangefinder's distance in cm
    // @Values: 0:Disabled,1:Enabled

    // @Param: RNGF_X
    // @DisplayName: RNGF_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: RNGF_Y
    // @DisplayName: RNGF_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(rngf, "RNGF", 60, AP_OSD_Screen, AP_OSD_Setting),

    AP_GROUPEND
};

const AP_Param::GroupInfo AP_OSD_Screen::var_info2[] = {
    // duplicate of OSDn_ENABLE to ensure params are hidden when not enabled
    AP_GROUPINFO_FLAGS("ENABLE", 2, AP_OSD_Screen, enabled, 0, AP_PARAM_FLAG_ENABLE | AP_PARAM_FLAG_HIDDEN),

    // @Param: LINK_Q_EN
    // @DisplayName: LINK_Q_EN
    // @Description: Displays Receiver link quality and also RF mode if bit 20 of OSD_OPTIONS is set
    // @Values: 0:Disabled,1:Enabled

    // @Param: LINK_Q_X
    // @DisplayName: LINK_Q_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: LINK_Q_Y
    // @DisplayName: LINK_Q_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(link_quality, "LINK_Q", 1, AP_OSD_Screen, AP_OSD_Setting),
    
    //Custom Entries: (IDs start from 63 and go down to avoid conflicts with upstream)

    // @Param: NRG_CONS_EN
    // @DisplayName: NRG_CONS_EN
    // @Description: Displays total energy consumed from primary battery
    // @Values: 0:Disabled,1:Enabled

    // @Param: NRG_CONS_X
    // @DisplayName: NRG_CONS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: NRG_CONS_Y
    // @DisplayName: NRG_CONS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(energy_consumed, "NRG_CONS", 63, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: RC_THR_EN
    // @DisplayName: RC_THR_EN
    // @Description: Displays input throttle value (plane only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: RC_THR_X
    // @DisplayName: RC_THR_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: RC_THR_Y
    // @DisplayName: RC_THR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(rc_throttle, "RC_THR", 62, AP_OSD_Screen, AP_OSD_Setting),

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // @Param: ASPD_DEM_EN
    // @DisplayName: ASPD_DEM_EN
    // @Description: Displays the demanded airspeed in auto throttle modes (plane only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: ASPD_DEM_X
    // @DisplayName: ASPD_DEM_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ASPD_DEM_Y
    // @DisplayName: ASPD_DEM_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(aspd_dem, "ASPD_DEM", 61, AP_OSD_Screen, AP_OSD_Setting),
#endif

    // @Param: ACC_LONG_EN
    // @DisplayName: ACC_LONG_EN
    // @Description: Displays the aircraft's longitudinal acceleration in g
    // @Values: 0:Disabled,1:Enabled

    // @Param: ACC_LONG_X
    // @DisplayName: ACC_LONG_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ACC_LONG_Y
    // @DisplayName: ACC_LONG_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(acc_long, "ACC_LONG", 60, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ACC_LAT_EN
    // @DisplayName: ACC_LAT_EN
    // @Description: Displays the aircraft's lateral acceleration in g
    // @Values: 0:Disabled,1:Enabled

    // @Param: ACC_LAT_X
    // @DisplayName: ACC_LAT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ACC_LAT_Y
    // @DisplayName: ACC_LAT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(acc_lat, "ACC_LAT", 59, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ACC_VERT_EN
    // @DisplayName: ACC_VERT_EN
    // @Description: Displays the aircraft's vertical acceleration in g
    // @Values: 0:Disabled,1:Enabled

    // @Param: ACC_VERT_X
    // @DisplayName: ACC_VERT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ACC_VERT_Y
    // @DisplayName: ACC_VERT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(acc_vert, "ACC_VERT", 58, AP_OSD_Screen, AP_OSD_Setting),

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // @Param: AUTO_FLP_EN
    // @DisplayName: AUTO_FLAPS_EN
    // @Description: Displays the requested auto flaps position
    // @Values: 0:Disabled,1:Enabled

    // @Param: AUTO_FLP_X
    // @DisplayName: AUTO_FLAPS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: AUTO_FLP_Y
    // @DisplayName: AUTO_FLAPS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(auto_flaps, "AUTO_FLP", 57, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: AOA_EN
    // @DisplayName: AOA_EN
    // @Description: Displays the estimated angle of attack
    // @Values: 0:Disabled,1:Enabled

    // @Param: AOA_X
    // @DisplayName: AOA_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: AOA_Y
    // @DisplayName: AOA_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(aoa, "AOA", 56, AP_OSD_Screen, AP_OSD_Setting),
#endif

    // @Param: CRSFPWR_EN
    // @DisplayName: CRSFPWR_EN
    // @Description: Displays the TX power when using the CRSF RC protocol
    // @Values: 0:Disabled,1:Enabled

    // @Param: CRSFPWR_X
    // @DisplayName: CRSFPWR_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CRSFPWR_Y
    // @DisplayName: CRSFPWR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(crsf_tx_power, "CRSFPWR", 55, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: CRSFRSSI_EN
    // @DisplayName: CRSFRSSI_EN
    // @Description: Displays RC signal strength in dBm for CRSF
    // @Values: 0:Disabled,1:Enabled

    // @Param: CRSFRSSI_X
    // @DisplayName: CRSFRSSI_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CRSFRSSI_Y
    // @DisplayName: CRSFRSSI_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(crsf_rssi_dbm, "CRSFRSSI", 54, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: CRSFSNR_EN
    // @DisplayName: CRSFSNR_EN
    // @Description: Displays RC signal to noise ratio in dB for CRSF
    // @Values: 0:Disabled,1:Enabled

    // @Param: CRSFSNR_X
    // @DisplayName: CRSFSNR_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CRSFSNR_Y
    // @DisplayName: CRSFSNR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(crsf_snr, "CRSFSNR", 53, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: CRSFANT_EN
    // @DisplayName: CRSFANT_EN
    // @Description: Displays the current active antenna for CRSF
    // @Values: 0:Disabled,1:Enabled

    // @Param: CRSFANT_X
    // @DisplayName: CRSFANT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CRSFANT_Y
    // @DisplayName: CRSFANT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(crsf_active_antenna, "CRSFANT", 52, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: BAT_PCT_EN
    // @DisplayName: BAT_PCT_EN
    // @Description: Displays the remaining battery capacity as a percentage
    // @Values: 0:Disabled,1:Enabled

    // @Param: BAT_PCT_X
    // @DisplayName: BAT_PCT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BAT_PCT_Y
    // @DisplayName: BAT_PCT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(bat_pct, "BAT_PCT", 51, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: R_AVG_CV_EN
    // @DisplayName: AVGCELLV_EN
    // @Description: Displays average resting cell voltage. WARNING: this can be inaccurate if the cell count is not detected properly. If the cell count detection voltage is not right or the battery is far from fully charged the detected cell count might not be accurate. Use BATT_CELL_COUNT to force the number of cells.
    // @Values: 0:Disabled,1:Enabled

    // @Param: R_AVG_CV_X
    // @DisplayName: R_AVG_CV_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: R_AVG_CV_Y
    // @DisplayName: R_AVG_CV_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(resting_avgcellvolt, "R_AVG_CV", 50, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: NRG_REM_EN
    // @DisplayName: NRG_REM_EN
    // @Description: Displays energy remaining in primary battery
    // @Values: 0:Disabled,1:Enabled

    // @Param: NRG_REM_X
    // @DisplayName: NRG_REM_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: NRG_REM_Y
    // @DisplayName: NRG_REM_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(energy_remaining, "NRG_REM", 49, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: AVG_EFFG_EN
    // @DisplayName: AVG_EFFG_EN
    // @Description: Displays average ground flight efficiency (mAh or Wh / km or mi)
    // @Values: 0:Disabled,1:Enabled

    // @Param: AVG_EFFG_X
    // @DisplayName: AVG_EFFG_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: AVG_EFFG_Y
    // @DisplayName: AVG_EFFG_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(avg_eff_ground, "AVG_EFFG", 48, AP_OSD_Screen, AP_OSD_Setting),

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // @Param: AVG_EFFA_EN
    // @DisplayName: AVG_EFFA_EN
    // @Description: Displays average air flight efficiency (mAh or Wh / km or mi)
    // @Values: 0:Disabled,1:Enabled

    // @Param: AVG_EFFA_X
    // @DisplayName: AVG_EFFA_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: AVG_EFFA_Y
    // @DisplayName: AVG_EFFA_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(avg_eff_air, "AVG_EFFA", 47, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: EFFA_EN
    // @DisplayName: EFFA_EN
    // @Description: Displays air flight efficiency (mAh or Wh / km or mi)
    // @Values: 0:Disabled,1:Enabled

    // @Param: EFFA_X
    // @DisplayName: EFFA_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: EFFA_Y
    // @DisplayName: EFFA_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(eff_air, "EFFA", 46, AP_OSD_Screen, AP_OSD_Setting),
#endif

    // @Param: BATREM_EN
    // @DisplayName: BATREM_EN
    // @Description: Displays primary battery mAh remaining
    // @Values: 0:Disabled,1:Enabled

    // @Param: BATREM_X
    // @DisplayName: BATREM_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BATREM_Y
    // @DisplayName: BATREM_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(batrem, "BATREM", 45, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: BAT2REM_EN
    // @DisplayName: BAT2REM_EN
    // @Description: Displays secondary battery mAh consumed
    // @Values: 0:Disabled,1:Enabled

    // @Param: BAT2REM_X
    // @DisplayName: BAT2REM_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BAT2REM_Y
    // @DisplayName: BAT2REM_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(bat2rem, "BAT2REM", 44, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: TUNED_PN_EN
    // @DisplayName: TUNED_PN_EN
    // @Description: Displays the currently tuned parameter name when the parameter value is changing
    // @Values: 0:Disabled,1:Enabled

    // @Param: TUNED_PN_X
    // @DisplayName: TUNED_PN_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: TUNED_PN_Y
    // @DisplayName: TUNED_PN_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(tuned_param_name, "TUNED_PN", 43, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: TUNED_PV_EN
    // @DisplayName: TUNED_PV_EN
    // @Description: Displays the currently tuned parameter value when it is changing
    // @Values: 0:Disabled,1:Enabled

    // @Param: TUNED_PV_X
    // @DisplayName: TUNED_PV_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: TUNED_PV_Y
    // @DisplayName: TUNED_PV_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(tuned_param_value, "TUNED_PV", 42, AP_OSD_Screen, AP_OSD_Setting),

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // @Param: PEAK_RR_EN
    // @DisplayName: PEAK_RR_EN
    // @Description: Displays the peak roll rate in the last OSD_PEAKR_TMOUT seconds
    // @Values: 0:Disabled,1:Enabled

    // @Param: PEAK_RR_X
    // @DisplayName: PEAK_RR_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: PEAK_RR_Y
    // @DisplayName: PEAK_RR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(peak_roll_rate, "PEAK_RR", 41, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: PEAK_PR_EN
    // @DisplayName: PEAK_PR_EN
    // @Description: Displays the peak pitch rate in the last OSD_PEAKR_TMOUT seconds
    // @Values: 0:Disabled,1:Enabled

    // @Param: PEAK_PR_X
    // @DisplayName: PEAK_PR_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: PEAK_PR_Y
    // @DisplayName: PEAK_PR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(peak_pitch_rate, "PEAK_PR", 40, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: CRS_HEAD_EN
    // @DisplayName: CRS_HEAD_EN
    // @Description: Displays the locked heading when in cruise mode (plane only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: CRS_HEAD_X
    // @DisplayName: CRS_HEAD_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CRS_HEAD_Y
    // @DisplayName: CRS_HEAD_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(cruise_heading, "CRS_HEAD", 39, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: CRS_HADJ_EN
    // @DisplayName: CRS_HADJ_EN
    // @Description: Displays the recent cruise mode heading adjustment (plane only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: CRS_HADJ_X
    // @DisplayName: CRS_HADJ_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CRS_HADJ_Y
    // @DisplayName: CRS_HADJ_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(cruise_heading_adjustment, "CRS_HADJ", 38, AP_OSD_Screen, AP_OSD_Setting),
#endif

    // @Param: RC_FS_EN
    // @DisplayName: RC_FS_EN
    // @Description: Displays the RC failsafe status
    // @Values: 0:Disabled,1:Enabled

    // @Param: RC_FS_X
    // @DisplayName: RC_FS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: RC_FS_Y
    // @DisplayName: RC_FS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(rc_failsafe, "RC_FS", 37, AP_OSD_Screen, AP_OSD_Setting),

#if OSD_DEBUG_ELEMENT
    // @Param: DEBUG_EN
    // @DisplayName: DEBUG_EN
    // @Description: Displays debug value
    // @Values: 0:Disabled,1:Enabled

    // @Param: DEBUG_X
    // @DisplayName: DEBUG_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: DEBUG_Y
    // @DisplayName: DEBUG_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(debug, "DEBUG", 36, AP_OSD_Screen, AP_OSD_Setting),
#endif

    AP_GROUPEND
};


uint8_t AP_OSD_AbstractScreen::symbols_lookup_table[AP_OSD_NUM_SYMBOLS];

// Symbol indexes to acces _symbols[index][set]
#define SYM_M 0
#define SYM_KM 1
#define SYM_FT 2
#define SYM_MI 3
#define SYM_ALT_M 4
#define SYM_ALT_FT 5
#define SYM_BATT_FULL 6
#define SYM_RSSI 7

#define SYM_VOLT 8
#define SYM_AMP 9
#define SYM_MAH 10
#define SYM_MS 11
#define SYM_FS 12
#define SYM_KMH 13
#define SYM_MPH 14
#define SYM_DEGR 15
#define SYM_PCNT 16
#define SYM_RPM 17
#define SYM_ASPD 18
#define SYM_GSPD 19
#define SYM_WSPD 20
#define SYM_VSPD 21
#define SYM_WPNO 22
#define SYM_WPDIR 23
#define SYM_WPDST 24
#define SYM_FTMIN 25
#define SYM_FTSEC 26

#define SYM_SAT_L 27
#define SYM_SAT_R 28
#define SYM_HDOP_L 29
#define SYM_HDOP_R 30

#define SYM_HOME 31
#define SYM_WIND 32

#define SYM_ARROW_START 33
#define SYM_ARROW_COUNT 34
#define SYM_AH_H_START 35
#define SYM_AH_H_COUNT 36

#define SYM_AH_V_START 37
#define SYM_AH_V_COUNT 38

#define SYM_AH_CENTER_LINE_LEFT 39
#define SYM_AH_CENTER_LINE_RIGHT 40
#define SYM_AH_CENTER 41

#define SYM_HEADING_N 42
#define SYM_HEADING_S 43
#define SYM_HEADING_E 44
#define SYM_HEADING_W 45
#define SYM_HEADING_DIVIDED_LINE 46
#define SYM_HEADING_LINE 47

#define SYM_UP_UP 48
#define SYM_UP 49
#define SYM_DOWN 50
#define SYM_DOWN_DOWN 51

#define SYM_DEGREES_C 52
#define SYM_DEGREES_F 53
#define SYM_GPS_LAT 54
#define SYM_GPS_LONG 55
#define SYM_ARMED 56
#define SYM_DISARMED 57
#define SYM_ROLL0 58
#define SYM_ROLLR 59
#define SYM_ROLLL 60
#define SYM_PTCH0 61
#define SYM_PTCHUP 62
#define SYM_PTCHDWN 63
#define SYM_XERR 64
#define SYM_KN 65
#define SYM_NM 66
#define SYM_DIST 67
#define SYM_FLY 68
#define SYM_EFF 69
#define SYM_AH 70
#define SYM_MW 71
#define SYM_CLK 72
#define SYM_KILO 73
#define SYM_TERALT 74
#define SYM_FENCE_ENABLED 75
#define SYM_FENCE_DISABLED 76
#define SYM_RNGFD 77
#define SYM_LQ 78

#define SYM_SIDEBAR_R_ARROW 79
#define SYM_SIDEBAR_L_ARROW 80
#define SYM_SIDEBAR_A 81
#define SYM_SIDEBAR_B 82
#define SYM_SIDEBAR_C 83
#define SYM_SIDEBAR_D 84
#define SYM_SIDEBAR_E 85
#define SYM_SIDEBAR_F 86
#define SYM_SIDEBAR_G 87
#define SYM_SIDEBAR_H 88
#define SYM_SIDEBAR_I 89
#define SYM_SIDEBAR_J 90

#define SYM_WATT 91
#define SYM_WH 92
#define SYM_DB 93
#define SYM_DBM 94
#define SYM_SNR 95
#define SYM_ANT 96
#define SYM_ARROW_RIGHT 97
#define SYM_ARROW_LEFT 98

#define SYM_G 99
#define SYM_BATT_UNKNOWN 100
#define SYM_ROLL 101
#define SYM_PITCH 102
#define SYM_DPS 103
#define SYM_HEADING 104

#define SYMBOL(n) AP_OSD_AbstractScreen::symbols_lookup_table[n]

// constructor
AP_OSD_Screen::AP_OSD_Screen()
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::setup_object_defaults(this, var_info2);
}

void AP_OSD_AbstractScreen::set_backend(AP_OSD_Backend *_backend)
{
    backend = _backend;
    osd = _backend->get_osd();
};

bool AP_OSD_AbstractScreen::check_option(uint32_t option)
{
    return osd?(osd->options & option) != 0 : false;
}

/*
  get the right units icon given a unit
 */
char AP_OSD_AbstractScreen::u_icon(enum unit_type unit)
{
    static const uint8_t icons_metric[UNIT_TYPE_LAST] {
        SYM_ALT_M,    //ALTITUDE
        SYM_KMH,      //SPEED
        SYM_MS,       //VSPEED
        SYM_M,        //DISTANCE
        SYM_KM,       //DISTANCE_LONG
        SYM_DEGREES_C //TEMPERATURE
    };
    static const uint8_t icons_imperial[UNIT_TYPE_LAST] {
        SYM_ALT_FT,   //ALTITUDE
        SYM_MPH,      //SPEED
        SYM_FS,       //VSPEED
        SYM_FT,       //DISTANCE
        SYM_MI,       //DISTANCE_LONG
        SYM_DEGREES_F //TEMPERATURE
    };
    static const uint8_t icons_SI[UNIT_TYPE_LAST] {
        SYM_ALT_M,    //ALTITUDE
        SYM_MS,       //SPEED
        SYM_MS,       //VSPEED
        SYM_M,        //DISTANCE
        SYM_KM,       //DISTANCE_LONG
        SYM_DEGREES_C //TEMPERATURE
    };
    static const uint8_t icons_aviation[UNIT_TYPE_LAST] {
        SYM_ALT_FT,   //ALTITUDE Ft
        SYM_KN,       //SPEED Knots
        SYM_FTMIN,    //VSPEED
        SYM_FT,       //DISTANCE
        SYM_NM,       //DISTANCE_LONG Nm
        SYM_DEGREES_C //TEMPERATURE
    };
    static const uint8_t* icons[AP_OSD::UNITS_LAST] = {
        icons_metric,
        icons_imperial,
        icons_SI,
        icons_aviation,
    };
    return (char)SYMBOL(icons[constrain_int16(osd->units, 0, AP_OSD::UNITS_LAST-1)][unit]);
}

/*
  scale a value for the user selected units
 */
float AP_OSD_AbstractScreen::u_scale(enum unit_type unit, float value)
{
    static const float scale_metric[UNIT_TYPE_LAST] = {
        1.0,       //ALTITUDE m
        3.6,       //SPEED km/hr
        1.0,       //VSPEED m/s
        1.0,       //DISTANCE m
        1.0/1000,  //DISTANCE_LONG km
        1.0,       //TEMPERATURE C
    };
    static const float scale_imperial[UNIT_TYPE_LAST] = {
        3.28084,     //ALTITUDE ft
        2.23694,     //SPEED mph
        3.28084,     //VSPEED ft/s
        3.28084,     //DISTANCE ft
        1.0/1609.34, //DISTANCE_LONG miles
        1.8,         //TEMPERATURE F
    };
    static const float offset_imperial[UNIT_TYPE_LAST] = {
        0.0,          //ALTITUDE
        0.0,          //SPEED
        0.0,          //VSPEED
        0.0,          //DISTANCE
        0.0,          //DISTANCE_LONG
        32.0,         //TEMPERATURE F
    };
    static const float scale_SI[UNIT_TYPE_LAST] = {
        1.0,       //ALTITUDE m
        1.0,       //SPEED m/s
        1.0,       //VSPEED m/s
        1.0,       //DISTANCE m
        1.0/1000,  //DISTANCE_LONG km
        1.0,       //TEMPERATURE C
    };
    static const float scale_aviation[UNIT_TYPE_LAST] = {
        3.28084,   //ALTITUDE Ft
        1.94384,   //SPEED Knots
        196.85,    //VSPEED ft/min
        3.28084,   //DISTANCE ft
        0.000539957,  //DISTANCE_LONG Nm
        1.0,       //TEMPERATURE C
    };
    static const float *scale[AP_OSD::UNITS_LAST] = {
        scale_metric,
        scale_imperial,
        scale_SI,
        scale_aviation
    };
    static const float *offsets[AP_OSD::UNITS_LAST] = {
        nullptr,
        offset_imperial,
        nullptr,
        nullptr
    };
    uint8_t units = constrain_int16(osd->units, 0, AP_OSD::UNITS_LAST-1);
    return value * scale[units][unit] + (offsets[units]?offsets[units][unit]:0);
}

void AP_OSD_Screen::draw_altitude(uint8_t x, uint8_t y, bool blink, float alt, bool available)
{
    if (!available) {
        backend->write(x, y, blink, "----%c", u_icon(ALTITUDE));
        return;
    }

    backend->write(x, y, blink, "%4d%c", (int)u_scale(ALTITUDE, alt), u_icon(ALTITUDE));
}

void AP_OSD_Screen::draw_altitude(uint8_t x, uint8_t y)
{
    float alt;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        ahrs.get_relative_position_D_home(alt);
    }
    alt = -alt;
    draw_altitude(x, y, false, alt);
}

void AP_OSD_Screen::draw_voltage(uint8_t x, uint8_t y, const float voltage, const bool two_decimals, const bool blink_voltage, const bool show_batt_symbol, const bool available)
{
    if (!available) {
        backend->write(x + 1, y, blink_voltage, "---%c", SYMBOL(SYM_VOLT));
        return;
    }

    AP_BattMonitor &battery = AP::battery();
    uint8_t pct;
    const char *fmt = two_decimals ? "%.2f%c" : "%.1f%c";
    if (show_batt_symbol && battery.capacity_remaining_pct(pct)) {
        const uint8_t p = (100 - pct) / 16.6;
        const uint8_t batt_symbol = SYMBOL(SYM_BATT_FULL) + p;
        const bool blink_batt = battery.remaining_mah_is_low() || battery.remaining_wh_is_low();
        backend->write(x, y, blink_batt, "%c", batt_symbol);
        backend->write(x+1, y, blink_voltage, fmt, voltage, SYMBOL(SYM_VOLT));
    } else if (show_batt_symbol && battery.capacity_has_been_configured()) {
        backend->write(x, y, false, "%c", SYMBOL(SYM_BATT_UNKNOWN));
        backend->write(x+1, y, blink_voltage, fmt, voltage, SYMBOL(SYM_VOLT));
    } else {
        backend->write(x+1, y, blink_voltage, fmt, voltage, SYMBOL(SYM_VOLT));
    }
}

void AP_OSD_Screen::draw_avgcellvolt(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP::battery();
    float cell_voltage;
    const bool cell_voltage_available = battery.cell_avg_voltage(cell_voltage);
    const bool blink_voltage = battery.voltage_is_low();
    draw_voltage(x, y, cell_voltage, true, blink_voltage, true, cell_voltage_available);
}

void AP_OSD_Screen::draw_resting_avgcellvolt(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP::battery();
    float resting_cell_voltage;
    const bool resting_cell_voltage_available = battery.resting_cell_avg_voltage(resting_cell_voltage);
    const bool blink_voltage = battery.resting_voltage_is_low();
    draw_voltage(x, y, resting_cell_voltage, true, blink_voltage, true, resting_cell_voltage_available);
}

void AP_OSD_Screen::draw_restvolt(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP::battery();
    float v = battery.voltage_resting_estimate();
    const bool blink_voltage = battery.resting_voltage_is_low();
    draw_voltage(x, y, v, false, blink_voltage);
}

void AP_OSD_Screen::draw_bat_volt(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP::battery();
    float v = battery.voltage();
    const bool blink_voltage = battery.voltage_is_low();
    draw_voltage(x, y, v, false, blink_voltage);
}

void AP_OSD_Screen::draw_bat_pct(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP::battery();

    const bool blink = battery.voltage_is_low() || battery.remaining_mah_is_low() || battery.remaining_wh_is_low();

    uint8_t pct;

    if (!battery.capacity_remaining_pct(pct)) {
        if (!AP_Notify::flags.armed) {
            backend->write(x , y , false , "---%c" , SYMBOL(SYM_PCNT));
        }
        return;
    }

    backend->write(x, y, blink, "%3d%c", (uint16_t)lrintf(pct), SYMBOL(SYM_PCNT));
}

void AP_OSD_Screen::draw_rssi(uint8_t x, uint8_t y)
{
    AP_RSSI *ap_rssi = AP_RSSI::get_singleton();
    if (ap_rssi) {
        const uint8_t rssiv = ap_rssi->read_receiver_rssi() * 100;
        backend->write(x, y, rssiv < osd->warn_rssi, "%c%2d", SYMBOL(SYM_RSSI), rssiv);
    }
}

void AP_OSD_Screen::draw_link_quality(uint8_t x, uint8_t y)
{
    AP_RSSI *ap_rssi = AP_RSSI::get_singleton();
    if (ap_rssi) {
        const int16_t lqv = ap_rssi->read_receiver_link_quality();
        const int16_t rf_mode = AP::crsf()->get_link_status().rf_mode;

        // rf_mode of 0 of higher indicates we are using the CRSF protocol
        // Display the RF mode at the start of the LQ element
        if (check_option(AP_OSD::OPTION_RF_MODE_ALONG_WITH_LQ) && rf_mode >= 0) {
            if (lqv < 0) {
                backend->write(x, y, false, "%c-:---%c", SYMBOL(SYM_LQ), SYMBOL(SYM_PCNT));
            } else {
                backend->write(x, y, false, "%c%d:%2d%c", SYMBOL(SYM_LQ), rf_mode, lqv, SYMBOL(SYM_PCNT));
            }
        } else {
            if (lqv < 0) {
                backend->write(x, y, false, "%c---%c", SYMBOL(SYM_LQ), SYMBOL(SYM_PCNT));
            } else {
                backend->write(x, y, false, "%c%2d%c", SYMBOL(SYM_LQ), lqv, SYMBOL(SYM_PCNT));
            }
        }
        
    }
}

void AP_OSD_Screen::draw_current(uint8_t x, uint8_t y, const bool available, const bool blink, const float value)
{
    if (available) {
        const char* const fmt = (value < 9.995 ? "%1.2f%c" : (value < 99.95 ? "%2.1f%c" : "%3.0f%c"));
        backend->write(x, y, blink, fmt, value, SYMBOL(SYM_AMP));
    } else {
        backend->write(x, y, blink, "---%c", SYMBOL(SYM_AMP));
    }
}

void AP_OSD_Screen::draw_current(uint8_t instance, uint8_t x, uint8_t y)
{
    float amps;
    if (AP::battery().current_amps(amps, instance)) {
        osd->filtered.current_a += (amps - osd->filtered.current_a) * 0.33;
        draw_current(x, y, true, false, osd->filtered.current_a);
    } else {
        draw_current(x, y, false);
    }
}

void AP_OSD_Screen::draw_current(uint8_t x, uint8_t y)
{
    draw_current(0, x, y);
}

void AP_OSD_Screen::draw_power(uint8_t x, uint8_t y, const bool available, const bool blink, const float value)
{
    if (available) {
        const char* fmt;
        uint8_t spaces;
        if (value < 9.995) {
            fmt = "%.2f%c";
            spaces = 1;
        } else if (value < 99.95) {
            fmt = "%.1f%c";
            spaces = 1;
        } else {
            fmt = "%.0f%c";
            spaces = value < 999.5 ? 1 : 0;
        }
        backend->write(x+spaces, y, blink, fmt, value, SYMBOL(SYM_WATT));
    } else {
        backend->write(x, y, blink, "----%c", SYMBOL(SYM_WATT));
    }
}

void AP_OSD_Screen::draw_power(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP::battery();
    float power_w;
    if (battery.power_watts(power_w)) {
        osd->filtered.power_w += (power_w - osd->filtered.power_w) * 0.33;
        draw_power(x, y, true, false, osd->filtered.power_w);
    } else {
        draw_power(x, y, false);
    }
}

void AP_OSD_Screen::draw_energy(uint8_t x, uint8_t y, bool available, bool blink, float energy_wh, bool can_be_negative)
{
    if (!available) {
        const char *fmt = can_be_negative ? "-----%c" : "----%c";
        backend->write(x, y, false, fmt, SYMBOL(SYM_WH));
        return;
    }
    const uint8_t spaces = !can_be_negative || signbit(energy_wh) ? 0 : 1;
    const char* const fmt = (energy_wh < 9.9995 ? "%1.3f%c" : (energy_wh < 99.995 ? "%2.2f%c" : "%3.1f%c"));
    backend->write(x + spaces, y, blink, fmt, energy_wh, SYMBOL(SYM_WH));
}

void AP_OSD_Screen::draw_energy_consumed(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP::battery();
    float energy_wh;
    const bool available = battery.consumed_wh(energy_wh);
    draw_energy(x, y, available, battery.remaining_wh_is_low(), energy_wh, false);
}

void AP_OSD_Screen::draw_energy_remaining(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP::battery();
    float energy_wh;
    const bool available = battery.remaining_wh(energy_wh);
    draw_energy(x, y, available, battery.remaining_wh_is_low(), energy_wh, true);
}

void AP_OSD_Screen::draw_fltmode(uint8_t x, uint8_t y)
{
    AP_Notify * notify = AP_Notify::get_singleton();
    char arm;
    if (AP_Notify::flags.armed && !AP_Notify::flags.throttle_cut) {
        arm = SYMBOL(SYM_ARMED);
    } else {
        arm = SYMBOL(SYM_DISARMED);
    }
    if (notify) {
        backend->write(x, y, false, "%s", notify->get_flight_mode_str());
        backend->write(x+4, y, AP_Notify::flags.throttle_cut, "%c", arm);
    }
}

void AP_OSD_Screen::draw_sats(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    uint8_t nsat = gps.num_sats();
    bool flash = (nsat < osd->warn_nsat) || (gps.status() < AP_GPS::GPS_OK_FIX_3D);
    backend->write(x, y, flash, "%c%c%2u", SYMBOL(SYM_SAT_L), SYMBOL(SYM_SAT_R), nsat);
}

void AP_OSD_Screen::draw_mah(uint8_t x, uint8_t y, bool available, bool blink, uint mah, bool can_be_negative)
{
    if (!available) {
        backend->write(x, y, blink, "----%c", SYMBOL(SYM_MAH));
        return;
    }

    const uint8_t spaces = !can_be_negative || signbit(mah) ? 0 : 1;
    if (mah < 10000) {
        backend->write(x+spaces, y, blink, "%4d%c", mah, SYMBOL(SYM_MAH));
    } else {
        const float ah = mah * 1e-3f;
        backend->write(x+spaces, y, blink, "%2.2f%c", ah, SYMBOL(SYM_AH));
    }
}

void AP_OSD_Screen::draw_batused(uint8_t x, uint8_t y, uint8_t instance)
{
    float mah;
    AP_BattMonitor &battery = AP::battery();
    const bool consumed_mah_available = battery.consumed_mah(mah, instance);
    const bool blink = battery.remaining_mah_is_low();
    draw_mah(x, y, consumed_mah_available, blink, mah, false);
}

void AP_OSD_Screen::draw_batused(uint8_t x, uint8_t y)
{
    draw_batused(x, y, 0);
}

void AP_OSD_Screen::draw_batrem(uint8_t x, uint8_t y, uint8_t instance)
{
    float mah;
    AP_BattMonitor &battery = AP::battery();
    const bool mah_available = battery.remaining_mah(mah, instance);
    const bool blink = battery.remaining_mah_is_low();
    draw_mah(x, y, mah_available, blink, mah, true);
}

void AP_OSD_Screen::draw_batrem(uint8_t x, uint8_t y)
{
    draw_batrem(x, y, 0);
}

//Autoscroll message is the same as in minimosd-extra.
//Thanks to night-ghost for the approach.
void AP_OSD_Screen::draw_message(uint8_t x, uint8_t y)
{
    AP_Notify * notify = AP_Notify::get_singleton();
    if (notify) {
        int32_t visible_time = AP_HAL::millis() - notify->get_text_updated_millis();
        if (visible_time < osd->msgtime_s *1000) {
            char buffer[NOTIFY_TEXT_BUFFER_SIZE];
            strncpy(buffer, notify->get_text(), sizeof(buffer));
            int16_t len = strnlen(buffer, sizeof(buffer));

            for (int16_t i=0; i<len; i++) {
                //converted to uppercase,
                //because we do not have small letter chars inside used font
                buffer[i] = toupper(buffer[i]);
                //normalize whitespace
                if (isspace(buffer[i])) {
                    buffer[i] = ' ';
                }
            }

            int16_t start_position = 0;
            //scroll if required
            //scroll pattern: wait, scroll to the left, wait, scroll to the right
            if (len > message_visible_width) {
                int16_t chars_to_scroll = len - message_visible_width;
                int16_t total_cycles = 2*message_scroll_delay + 2*chars_to_scroll;
                int16_t current_cycle = (visible_time / message_scroll_time_ms) % total_cycles;

                //calculate scroll start_position
                if (current_cycle < total_cycles/2) {
                    //move to the left
                    start_position = current_cycle - message_scroll_delay;
                } else {
                    //move to the right
                    start_position = total_cycles - current_cycle;
                }
                start_position = constrain_int16(start_position, 0, chars_to_scroll);
                int16_t end_position = start_position + message_visible_width;

                //ensure array boundaries
                start_position = MIN(start_position, int(sizeof(buffer)-1));
                end_position = MIN(end_position, int(sizeof(buffer)-1));

                //trim invisible part
                buffer[end_position] = 0;
            }

            backend->write(x, y, buffer + start_position);
        }
    }
}

void AP_OSD_Screen::draw_speed(uint8_t x, uint8_t y, bool available, float magnitude, bool blink)
{
    if (!available) {
        backend->write(x, y, blink, "---%c", u_icon(SPEED));
        return;
    }

    const float magnitude_scaled = u_scale(SPEED, magnitude);
    const char *fmt;
    uint8_t spaces;
    
    if (magnitude_scaled < 9.95) {
        fmt = "%.1f%c";
        spaces = 1;
    } else {
        fmt = "%.0f%c";
        spaces = magnitude_scaled < 99.5 ? 1 : 0;
    }

    backend->write(x+spaces, y, blink, fmt, magnitude_scaled, u_icon(SPEED));
}

// draw a arrow at the given angle, and print the given magnitude
void AP_OSD_Screen::draw_speed_with_arrow(uint8_t x, uint8_t y, float angle_rad, float magnitude, bool blink)
{
    static const int32_t arrow_interval = 36000 / SYMBOL(SYM_ARROW_COUNT);
    char arrow = SYMBOL(SYM_ARROW_START) + ((int32_t(angle_rad*DEGX100) + arrow_interval / 2) / arrow_interval) % SYMBOL(SYM_ARROW_COUNT);
    backend->write(x, y, false, "%c", arrow);
    draw_speed(x+1, y, true, magnitude, blink);
}

void AP_OSD_Screen::draw_gspeed(uint8_t x, uint8_t y)
{
    Vector2f v;
    float yaw;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        v = ahrs.groundspeed_vector();
        yaw = ahrs.yaw;
    }
    backend->write(x, y, false, "%c", SYMBOL(SYM_GSPD));

    float angle = 0;
    const float length = v.length();
    if (length > 1.0f) {
        angle = wrap_2PI(atan2f(v.y, v.x) - yaw);
    }

    draw_speed_with_arrow(x + 1, y, angle, length);
}

//Thanks to betaflight/inav for simple and clean artificial horizon visual design
void AP_OSD_Screen::draw_horizon(uint8_t x, uint8_t y)
{
    float roll;
    float pitch;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        AP::vehicle()->get_osd_roll_pitch_rad(roll, pitch);
    }
    pitch *= -1;

    //inverted roll AH
    if (check_option(AP_OSD::OPTION_INVERTED_AH_ROLL)) {
        roll = -roll;
    }

    const float ah_pitch_max = ToRad(osd->ah_pitch_max);
    const float pitch_rad_to_char = 8.8f / (2 * ah_pitch_max); // 9 lines but remove a little bit so that half of the line doesn't disappear for very small roll angles

    pitch = constrain_float(pitch, -ah_pitch_max, ah_pitch_max);
    float ky = sinf(roll);
    float kx = cosf(roll);

    float ratio = backend->get_aspect_ratio_correction();

    if (fabsf(ky) < fabsf(kx)) {
        for (int dx = -4; dx <= 4; dx++) {
            float fy = (ratio * dx) * (ky/kx) + pitch * pitch_rad_to_char + 0.5f;
            int dy = floorf(fy);
            char c = (fy - dy) * SYMBOL(SYM_AH_H_COUNT);
            //chars in font in reversed order
            c = SYMBOL(SYM_AH_H_START) + ((SYMBOL(SYM_AH_H_COUNT) - 1) - c);
            if (dy >= -4 && dy <= 4) {
                backend->write(x + dx, y - dy, false, "%c", c);
            }
        }
    } else {
        for (int dy=-4; dy<=4; dy++) {
            float fx = ((dy / ratio) - pitch * pitch_rad_to_char) * (kx/ky) + 0.5f;
            int dx = floorf(fx);
            char c = (fx - dx) * SYMBOL(SYM_AH_V_COUNT);
            c = SYMBOL(SYM_AH_V_START) + c;
            if (dx >= -4 && dx <=4) {
                backend->write(x + dx, y - dy, false, "%c", c);
            }
        }
    }

    if (!check_option(AP_OSD::OPTION_DISABLE_CROSSHAIR)) {
        backend->write(x-1,y, false, "%c%c%c", SYMBOL(SYM_AH_CENTER_LINE_LEFT), SYMBOL(SYM_AH_CENTER), SYMBOL(SYM_AH_CENTER_LINE_RIGHT));
    }

}

void AP_OSD_Screen::draw_distance(uint8_t x, uint8_t y, float distance, bool can_only_be_positive, bool available)
{
    char unit_icon = u_icon(DISTANCE);

    if (!available) {
        const char *const format = can_only_be_positive ? "----%c" : "-----%c";
        backend->write(x, y, false, format, unit_icon);
        return;
    }

    float distance_scaled = u_scale(DISTANCE, distance);

    const char *format;
    uint8_t spaces;

    if (distance_scaled < 9999.5f) {

        const float distance_scaled_abs = abs(distance_scaled);

        format = "%.0f%c";

        if (distance_scaled_abs < 9.95) {
            format = "%1.1f%c";
            spaces = 3;
        } else if (distance_scaled_abs < 99.5) {
            spaces = 3;
        } else if (distance_scaled_abs < 999.5) {
            spaces = 2;
        } else {
            spaces = 1;
        }

    } else {

        distance_scaled = u_scale(DISTANCE_LONG, distance);
        unit_icon= u_icon(DISTANCE_LONG);

        const float distance_scaled_abs = abs(distance_scaled);

        if (distance_scaled_abs < 9.995) {
            format = "%1.3f%c";
        } else if (distance_scaled_abs < 99.95) {
            format = "%2.2f%c";
        } else if (distance_scaled_abs < 999.5) {
            format = "%3.1f%c";
        } else {
            format = "%4.0f%c";
        }

        spaces = 1;

    }

    if (can_only_be_positive || signbit(distance_scaled)) {
        spaces -= 1;
    }

    backend->write(x+spaces, y, false, format, distance_scaled, unit_icon);
}

void AP_OSD_Screen::draw_home(uint8_t x, uint8_t y)
{
    Location loc;
    Location home_loc;
    int32_t yaw_sensor;
    bool home_is_set;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        home_is_set = ahrs.get_location(loc) && ahrs.home_is_set();
        if (home_is_set) {
            home_loc = ahrs.get_home();
            yaw_sensor = ahrs.yaw_sensor;
        }
    }
    if (home_is_set) {
        // const Location &home_loc = ahrs.get_home();
        float distance = home_loc.get_distance(loc);
        int32_t angle = wrap_360_cd(loc.get_bearing_to(home_loc) - yaw_sensor);
        int32_t interval = 36000 / SYMBOL(SYM_ARROW_COUNT);
        if (distance < 2.0f) {
            //avoid fast rotating arrow at small distances
            angle = 0;
        }
        char arrow = SYMBOL(SYM_ARROW_START) + ((angle + interval / 2) / interval) % SYMBOL(SYM_ARROW_COUNT);
        backend->write(x, y, false, "%c%c", SYMBOL(SYM_HOME), arrow);
        draw_distance(x+2, y, distance, true);
    } else {
        backend->write(x, y, true, "%c-", SYMBOL(SYM_HOME));
        draw_distance(x+2, y, 0, true, false);
    }
}

void AP_OSD_Screen::draw_heading(uint8_t x, uint8_t y)
{
    float yaw_sensor;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        yaw_sensor = ahrs.yaw_sensor;
    }
    uint16_t yaw = yaw_sensor / 100;
    backend->write(x, y, false, "%c%3d%c", SYMBOL(SYM_HEADING), yaw, SYMBOL(SYM_DEGR));
}

void AP_OSD_Screen::draw_throttle_value(uint8_t x, uint8_t y, float throttle_v, bool blink)
{
    const char *format;
    uint8_t spaces;
    float throttle_abs = fabsf(throttle_v);
    if (osd->options & AP_OSD::OPTION_ONE_DECIMAL_THROTTLE) {
        if (throttle_abs < 9.95) {
            spaces = 2;
            format = "%1.1f%c";
        } else if (throttle_abs < 99.95) {
            spaces = 1;
            format = "%2.1f%c";
        } else {
            spaces = 1;
            format = "%3.0f%c";
        }
    } else {
        if (throttle_abs < 9.5) {
            spaces = 3;
            format = "%1.0f%c";
        } else if (throttle_abs < 99.5) {
            spaces = 2;
            format = "%2.0f%c";
        } else {
            spaces = 1;
            format = "%3.0f%c";
        }
    }
    if (signbit(throttle_v)) {
        spaces -= 1;
    }
    backend->write(x + spaces, y, blink, format, throttle_v, SYMBOL(SYM_PCNT));
}

void AP_OSD_Screen::draw_throttle_output(uint8_t x, uint8_t y)
{
    draw_throttle_value(x, y, gcs().get_hud_throttle(), AP_Notify::flags.throttle_cut || AP::vehicle()->is_auto_throttle_gliding());
}

#if HAL_OSD_SIDEBAR_ENABLE

void AP_OSD_Screen::draw_sidebars(uint8_t x, uint8_t y)
{
    const int8_t total_sectors = 18;
    static const uint8_t sidebar_sectors[total_sectors] = {
        SYM_SIDEBAR_A,
        SYM_SIDEBAR_B,
        SYM_SIDEBAR_C,
        SYM_SIDEBAR_D,
        SYM_SIDEBAR_E,
        SYM_SIDEBAR_F,
        SYM_SIDEBAR_G,
        SYM_SIDEBAR_E,
        SYM_SIDEBAR_F,
        SYM_SIDEBAR_G,
        SYM_SIDEBAR_E,
        SYM_SIDEBAR_F,
        SYM_SIDEBAR_G,
        SYM_SIDEBAR_E,
        SYM_SIDEBAR_F,
        SYM_SIDEBAR_H,
        SYM_SIDEBAR_I,
        SYM_SIDEBAR_J,
    };

    // Get altitude and airspeed, scaled to appropriate units
    float aspd;
    float alt = 0.0f;
    bool have_airspeed_estimate;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        have_airspeed_estimate = ahrs.airspeed_estimate(aspd);
        ahrs.get_relative_position_D_home(alt);
    }
    if (!have_airspeed_estimate) { aspd = 0.0f; }
    float scaled_aspd = u_scale(SPEED, aspd);
    float scaled_alt = u_scale(ALTITUDE, -alt);
    static const int aspd_interval = 10; //units between large tick marks
    int alt_interval = (osd->units == AP_OSD::UNITS_AVIATION || osd->units == AP_OSD::UNITS_IMPERIAL) ? 20 : 10;

    // render airspeed ladder
    int aspd_symbol_index = fmodf(scaled_aspd, aspd_interval) / aspd_interval * total_sectors;
    for (int i = 0; i < 7; i++){
        if (i == 3) {
            // the middle section of the ladder with the currrent airspeed
            backend->write(x, y+i, false, "%3d%c%c", (int) scaled_aspd, u_icon(SPEED), SYMBOL(SYM_SIDEBAR_R_ARROW));
        } else {
            backend->write(x+4, y+i, false,  "%c", SYMBOL(sidebar_sectors[aspd_symbol_index]));
        }
        aspd_symbol_index = (aspd_symbol_index + 12) % 18;
    }

    // render the altitude ladder
    // similar formula to above, but accounts for negative altitudes
    int alt_symbol_index = fmodf(fmodf(scaled_alt, alt_interval) + alt_interval, alt_interval) / alt_interval * total_sectors;
    for (int i = 0; i < 7; i++){
        if (i == 3) {
            // the middle section of the ladder with the currrent altitude
            backend->write(x+16, y+i, false, "%c%d%c", SYMBOL(SYM_SIDEBAR_L_ARROW), (int) scaled_alt, u_icon(ALTITUDE));
        } else {
            backend->write(x+16, y+i, false,  "%c", SYMBOL(sidebar_sectors[alt_symbol_index]));
        }
        alt_symbol_index = (alt_symbol_index + 12) % 18;
    }
}

#endif // HAL_OSD_SIDEBAR_ENABLE

//Thanks to betaflight/inav for simple and clean compass visual design
void AP_OSD_Screen::draw_compass(uint8_t x, uint8_t y)
{
    const int8_t total_sectors = 16;
    static const uint8_t compass_circle[total_sectors] = {
        SYM_HEADING_N,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_E,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_S,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_W,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
    };
    int32_t yaw_sensor;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        yaw_sensor = ahrs.yaw_sensor;
    }
    int32_t interval = 36000 / total_sectors;
    int8_t center_sector = ((yaw_sensor + interval / 2) / interval) % total_sectors;
    for (int8_t i = -4; i <= 4; i++) {
        int8_t sector = center_sector + i;
        sector = (sector + total_sectors) % total_sectors;
        backend->write(x + i, y, false,  "%c", SYMBOL(compass_circle[sector]));
    }
}

void AP_OSD_Screen::draw_wind(uint8_t x, uint8_t y)
{
#if !APM_BUILD_TYPE(APM_BUILD_Rover)
    float yaw;
    Vector3f v;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        v = ahrs.wind_estimate();
        yaw = ahrs.yaw;
    }
    float angle = 0;
    const float length = v.length();
    if (length > 1.0f) {
        if (check_option(AP_OSD::OPTION_INVERTED_WIND)) {
            angle = M_PI;
        }
        angle = wrap_2PI(angle + atan2f(v.y, v.x) - yaw);
    }
    draw_speed_with_arrow(x + 1, y, angle, length);

#else
    const AP_WindVane* windvane = AP_WindVane::get_singleton();
    if (windvane != nullptr) {
        draw_speed_with_arrow(x + 1, y, wrap_2PI(windvane->get_apparent_wind_direction_rad() + M_PI), windvane->get_apparent_wind_speed());
    }
#endif

    backend->write(x, y, false, "%c", SYMBOL(SYM_WSPD));
}

void AP_OSD_Screen::draw_vspeed(uint8_t x, uint8_t y)
{
    Vector3f v;
    float vspd_mps;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        if (ahrs.get_velocity_NED(v)) {
            vspd_mps = -v.z;
        } else {
            auto &baro = AP::baro();
            WITH_SEMAPHORE(baro.get_semaphore());
            vspd_mps = baro.get_climb_rate();
        }
    }

    vspd_mps = osd->filtered.vspd_mps += (vspd_mps - osd->filtered.vspd_mps) * 0.33f;

    const float vs_scaled_abs = u_scale(VSPEED, abs(vspd_mps));

    float edge_sym_slow;
    const float edge_sym_high = 3.0f;

    const char *fmt;
    if (osd->options & AP_OSD::OPTION_TWO_DECIMALS_VERTICAL_SPEED) {

        edge_sym_slow = 0.005;

        if (vs_scaled_abs < 9.995) {
            fmt = "%c%.2f%c";
        } else {
            fmt = "%c%.1f%c";
        }

    } else {

        edge_sym_slow = 0.05;

        if (vs_scaled_abs < 9.95) {
            fmt = "%c%.1f%c";
        } else {
            fmt = "%c%.0f%c";
        }

    }

    char sym = signbit(vspd_mps) ? SYMBOL(SYM_DOWN) : SYMBOL(SYM_UP);
    if (abs(vspd_mps) < edge_sym_slow) {
        sym = ' ';
    } else if (vspd_mps <= -edge_sym_high) {
        sym = SYMBOL(SYM_DOWN_DOWN);
    } else if (vspd_mps >= edge_sym_high) {
        sym = SYMBOL(SYM_UP_UP);
    }

    backend->write(x, y, false, fmt, sym, vs_scaled_abs, u_icon(VSPEED));
}

void AP_OSD_Screen::draw_temperature(uint8_t x, uint8_t y, bool available, float value, bool blink)
{
    if (!available) {
        backend->write(x, y, blink, "---%c", u_icon(TEMPERATURE));
        return;
    }

    backend->write(x, y, blink, "%3d%c", (int)u_scale(TEMPERATURE, value), u_icon(TEMPERATURE));
}

#if HAL_WITH_ESC_TELEM
void AP_OSD_Screen::draw_highest_esc_temp(uint8_t x, uint8_t y)
{
    int16_t etemp;
    if (!AP::esc_telem().get_highest_temperature(etemp)) {
        draw_temperature(x, y, false);
        return;
    }

    etemp /= 100;
    const bool blink = is_positive(osd->warn_blhtemp) && etemp > osd->warn_blhtemp;
    draw_temperature(x, y, true, etemp, blink);
}

void AP_OSD_Screen::draw_rpm(uint8_t x, uint8_t y, float rpm)
{
    float krpm = rpm * 0.001f;
    const char *format = krpm < 9.995 ? "%.2f%c%c" : (krpm < 99.95 ? "%.1f%c%c" : "%.0f%c%c");
    const bool warn_high = osd->warn_blh_high_rpm > 0 && krpm > osd->warn_blh_high_rpm;
    const bool warn_low = osd->warn_blh_low_rpm > 0 && krpm < osd->warn_blh_low_rpm && gcs().get_hud_throttle() >= 5;
    backend->write(x, y, warn_high || warn_low, format, krpm, SYMBOL(SYM_KILO), SYMBOL(SYM_RPM));
}

void AP_OSD_Screen::draw_avg_esc_rpm(uint8_t x, uint8_t y)
{
    draw_rpm(x, y, AP::esc_telem().get_average_motor_rpm());
}

void AP_OSD_Screen::draw_highest_esc_rpm(uint8_t x, uint8_t y)
{
    float rpm;
    if (AP::esc_telem().get_highest_motor_rpm(rpm)) {
        draw_rpm(x, y, rpm);
    }
}

void AP_OSD_Screen::draw_avg_esc_amps(uint8_t x, uint8_t y)
{
    float amps;
    if (!AP::esc_telem().get_average_current(amps)) {
        return;
    }
    backend->write(x, y, false, "%4.1f%c", amps, SYMBOL(SYM_AMP));
}

void AP_OSD_Screen::draw_highest_esc_amps(uint8_t x, uint8_t y)
{
    float amps;
    if (!AP::esc_telem().get_highest_current(amps)) {
        return;
    }
    backend->write(x, y, false, "%4.1f%c", amps, SYMBOL(SYM_AMP));
}

void AP_OSD_Screen::draw_total_esc_amps(uint8_t x, uint8_t y)
{
    float amps;
    if (!AP::esc_telem().get_total_current(amps)) {
        return;
    }
    backend->write(x, y, false, "%4.1f%c", amps, SYMBOL(SYM_AMP));
}
#endif

void AP_OSD_Screen::draw_gps_latitude(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    const Location &loc = gps.location();   // loc.lat and loc.lng
    int32_t dec_portion, frac_portion;
    int32_t abs_lat = labs(loc.lat);

    dec_portion = loc.lat / 10000000L;
    frac_portion = abs_lat - labs(dec_portion)*10000000UL;

    backend->write(x, y, false, "%c%4ld.%07ld", SYMBOL(SYM_GPS_LAT), (long)dec_portion,(long)frac_portion);
}

void AP_OSD_Screen::draw_gps_longitude(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    const Location &loc = gps.location();   // loc.lat and loc.lng
    int32_t dec_portion, frac_portion;
    int32_t abs_lon = labs(loc.lng);

    dec_portion = loc.lng / 10000000L;
    frac_portion = abs_lon - labs(dec_portion)*10000000UL;

    backend->write(x, y, false, "%c%4ld.%07ld", SYMBOL(SYM_GPS_LONG), (long)dec_portion,(long)frac_portion);
}

void AP_OSD_Screen::draw_roll_angle(uint8_t x, uint8_t y)
{
    float roll;
    float pitch;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        AP::vehicle()->get_osd_roll_pitch_rad(roll, pitch);
    }
    roll = ToDeg(roll);
    const bool one_decimal = osd->options & AP_OSD::OPTION_ONE_DECIMAL_ATTITUDE;
    const float level_symbol_max_angle = one_decimal ? 0.049f : 0.45f;
    const float roll_abs = abs(roll);
    char r;
    if (roll_abs > level_symbol_max_angle) {
        r = signbit(roll) ? SYMBOL(SYM_ROLLL) : SYMBOL(SYM_ROLLR);
    } else {
        r = SYMBOL(SYM_ROLL0);
    }
    if (one_decimal) {
        const char *format = roll_abs < 9.95 ? "%c  %.1f%c" : (roll_abs < 99.95 ? "%c %.1f%c" : "%c%.1f%c");
        backend->write(x, y, false, format, r, roll_abs, SYMBOL(SYM_DEGR));
    } else {
        const char *format = roll_abs < 9.95 ? "%c  %.0f%c" : (roll_abs < 99.95 ? "%c %.0f%c" : "%c%.0f%c");
        backend->write(x, y, false, format, r, roll_abs, SYMBOL(SYM_DEGR));
    }
}

void AP_OSD_Screen::draw_pitch(uint8_t x, uint8_t y, float pitch)
{
    const bool one_decimal = osd->options & AP_OSD::OPTION_ONE_DECIMAL_ATTITUDE;
    const float level_symbol_max_angle = one_decimal ? 0.049f : 0.45f;
    const float pitch_abs = abs(pitch);
    char p;
    if (pitch_abs > level_symbol_max_angle) {
        p = signbit(pitch) ? SYMBOL(SYM_PTCHDWN) : SYMBOL(SYM_PTCHUP);
    } else {
        p = SYMBOL(SYM_PTCH0);
    }
    if (one_decimal) {
        const char *format = pitch_abs < 9.95 ? "%c %.1f%c" : "%c%.1f%c";
        backend->write(x, y, false, format, p, pitch_abs, SYMBOL(SYM_DEGR));
    } else {
        const char *format = pitch_abs < 9.5 ? "%c %.0f%c" : "%c%.0f%c";
        backend->write(x, y, false, format, p, pitch_abs, SYMBOL(SYM_DEGR));
    }
}

void AP_OSD_Screen::draw_pitch_angle(uint8_t x, uint8_t y)
{
    float roll;
    float pitch;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        AP::vehicle()->get_osd_roll_pitch_rad(roll, pitch);
    }
    draw_pitch(x, y, ToDeg(pitch));
}

void AP_OSD_Screen::draw_acc_value(uint8_t x, uint8_t y, float value, bool blink, bool can_be_negative)
{
    const char *format;
    uint8_t spaces;
    const float value_abs = fabsf(value);
    if (value_abs < 9.95) {
        spaces = 2;
        format = "%1.1f%c";
    } else {
        spaces = 1;
        format = "%2.1f%c";
    }

    if (!can_be_negative || signbit(value)) {
        spaces -= 1;
    }

    backend->write(x + spaces, y, false, format, value, SYMBOL(SYM_G));
}

// helper to draw accelerator for vertical and lateral accelerations
void AP_OSD_Screen::draw_acc_vert_lat(uint8_t x, uint8_t y, float acc, uint8_t neg_symbol, uint8_t zero_symbol, uint8_t pos_symbol, float warn)
{
    const float acc_abs = fabsf(acc);
    const uint8_t symbol = SYMBOL(acc_abs < 0.05 ? zero_symbol : (signbit(acc) ? neg_symbol : pos_symbol));
    backend->write(x, y, false, "%c", symbol);
    const bool blink = is_positive(warn) && acc_abs > warn;
    draw_acc_value(x+1, y, acc_abs, blink, false);
}

void AP_OSD_Screen::draw_acc_long(uint8_t x, uint8_t y) {
    float rotMat_c_x;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
        rotMat_c_x = rotMat.c.x;
    }
    const float acc = _acc_long_filter.apply(rotMat_c_x * GRAVITY_MSS + AP::ins().get_accel().x) / GRAVITY_MSS;
    draw_acc_value(x, y, acc, false, true);
}

void AP_OSD_Screen::draw_tx_power(uint8_t x, uint8_t y, int16_t value, bool blink)
{
    if (value > 0) {
        if (value < 1000) {
            backend->write(x, y, false, "%3d%c", value, SYMBOL(SYM_MW));
        } else {
            const float value_w = float(value) * 0.001f;
            backend->write(x, y, false, "%.2f%c", value_w, SYMBOL(SYM_WATT));
        }
    } else {
        backend->write(x, y, false, "---%c", SYMBOL(SYM_MW));
    }
}

void AP_OSD_Screen::draw_crsf_tx_power(uint8_t x, uint8_t y)
{
    const int16_t tx_power = AP::crsf()->get_link_status().tx_power;
    draw_tx_power(x, y, tx_power);
}

void AP_OSD_Screen::draw_rssi_dbm(uint8_t x, uint8_t y, int8_t value, bool blink)
{
    if (value >= 0) {
        backend->write(x, y, blink, "%4d%c", -value, SYMBOL(SYM_DBM));
    } else {
        backend->write(x, y, blink, "----%c", SYMBOL(SYM_DBM));
    }
}

void AP_OSD_Screen::draw_crsf_rssi_dbm(uint8_t x, uint8_t y)
{
    const int8_t rssidbm = AP::crsf()->get_link_status().rssi_dbm;
    const bool blink = rssidbm > osd->warn_rssi;
    backend->write(x, y, blink, "%c", SYMBOL(SYM_RSSI));
    draw_rssi_dbm(x+1, y, rssidbm, blink);
}

void AP_OSD_Screen::draw_crsf_snr(uint8_t x, uint8_t y)
{
    const int8_t snr = AP::crsf()->get_link_status().snr;
    if (snr == INT8_MIN) {
        backend->write(x, y, false, "%c---%c", SYMBOL(SYM_SNR), SYMBOL(SYM_DB));
    } else {
        backend->write(x, y, false, "%c%3d%c", SYMBOL(SYM_SNR), snr, SYMBOL(SYM_DB));
    }
}

void AP_OSD_Screen::draw_crsf_active_antenna(uint8_t x, uint8_t y)
{
    const int8_t active_antenna = AP::crsf()->get_link_status().active_antenna;
    if (active_antenna < 0) {
        backend->write(x, y, false, "%c-", SYMBOL(SYM_ANT));
    } else {
        backend->write(x, y, false, "%c%d", SYMBOL(SYM_ANT), active_antenna + 1);
    }
}

void AP_OSD_Screen::draw_acc_lat(uint8_t x, uint8_t y) {
    float rotMat_c_y;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
        rotMat_c_y = rotMat.c.y;
    }
    const float acc = _acc_lat_filter.apply(rotMat_c_y * GRAVITY_MSS + AP::ins().get_accel().y) / GRAVITY_MSS;
    draw_acc_vert_lat(x, y, acc, SYM_ARROW_LEFT, SYM_ROLL0, SYM_ARROW_RIGHT, 0);
}

void AP_OSD_Screen::draw_acc_vert(uint8_t x, uint8_t y) {
    float rotMat_c_z;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
        rotMat_c_z = rotMat.c.z;
    }
    const float acc = _acc_vert_filter.apply(rotMat_c_z * GRAVITY_MSS + AP::ins().get_accel().z) / GRAVITY_MSS;
    draw_acc_vert_lat(x, y, acc, SYM_PTCHUP, SYM_PTCH0, SYM_PTCHDWN, osd->warn_vert_acc);
}

void AP_OSD_Screen::draw_temp(uint8_t x, uint8_t y)
{
    AP_Baro &barometer = AP::baro();
    float tmp = barometer.get_temperature();
    backend->write(x, y, false, "%3d%c", (int)u_scale(TEMPERATURE, tmp), u_icon(TEMPERATURE));
}

void AP_OSD_Screen::draw_hdop(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    float hdp = gps.get_hdop() * 0.01f;
    backend->write(x, y, false, "%c%c%3.2f", SYMBOL(SYM_HDOP_L), SYMBOL(SYM_HDOP_R), hdp);
}

void AP_OSD_Screen::draw_waypoint(uint8_t x, uint8_t y)
{
    int32_t yaw_sensor;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        yaw_sensor = ahrs.yaw_sensor;
    }
    int32_t angle = wrap_360_cd(osd->nav_info.wp_bearing - yaw_sensor);
    int32_t interval = 36000 / SYMBOL(SYM_ARROW_COUNT);
    if (osd->nav_info.wp_distance < 2.0f) {
        //avoid fast rotating arrow at small distances
        angle = 0;
    }
    char arrow = SYMBOL(SYM_ARROW_START) + ((angle + interval / 2) / interval) % SYMBOL(SYM_ARROW_COUNT);
    backend->write(x,y, false, "%c%2u%c",SYMBOL(SYM_WPNO), osd->nav_info.wp_number, arrow);
    draw_distance(x+4, y, osd->nav_info.wp_distance, true);
}

void AP_OSD_Screen::draw_xtrack_error(uint8_t x, uint8_t y)
{
    backend->write(x, y, false, "%c", SYMBOL(SYM_XERR));
    draw_distance(x+1, y, osd->nav_info.wp_xtrack_error, false);
}

void AP_OSD_Screen::draw_stats(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP::battery();
    const auto ap_stats = AP::stats();
    WITH_SEMAPHORE(ap_stats->get_semaphore());
    const bool have_stats = ap_stats->available();
    const uint8_t col_offset = 11;

    backend->write(x, y, false, "MIN BAV");
    draw_voltage(x+col_offset, y, ap_stats->get_boot_min_voltage_v(), false, false, false, have_stats);
    float cell_voltage;
    const bool cell_voltage_is_available = battery.cell_avg_voltage(cell_voltage);
    if (cell_voltage_is_available) {
        backend->write(x+7, y, false, "/BCV");
        backend->write(x+col_offset+5, y, false, "/");
        draw_voltage(x+col_offset+6, y, ap_stats->get_boot_min_cell_voltage_v(), true, false, false, have_stats);
    }

    y += 1;
    backend->write(x, y, false, "AVG CUR/POW");
    draw_current(x+col_offset+1, y, have_stats, false, ap_stats->get_boot_avg_flying_current_a());
    backend->write(x+col_offset+5, y, false, "/");
    draw_power(x+col_offset+6, y, have_stats, false, ap_stats->get_boot_avg_flying_power_w());

    y += 1;
    backend->write(x, y, false, "MAX CUR/POW");
    draw_current(x+col_offset+1, y, have_stats, false, ap_stats->get_boot_max_flying_current_a());
    backend->write(x+col_offset+5, y, false, "/");
    draw_power(x+col_offset+6, y, have_stats, false, ap_stats->get_boot_max_flying_power_w());

    y += 1;
    backend->write(x, y, false, "USD CAPA");
    draw_mah(x+col_offset, y, have_stats && ap_stats->mah_is_available(), false, ap_stats->get_boot_flying_mah());
    backend->write(x+col_offset+5, y, false, "/");
    draw_energy(x+col_offset+6, y, have_stats && ap_stats->energy_is_available(), false, ap_stats->get_boot_flying_energy_wh());

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    y += 1;
    backend->write(x, y, false, "AVG %c/%c/%c", SYMBOL(SYM_ASPD), SYMBOL(SYM_GSPD), SYMBOL(SYM_WSPD));
    draw_speed(x+col_offset+1, y, ap_stats->avg_air_speed_is_available(), ap_stats->get_boot_avg_air_speed_mps());
    backend->write(x+col_offset+5, y, false, "/");
    draw_speed(x+col_offset+6, y, ap_stats->avg_ground_speed_is_available(), ap_stats->get_boot_avg_ground_speed_mps());
    backend->write(x+col_offset+6+4, y, false, "/");
    draw_speed(x+col_offset+6+5, y, have_stats, ap_stats->get_boot_avg_wind_speed_mps());

    y += 1;
    backend->write(x, y, false, "MAX %c/%c/%c", SYMBOL(SYM_ASPD), SYMBOL(SYM_GSPD), SYMBOL(SYM_WSPD));
    draw_speed(x+col_offset+1, y, have_stats, ap_stats->get_boot_max_air_speed_mps());
    backend->write(x+col_offset+5, y, false, "/");
    draw_speed(x+col_offset+6, y, have_stats, ap_stats->get_boot_max_ground_speed_mps());
    backend->write(x+col_offset+6+4, y, false, "/");
    draw_speed(x+col_offset+6+5, y, have_stats, ap_stats->get_boot_max_wind_speed_mps());

    y += 1;
    backend->write(x, y, false, "EFF A/G");
    draw_avg_eff_air(x+col_offset+1, y, false);
    backend->write(x+col_offset+5, y, false, "/");
    draw_avg_eff_ground(x+col_offset+6, y, false);

    y += 1;
    backend->write(x, y, false, "TRV A/G");
    draw_distance(x+col_offset, y, ap_stats->get_boot_flying_air_traveled_m(), true, have_stats);
    backend->write(x+col_offset+5, y, false, "/");
    draw_distance(x+col_offset+6, y, ap_stats->get_boot_flying_ground_traveled_m(), true, have_stats);
#else // APM_BUILD_TYPE(APM_BUILD_ArduPlane)

    y += 1;
    backend->write(x, y, false, "AVG %c/%c", SYMBOL(SYM_GSPD), SYMBOL(SYM_WSPD));
    draw_speed(x+col_offset+1, y, ap_stats->avg_ground_speed_is_available(), ap_stats->get_boot_avg_ground_speed_mps());
    backend->write(x+col_offset+5, y, false, "/");
    draw_speed(x+col_offset+6, y, have_stats, ap_stats->get_boot_avg_wind_speed_mps());

    y += 1;
    backend->write(x, y, false, "MAX %c/%c", SYMBOL(SYM_GSPD), SYMBOL(SYM_WSPD));
    draw_speed(x+col_offset+1, y, have_stats, ap_stats->get_boot_max_ground_speed_mps());
    backend->write(x+col_offset+5, y, false, "/");
    draw_speed(x+col_offset+6, y, have_stats, ap_stats->get_boot_max_wind_speed_mps());

    y += 1;
    backend->write(x, y, false, "EFF");
    draw_avg_eff_ground(x+col_offset+1, y, false);

    y += 1;
    backend->write(x, y, false, "TRV");
    draw_distance(x+col_offset, y, ap_stats->get_boot_flying_ground_traveled_m(), true, have_stats);

#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)

    y += 1;
    backend->write(x, y, false, "DST AG/MX");
    draw_distance(x+col_offset, y, ap_stats->get_boot_avg_home_distance_m(), true, have_stats);
    backend->write(x+col_offset+5, y, false, "/");
    draw_distance(x+col_offset+5+1, y, ap_stats->get_boot_max_home_distance_m(), true, have_stats);

    y += 1;
    backend->write(x, y, false, "MAX ALT");
    draw_altitude(x+col_offset, y, false, ap_stats->get_boot_max_relative_altitude_m(), have_stats);

    y += 1;
    backend->write(x, y, false, "FLT TIME");
    uint32_t flight_time_s = ap_stats->get_boot_flying_time_s();
    if (!flight_time_s) {
        backend->write(x+col_offset+1, y, false, "---:--");
    } else {
        backend->write(x+col_offset+1, y, false, "%3u:%02u", unsigned(flight_time_s/60), unsigned(flight_time_s%60));
    }

    if (AP::rssi()->enabled()) {
        y += 1;
        backend->write(x, y, false, "MIN RSSI");
        uint8_t value_col_offset = col_offset + 2;
        if (have_stats) {
            backend->write(x+value_col_offset, y, false, "%2d", MAX(0, (int)lrintf(ap_stats->get_boot_min_rc_rssi() * 99)));
            value_col_offset += 3;
            if (ap_stats->min_rc_rssi_dbm_is_available()) {
                backend->write(x+value_col_offset, y, false, "/");
                draw_rssi_dbm(x+value_col_offset+1, y, ap_stats->get_boot_min_rc_rssi_dbm());
                value_col_offset += 1 + 5;
            }
            if (ap_stats->max_rc_tx_power_is_available()) {
                backend->write(x+value_col_offset, y, false, "/");
                draw_tx_power(x+value_col_offset+1, y, ap_stats->get_boot_max_rc_tx_power_mw());
            }
        } else {
            backend->write(x+value_col_offset, y, false, "--");
            value_col_offset += 3;
            const int8_t rssi_dbm = AP::crsf()->get_link_status().rssi_dbm;
            if (rssi_dbm >= 0) {
                backend->write(x+value_col_offset, y, false, "/");
                draw_rssi_dbm(x+value_col_offset+1, y, -1);
                value_col_offset += 1 + 5;
            }
            const int16_t tx_power = AP::crsf()->get_link_status().tx_power;
            if (tx_power > 0) {
                backend->write(x+value_col_offset, y, false, "/");
                draw_tx_power(x+value_col_offset+1, y, 0);
            }
        }
    }

    if (ap_stats->esc_temperatures_are_available()) {
        y += 1;
        backend->write(x, y, false, "MAX/AVG ESCT");
        draw_temperature(x+col_offset+1, y, have_stats, ap_stats->get_boot_max_esc_temperature_degc(), false);
        backend->write(x+col_offset+1+4, y, false, "/");
        draw_temperature(x+col_offset+1+4+1, y, have_stats, ap_stats->get_boot_avg_esc_temperature_degc(), false);
    }

}

bool AP_OSD_Screen::has_tuned_param_changed()
{
    static uint32_t last_changed;
    static float last_value;
    static const AP_Float *last_param_pointer = nullptr;
    const uint32_t now = AP_HAL::millis();
    const auto tuning_object = AP::vehicle()->get_tuning_object();

    if (tuning_object == nullptr) {
        return false;
    }

    const AP_Float *param_pointer = tuning_object->get_param_pointer();

    if (param_pointer != last_param_pointer) {
        last_param_pointer = param_pointer;
        last_changed = now;
        return true;
    } else if (param_pointer != nullptr) {
        const float value = param_pointer->get();
        if (abs(value - last_value) > FLT_EPSILON) {
            last_value = value;
            last_changed = now;
            return true;
        }
    }

    return now - last_changed < osd->tune_display_timeout * 1000;
}

void AP_OSD_Screen::draw_tuned_param_name(uint8_t x, uint8_t y)
{
    if (has_tuned_param_changed()) {
        const auto tuning_object = AP::vehicle()->get_tuning_object();

        const char *tuned_name = tuning_object->get_tuning_name();
        char buffer[AP_OSD::max_tuned_pn_display_len+1];
        strncpy(buffer, tuned_name, sizeof(buffer)-1);
        const int16_t name_len = strnlen(buffer, sizeof(buffer));

        for (int16_t i = 0; i < name_len; ++i) {
            //converted to uppercase,
            //because we do not have small letter chars inside used font
            buffer[i] = toupper(buffer[i]);
            //normalize whitespace
            if (isspace(buffer[i])) {
                buffer[i] = ' ';
            }
        }

        buffer[sizeof(buffer)-1] = '\0';

        const uint8_t spaces = check_option(AP_OSD::OPTION_RIGHT_JUSTIFY_TUNED_PN) ? AP_OSD::max_tuned_pn_display_len - name_len : 0;

        backend->write(x + spaces, y, false, "%s", buffer);
    } else if (!AP_Notify::flags.armed) {
        for (int i = 0; i < AP_OSD::max_tuned_pn_display_len; ++i) {
            backend->write(x + i, y, false, "-");
        }
    }
}

void AP_OSD_Screen::draw_tuned_param_value(uint8_t x, uint8_t y)
{
    if (has_tuned_param_changed()) {
        const auto tuning_object = AP::vehicle()->get_tuning_object();
        const AP_Float *const param_value_ptr = tuning_object->get_param_pointer();
        if (param_value_ptr != nullptr) {
            const float value = param_value_ptr->get();
            const char* const fmt = (value < 9.9995 ? "%1.3f" : (value < 99.995 ? "%2.2f" : (value < 999.95 ? "%3.1f" : "%4.0f")));
            const uint8_t spaces = signbit(value) ? 0 : 1;
            backend->write(x+spaces, y, false, fmt, value);
        }
    } else if (!AP_Notify::flags.armed) {
        backend->write(x, y, false, "-----");
    }
}

void AP_OSD_Screen::draw_traveled_ground_distance(uint8_t x, uint8_t y)
{
    backend->write(x, y, false, "%c", SYMBOL(SYM_DIST));
    const auto ap_stats = AP::stats();
    WITH_SEMAPHORE(ap_stats->get_semaphore());
    draw_distance(x+1, y, ap_stats->get_boot_flying_ground_traveled_m(), true);
}

void  AP_OSD_Screen::draw_flightime(uint8_t x, uint8_t y)
{
    const auto ap_stats = AP::stats();
    WITH_SEMAPHORE(ap_stats->get_semaphore());
    if (ap_stats) {
        uint32_t t = ap_stats->get_boot_flying_time_s();
        backend->write(x, y, false, "%c%3u:%02u", SYMBOL(SYM_FLY), unsigned(t/60), unsigned(t%60));
    }
}

void AP_OSD_Screen::draw_eff_mah(uint8_t x, uint8_t y, uint16_t value)
{
    backend->write(x, y, false, "%c%3d%c", SYMBOL(SYM_EFF), value, SYMBOL(SYM_MAH));
}

void AP_OSD_Screen::draw_eff_wh(uint8_t x, uint8_t y, float value)
{
    const char* const fmt = (value < 9.995 ? "%c%1.2f%c" : (value < 99.95 ? "%c%2.1f%c" : "%c%3.0f%c"));
    backend->write(x, y, false, fmt, SYMBOL(SYM_EFF), value, SYMBOL(SYM_WH));
}

// returns false if the efficiency is not available (current or power not available or result not finite)
bool AP_OSD_Screen::calculate_efficiency(float speed, float &efficiency)
{
    AP_BattMonitor &battery = AP::battery();
    speed = u_scale(SPEED, speed);
    int8_t eff_unit_base = osd->efficiency_unit_base;
    if (eff_unit_base == AP_OSD::EFF_UNIT_BASE_MAH) {
        float current_amps;
        if (!battery.current_amps(current_amps) || is_negative(current_amps)) return false;
        efficiency = 1000.0f * current_amps / speed;
    } else {
        float power_w;
        if (!battery.power_watts_without_losses(power_w) || is_negative(power_w)) return false;
        efficiency = power_w / speed;
    }
    return isfinite(efficiency);
}

void AP_OSD_Screen::draw_eff(uint8_t x, uint8_t y, bool available, float efficiency)
{
    int8_t eff_unit_base = osd->efficiency_unit_base;
    if (!available) goto invalid;
    if (eff_unit_base == AP_OSD::EFF_UNIT_BASE_MAH) {
        const uint16_t efficiency_int = roundf(efficiency);
        if (efficiency_int > 999) goto invalid;
        draw_eff_mah(x, y, efficiency_int);
    } else {
        if (!isfinite(efficiency) || roundf(efficiency) > 999) goto invalid;
        draw_eff_wh(x, y, efficiency);
    }
    return;
invalid:
    const uint8_t unit_symbol = SYMBOL(eff_unit_base == AP_OSD::EFF_UNIT_BASE_MAH ? SYM_MAH : SYM_WH);
    backend->write(x, y, false, "%c---%c", SYMBOL(SYM_EFF), unit_symbol);
}

void AP_OSD_Screen::draw_eff_ground(uint8_t x, uint8_t y)
{
    Vector2f ground_speed_vector;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        ground_speed_vector = ahrs.groundspeed_vector();
    }
    const float ground_speed = ground_speed_vector.length();
    float efficiency;
    const bool efficiency_available = calculate_efficiency(ground_speed, efficiency);
    if (efficiency_available) {
        osd->filtered.eff_ground += (efficiency - osd->filtered.eff_ground) * 0.2f;
    }
    draw_eff(x, y, efficiency_available, osd->filtered.eff_ground);
}

void AP_OSD_Screen::draw_avg_eff(uint8_t x, uint8_t y, const float distance_travelled_m, const bool draw_eff_symbol)
{
    const int8_t eff_unit_base = osd->efficiency_unit_base;

    uint8_t value_offset = 0;
    if (draw_eff_symbol) {
        backend->write(x, y, false, "%c", SYMBOL(SYM_EFF));
        value_offset = 1;
    }

    const auto ap_stats = AP::stats();
    WITH_SEMAPHORE(ap_stats->get_semaphore());
    const float distance_travelled_km = distance_travelled_m * 0.001f;
    if (is_positive(distance_travelled_km)) {
        if (eff_unit_base == AP_OSD::EFF_UNIT_BASE_MAH) {
            if (!ap_stats->mah_is_available()) goto invalid;
            const uint16_t efficiency = roundf(ap_stats->get_boot_flying_mah() / distance_travelled_km);
            if (efficiency > 999) goto invalid;
            backend->write(x+value_offset, y, false, "%3d%c", efficiency, SYMBOL(SYM_MAH));
        } else {
            if (!ap_stats->energy_is_available()) goto invalid;
            const float efficiency = ap_stats->get_boot_flying_energy_wh() / distance_travelled_km;
            if (!isfinite(efficiency) || roundf(efficiency) > 999) goto invalid;
            const char* const fmt = (efficiency < 9.995 ? "%1.2f%c" : (efficiency < 99.95 ? "%2.1f%c" : "%3.0f%c"));
            backend->write(x+value_offset, y, false, fmt, efficiency, SYMBOL(SYM_WH));
        }
        return;
    }
invalid:
    const uint8_t unit_symbol = SYMBOL(eff_unit_base == AP_OSD::EFF_UNIT_BASE_MAH ? SYM_MAH : SYM_WH);
    backend->write(x+value_offset, y, false, "---%c", unit_symbol);
}

void AP_OSD_Screen::draw_avg_eff_ground(uint8_t x, uint8_t y, bool draw_eff_symbol)
{
    const auto ap_stats = AP::stats();
    WITH_SEMAPHORE(ap_stats->get_semaphore());
    draw_avg_eff(x, y, ap_stats->get_boot_flying_ground_traveled_m(), draw_eff_symbol);
}

void AP_OSD_Screen::draw_climbeff(uint8_t x, uint8_t y)
{
    char unit_icon = u_icon(DISTANCE);
    Vector3f v;
    float vspd;
    do {
        {
            auto &ahrs = AP::ahrs();
            WITH_SEMAPHORE(ahrs.get_semaphore());
            if (ahrs.get_velocity_NED(v)) {
                vspd = -v.z;
                break;
            }
        }
        auto &baro = AP::baro();
        WITH_SEMAPHORE(baro.get_semaphore());
        vspd = baro.get_climb_rate();
    } while (false);
    if (is_positive(vspd)) {
        AP_BattMonitor &battery = AP::battery();
        float efficiency;
        if (osd->efficiency_unit_base == AP_OSD::EFF_UNIT_BASE_MAH) {
            float amps;
            if (!battery.current_amps(amps) || !is_positive(amps)) goto invalid;
            efficiency = 3600.0f * vspd / amps;
        } else {
            float power_w;
            if (!battery.power_watts(power_w) || !is_positive(power_w)) goto invalid;
            efficiency = 3600.0f * vspd / power_w;
        }
        osd->filtered.eff_climb += (efficiency - osd->filtered.eff_climb) * 0.2f;
        if (osd->filtered.eff_climb < 1000) {
            backend->write(x, y, false,"%c%c%3u%c", SYMBOL(SYM_PTCHUP), SYMBOL(SYM_EFF), (uint16_t)lrintf(osd->filtered.eff_climb), unit_icon);
            return;
        }
    }
invalid:
    backend->write(x, y, false, "%c%c---%c", SYMBOL(SYM_PTCHUP), SYMBOL(SYM_EFF), unit_icon);
}

#if BARO_MAX_INSTANCES > 1
void AP_OSD_Screen::draw_btemp(uint8_t x, uint8_t y)
{
    AP_Baro &barometer = AP::baro();
    float btmp = barometer.get_temperature(1);
    backend->write(x, y, false, "%3d%c", (int)u_scale(TEMPERATURE, btmp), u_icon(TEMPERATURE));
}
#endif

void AP_OSD_Screen::draw_atemp(uint8_t x, uint8_t y)
{
#if AP_AIRSPEED_ENABLED
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (!airspeed) {
        return;
    }
    float temperature = 0;
    airspeed->get_temperature(temperature);
    if (airspeed->healthy()) {
        backend->write(x, y, false, "%3d%c", (int)u_scale(TEMPERATURE, temperature), u_icon(TEMPERATURE));
    } else {
        backend->write(x, y, false, "--%c", u_icon(TEMPERATURE));
    }
#endif
}

void AP_OSD_Screen::draw_bat2_vlt(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP::battery();
    uint8_t pct2 = 0;
    float v2 = battery.voltage(1);
    const bool blink = battery.voltage_is_low(1);
    if (!battery.capacity_remaining_pct(pct2, 1)) {
        // Do not show battery percentage
        backend->write(x+1,y, blink, "%2.1f%c", v2, SYMBOL(SYM_VOLT));
        return;
    }
    uint8_t p2 = (100 - pct2) / 16.6;
    backend->write(x,y, blink, "%c%2.1f%c", SYMBOL(SYM_BATT_FULL) + p2, v2, SYMBOL(SYM_VOLT));
}

void AP_OSD_Screen::draw_bat2used(uint8_t x, uint8_t y)
{
    draw_batused(x, y, 1);
}

void AP_OSD_Screen::draw_bat2rem(uint8_t x, uint8_t y)
{
    draw_batrem(x, y, 1);
}

void AP_OSD_Screen::draw_clk(uint8_t x, uint8_t y)
{
    AP_RTC &rtc = AP::rtc();
    uint8_t hour, min, sec;
    uint16_t ms;
    if (!rtc.get_local_time(hour, min, sec, ms)) {
        backend->write(x, y, false, "%c--:--", SYMBOL(SYM_CLK));
    } else {
        backend->write(x, y, false, "%c%02u:%02u", SYMBOL(SYM_CLK), hour, min);
    }
}

#if HAL_PLUSCODE_ENABLE
void AP_OSD_Screen::draw_pluscode(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    const Location &loc = gps.location();
    char buff[16];
    if (gps.status() == AP_GPS::NO_GPS || gps.status() == AP_GPS::NO_FIX) {
        backend->write(x, y, false, check_option(AP_OSD::OPTION_SHORTEN_PLUSCODE) ? "----+---" : "--------+---");
    } else {
        AP_OLC::olc_encode(loc.lat, loc.lng, 11, check_option(AP_OSD::OPTION_SHORTEN_PLUSCODE), buff, sizeof(buff));
        backend->write(x, y, false, "%s", buff);
    }
}
#endif

/*
  support callsign display from a file called callsign.txt
 */
void AP_OSD_Screen::draw_callsign(uint8_t x, uint8_t y)
{
#if HAVE_FILESYSTEM_SUPPORT
    if (!callsign_data.load_attempted) {
        callsign_data.load_attempted = true;
        FileData *fd = AP::FS().load_file("callsign.txt");
        if (fd != nullptr) {
            uint32_t len = fd->length;
            // trim off whitespace
            while (len > 0 && isspace(fd->data[len-1])) {
                len--;
            }
            callsign_data.str = strndup((const char *)fd->data, len);
            delete fd;
        }
    }
    if (callsign_data.str != nullptr) {
        backend->write(x, y, false, "%s", callsign_data.str);
    }
#endif
}

void AP_OSD_Screen::draw_current2(uint8_t x, uint8_t y)
{
    draw_current(1, x, y);
}

void AP_OSD_Screen::draw_vtx_power(uint8_t x, uint8_t y)
{
    AP_VideoTX *vtx = AP_VideoTX::get_singleton();
    if (!vtx) {
        return;
    }
    uint16_t powr = 0;
    // If currently in pit mode, just render 0mW to the screen
    if(!vtx->has_option(AP_VideoTX::VideoOptions::VTX_PITMODE)){
        powr = vtx->get_power_mw();
    }
    backend->write(x, y, !vtx->is_configuration_finished(), "%4hu%c", powr, SYMBOL(SYM_MW));
}
#if AP_TERRAIN_AVAILABLE
void AP_OSD_Screen::draw_hgt_abvterr(uint8_t x, uint8_t y)
{
    AP_Terrain *terrain = AP::terrain();

    float terrain_altitude;
    if (terrain != nullptr && terrain->height_above_terrain(terrain_altitude,true)) {
        bool blink = (osd->warn_terr != -1)? (terrain_altitude < osd->warn_terr) : false; //blink if warn_terr is not disabled and alt above terrain is below warning value
        backend->write(x, y, blink, "%4d%c%c", (int)u_scale(ALTITUDE, terrain_altitude), u_icon(ALTITUDE), SYMBOL(SYM_TERALT));
     } else {
        backend->write(x, y, false, " ---%c%c", u_icon(ALTITUDE),SYMBOL(SYM_TERALT));
     }
}
#endif

#if AP_FENCE_ENABLED
void AP_OSD_Screen::draw_fence(uint8_t x, uint8_t y)
{
    AC_Fence *fenceptr = AP::fence();
    if (fenceptr == nullptr) {
       return;
    }
    if (fenceptr->enabled() && fenceptr->present()) {
        backend->write(x, y, fenceptr->get_breaches(), "%c", SYMBOL(SYM_FENCE_ENABLED));
    } else {
        backend->write(x, y, false, "%c", SYMBOL(SYM_FENCE_DISABLED));
    }
}
#endif

void AP_OSD_Screen::draw_rngf(uint8_t x, uint8_t y)
{
    RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder == nullptr) {
       return;
    }
    if (rangefinder->status_orient(ROTATION_PITCH_270) < RangeFinder::Status::Good) {
        backend->write(x, y, false, "----%c%c", u_icon(ALTITUDE), SYMBOL(SYM_RNGFD));
    } else {
        float attitude_angle;
        {
            AP_AHRS &ahrs = AP::ahrs();
            WITH_SEMAPHORE(ahrs.get_semaphore());
            attitude_angle = ahrs.get_rotation_body_to_ned().c.z;
        }
        const float distance = (rangefinder->distance_orient(ROTATION_PITCH_270) - rangefinder->ground_clearance_cm_orient(ROTATION_PITCH_270) * 0.01f) * attitude_angle;
        const float distance_abs = abs(distance);

        uint8_t spaces;
        const char *format;
        if (distance_abs < 9.995) {
            spaces = 1;
            format = "%1.2f%c%c";
        } else if (distance_abs < 99.95) {
            spaces = 0;
            format = "%2.2f%c%c";
        } else {
            spaces = 0;
            format = "%3.1f%c%c";
        }
        if (signbit(distance)) {
            spaces -= 1;
        }

        backend->write(x + spaces, y, false, format, u_scale(ALTITUDE, distance), u_icon(ALTITUDE), SYMBOL(SYM_RNGFD));
    }
}

void AP_OSD_Screen::draw_rc_throttle(uint8_t x, uint8_t y) {
    draw_throttle_value(x, y, AP::vehicle()->get_throttle_input(true));
}

void AP_OSD_Screen::draw_rc_failsafe(uint8_t x, uint8_t y)
{
    if (AP::vehicle()->rc_failsafe()) {
        backend->write(x, y, true, "!!! RC FS !!!");
        return;
    }

    if (!AP_Notify::flags.armed) {
        backend->write(x, y, false, "--- RC FS ---");
    }
}

#if OSD_DEBUG_ELEMENT
void AP_OSD_Screen::draw_debug(uint8_t x, uint8_t y)
{
    WITH_SEMAPHORE(osd->get_semaphore());
    backend->write(x, y, false, "%.1f", osd->_debug);
}
#endif

// Plane specific elements

void AP_OSD_Screen::draw_aspeed(uint8_t x, uint8_t y)
{
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    float aspd;
    bool have_estimate;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        have_estimate = ahrs.airspeed_estimate(aspd);
    }
    if (have_estimate) {
        const bool blink = (is_positive(osd->warn_aspd_low) && aspd < osd->warn_aspd_low) || (is_positive(osd->warn_aspd_high) && aspd > osd->warn_aspd_high);
        backend->write(x, y, blink, "%c", SYMBOL(SYM_ASPD));
        draw_speed(x+2, y, true, aspd, blink);
    } else {
        backend->write(x, y, false, "%c", SYMBOL(SYM_ASPD));
        draw_speed(x+2, y, false);
    }
#endif
}

void AP_OSD_Screen::draw_aspd_dem(uint8_t x, uint8_t y)
{
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    if (!AP_Notify::flags.armed) {
        backend->write(x, y, false, "%c", SYMBOL(SYM_ASPD));
        draw_speed(x+2, y, false);
    } else if (AP::vehicle()->control_mode_does_auto_throttle()) {
        const float aspd = AP::vehicle()->demanded_airspeed();
        const bool blink = (is_positive(osd->warn_aspd_low) && aspd < osd->warn_aspd_low) || (is_positive(osd->warn_aspd_high) && aspd > osd->warn_aspd_high);
        backend->write(x, y, blink, "%c", SYMBOL(SYM_ASPD));
        draw_speed(x+2, y, true, aspd, blink);
    }
#endif
}

void AP_OSD_Screen::draw_aspd1(uint8_t x, uint8_t y)
{
#if AP_AIRSPEED_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (!airspeed) {
        return;
    }
    float asp1 = airspeed->get_airspeed();
    const bool blink = (is_positive(osd->warn_aspd_low) && asp1 < osd->warn_aspd_low) || (is_positive(osd->warn_aspd_high) && asp1 > osd->warn_aspd_high);
    backend->write(x, y, blink, "%c", SYMBOL(SYM_ASPD));
    draw_speed(x+2, y, true, asp1, blink);
#endif
}

void AP_OSD_Screen::draw_aspd2(uint8_t x, uint8_t y)
{
#if AP_AIRSPEED_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (!airspeed) {
        return;
    }
    float asp2 = airspeed->get_airspeed();
    const bool blink = (is_positive(osd->warn_aspd_low) && asp2 < osd->warn_aspd_low) || (is_positive(osd->warn_aspd_high) && asp2 > osd->warn_aspd_high);
    backend->write(x, y, blink, "%c", SYMBOL(SYM_ASPD));
    draw_speed(x+2, y, true, asp2, blink);
#endif
}

void AP_OSD_Screen::draw_peak_roll_rate(uint8_t x, uint8_t y) {
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    static float last_max_roll_rate;
    static uint32_t last_peak_tstamp;
    float rate_x_abs;

    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        rate_x_abs = abs(ahrs.get_gyro().x);
    }

    uint32_t now = AP_HAL::millis();
    if ((rate_x_abs > last_max_roll_rate) || (now - last_peak_tstamp > osd->peak_rate_timeout * 1000)) {
        last_peak_tstamp = now;
        last_max_roll_rate = rate_x_abs;
    }

    backend->write(x, y, false, "%c%3u%c", SYMBOL(SYM_ROLL), (uint)lrintf(ToDeg(last_max_roll_rate)), SYMBOL(SYM_DPS));
#endif
}

void AP_OSD_Screen::draw_peak_pitch_rate(uint8_t x, uint8_t y) {
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    static float last_max_pitch_rate;
    static uint32_t last_peak_tstamp;
    float rate_y_abs;

    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        rate_y_abs = abs(ahrs.get_gyro().y);
    }

    uint32_t now = AP_HAL::millis();
    if ((rate_y_abs > last_max_pitch_rate) || (now - last_peak_tstamp > osd->peak_rate_timeout * 1000)) {
        last_peak_tstamp = now;
        last_max_pitch_rate = rate_y_abs;
    }

    backend->write(x, y, false, "%c%3u%c", SYMBOL(SYM_PITCH), (uint)lrintf(ToDeg(last_max_pitch_rate)), SYMBOL(SYM_DPS));
#endif
}

bool AP_OSD_Screen::cruise_heading_changed(uint16_t &locked_heading)
{
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    static uint32_t last_changed;
    static uint16_t last_value;
    const bool heading_locked = AP::vehicle()->get_cruise_locked_heading(locked_heading);

    if (!heading_locked) {
        last_changed = 0;
        return false;
    }

    const uint32_t now = AP_HAL::millis();
    if (locked_heading != last_value) {
        if (last_changed) last_value = locked_heading;
        last_changed = now;
    }
    if (!last_changed || now - last_changed > 2000) {
        return false;
    }

    return true;
#else
    return false;
#endif
}

void AP_OSD_Screen::draw_cruise_heading_adjustment(uint8_t x, uint8_t y)
{
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    if (!AP_Notify::flags.armed) {
        backend->write(x, y, false, "%c----%c", SYMBOL(SYM_HEADING), SYMBOL(SYM_DEGR));
        return;
    }

    static int16_t last_fixed = -1;
    uint16_t locked_heading;
    if (cruise_heading_changed(locked_heading) && last_fixed != -1) {
        int16_t heading_adj = locked_heading - last_fixed;
        if (heading_adj > 180) heading_adj -= 360;
        if (heading_adj < -180) heading_adj += 360;
        backend->write(x, y, false, "%c%4d%c", SYMBOL(SYM_HEADING), heading_adj, SYMBOL(SYM_DEGR));
    } else {
        last_fixed = locked_heading;
    }
#endif
}

void AP_OSD_Screen::draw_cruise_heading(uint8_t x, uint8_t y)
{
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    if (!AP_Notify::flags.armed) {
        backend->write(x, y, false, "%c---%c", SYMBOL(SYM_HEADING), SYMBOL(SYM_DEGR));
        return;
    }

    uint16_t locked_heading;
    if (cruise_heading_changed(locked_heading)) {
        backend->write(x, y, false, "%c%3d%c", SYMBOL(SYM_HEADING), locked_heading, SYMBOL(SYM_DEGR));
    }
#endif
}

void AP_OSD_Screen::draw_auto_flaps(uint8_t x, uint8_t y)
{
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    if (!AP_Notify::flags.armed) {
        backend->write(x, y, false, "---%c", SYMBOL(SYM_PCNT));
    } else if (AP::vehicle()->control_mode_does_auto_throttle()) {
        uint8_t flaps_pcnt = AP::vehicle()->auto_flap_percent();
        backend->write(x, y, false, "%3u%c", (uint)lrintf(flaps_pcnt), SYMBOL(SYM_PCNT));
    }
#endif
}

void AP_OSD_Screen::draw_aoa(uint8_t x, uint8_t y)
{
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    float aoa_val;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        aoa_val = ahrs.getAOA();
    }
    draw_pitch(x, y, aoa_val);
#endif
}

void AP_OSD_Screen::draw_eff_air(uint8_t x, uint8_t y)
{
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    bool have_airspeed_estimate;
    float airspeed_mps;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        have_airspeed_estimate = ahrs.airspeed_estimate(airspeed_mps);
    }
    float efficiency;
    bool efficiency_available;
    if (have_airspeed_estimate) {
        efficiency_available = calculate_efficiency(airspeed_mps, efficiency);
        if (efficiency_available) {
            osd->filtered.eff_air += (efficiency - osd->filtered.eff_air) * 0.2f;
        }
    }
    draw_eff(x, y, have_airspeed_estimate && efficiency_available, osd->filtered.eff_air);
#endif
}

void AP_OSD_Screen::draw_avg_eff_air(uint8_t x, uint8_t y, bool draw_eff_symbol)
{
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    const auto ap_stats = AP::stats();
    WITH_SEMAPHORE(ap_stats->get_semaphore());
    draw_avg_eff(x, y, ap_stats->get_boot_flying_air_traveled_m(), draw_eff_symbol);
#endif
}

// End plane specific elements

#define DRAW_SETTING(n) if (n.enabled) draw_ ## n(n.xpos, n.ypos)

#if HAL_WITH_OSD_BITMAP || HAL_WITH_MSP_DISPLAYPORT
void AP_OSD_Screen::draw(void)
{
    if (!enabled || !backend) {
        return;
    }
    //Note: draw order should be optimized.
    //Big and less important items should be drawn first,
    //so they will not overwrite more important ones.
#if HAL_OSD_SIDEBAR_ENABLE
    DRAW_SETTING(sidebars);
#endif

    DRAW_SETTING(message);
    DRAW_SETTING(horizon);
    DRAW_SETTING(compass);
    DRAW_SETTING(altitude);

#if AP_TERRAIN_AVAILABLE
    DRAW_SETTING(hgt_abvterr);
#endif

    DRAW_SETTING(rngf);
    DRAW_SETTING(waypoint);
    DRAW_SETTING(xtrack_error);
    DRAW_SETTING(bat_volt);
    DRAW_SETTING(restvolt);
    DRAW_SETTING(avgcellvolt);
    DRAW_SETTING(bat2_vlt);
    DRAW_SETTING(rssi);
    DRAW_SETTING(link_quality);
    DRAW_SETTING(current);
    DRAW_SETTING(power);
    DRAW_SETTING(energy_consumed);
    DRAW_SETTING(energy_remaining);
    DRAW_SETTING(batused);
    DRAW_SETTING(batrem);
    DRAW_SETTING(bat2used);
    DRAW_SETTING(bat2rem);
    DRAW_SETTING(sats);
    DRAW_SETTING(fltmode);
    DRAW_SETTING(gspeed);
    DRAW_SETTING(vspeed);
    DRAW_SETTING(throttle_output);
    DRAW_SETTING(heading);
    DRAW_SETTING(wind);
    DRAW_SETTING(home);
#if AP_FENCE_ENABLED
    DRAW_SETTING(fence);
#endif
    DRAW_SETTING(roll_angle);
    DRAW_SETTING(pitch_angle);
    DRAW_SETTING(acc_long);
    DRAW_SETTING(acc_lat);
    DRAW_SETTING(acc_vert);
    DRAW_SETTING(temp);
#if BARO_MAX_INSTANCES > 1
    DRAW_SETTING(btemp);
#endif
    DRAW_SETTING(atemp);
    DRAW_SETTING(hdop);
    DRAW_SETTING(flightime);
    DRAW_SETTING(clk);
    DRAW_SETTING(vtx_power);

#if HAL_WITH_ESC_TELEM
    DRAW_SETTING(highest_esc_temp);
    DRAW_SETTING(avg_esc_rpm);
    DRAW_SETTING(highest_esc_rpm);
    DRAW_SETTING(avg_esc_amps);
    DRAW_SETTING(highest_esc_amps);
    DRAW_SETTING(total_esc_amps);
#endif

    DRAW_SETTING(gps_latitude);
    DRAW_SETTING(gps_longitude);
#if HAL_PLUSCODE_ENABLE
    DRAW_SETTING(pluscode);
#endif
    DRAW_SETTING(traveled_ground_distance);
    DRAW_SETTING(stats);
    DRAW_SETTING(climbeff);
    DRAW_SETTING(eff_ground);
    DRAW_SETTING(avg_eff_ground);
    DRAW_SETTING(callsign);
    DRAW_SETTING(current2);
    DRAW_SETTING(rc_throttle);
    DRAW_SETTING(crsf_tx_power);
    DRAW_SETTING(crsf_rssi_dbm);
    DRAW_SETTING(crsf_snr);
    DRAW_SETTING(crsf_active_antenna);
    DRAW_SETTING(bat_pct);
    DRAW_SETTING(tuned_param_name);
    DRAW_SETTING(tuned_param_value);
    DRAW_SETTING(rc_failsafe);
#if OSD_DEBUG_ELEMENT
    DRAW_SETTING(debug);
#endif

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    DRAW_SETTING(aspeed);
    DRAW_SETTING(aspd1);
    DRAW_SETTING(aspd2);
    DRAW_SETTING(aspd_dem);
    DRAW_SETTING(peak_roll_rate);
    DRAW_SETTING(peak_pitch_rate);
    DRAW_SETTING(cruise_heading);
    DRAW_SETTING(cruise_heading_adjustment);
    DRAW_SETTING(auto_flaps);
    DRAW_SETTING(aoa);
    DRAW_SETTING(eff_air);
    DRAW_SETTING(avg_eff_air);
#endif
}
#endif
#endif // OSD_ENABLED
