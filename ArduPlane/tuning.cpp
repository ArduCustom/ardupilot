#include "Plane.h"

/*
  the vehicle class has its own var table for TUNE_PARAM so it can
  have separate parameter docs for the list of available parameters
 */
const AP_Param::GroupInfo AP_Tuning_Plane::var_info[] = {
    // @Param: PARAM
    // @DisplayName: Transmitter tuning parameter or set of parameters
    // @Description: This sets which parameter or set of parameters will be tuned. Values greater than 100 indicate a set of parameters rather than a single parameter. Parameters less than 50 are for QuadPlane vertical lift motors only.
    // @Values: 0:None,1:RateRollPI,2:RateRollP,3:RateRollI,4:RateRollD,5:RatePitchPI,6:RatePitchP,7:RatePitchI,8:RatePitchD,9:RateYawPI,10:RateYawP,11:RateYawI,12:RateYawD,13:AngleRollP,14:AnglePitchP,15:AngleYawP,16:PosXYP,17:PosZP,18:VelXYP,19:VelXYI,20:VelZP,21:AccelZP,22:AccelZI,23:AccelZD,24:RatePitchFF,25:RateRollFF,26:RateYawFF,50:FixedWingRollP,51:FixedWingRollI,52:FixedWingRollD,53:FixedWingRollFF,54:FixedWingPitchP,55:FixedWingPitchI,56:FixedWingPitchD,57:FixedWingPitchFF,58:TRIM_THROTTLE,59:TRIM_PITCH,60:KFF_THRAT2PTCH,61:FW STAB PITCH DOWN,62:FW STAB PITCH DOWN curve,63:RLL2PTCH,64:KFF_RDDRMIX,65:TECSTFFDAMP,66:TECSTFF_FILT,67:FWAglRollP,68:FWAglRollI,69:AglRollD,70:AglRollFLTT,71:FWAglPitchP,72:FWAglPitchI,73:AglPitchD,74:AglPitchFLTT,75:MixingDiff,76:MixingOffset,77:THR expo manual,78:THR expo auto,79:Flap retracted speed,80:Flap extended speed,81:Flap extended percent,82:KFF_THRAT2ELEV,83:KFF_FLAP2ELEV,84:Ailerons diff,85:Elevator diff,101:Set_RateRollPitch,102:Set_RateRoll,103:Set_RatePitch,104:Set_RateYaw,105:Set_AngleRollPitch,106:Set_VelXY,107:Set_AccelZ,108:Set_TRIM_THR_PTCH,109:Set turn coordination,110:TECSTHRFF,111:Set_AglRollPitch,112:Set_AglRoll,113:Set_AglPitch,114:Set_Mixing,115:Set_THRExpo,116:Set_flap
    // @User: Standard
    AP_GROUPINFO("PARAM", 1, AP_Tuning_Plane, parmset, 0),

    // the rest of the parameters are from AP_Tuning
    AP_NESTEDGROUPINFO(AP_Tuning, 0),

    AP_GROUPEND
};


/*
  tables of tuning sets
 */
const uint8_t AP_Tuning_Plane::tuning_set_rate_roll_pitch[] = { TUNING_RATE_ROLL_D, TUNING_RATE_ROLL_PI,
                                                                TUNING_RATE_PITCH_D, TUNING_RATE_PITCH_PI};
const uint8_t AP_Tuning_Plane::tuning_set_rate_roll[] =       { TUNING_RATE_ROLL_D, TUNING_RATE_ROLL_PI };
const uint8_t AP_Tuning_Plane::tuning_set_rate_pitch[] =      { TUNING_RATE_PITCH_D, TUNING_RATE_PITCH_PI };
const uint8_t AP_Tuning_Plane::tuning_set_rate_yaw[] =        { TUNING_RATE_YAW_P, TUNING_RATE_YAW_I, TUNING_RATE_YAW_D };
const uint8_t AP_Tuning_Plane::tuning_set_ang_roll_pitch[] =  { TUNING_ANG_ROLL_P, TUNING_ANG_PITCH_P };
const uint8_t AP_Tuning_Plane::tuning_set_vxy[] =             { TUNING_VXY_P, TUNING_VXY_I };
const uint8_t AP_Tuning_Plane::tuning_set_az[] =              { TUNING_AZ_P, TUNING_AZ_I, TUNING_AZ_D };
const uint8_t AP_Tuning_Plane::tuning_set_trim_thr_pitch[] =  { TUNING_TRIM_THROTTLE, TUNING_TRIM_PITCH, TUNING_KFF_THRAT2PTCH, TUNING_STAB_PITCH_DOWN, TUNING_STAB_PITCH_DCRV };
const uint8_t AP_Tuning_Plane::tuning_set_coordination[] =    { TUNING_RLL2PTCH, TUNING_KFF_RDDRMIX };
const uint8_t AP_Tuning_Plane::tuning_set_tecs_thr_ff[] =     { TUNING_TECS_THR_FF_DAMP, TUNING_TECS_THR_FF_FILT };
const uint8_t AP_Tuning_Plane::tuning_set_angle_roll_pitch[] = { TUNING_AGL_ROLL_P, TUNING_AGL_ROLL_D, TUNING_AGL_ROLL_FLTT,
                                                                 TUNING_AGL_PITCH_P, TUNING_AGL_PITCH_D, TUNING_AGL_PITCH_FLTT };
const uint8_t AP_Tuning_Plane::tuning_set_angle_roll[] =      { TUNING_AGL_ROLL_D, TUNING_AGL_ROLL_P, TUNING_AGL_ROLL_FLTT };
const uint8_t AP_Tuning_Plane::tuning_set_angle_pitch[] =     { TUNING_AGL_PITCH_D, TUNING_AGL_PITCH_P, TUNING_AGL_PITCH_FLTT };
const uint8_t AP_Tuning_Plane::tuning_set_mixing[] =          { TUNING_MIXING_DIFF, TUNING_AILERONS_DIFF, TUNING_ELEVATOR_DIFF, TUNING_MIXING_OFFSET, TUNING_KFF_THRAT2ELEV, TUNING_KFF_FLAP2ELEV };
const uint8_t AP_Tuning_Plane::tuning_set_thr_expo[] =        { TUNING_THR_EXPO_MANUAL, TUNING_THR_EXPO_AUTO };
const uint8_t AP_Tuning_Plane::tuning_set_flap[] =            { TUNING_FLAP_RETED_SPD, TUNING_FLAP_EXTED_SPD, TUNING_FLAP_EXTED_PCT };

// macro to prevent getting the array length wrong
#define TUNING_ARRAY(v) ARRAY_SIZE(v), v

// list of tuning sets
const AP_Tuning_Plane::tuning_set AP_Tuning_Plane::tuning_sets[] = {
    { TUNING_SET_RATE_ROLL_PITCH, TUNING_ARRAY(tuning_set_rate_roll_pitch) },
    { TUNING_SET_RATE_ROLL,       TUNING_ARRAY(tuning_set_rate_roll) },
    { TUNING_SET_RATE_PITCH,      TUNING_ARRAY(tuning_set_rate_pitch) },
    { TUNING_SET_RATE_YAW,        TUNING_ARRAY(tuning_set_rate_yaw) },
    { TUNING_SET_ANG_ROLL_PITCH,  TUNING_ARRAY(tuning_set_ang_roll_pitch) },
    { TUNING_SET_VXY,             TUNING_ARRAY(tuning_set_vxy) },
    { TUNING_SET_AZ,              TUNING_ARRAY(tuning_set_az) },
    { TUNING_SET_TRIM_THR_PTCH,   TUNING_ARRAY(tuning_set_trim_thr_pitch) },
    { TUNING_SET_COORDINATION,    TUNING_ARRAY(tuning_set_coordination) },
    { TUNING_SET_TECS_THR_FF,     TUNING_ARRAY(tuning_set_tecs_thr_ff) },
    { TUNING_SET_AGL_ROLL_PITCH,  TUNING_ARRAY(tuning_set_angle_roll_pitch) },
    { TUNING_SET_AGL_ROLL,        TUNING_ARRAY(tuning_set_angle_roll) },
    { TUNING_SET_AGL_PITCH,       TUNING_ARRAY(tuning_set_angle_pitch) },
    { TUNING_SET_MIXING,          TUNING_ARRAY(tuning_set_mixing) },
    { TUNING_SET_THR_EXPO,        TUNING_ARRAY(tuning_set_thr_expo) },
    { TUNING_SET_FLAP,            TUNING_ARRAY(tuning_set_flap) },
    { 0, 0, nullptr }
};

/*
  table of tuning names
 */
const AP_Tuning_Plane::tuning_name AP_Tuning_Plane::tuning_names[] = {
    { TUNING_RATE_ROLL_PI, "RateRollPI" },
    { TUNING_RATE_ROLL_P,  "RateRollP" },
    { TUNING_RATE_ROLL_I,  "RateRollI" },
    { TUNING_RATE_ROLL_D,  "RateRollD" },
    { TUNING_RATE_PITCH_PI,"RatePitchPI" },
    { TUNING_RATE_PITCH_P, "RatePitchP" },
    { TUNING_RATE_PITCH_I, "RatePitchI" },
    { TUNING_RATE_PITCH_D, "RatePitchD" },
    { TUNING_RATE_YAW_PI,  "RateYawPI" },
    { TUNING_RATE_YAW_P,   "RateYawP" },
    { TUNING_RATE_YAW_I,   "RateYawI" },
    { TUNING_RATE_YAW_D,   "RateYawD" },
    { TUNING_ANG_ROLL_P,   "AngRollP" },
    { TUNING_ANG_PITCH_P,  "AngPitchP" },
    { TUNING_ANG_YAW_P,    "AngYawP" },
    { TUNING_RATE_PITCH_FF, "RatePitchFF" },
    { TUNING_RATE_ROLL_FF, "RateRollFF" },
    { TUNING_RATE_YAW_FF, "RateYawFF" },
    { TUNING_PXY_P,        "PXY_P" },
    { TUNING_PZ_P,         "PZ_P" },
    { TUNING_VXY_P,        "VXY_P" },
    { TUNING_VXY_I,        "VXY_I" },
    { TUNING_VZ_P,         "VZ_P" },
    { TUNING_AZ_P,         "RateAZ_P" },
    { TUNING_AZ_I,         "RateAZ_I" },
    { TUNING_AZ_D,         "RateAZ_D" },
    { TUNING_RLL_P,        "RollP" },
    { TUNING_RLL_I,        "RollI" },
    { TUNING_RLL_D,        "RollD" },
    { TUNING_RLL_FF,       "RollFF" },
    { TUNING_PIT_P,        "PitchP" },
    { TUNING_PIT_I,        "PitchI" },
    { TUNING_PIT_D,        "PitchD" },
    { TUNING_PIT_FF,       "PitchFF" },
    { TUNING_TRIM_THROTTLE, "TRIM_THROTTLE" },
    { TUNING_TRIM_PITCH,   "TRIM_PITCH" },
    { TUNING_KFF_THRAT2PTCH, "THRAT2PTCH" },
    { TUNING_STAB_PITCH_DOWN, "STAB_PITCH_DOWN" },
    { TUNING_STAB_PITCH_DCRV, "STAB_PITCH_DCRV" },
    { TUNING_RLL2PTCH,      "RLL2PTCH" },
    { TUNING_KFF_RDDRMIX,   "RDRMIX" },
    { TUNING_TECS_THR_FF_DAMP, "TTHR_FF_DAMP" },
    { TUNING_TECS_THR_FF_FILT, "TTHR_FF_FILT" },
    { TUNING_AGL_ROLL_P,   "AglRollP" },
    { TUNING_AGL_ROLL_I,   "AglRollI" },
    { TUNING_AGL_ROLL_D,   "AglRollD" },
    { TUNING_AGL_ROLL_FLTT,"AglRollTF" },
    { TUNING_AGL_PITCH_P,  "AglPitchP" },
    { TUNING_AGL_PITCH_I,  "AglPitchI" },
    { TUNING_AGL_PITCH_D,  "AglPitchD" },
    { TUNING_AGL_PITCH_FLTT,"AglPitchTF" },
    { TUNING_MIXING_DIFF,  "MixingDiff"},
    { TUNING_MIXING_OFFSET,"MixingOffset"},
    { TUNING_THR_EXPO_MANUAL,"THR EXPO MAN"},
    { TUNING_THR_EXPO_AUTO,"THR EXPO AUTO"},
    { TUNING_FLAP_RETED_SPD, "FLAP_RETED_SPD"},
    { TUNING_FLAP_EXTED_SPD, "FLAP_EXTED_SPD"},
    { TUNING_FLAP_EXTED_PCT, "FLAP_EXTED_PCT"},
    { TUNING_KFF_THRAT2ELEV, "THRAT2ELEV"},
    { TUNING_KFF_FLAP2ELEV,  "FLAP2ELEV"},
    { TUNING_AILERONS_DIFF,  "AILERONS_DIFF"},
    { TUNING_ELEVATOR_DIFF,  "ELEVATOR_DIFF"},
    { TUNING_NONE, nullptr }
};

/*
  get a pointer to an AP_Float for a parameter, or nullptr on fail
 */
AP_Float *AP_Tuning_Plane::get_param_pointer(uint8_t parm)
{
#if HAL_QUADPLANE_ENABLED
    if (parm < TUNING_FIXED_WING_BASE && !plane.quadplane.available()) {
        // quadplane tuning options not available
        return nullptr;
    }
#endif

    switch(parm) {

#if HAL_QUADPLANE_ENABLED
    case TUNING_RATE_ROLL_PI:
        // use P for initial value when tuning PI
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kP();

    case TUNING_RATE_ROLL_P:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kP();

    case TUNING_RATE_ROLL_I:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kI();

    case TUNING_RATE_ROLL_D:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kD();

    case TUNING_RATE_PITCH_PI:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kP();

    case TUNING_RATE_PITCH_P:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kP();

    case TUNING_RATE_PITCH_I:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kI();

    case TUNING_RATE_PITCH_D:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kD();

    case TUNING_RATE_YAW_PI:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kP();

    case TUNING_RATE_YAW_P:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kP();

    case TUNING_RATE_YAW_I:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kI();

    case TUNING_RATE_YAW_D:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kD();

    case TUNING_ANG_ROLL_P:
        return &plane.quadplane.attitude_control->get_angle_roll_p().kP();

    case TUNING_ANG_PITCH_P:
        return &plane.quadplane.attitude_control->get_angle_pitch_p().kP();

    case TUNING_ANG_YAW_P:
        return &plane.quadplane.attitude_control->get_angle_yaw_p().kP();

    case TUNING_PXY_P:
        return &plane.quadplane.pos_control->get_pos_xy_p().kP();

    case TUNING_PZ_P:
        return &plane.quadplane.pos_control->get_pos_z_p().kP();

    case TUNING_VXY_P:
        return &plane.quadplane.pos_control->get_vel_xy_pid().kP();

    case TUNING_VXY_I:
        return &plane.quadplane.pos_control->get_vel_xy_pid().kI();

    case TUNING_VZ_P:
        return &plane.quadplane.pos_control->get_vel_z_pid().kP();

    case TUNING_AZ_P:
        return &plane.quadplane.pos_control->get_accel_z_pid().kP();

    case TUNING_AZ_I:
        return &plane.quadplane.pos_control->get_accel_z_pid().kI();

    case TUNING_AZ_D:
        return &plane.quadplane.pos_control->get_accel_z_pid().kD();

    case TUNING_RATE_PITCH_FF:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().ff();

    case TUNING_RATE_ROLL_FF:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().ff();

    case TUNING_RATE_YAW_FF:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().ff();
#endif // HAL_QUADPLANE_ENABLED

    // fixed wing tuning parameters
    case TUNING_RLL_P:
        return &plane.rollController.kP();

    case TUNING_RLL_I:
        return &plane.rollController.kI();

    case TUNING_RLL_D:
        return &plane.rollController.kD();

    case TUNING_RLL_FF:
        return &plane.rollController.kFF();

    case TUNING_PIT_P:
        return &plane.pitchController.kP();

    case TUNING_PIT_I:
        return &plane.pitchController.kI();

    case TUNING_PIT_D:
        return &plane.pitchController.kD();

    case TUNING_PIT_FF:
        return &plane.pitchController.kFF();

    case TUNING_KFF_THRAT2PTCH:
        return &plane.g.kff_throttle_above_trim_to_pitch;

    case TUNING_STAB_PITCH_DOWN:
        return &plane.g.stab_pitch_down;

    case TUNING_STAB_PITCH_DCRV:
        return &plane.g.stab_pitch_down_curve;

    case TUNING_RLL2PTCH:
        return &plane.pitchController.rollFF();

    case TUNING_TRIM_THROTTLE:
        return &plane.aparm.throttle_cruise;

    case TUNING_TRIM_PITCH:
        return &plane.g.pitch_trim;

    case TUNING_KFF_RDDRMIX:
        return &plane.g.kff_rudder_mix;

    case TUNING_TECS_THR_FF_DAMP:
        return &plane.TECS_controller.thr_ff_damp();

    case TUNING_TECS_THR_FF_FILT:
        return &plane.TECS_controller.thr_ff_filter();

    case TUNING_AGL_ROLL_P:
        return &plane.rollController.angle_kP();

    case TUNING_AGL_ROLL_I:
        return &plane.rollController.angle_kI();

    case TUNING_AGL_ROLL_D:
        return &plane.rollController.angle_kD();

    case TUNING_AGL_ROLL_FLTT:
        return &plane.rollController.angle_fltt();

    case TUNING_AGL_PITCH_P:
        return &plane.pitchController.angle_kP();

    case TUNING_AGL_PITCH_I:
        return &plane.pitchController.angle_kI();

    case TUNING_AGL_PITCH_D:
        return &plane.pitchController.angle_kD();

    case TUNING_AGL_PITCH_FLTT:
        return &plane.pitchController.angle_fltt();

    case TUNING_MIXING_DIFF:
        return &plane.g.mixing_diff;

    case TUNING_MIXING_OFFSET:
        return &plane.g.mixing_offset;

    case TUNING_THR_EXPO_MANUAL:
        return &plane.g2.throttle_expo_manual;

    case TUNING_THR_EXPO_AUTO:
        return &plane.g2.throttle_expo_auto;
    
    case TUNING_FLAP_RETED_SPD:
        return &plane.g.flap_retracted_speed;

    case TUNING_FLAP_EXTED_SPD:
        return &plane.g.flap_deployed_speed;

    case TUNING_FLAP_EXTED_PCT:
        return &plane.g.flap_deployed_percent;

    case TUNING_KFF_THRAT2ELEV:
        return &plane.g.kff_throttle_above_trim_to_elevator;

    case TUNING_KFF_FLAP2ELEV:
        return &plane.g.kff_flap_to_elevator;

    case TUNING_AILERONS_DIFF:
        return &plane.g2.ailerons_diff;

    case TUNING_ELEVATOR_DIFF:
        return &plane.g2.elevator_diff;
    }

    return nullptr;
}


/*
  save a parameter
 */
void AP_Tuning_Plane::save_value(uint8_t parm)
{
    switch(parm) {
    // special handling of dual-parameters
    case TUNING_RATE_ROLL_PI:
        save_value(TUNING_RATE_ROLL_P);
        save_value(TUNING_RATE_ROLL_I);
        break;
    case TUNING_RATE_PITCH_PI:
        save_value(TUNING_RATE_PITCH_P);
        save_value(TUNING_RATE_PITCH_I);
        break;
    default:
        AP_Float *f = get_param_pointer(parm);
        if (f != nullptr) {
            f->save();
        }
        break;
    }
}

/*
  set a parameter
 */
void AP_Tuning_Plane::set_value(uint8_t parm, float value)
{
    switch(parm) {
    // special handling of dual-parameters
    case TUNING_RATE_ROLL_PI:
        set_value(TUNING_RATE_ROLL_P, value);
        set_value(TUNING_RATE_ROLL_I, value);
        break;
    case TUNING_RATE_PITCH_PI:
        set_value(TUNING_RATE_PITCH_P, value);
        set_value(TUNING_RATE_PITCH_I, value);
        break;
    default:
        AP_Float *f = get_param_pointer(parm);
        if (f != nullptr) {
            uint64_t param_bit = (1ULL << parm);
            if (!(param_bit & have_set)) {
                // first time this param has been set by tuning. We
                // need to see if a reversion value is available in
                // FRAM, and if not then save one
                float current_value = f->get();
                if (!f->load()) {
                    // there is no value in FRAM, set one
                    f->set_and_save(current_value);
                }
                have_set |= param_bit;
            }
            f->set_and_notify(value);
        }
        break;
    }
}

/*
  reload a parameter
 */
void AP_Tuning_Plane::reload_value(uint8_t parm)
{
    switch(parm) {
    // special handling of dual-parameters
    case TUNING_RATE_ROLL_PI:
        reload_value(TUNING_RATE_ROLL_P);
        reload_value(TUNING_RATE_ROLL_I);
        break;
    case TUNING_RATE_PITCH_PI:
        reload_value(TUNING_RATE_PITCH_P);
        reload_value(TUNING_RATE_PITCH_I);
        break;
    default:
        AP_Float *f = get_param_pointer(parm);
        if (f != nullptr) {
            uint64_t param_bit = (1ULL << parm);
            // only reload if we have set this parameter at some point
            if (param_bit & have_set) {
                f->load();
            }
        }
        break;
    }
}
