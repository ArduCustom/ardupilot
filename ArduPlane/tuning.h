#include <AP_Tuning/AP_Tuning.h>

/*
  Plane transmitter tuning
 */
class AP_Tuning_Plane : public AP_Tuning
{
private:
    // table of tuning sets
    static const tuning_set tuning_sets[];

    // table of tuning parameter names for reporting
    static const tuning_name tuning_names[];
    
public:
    // constructor
    AP_Tuning_Plane(void) : AP_Tuning(tuning_sets, tuning_names) {}

    static const struct AP_Param::GroupInfo  var_info[];
    
private:

    // individual tuning parameters
    enum tuning_func {
        TUNING_NONE =                          0,

        // quadplane tuning
        TUNING_RATE_ROLL_PI =                  1,
        TUNING_RATE_ROLL_P =                   2,
        TUNING_RATE_ROLL_I =                   3,
        TUNING_RATE_ROLL_D =                   4,

        TUNING_RATE_PITCH_PI =                 5,
        TUNING_RATE_PITCH_P =                  6,
        TUNING_RATE_PITCH_I =                  7,
        TUNING_RATE_PITCH_D =                  8,

        TUNING_RATE_YAW_PI =                   9,
        TUNING_RATE_YAW_P =                   10,
        TUNING_RATE_YAW_I =                   11,
        TUNING_RATE_YAW_D =                   12,

        TUNING_ANG_ROLL_P =                   13,
        TUNING_ANG_PITCH_P =                  14,
        TUNING_ANG_YAW_P =                    15,

        TUNING_PXY_P =                        16,
        TUNING_PZ_P  =                        17,

        TUNING_VXY_P =                        18,
        TUNING_VXY_I =                        19,
        TUNING_VZ_P  =                        20,

        TUNING_AZ_P =                         21,
        TUNING_AZ_I =                         22,
        TUNING_AZ_D  =                        23,

        TUNING_RATE_PITCH_FF =         24,
        TUNING_RATE_ROLL_FF =         25,
        TUNING_RATE_YAW_FF =         26,

        // fixed wing tuning
        TUNING_FIXED_WING_BASE =              50,
        TUNING_RLL_P =                        50,
        TUNING_RLL_I =                        51,
        TUNING_RLL_D =                        52,
        TUNING_RLL_FF =                       53,

        TUNING_PIT_P =                        54,
        TUNING_PIT_I =                        55,
        TUNING_PIT_D =                        56,
        TUNING_PIT_FF =                       57,

        TUNING_KFF_THRAT2PTCH =               58,
        TUNING_STAB_PITCH_DOWN =              59,

        TUNING_RLL2PTCH =                     60,
        TUNING_KFF_RDDRMIX =                  61,

        TUNING_TECS_THR_FF_DAMP =             62,
        TUNING_TECS_THR_FF_FILT =             63,

        TUNING_AGL_ROLL_P =                   64,
        TUNING_AGL_ROLL_I =                   65,
        TUNING_AGL_ROLL_D =                   66,
        TUNING_AGL_ROLL_FLTT =                67,
        TUNING_AGL_PITCH_P =                  68,
        TUNING_AGL_PITCH_I =                  69,
        TUNING_AGL_PITCH_D =                  70,
        TUNING_AGL_PITCH_FLTT =               71,
    };

    /*
      sets of tuning values, chosen with TUNE_PARMSET over 100
     */
    enum tuning_sets {
        TUNING_SET_RATE_ROLL_PITCH =         1,
        TUNING_SET_RATE_ROLL =               2,
        TUNING_SET_RATE_PITCH =              3,
        TUNING_SET_RATE_YAW =                4,
        TUNING_SET_ANG_ROLL_PITCH =          5,
        TUNING_SET_VXY =                     6,
        TUNING_SET_AZ =                      7,
        TUNING_SET_THR2PTCH_MIX =            8,
        TUNING_SET_COORDINATION =            9,
        TUNING_SET_TECS_THR_FF =            10,
        TUNING_SET_AGL_ROLL_PITCH =         11,
        TUNING_SET_AGL_ROLL =               12,
        TUNING_SET_AGL_PITCH =              13,
    };

    AP_Float *get_param_pointer(uint8_t parm) override;
    void save_value(uint8_t parm) override;
    void reload_value(uint8_t parm) override;
    void set_value(uint8_t parm, float value) override;

    // tuning set arrays
    static const uint8_t tuning_set_rate_roll_pitch[];
    static const uint8_t tuning_set_rate_roll[];
    static const uint8_t tuning_set_rate_pitch[];
    static const uint8_t tuning_set_rate_yaw[];
    static const uint8_t tuning_set_ang_roll_pitch[];
    static const uint8_t tuning_set_vxy[];
    static const uint8_t tuning_set_az[];
    static const uint8_t tuning_set_thr2ptch[];
    static const uint8_t tuning_set_coordination[];
    static const uint8_t tuning_set_tecs_thr_ff[];
    static const uint8_t tuning_set_angle_roll_pitch[];
    static const uint8_t tuning_set_angle_roll[];
    static const uint8_t tuning_set_angle_pitch[];

    // mask of what params have been set
    uint64_t have_set;
};
