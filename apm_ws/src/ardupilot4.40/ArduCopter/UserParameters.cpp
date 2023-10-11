#include "UserParameters.h"

// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 0),
    AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),
    AP_GROUPINFO("_S2LOIT_ALT", 3, UserParameters, stab_loit_alt, 60),
    AP_GROUPINFO("_L2STAB_ALT", 4, UserParameters, loit_to_stab_alt, 40),
    AP_GROUPINFO("_S2LOIT_SPD", 5, UserParameters, stab2loit_up, 30),
    AP_GROUPINFO("_L2STAB_SPD", 6, UserParameters, loit2stab_down, 50),
    AP_GROUPINFO("_REC_TIME", 7, UserParameters, speed_recovery_time, 6000),
    AP_GROUPINFO("_RNG_TIMES", 8, UserParameters, ground_rng_times, 100),
    AP_GROUPINFO("_GNDOK_TIMES", 9, UserParameters, ground_ok_times, 100),
    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
