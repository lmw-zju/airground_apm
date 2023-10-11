#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    AP_Int8 get_int8Param() const { return _int8; }
    AP_Int16 get_int16Param() const { return _int16; }
    AP_Float get_floatParam() const { return _float; }
    
    AP_Int16 stab_loit_alt;//shuould be high
    AP_Int16 loit_to_stab_alt;//should be short
    AP_Int16 stab2loit_up;
    AP_Int16 loit2stab_down;
    AP_Int16 speed_recovery_time;
    AP_Int16 ground_rng_times;
    AP_Int16 ground_ok_times;

private:
    // Put your parameter variable definitions here
    AP_Int8 _int8;
    AP_Int16 _int16;
    AP_Float _float;
};
