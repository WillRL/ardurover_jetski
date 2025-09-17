#pragma once

#include <stdint.h>
#include <AP_DAC/AP_DAC.h>
#include "AP_DAC_Channel_Params.h"

class AP_DAC_Channel;

class AP_AnalogOutput {
    public:
        // Setpoint for the target angle
        friend class AP_DAC_Channel;
        static const struct AP_Param::GroupInfo var_info[];

        // Constructor
        AP_AnalogOutput(AP_DAC& dac);
        ~AP_AnalogOutput();

        void init();
        
        bool update();
        // bool restart_dac();
        // AP_DAC *get_dac();
        

        AP_Int8 is_active; // Boolean, if analog output is enabled.
        struct Commands {
            static float throttle;
            static float steering;
            static float brake;
            static bool armed;
            static float lua1;
            static float lua2;
            static float lua3;
            static float lua4;
            static float lua5;
            static float lua6;
            static float lua7;
            static float lua8;
        };


        AP_DAC_Channel_Params *get_param(int channel);
    private:
        AP_DAC& _dac;
        AP_DAC_Channel* _channels[8];
        AP_DAC_Channel_Params _params[8];

};