/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 /*
 Closed loop stepper motor position controller by adjusting PWM frequency to control stepper motor speed.
 */

#include "AP_AnalogOutput.h"
#include "AP_DAC_Channel.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <cmath>
extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_AnalogOutput::var_info[] = {
    // @Param: STPR_EN
    // @DisplayName: Enable stepper motor for steering
    // @Description: Allows stepper motor control for steering
    // @Values: 0:False,1:True
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("EN", 1, AP_AnalogOutput, is_active, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: CHAN0_
    // @Path: AP_DAC_Channel
    AP_SUBGROUPINFO(_params[0], "CHAN0_",  2, AP_AnalogOutput, AP_DAC_Channel_Params),

    // @Param: CHAN1_
    // @Path: AP_DAC_Channel
    AP_SUBGROUPINFO(_params[1], "CHAN1_",  3, AP_AnalogOutput, AP_DAC_Channel_Params),

    // @Param: CHAN2_
    // @Path: AP_DAC_Channel
    AP_SUBGROUPINFO(_params[2], "CHAN2_",  4, AP_AnalogOutput, AP_DAC_Channel_Params),

    // @Param: CHAN3_
    // @Path: AP_DAC_Channel
    AP_SUBGROUPINFO(_params[3], "CHAN3_",  5, AP_AnalogOutput, AP_DAC_Channel_Params),

    // @Param: CHAN4_
    // @Path: AP_DAC_Channel
    AP_SUBGROUPINFO(_params[4], "CHAN4_",  6, AP_AnalogOutput, AP_DAC_Channel_Params),

    // @Param: CHAN5_
    // @Path: AP_DAC_Channel
    AP_SUBGROUPINFO(_params[5], "CHAN5_",  7, AP_AnalogOutput, AP_DAC_Channel_Params),

    // @Param: CHAN6_
    // @Path: AP_DAC_Channel
    AP_SUBGROUPINFO(_params[6], "CHAN6_",  8, AP_AnalogOutput, AP_DAC_Channel_Params),

    // @Param: CHAN7_
    // @Path: AP_DAC_Channel
    AP_SUBGROUPINFO(_params[7], "CHAN7_",  9, AP_AnalogOutput, AP_DAC_Channel_Params),

    
    AP_GROUPEND
};

// Constructor
AP_AnalogOutput::AP_AnalogOutput(AP_DAC& dac) :
    _dac(dac)
{
    AP_Param::setup_object_defaults(this, var_info);
}

AP_AnalogOutput::~AP_AnalogOutput() 
{
    // Remember to deallocate memory in the destructor to prevent memory leaks
    for (int i = 0; i < 8; i++) {
        delete _channels[i];
    }
}

// Constructor
void AP_AnalogOutput::init()
{
    _dac.init();
    for (int i = 0; i < 8; i++) {
        _channels[i] = NEW_NOTHROW AP_DAC_Channel(*this, i);
    }
}

bool AP_AnalogOutput::update(){
    bool ret = 1;
    for (int i = 0; i <= 7; i++){
        if (!_params[i].enabled) continue;
        ret &= _channels[i]->output();
    }
    return ret;
}


// Used for example only!
AP_DAC_Channel_Params *AP_AnalogOutput::get_param(int channel)
{
    return &_params[channel];
}