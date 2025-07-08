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
#pragma once

#include "AP_StepperEncoder_config.h"
#if AP_StepperEncoder_ENABLED

#include <AP_Param/AP_Param.h>
#include <Filter/Filter.h>
#include <GCS_MAVLink/GCS_MAVLink.h>


class AP_StepperEncoder_Backend;

class AP_StepperEncoder
{
    friend class AP_StepperEncoder_Backend;

public:
    AP_StepperEncoder();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_StepperEncoder);

    static AP_StepperEncoder *get_singleton();
    void init();

    // update wind vane
    void update();

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

    float theta;                
    float omega;                    
    float alpha; 
    float latest_measurement_time;
    enum Type {
        NONE        = 0,
        AS5600I2C   = 1,

    };

private:
    // parameters
    AP_Int8 _encoder_type;                          // type of windvane being used
    AP_StepperEncoder_Backend *_driver;
};

#endif  // AP_StepperEncoder_ENABLED
