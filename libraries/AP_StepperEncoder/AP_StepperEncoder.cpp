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

#include "AP_StepperEncoder_config.h"
#if AP_StepperEncoder_ENABLED

#include "AP_StepperEncoder.h"
#include "AP_StepperEncoder_AS5600I2C.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>



extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_StepperEncoder::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Encoder Type
    // @Description: Encoder type
    // @Values: 0:None, 1:AS5600I2C
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_StepperEncoder, _encoder_type, 0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

// constructor
AP_StepperEncoder::AP_StepperEncoder()
{
    AP_Param::setup_object_defaults(this, var_info);
}


// Initialize the Wind Vane object and prepare it for use
void AP_StepperEncoder::init()
{
    // don't construct twice
    if (_driver != nullptr) {
        return;
    }

    // detect type of encoder used and create the link.
    switch (_encoder_type) {
        case Type::NONE:
            // STEPPERENCODER disabled
            return;
        
        #if AP_GENERICENCODER_AS5600I2C_ENABLED
        case Type::AS5600I2C:
            _driver = NEW_NOTHROW AP_StepperEncoder_AS5600I2C(*this);
            _driver->init();
            return; 
        #endif

    }
}

// update encoder expected to be called at 400hz
void AP_StepperEncoder::update()
{
    if (_driver == nullptr) {
        return;
    }
    _driver->update();
}

#endif  // AP_StepperEncoder_ENABLED
