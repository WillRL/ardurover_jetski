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

#include "AP_StepperDriver_config.h"
#if AP_STEPPERDRIVER_ENABLED

#include "AP_StepperDriver.h"
#include "AP_StepperDriver_DRV8462SPI.h"
#include <cmath>
extern const AP_HAL::HAL& hal;

class AP_StepperDriver_Backend;
const AP_Param::GroupInfo AP_StepperDriver::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Encoder Type
    // @Description: Encoder type
    // @Values: 0:None, 1:DRV8462SPI
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_StepperDriver, _driver_type, 0, AP_PARAM_FLAG_ENABLE),
    AP_GROUPINFO("RUN_I", 2, AP_StepperDriver, _run_I, 1.0f),
    AP_GROUPINFO("HOLD_I", 3, AP_StepperDriver, _hold_I, 0.5f),
    AP_GROUPINFO("MSTP", 4, AP_StepperDriver, _microstep_mode, AP_StepperDriver::MicrostepMode::SIXTEENTH),
    AP_GROUPEND
};

AP_StepperDriver *AP_StepperDriver::_singleton = nullptr;

// constructor
AP_StepperDriver::AP_StepperDriver()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_StepperDriver must be singleton");
    }
    _singleton = this;
}


// Initialize the Wind Vane object and prepare it for use
void AP_StepperDriver::init()
{
    // don't construct twice
    if (_driver != nullptr) {
        return;
    }

    // detect type of encoder used and create the link.
    switch (_driver_type) {
        case Type::NONE:
            // STEPPERDRIVER disabled
            return;
        
        #if AP_STEPPERDRIVER_DRV8462SPI_ENABLED
        case Type::DRV8462SPI:
            _driver = NEW_NOTHROW AP_StepperDriver_DRV8462SPI(*this);
            _driver->init();
            set_hold_I(_hold_I);
            set_run_I(_run_I);
            set_microstep_mode(_microstep_mode);
            return; 
        #endif

    }
}

bool AP_StepperDriver::step()
{
    if (_driver == nullptr) {
        return false;
    }
    return _driver->step();
}

bool AP_StepperDriver::dir(bool dir)
{
    if (_driver == nullptr) {
        return false;
    }
    return _driver->dir(dir);
}

bool AP_StepperDriver::enable()
{
    if (_driver == nullptr) {
        return false;
    }
    return _driver->enable();
}

bool AP_StepperDriver::set_run_I(float I)
{
    if (_driver == nullptr) {
        return false;
    }
    return _driver->set_run_I(I);
}

bool AP_StepperDriver::set_hold_I(float I)
{
    if (_driver == nullptr) {
        return false;
    }
    return _driver->set_hold_I(I);
}

bool AP_StepperDriver::set_microstep_mode(MicrostepMode mode)
{
    if (_driver == nullptr) {
        return false;
    }
    return _driver->set_microstep_mode(mode);
}

bool AP_StepperDriver::use_SPI(bool use_spi)
{
    if (_driver == nullptr) {
        return false;
    }
    return _driver->use_SPI(use_spi);
}

bool AP_StepperDriver::use_IVREF(bool use_ivref)
{
    if (_driver == nullptr) {
        return false;
      }
    return _driver->use_IVREF(use_ivref);
}
#endif  // AP_StepperDriver_ENABLED
