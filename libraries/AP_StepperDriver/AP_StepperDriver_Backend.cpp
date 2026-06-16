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

#include "AP_StepperDriver_Backend.h"

#include <GCS_MAVLink/GCS.h>

// base class constructor.
AP_StepperDriver_Backend::AP_StepperDriver_Backend(AP_StepperDriver &frontend) :
        _frontend(frontend)
{
}

void AP_StepperDriver_Backend::init() {};

bool AP_StepperDriver_Backend::write_config(const config &cfg) {return false;}
bool AP_StepperDriver_Backend::read_reg(uint8_t addr, uint8_t *recv) {return false;}
bool AP_StepperDriver_Backend::write_reg(uint8_t addr, uint8_t data) {return false;}
bool AP_StepperDriver_Backend::step() {return false;}
bool AP_StepperDriver_Backend::dir(bool dir) {return false;}
bool AP_StepperDriver_Backend::enable() {return false;}
bool AP_StepperDriver_Backend::set_run_I(float I) {     return false;}
bool AP_StepperDriver_Backend::set_hold_I(float I) {return false;}
bool AP_StepperDriver_Backend::set_microstep_mode(AP_StepperDriver::MicrostepMode microstep_mode) {return false;}
bool AP_StepperDriver_Backend::use_SPI(bool use_spi) {return false;}
bool AP_StepperDriver_Backend::use_IVREF(bool use_ivref) {return false;}


#endif  // AP_STEPPERDRIVER_ENABLED
