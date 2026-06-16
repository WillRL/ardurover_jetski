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

#include "AP_StepperDriver_config.h"

#if AP_STEPPERDRIVER_ENABLED
#include "AP_StepperDriver.h"

class AP_StepperDriver_Backend
{
public:
    // constructor. This incorporates initialization as well.
    AP_StepperDriver_Backend(AP_StepperDriver &frontend);

    // we declare a virtual destructor so that STEPPERDRIVER drivers can
    // override with a custom destructor if need be
    virtual ~AP_StepperDriver_Backend(void) {};

    // initialization
    virtual void init();
    struct config {};
    
    virtual bool write_config(const config &cfg);
    virtual bool read_reg(uint8_t addr, uint8_t *recv);
    virtual bool write_reg(uint8_t addr, uint8_t data);
    virtual bool step();
    virtual bool dir(bool dir);
    virtual bool enable();
    virtual bool set_run_I(float I);
    virtual bool set_hold_I(float I);
    virtual bool set_microstep_mode(AP_StepperDriver::MicrostepMode microstep_mode);
    virtual bool use_SPI(bool use_spi);
    virtual bool use_IVREF(bool use_ivref);

protected:

    AP_StepperDriver &_frontend;
};

#endif  // AP_StepperDriver_ENABLED
