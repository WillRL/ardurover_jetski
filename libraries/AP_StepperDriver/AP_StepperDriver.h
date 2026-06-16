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
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Param/AP_Param.h>
#if AP_STEPPERDRIVER_ENABLED
class AP_StepperDriver_Backend;

class AP_StepperDriver
{
    friend class AP_StepperDriver_Backend;
    public:
    // constructor. This incorporates initialization as well.
        CLASS_NO_COPY(AP_StepperDriver);
        static AP_StepperDriver *get_singleton() { return _singleton; }
        AP_StepperDriver();
        virtual ~AP_StepperDriver(void) {};
    
        virtual void init();

        enum Type {
        NONE         = 0,
        DRV8462SPI   = 1,
        };

        enum MicrostepMode {
        FULL            = 0,
        HALF            = 1,
        QUARTER         = 2,
        EIGHTH          = 3,
        SIXTEENTH       = 4,
        THIRTY_SECOND   = 5,
        SIXTY_FOURTH    = 6,
        ONE28TH         = 7,
        TWO56TH         = 8
        };

        static const struct AP_Param::GroupInfo var_info[];
        
        bool step();
        bool dir(bool dir);
        bool enable();
        bool set_run_I(float I);
        bool set_hold_I(float I);
        bool set_microstep_mode(MicrostepMode mode);
        bool use_IVREF(bool use_ivref);
        bool use_SPI(bool use_spi);


    private:
        static AP_StepperDriver *_singleton;
        AP_Int8 _driver_type;
        AP_StepperDriver_Backend *_driver;
        AP_Float _run_I;
        AP_Float _hold_I;
        AP_Enum16<MicrostepMode> _microstep_mode;
};

#endif  // AP_STEPPERDRIVER_ENABLED