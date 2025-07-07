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
#include <AP_GenericEncoder/AP_GenericEncoder_config.h>

#if AP_GENERICENCODER_ENABLED
#if AP_GENERICENCODER_AS5600I2C_ENABLED
#if AP_StepperEncoder_ENABLED


#include "AP_StepperEncoder.h"
#include "AP_StepperEncoder_Backend.h"

#include <AP_GenericEncoder/AP_GenericEncoder_AS5600I2C.h>
#include <AP_HAL/I2CDevice.h>


class AP_StepperEncoder_AS5600I2C: public AP_StepperEncoder_Backend
{
  public:                                                      
    // constructor
    AP_StepperEncoder_AS5600I2C(AP_StepperEncoder &frontend);
    void update() override;
    void init() override;
  
  protected:
    AP_GenericEncoder_AS5600I2C _encoder;
};

#endif //AP_StepperEncoder_ENABLED
#endif //AP_GENERICENCODER_AS5600I2C_ENABLED
#endif //AP_GENERICENCODER_ENABLED