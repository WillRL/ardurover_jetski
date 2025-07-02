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

#include "AP_SteeringEncoder_config.h"

#if AP_STEERINGENCODER_ENABLED

#include "AP_SteeringEncoder.h"

class AP_SteeringEncoder_Backend
{
public:
    // constructor. This incorporates initialization as well.
    AP_SteeringEncoder_Backend(AP_SteeringEncoder &frontend);

    // we declare a virtual destructor so that SteeringEncoder drivers can
    // override with a custom destructor if need be
    virtual ~AP_SteeringEncoder_Backend() {}

    // initialization
    virtual void init() {};

    // update the state structure
    virtual void update() {};
    virtual void calibrate();

protected:

    AP_SteeringEncoder &_frontend;

};

#endif  // AP_WINDVANE_ENABLED
