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
#include "AP_StepperEncoder_Backend.h"

#include <GCS_MAVLink/GCS.h>

// base class constructor.
AP_StepperEncoder_Backend::AP_StepperEncoder_Backend(AP_StepperEncoder &frontend) :
        _frontend(frontend)
{
}

void AP_StepperEncoder_Backend::init() {};
void AP_StepperEncoder_Backend::update() {};
void AP_StepperEncoder_Backend::calibrate() {};

#endif  // AP_StepperEncoder_ENABLED
