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

#include "AP_SteeringEncoder_config.h"

#if AP_STEERINGENCODER_ENABLED

#include "AP_SteeringEncoder.h"
#include "AP_SteeringEncoder_Backend.h"

#include <GCS_MAVLink/GCS.h>

// base class constructor.
AP_SteeringEncoder_Backend::AP_SteeringEncoder_Backend(AP_SteeringEncoder &frontend) :
        _frontend(frontend)
{
}

// calibrate SteeringEncoder
void AP_SteeringEncoder_Backend::calibrate()
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SteeringEncoder: No cal required");
    _frontend._calibration.set_and_save(0);
    return;
}

#endif  // AP_WINDVANE_ENABLED
