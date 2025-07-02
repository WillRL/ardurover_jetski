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

#include "AP_GenericEncoder_config.h"
#include "AP_GenericEncoder.h"
#include <cmath>

#if AP_GENERICENCODER_ENABLED

AP_GenericEncoder::AP_GenericEncoder()
{
    // Initialize the position, velocity, and acceleration to zero
    _pos = 0.0f;
    _dt_pos = 0.0f;
    _ddt_pos = 0.0f;
}

void AP_GenericEncoder::read(float *retvals)
{
    // Ensure retvals is not null
    if (retvals == nullptr) {
        return;
    }

    // Read the latest values into the provided array
    retvals[0] = _pos;      // Position
    retvals[1] = _dt_pos;    // Velocity
    retvals[2] = _ddt_pos;   // Acceleration
}

/*
https://stackoverflow.com/questions/50833008/detect-if-magnetic-encoder-passes-360-or-0-and-in-which-direction-roll-over-wra
Calculates the dpos by assuming direction traveled is the shortest path.
*/
float AP_GenericEncoder::calc_abs_rotary_d_pos(float new_angle, float old_angle)
{
  float result = fmod((2 * M_PI + new_angle - old_angle), (2 * M_PI));
  return (result > M_PI) ? result - (2 * M_PI) : result;
}

#endif  // AP_GENERICENCODER_ENABLED