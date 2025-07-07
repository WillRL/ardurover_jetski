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
#if AP_GENERICENCODER_ENABLED

#include "AP_GenericEncoder.h"
#include <cmath>
extern const AP_HAL::HAL& hal;

AP_GenericEncoder::AP_GenericEncoder()
{
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

float AP_GenericEncoder::round_to_decimal_place(float value, int decimals)
{
    int rounding = pow(10, decimals);
    return round(value * rounding) / rounding;
}

void AP_GenericEncoder::init(float *_ptr_pos, float *_ptr_dt_pos, float *_ptr_ddt_pos, float *_ptr_latest_measurement_time) 
{
    _pos = _ptr_pos;
    _dt_pos = _ptr_dt_pos;
    _ddt_pos = _ptr_ddt_pos;
    _latest_measurement_time = _ptr_latest_measurement_time; 
}

void AP_GenericEncoder::update() {
    // Default implementation (can be overridden)
}

void AP_GenericEncoder::update_pos() {
    // Default implementation (can be overridden)
}

void AP_GenericEncoder::update_velocity() {
    // Default implementation (can be overridden)
}

void AP_GenericEncoder::update_acceleration() {
    // Default implementation (can be overridden)
}

void AP_GenericEncoder::calibrate() {
    // Default implementation (can be overridden)
}

void AP_GenericEncoder::setup() {
    // Default implementation (can be overridden)
}

#endif  // AP_GENERICENCODER_ENABLED