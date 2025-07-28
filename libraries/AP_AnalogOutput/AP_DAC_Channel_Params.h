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

#include <AP_Param/AP_Param.h>
// #include "AP_DAC_Channel.h"

class AP_DAC_Channel_Params {
public:
    friend class AP_DAC_Channel;
    AP_DAC_Channel_Params();
    static const struct AP_Param::GroupInfo var_info[];
    // DAC types
    enum class Type : uint8_t {
        NONE            = 0,
        THROTTLE        = 1,
        THROTTLE_ABS    = 2,
        REVERSE         = 3,
        STEERING        = 4,
    };
    AP_Enum<Type> binding;             // 0=disabled, others see frontend enum TYPE
    AP_Int8 enabled;

private:
    uint8_t _chan;
    AP_Float _ratio;
    AP_Float _voltage_min;
    AP_Float _voltage_max;
};

