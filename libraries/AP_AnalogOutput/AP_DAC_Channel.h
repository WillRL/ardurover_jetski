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
#include "AP_AnalogOutput.h"

class AP_DAC_Channel {
public:
    
    AP_DAC_Channel(AP_AnalogOutput &frontend, uint8_t chan);

    void init(AP_AnalogOutput &frontend, uint8_t chan);
    bool output();
    // bool enabled;

private:
    uint8_t _chan;
    AP_AnalogOutput &_frontend;
    
    float _convert_command();
    static float _scale_normalise(float val, float min_old, float max_old, float min_new, float max_new);
};

