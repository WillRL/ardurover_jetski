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

#include "AP_DAC_Channel_Params.h"


const AP_Param::GroupInfo AP_DAC_Channel_Params::var_info[] = {
    // @Param: EN
    // @DisplayName: Enable Channel
    // @Description: Enables this channel for the DAC
    // @Values: 0:False, 1:True
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("EN", 1, AP_DAC_Channel_Params, enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: BIND
    // @DisplayName: Channel Function
    // @Description: Selects the function this DAC channel is bound to (e.g., Throttle, Steering, etc.). Typically mapped to specific vehicle outputs.
    // @Values: 0:None, 1:Throttle, 2:Steering
    // @User: Standard
    AP_GROUPINFO("BIND", 2, AP_DAC_Channel_Params, type, 0),

    // @Param: MIN
    // @DisplayName: Minimum Voltage
    // @Description: The minimum output voltage (in volts) this DAC channel can output when the input value is at its minimum.
    // @Units: V
    // @Range: -inf inf
    // @User: Standard
    AP_GROUPINFO("MIN", 3, AP_DAC_Channel_Params, _voltage_min, 0),

    // @Param: MAX
    // @DisplayName: Maximum Voltage
    // @Description: The maximum output voltage (in volts) this DAC channel can output when the input value is at its maximum.
    // @Units: V
    // @Range: -inf inf
    // @User: Standard
    AP_GROUPINFO("MAX", 4, AP_DAC_Channel_Params, _voltage_max, 5),

    // @Param: RATIO
    // @DisplayName: Output Scaling Ratio
    // @Description: Multiplier applied to output voltage. Useful for dual redundancy with ratios.
    // @Range: -inf inf
    AP_GROUPINFO("RATIO", 5, AP_DAC_Channel_Params, _ratio, 1),
    AP_GROUPEND
};

AP_DAC_Channel_Params::AP_DAC_Channel_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

