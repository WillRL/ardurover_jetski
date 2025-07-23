#include "AP_DAC_Channel.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <cmath>

extern const AP_HAL::HAL& hal;
float AP_DAC_Channel::_minmax[] = {0.0f, 100.0f, 100.0f};

// base class constructor.
AP_DAC_Channel::AP_DAC_Channel(AP_AnalogOutput &frontend, uint8_t chan) :
        _frontend(frontend)
{
    _chan = chan;
}



bool AP_DAC_Channel::output()
{   
    float voltage = _convert_command();
    return _frontend._dac.set_voltage(0, _chan, voltage);
}

float AP_DAC_Channel::_convert_command()
{   
    float_t _ratio = _frontend._params[_chan]._ratio.get();
    float_t _voltage_min = _frontend._params[_chan]._voltage_min.get();
    float_t _voltage_max = _frontend._params[_chan]._voltage_max.get();
    uint8_t type = _frontend._params[_chan].type.get();

    switch (AP_DAC_Channel_Params::Type(type)){
        case AP_DAC_Channel_Params::Type::NONE:
            return 0;
        case AP_DAC_Channel_Params::Type::THROTTLE:
            return _normalise(_frontend.command.throttle, -_minmax[type], _minmax[type], _voltage_min, _voltage_max) * _ratio;
        case AP_DAC_Channel_Params::Type::REVERSE:
            return 0;
    }
    return 0;
}

float AP_DAC_Channel::_normalise(float val, float min_old, float max_old, float min_new, float max_new)
{
    if (min_old == max_old) {
        hal.console->printf("AP_DAC_Channel::_normalise(float val, float min_old, float max_old, float min_new, float max_new); min_old != max_old");
        return val;
    }

    return min_new + ((val - min_old) / (max_old - min_old)) * (max_new - min_new);
}