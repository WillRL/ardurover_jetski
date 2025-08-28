#include "AP_DAC_Channel.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <cmath>

extern const AP_HAL::HAL& hal;

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
    uint8_t type = _frontend._params[_chan].binding.get();

    switch (AP_DAC_Channel_Params::Type(type)){
        case AP_DAC_Channel_Params::Type::NONE:
            return 0;

        default:
            hal.console->printf("Unknown analog binding: %d", type);
            return 0;

        case AP_DAC_Channel_Params::Type::THROTTLE:
            return _scale_normalise(_frontend.command.throttle, -100.0f, 100.0f, _voltage_min, _voltage_max) * _ratio;

        case AP_DAC_Channel_Params::Type::THROTTLE_ABS:
            return _scale_normalise(abs(_frontend.command.throttle), 0.0f, 100.0f, _voltage_min, _voltage_max) * _ratio;

        case AP_DAC_Channel_Params::Type::REVERSE:
            return _scale_normalise(-MIN(_frontend.command.throttle, 0), 0.0f, 100.0f, _voltage_min, _voltage_max) * _ratio;

        case AP_DAC_Channel_Params::Type::STEERING:
            return _scale_normalise(_frontend.command.steering, -4500.0f, 4500.0f, _voltage_min, _voltage_max) * _ratio;
        
        case AP_DAC_Channel_Params::Type::BRAKE:
            return _scale_normalise(abs(_frontend.command.brake), 0.0f, 4500.0f, _voltage_min, _voltage_max) * _ratio;
    }
    return 0;
}

float AP_DAC_Channel::_scale_normalise(float val, float min_old, float max_old, float min_new, float max_new)
{
    if (abs(max_old - min_old) < 1e-6) {
        hal.console->printf("AP_DAC_Channel::_normalise(float val, float min_old, float max_old, float min_new, float max_new); min_old != max_old");
        return val;
    }

    return ((val - min_old) / (max_old - min_old)) * (max_new - min_new) + min_new;
}