/*
 *  AnalogOutput Test Code
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_AnalogOutput/AP_AnalogOutput.h>
#include <AP_AnalogOutput/AP_DAC_Channel_Params.h>
#include <AP_DAC/AP_DAC.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_DAC dac;
AP_AnalogOutput aout(dac);

namespace {
// try to set the object value but provide diagnostic if it failed
void set_object_value(const void *object_pointer,
                      const struct AP_Param::GroupInfo *group_info,
                      const char *name, float value)
    {
        // THIS DOES NOT WORK FOR SETTINGS IN SUBGROUPS.
        if (!AP_Param::set_object_value(object_pointer, group_info, name, value)) {
            hal.console->printf("WARNING: AP_Param::set object value \"%s::%s\" Failed.\n",
                                group_info->name, name);
        }
    }
}



void setup()
{
    // print welcome message
    hal.scheduler->delay(5000);
    hal.console->printf("Starting AnalogOutput test\n");
    set_object_value(&dac.params[0], dac.params[0].var_info, "TYPE", 3);
    set_object_value(&dac.params[0], dac.params[0].var_info, "VREF", 5);

    AP_DAC_Channel_Params *param0 = aout.get_param(0);
    set_object_value(param0, param0->var_info, "EN", 1);
    set_object_value(param0, param0->var_info, "BIND", (uint8_t)AP_DAC_Channel_Params::Type::THROTTLE);
    set_object_value(param0, param0->var_info, "MIN", 0.5);
    set_object_value(param0, param0->var_info, "MAX", 2.97);
    set_object_value(param0, param0->var_info, "RATIO", 1);

    AP_DAC_Channel_Params *param1 = aout.get_param(1);
    set_object_value(param1, param1->var_info, "EN", 1);
    set_object_value(param1, param1->var_info, "BIND", (uint8_t)AP_DAC_Channel_Params::Type::THROTTLE);
    set_object_value(param1, param1->var_info, "MIN", 0.5);
    set_object_value(param1, param1->var_info, "MAX", 2.97);
    set_object_value(param1, param1->var_info, "RATIO", 0.5);
    
    aout.init();
}

void loop()
{
    aout.command.throttle = sinf(10*AP_HAL::millis()/1000.0f) * 100;
    hal.console->printf("updated: %i\n", aout.update());
    hal.scheduler->delay(10);
}
AP_HAL_MAIN();
