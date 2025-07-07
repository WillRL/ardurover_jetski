/*
 *  RangeFinder test code
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_StepperEncoder/AP_StepperEncoder.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
AP_StepperEncoder encoder;

void setup()
{
    // print welcome message
    hal.console->printf("Steering encoder library test\n");
    AP_Param::set_object_value(&encoder, encoder.var_info, "TYPE", (uint8_t)AP_StepperEncoder::Type::AS5600I2C);

    // initialise sensor, delaying to make debug easier
    hal.scheduler->delay(2000);
    encoder.init();
}

void loop()
{
    // Delay between reads
    hal.scheduler->delay(100);
    encoder.update();
    hal.console->printf("pos %f, vel %f, acc %f \n", encoder.theta*57.2958, encoder.omega*57.2958, encoder.alpha*57.2958);
}
AP_HAL_MAIN();
