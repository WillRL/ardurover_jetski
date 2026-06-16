/*
 *  Stepper Encoder Test Code
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_StepperDriver/AP_StepperDriver.h>
#include <GCS_MAVLink/GCS_Dummy.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
AP_StepperDriver driver;

void setup()
{
    // print welcome message
    hal.console->printf("Steering encoder library test\n");
    AP_Param::set_object_value(&driver, driver.var_info, "SDRV_TYPE", (uint8_t)AP_StepperDriver::Type::DRV8462SPI);
    AP_Param::set_object_value(&driver, driver.var_info, "SDRV_RUN_I", 2.5f);
    AP_Param::set_object_value(&driver, driver.var_info, "SDRV_HOLD_I", 1.0f);
    AP_Param::set_object_value(&driver, driver.var_info, "SDRV_MSTP", (uint8_t)AP_StepperDriver::MicrostepMode::SIXTEENTH);
    // initialise sensor, delaying to make debug easier
    hal.scheduler->delay(2000);
    driver.init();
    driver.enable();
    driver.use_SPI(true);
    driver.use_IVREF(true);
}

const uint32_t steps_per_direction = 5000;
uint32_t step_count = 0;
bool direction = false;

void loop()
{
    if (step_count >= steps_per_direction) {
        direction = !direction;
        driver.dir(direction);
        hal.console->printf("Changing direction to %s\n", direction ? "forward" : "reverse");
        step_count = 0;
    }

    driver.step();
    step_count++;
    hal.scheduler->delay(1);
}

GCS_Dummy _gcs;
AP_HAL_MAIN();
