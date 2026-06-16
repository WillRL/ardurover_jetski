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
/*
    Printf.cpp: We demonstrate the use of the printf() and snprintf() functions
*/
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_StepperDriver/AP_StepperDriver_DRV8462SPI.h>
#include <AP_StepperDriver/AP_StepperDriver.h>
#include <GCS_MAVLink/GCS_Dummy.h>

void setup();    // declaration of the setup() function
void loop();     // declaration of the loop() function

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
AP_StepperDriver stepper_frontend;
AP_StepperDriver_DRV8462SPI stepper_driver(stepper_frontend);

void setup() {
    hal.scheduler->delay(5000);
    hal.console->printf("Starting DRV8462 test\n");    // print a starting message

    stepper_driver.init();
    AP_StepperDriver_DRV8462SPI::config cfg;
    cfg.en_out = 1;
    cfg.spi_step = 1;
    cfg.spi_dir = 1;
    cfg.vref_int_en = 1;
    cfg.trq_dac = 256 >> 2;  // 2.5A run current
    cfg.istsl = 256 >> 4;    // 25% of run
    cfg.microstep_mode = 0b0110;
    hal.console->printf("Writing config\n");
    bool ok = stepper_driver.write_config(cfg);

    uint8_t data;
    stepper_driver.read_reg(DRV8462_CTRL2_REG, &data);
    hal.console->printf("write:%i data: %u\n", ok, data);
}

// the loop function runs over and over again forever
void loop()
{   
    // give a delay of 1000ms or 1s
    hal.console->printf("stepping\n");
    hal.scheduler->delay(1);
    AP_StepperDriver_DRV8462SPI::config cfg;
    cfg.dir = 1;
    cfg.step = 1;
    stepper_driver.write_config(cfg);
}
GCS_Dummy _gcs;
AP_HAL_MAIN();    // HAL Macro that declares the main function. For more info see <https://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html/>


