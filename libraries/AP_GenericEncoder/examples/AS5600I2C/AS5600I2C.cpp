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
#include <AP_GenericEncoder/AP_GenericEncoder_AS5600I2C.h>
#include <AP_HAL/I2CDevice.h>

void setup();    // declaration of the setup() function
void loop();     // declaration of the loop() function

const AP_HAL::HAL& hal = AP_HAL::get_HAL(); // create an instance of the encoder
AP_GenericEncoder_AS5600I2C encoder;
AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev = nullptr;
float pos;
float dt_pos;
float ddt_pos;
float _latest_measurement_time;

void setup() {
    hal.scheduler->delay(5000);
    hal.console->printf("Starting AS5600I2C test\n");    // print a starting message

    uint8_t recv = 0;
    FOREACH_I2C(i) {
        hal.console->printf("Checking bus %li\n", i);
        for(int address = 1; address < 127; address++ ){
            _dev = hal.i2c_mgr->get_device(i, address);
            WITH_SEMAPHORE(_dev->get_semaphore());
            if (_dev->read_registers(0, &recv, 1)){
                hal.console->printf("Found device on bus %li address 0x%02x\n", i, address);
            } 
        }
    }

    encoder.init(&pos, &dt_pos, &ddt_pos, &_latest_measurement_time);
}

// the loop function runs over and over again forever
void loop()
{   
    encoder.update(); // update the encoder state
    hal.console->printf("Position: %f, Velocity: %f, Acceleration: %f\n",
                    pos*57.2958, dt_pos*57.2958, ddt_pos*57.2958); // print the values to the console
    // give a delay of 1000ms or 1s
    hal.scheduler->delay(1);
}

AP_HAL_MAIN();    // HAL Macro that declares the main function. For more info see <https://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html/>


