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
#include <AP_DAC/AP_DAC.h>
#include <AP_HAL/I2CDevice.h>

void setup();    // declaration of the setup() function
void loop();     // declaration of the loop() function

const AP_HAL::HAL& hal = AP_HAL::get_HAL(); // create an instance of the dac
AP_DAC dac;
AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev = nullptr;
namespace {
// try to set the object value but provide diagnostic if it failed
void set_object_value(const void *object_pointer,
                      const struct AP_Param::GroupInfo *group_info,
                      const char *name, float value)
    {
        // THIS DOES NOT WORK FOR SETTING SUBGROUPS.
        if (!AP_Param::set_object_value(object_pointer, group_info, name, value)) {
            hal.console->printf("WARNING: AP_Param::set object value \"%s::%s\" Failed.\n",
                                group_info->name, name);
        }
    }
}

void setup() {
    hal.scheduler->delay(5000);
    hal.console->printf("Initialising MCP47FxBxx test\nDoing I2C bus scan...\n");    // print a starting message

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
    
    set_object_value(&dac.params[0], dac.params[0].var_info, "TYPE", 3);
    set_object_value(&dac.params[0], dac.params[0].var_info, "VREF", 5);
    dac.init();
    hal.scheduler->delay(5000);
    hal.console->printf("Starting MCP47FxBxx test\n");    // print a starting message

}

// the loop function runs over and over again forever
float voltage = 0;
void loop()
{   
    voltage = sin(10*AP_HAL::millis()/1000.0f) * 2.5 + 2.5;
    dac.set_voltage(0, 0, voltage);
    dac.set_voltage(0, 1, voltage);
    dac.set_voltage(0, 2, voltage);
    dac.set_voltage(0, 3, voltage);
    hal.console->printf("voltage: %f\n", voltage);
    hal.scheduler->delay(1);
}

AP_HAL_MAIN();    // HAL Macro that declares the main function. For more info see <https://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html/>


