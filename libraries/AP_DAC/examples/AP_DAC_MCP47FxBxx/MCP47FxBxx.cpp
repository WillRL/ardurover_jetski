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
#include <AP_DAC/AP_DAC_Params.h>
#include <AP_DAC/AP_DAC_MCP47FxBxx.h>
#include <AP_HAL/I2CDevice.h>
#include <GCS_MAVLink/GCS_Dummy.h>
void setup();    // declaration of the setup() function
void loop();     // declaration of the loop() function

const AP_HAL::HAL& hal = AP_HAL::get_HAL(); // create an instance of the dac
AP_DAC_Params params;
AP_DAC_MCP47FxBxx dac(params);

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
    hal.console->printf("Initialising MCP47FxBxx test...\n");    // print a starting message
    
    set_object_value(&params, params.var_info, "TYPE", 3);
    set_object_value(&params, params.var_info, "VREF", 5);
    dac.init();
    hal.scheduler->delay(5000);
    hal.console->printf("Starting MCP47FxBxx test\n");    // print a starting message

}

// the loop function runs over and over again forever
float voltage = 0;
bool chan0;
bool chan1;
bool chan2;
bool chan3;
uint16_t por;
uint16_t eewa;
uint16_t wiperlock;
int delay = 10; // 10
int count = 0;
void loop()
{   
    voltage = sin(10*AP_HAL::millis()/1000.0f) * 2.5 + 2.5;
    chan0 = dac.set_voltage(0, voltage);
    // hal.scheduler->delay(delay);
    chan1 = dac.set_voltage(1, voltage);
    // hal.scheduler->delay(delay);
    chan2 = dac.set_voltage(2, voltage);
    // hal.scheduler->delay(delay);
    chan3 = dac.set_voltage(3, voltage);
    // hal.scheduler->delay(delay);
    count++;
    

    hal.console->printf("set: %i%i%i%i-%i\n", chan0, chan1, chan2, chan3, count);
    if (!(chan0 & chan1 & chan2 & chan3)){
        hal.console->printf("Failed setting channels. Resetting count.\n");
        hal.scheduler->delay(10000);
        count = 0;
    }
    // hal.scheduler->delay(100);
}
GCS_Dummy _gcs;
AP_HAL_MAIN();    // HAL Macro that declares the main function. For more info see <https://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html/>


