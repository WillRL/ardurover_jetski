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


const AP_HAL::HAL& hal = AP_HAL::get_HAL();
AP_GenericEncoder_AS5600I2C *encoder =  new AP_GenericEncoder_AS5600I2C(); // create an instance of the encoder

float retvals[3]; // array to hold the returned values
void setup() {
    hal.console->printf("Starting AS5600I2C test\n");    // print a starting message
    
}

// the loop function runs over and over again forever
void loop()
{   
    try{
        // encoder->update(); // update the encoder state
        // encoder->read(retvals); // read the encoder values
        // hal.console->printf("Position: %.2f, Velocity: %.2f, Acceleration: %.2f\n",
        //                retvals[0], retvals[1], retvals[2]); // print the values to the console
        // give a delay of 1000ms or 1s
        hal.scheduler->delay(1000);
    }
    catch (const std::exception &exc){
    // catch anything thrown within try block that derives from std::exception
        hal.console->printf("Exception caught: %s\n", exc.what()); // print the exception message
    }
    catch (...){
        hal.console->printf("Unknown exception caught\n"); // print a generic message for unknown exceptions
    }
    
}

AP_HAL_MAIN();    // HAL Macro that declares the main function. For more info see <https://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html/>


