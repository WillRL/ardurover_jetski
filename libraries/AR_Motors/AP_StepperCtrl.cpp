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
 Closed loop stepper motor position controller by adjusting PWM frequency to control stepper motor speed.
 */

#include "AP_StepperCtrl.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <cmath>
extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_StepperCtrl::var_info[] = {
    // @Param: STPR_STR
    // @DisplayName: Stepper motor for steering
    // @Description: Allows stepper motor control for steering
    // @Values: 0:False,1:True
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("STR", 1, AP_StepperCtrl, is_active, 0),

    // @Param: G_RATIO
    // @DisplayName: Stepper Motor Gear Ratio
    // @Description: The gear ratio between the encoder and the steering angle.
    // @User: Standard
    AP_GROUPINFO("G_RATIO", 2, AP_StepperCtrl, _gear_ratio, 1),

    // @Param: MINMAX
    // @DisplayName: Steering Angle Minimum/Maximum
    // @Description: Minimum and maximum steering angle for the stepper motor (+-MINMAX). Usually governed by physical limits.
    // @User: Standard
    AP_GROUPINFO("MINMAX", 3, AP_StepperCtrl, _min_max, 45),

    // @Param: DIR_PIN
    // @DisplayName: Stepper Direction Pin
    // @Description: Pin used for stepper motor direction control. This pin is used to set the direction of the stepper motor when using it for steering. Select Main or Aux depending on TTL requrired for stepper driver.
    // @Values: 101:MAIN1,102:MAIN2,103:MAIN3,104:MAIN4,105:MAIN5,106:MAIN6,107:MAIN7,108:MAIN8,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    AP_GROUPINFO("DIR_PIN", 4, AP_StepperCtrl, _stepper_direction_pin, 108),

    // @Param: SMTH
    // @DisplayName: Stepper Motor Frequency Smoothing
    // @Description: Smoothing for the stepper motor frequency control. This is used to smooth out jitters in the stepper motor.
    // @User: Standard
    AP_GROUPINFO("SMTH", 5, AP_StepperCtrl, _smoothing, 1),

    // @Param: LOOP_FREQ
    // @DisplayName: Stepper Motor Loop Frequency
    // @Description: The loop frequency for the stepper motor control. This is used to determine how often the control loop is updated.
    // @Units: Hz
    // @Range: 1 400
    // @User: Advanced
    AP_GROUPINFO("LOOP_FREQ", 6, AP_StepperCtrl, _loop_freq, 400),

    // @Param: PID_
    // @Path: ../PID/PID.cpp
    AP_SUBGROUPINFO(_pid, "",  7, AP_StepperCtrl, PID),
    
    
    AP_GROUPEND
};

// Constructor
AP_StepperCtrl::AP_StepperCtrl(){
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_StepperCtrl::update(float curr_state){
    // Calculate the next signal based on the setpoint and gear ratio
    // curr_state is the encoders current state. This is usually placed directly on the stepper motor shaft.

    // Limit the loop frequency to smooth stepper motor jitters.
    if (AP_HAL::millis() - _prev_time < 1/(_loop_freq) * 1000) {
        return;
    } else {
        _prev_time = AP_HAL::millis(); // Update the previous time to the current time
    }


    // Requested angle from autopilot is for the steering angle. Convert this to stepper motor angle (Or encoder angle if the encoder is not on stepper shaft)
    float stepper_setpoint = MIN(setpoint * _gear_ratio, 180); // 180 is the hard limit as this only supports absolute encoders.
    float stepper_min_max = _min_max * (float) _gear_ratio;
    stepper_setpoint = MAX(MIN(stepper_setpoint, stepper_min_max), -stepper_min_max);

    float error = stepper_setpoint - curr_state;
    

    float control_signal = _pid.get_pid(error, 1);
    control_signal = MAX(MIN(control_signal, 40000), -40000); // Keep the control signal within frequency limits.
    
    // Round the control signal to the nearest sig fig to reduce the amount of frequency changes. This lessens jitters slightly
    int rounding = pow(10, int(_smoothing));
    control_signal = rounding * round((control_signal/rounding));

    // Write direction and frequency.
    hal.gpio->pinMode(_stepper_direction_pin, HAL_GPIO_OUTPUT);
    hal.gpio->write(_stepper_direction_pin, signbit(control_signal));
    uint32_t motor_mask = SRV_Channels::get_output_channel_mask(SRV_Channel::k_steering);
    hal.rcout->set_freq(motor_mask, abs(control_signal));

    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "ct: %0.3f st: %0.3f, sp:  %0.3f, e: %0.3f", control_signal, curr_state, stepper_setpoint, error);
}