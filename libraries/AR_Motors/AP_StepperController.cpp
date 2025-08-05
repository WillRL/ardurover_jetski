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

#include "AP_StepperController.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <cmath>
extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_StepperController::var_info[] = {
    // @Param: STPR_EN
    // @DisplayName: Enable stepper motor for steering
    // @Description: Allows stepper motor control for steering
    // @Values: 0:False,1:True
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("EN", 1, AP_StepperController, is_active, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: G_RATIO
    // @DisplayName: Stepper Motor Gear Ratio
    // @Description: The gear ratio between the encoder and the steering angle.
    // @User: Standard
    AP_GROUPINFO("G_RATIO", 2, AP_StepperController, _gear_ratio, 1),

    // @Param: MINMAX
    // @DisplayName: Steering Angle Minimum/Maximum
    // @Description: Minimum and maximum steering angle for the stepper motor (+-MINMAX). Usually governed by physical limits.
    // @User: Standard
    AP_GROUPINFO("MINMAX", 3, AP_StepperController, _min_max, 45),

    // @Param: DIR_PIN
    // @DisplayName: Stepper Direction Pin
    // @Description: Pin used for stepper motor direction control. This pin is used to set the direction of the stepper motor when using it for steering. Select Main or Aux depending on TTL requrired for stepper driver.
    // @Values: 101:MAIN1,102:MAIN2,103:MAIN3,104:MAIN4,105:MAIN5,106:MAIN6,107:MAIN7,108:MAIN8,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    AP_GROUPINFO("DIR_PIN", 4, AP_StepperController, _stepper_direction_pin, 108),

    // @Param: MAX_FREQ
    // @DisplayName: Max stepping speed
    // @Description: Pin used for stepper motor direction control. This pin is used to set the direction of the stepper motor when using it for steering. Select Main or Aux depending on TTL requrired for stepper driver.
    // @Values: 0..40000
    // @User: Standard
    AP_GROUPINFO("MAX_FREQ", 5, AP_StepperController, _max_freq, 40000),

    // @Param: EN_PIN
    // @DisplayName: Stepper motor enable pin
    // @Description: Pin used for enable stepper motor power.
    // @Values: 101:MAIN1,102:MAIN2,103:MAIN3,104:MAIN4,105:MAIN5,106:MAIN6,107:MAIN7,108:MAIN8,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    AP_GROUPINFO("EN_PIN", 6, AP_StepperController, en_pin, 107),

    // @Param: DISARM_PWR
    // @DisplayName: Disarm Power
    // @Description: Provide power to stepper motor on disarm?
    // @Values: 0:No,1:Yes
    // @User: Standard
    AP_GROUPINFO("DISARM_PWR", 7, AP_StepperController, disarm_pwr, 0),

    // @Param: ENC_
    // @Path: ../AP_StepperEncoder/AP_StepperEncoder.cpp
    AP_SUBGROUPINFO(encoder_frontend, "ENC_",  8, AP_StepperController, AP_StepperEncoder),

    // @Param: PID_
    // @Path: ../PID/PID.cpp
    AP_SUBGROUPINFO(_pid_angle, "ANG_",  9, AP_StepperController, PID),

    // @Param: PID_
    // @Path: ../PID/PID.cpp
    // AP_SUBGROUPINFO(_pid_rate, "RAT_",  10, AP_StepperController, PID),
    
    AP_GROUPEND
};

// Constructor
AP_StepperController::AP_StepperController()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_StepperController::init(){
    encoder_frontend.init();
}

void AP_StepperController::update(){
    // Calculate the next signal based on the setpoint and gear ratio
    // curr_state is the encoders current state. This is usually placed directly on the stepper motor shaft.
    float curr_theta = encoder_frontend.theta * _rad2deg - 180.0f;
    // float curr_omega = encoder_frontend.omega * _rad2deg;
    // float dt = AP_HAL::millis() -  _prev_time; 
    

    // Requested angle from autopilot is for the steering angle. Convert this to stepper motor angle (Or encoder angle if the encoder is not on stepper shaft)
    float setpoint_pos = MAX(MIN(setpoint * _gear_ratio, 180), -180); // 180 is the hard limit as this only supports absolute encoders. (Not taking into wrapping of the angle)
    float stepper_min_max = _min_max * (float) _gear_ratio;
    setpoint_pos = MAX(MIN(setpoint_pos, stepper_min_max), -stepper_min_max);

    // Obtain control signal.
    // Position control
    float error_pos = setpoint_pos - curr_theta;
    float control_signal = _pid_angle.get_pid(error_pos, 1);

    // Velocity control
    // float error_omega = setpoint_omega - curr_omega;
    // float alpha = _pid_rate.get_pid(error_omega, 1);

    // Adjust velocity based of acceleration from control (Gives us a slew rate)
    // float control_signal = curr_omega + alpha * dt;
    control_signal = MAX(MIN(control_signal, _max_freq), -_max_freq);
    
    // Write direction and frequency.
    hal.gpio->pinMode(_stepper_direction_pin, HAL_GPIO_OUTPUT);
    hal.gpio->write(_stepper_direction_pin, signbit(control_signal));
    uint32_t motor_mask = SRV_Channels::get_output_channel_mask(SRV_Channel::k_steering);
    hal.rcout->set_freq(motor_mask, abs(control_signal));


    _prev_time = AP_HAL::millis(); // Update the previous time to the current time
    // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "set_om: %0.3f set_al: %0.3f ct: %0.3f", setpoint_omega, alpha, control_signal);
    // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "err_pos: %0.3f err_ome: %0.3f", error_pos, error_omega);
}