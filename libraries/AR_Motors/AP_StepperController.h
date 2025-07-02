#pragma once

#include <stdint.h>
#include <SRV_Channel/SRV_Channel.h>
#include <PID/PID.h>


class AP_StepperController {
    public:
        // Setpoint for the target angle
        float setpoint;

        // Constructor
        AP_StepperController();
        
        // Speed is defined as a PWM. Every rising edge is a single step, so a PWM 
        void update(float curr_state);

        static const struct AP_Param::GroupInfo var_info[];
        AP_Int8 is_active; // Boolean, if steering motor is a stepper motor.  
        AP_Int8 encoder_analog_pin; // Analog pin for encoder input.  
        
    private:
        AP_Float _gear_ratio; // Gear ratio between stepper motor and steering angle.
        AP_Float _min_max; // Min/Max angle for the stepper motor.
        AP_Int8 _stepper_direction_pin;  // PWM output type.
        AP_Int32 _loop_freq;
        PID _pid;
        float _prev_time; // Previous time.
};