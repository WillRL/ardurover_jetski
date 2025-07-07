#pragma once

#include <stdint.h>
#include <SRV_Channel/SRV_Channel.h>
#include <PID/PID.h>
#include <AP_StepperEncoder/AP_StepperEncoder.h>


class AP_StepperController {
    public:
        // Setpoint for the target angle
        float setpoint;

        // Constructor
        AP_StepperController();

        void init();
        
        // Speed is defined as a PWM. Every rising edge is a single step, so a PWM 
        void update();

        static const struct AP_Param::GroupInfo var_info[];
        AP_Int8 is_active; // Boolean, if steering motor is a stepper motor.  
        
        AP_StepperEncoder encoder_frontend;
    private:
        AP_Float _gear_ratio; // Gear ratio between stepper motor and steering angle.
        AP_Float _min_max; // Min/Max angle for the stepper motor.
        AP_Int8 _stepper_direction_pin;  // PWM output type.
        AP_Int32 _max_freq;
        PID _pid_angle;
        // PID _pid_rate;
        float _prev_time; // Previous time.
        float _rad2deg = (180/M_PI);
};