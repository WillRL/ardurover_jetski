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
#pragma once

#include "AP_SteeringEncoder_config.h"

#if AP_STEERINGENCODER_ENABLED

#include <AP_Param/AP_Param.h>
#include <Filter/Filter.h>
#include <GCS_MAVLink/GCS_MAVLink.h>


class AP_SteeringEncoder_Backend;

class AP_SteeringEncoder
{
    friend class AP_SteeringEncoder_Backend;
    friend class AP_SteeringEncoder_Home;
    friend class AP_SteeringEncoder_Analog;
    friend class AP_SteeringEncoder_SITL;
    friend class AP_SteeringEncoder_ModernDevice;
    friend class AP_SteeringEncoder_Airspeed;
    friend class AP_SteeringEncoder_RPM;
    friend class AP_SteeringEncoder_NMEA;

public:
    AP_SteeringEncoder();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_SteeringEncoder);

    static AP_SteeringEncoder *get_singleton();

    // return true if wind vane is enabled
    bool enabled() const;

    // return true if wind speed is enabled
    bool wind_speed_enabled() const;

    // Initialize the Wind Vane object and prepare it for use
    void init();

    // update wind vane
    void update();

    // get the apparent wind direction in body-frame in radians, 0 = head to wind
    float get_apparent_wind_direction_rad() const { return _direction_apparent; }

    // get the true wind direction in radians, 0 = wind coming from north
    float get_true_wind_direction_rad() const { return _direction_true; }

    // Return apparent wind speed
    float get_apparent_wind_speed() const { return _speed_apparent; }

    // Return true wind speed
    float get_true_wind_speed() const { return _speed_true; }

    // Return the apparent wind angle used to determin the current tack
    float get_tack_threshold_wind_dir_rad() const { return _direction_tack; }

    // enum defining current tack
    enum Sailboat_Tack {
        TACK_PORT,
        TACK_STARBOARD
    };

    // return the current tack
    Sailboat_Tack get_current_tack() const { return _current_tack; }

    // record home heading for use as wind direction if no sensor is fitted
    void record_home_heading();

    // start calibration routine
    bool start_direction_calibration();
    bool start_speed_calibration();

    // send mavlink wind message
    void send_wind(mavlink_channel_t chan) const;

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:

    // parameters
    AP_Int8 _encoder_type;                        // type of windvane being used
    AP_Int8 _dir_analog_pin;                        // analog pin connected to wind vane direction sensor
    AP_Float _dir_analog_volt_min;                  // minimum voltage read by windvane
    AP_Float _dir_analog_volt_max;                  // maximum voltage read by windvane
    AP_Float _dir_analog_bearing_offset;            // angle offset when windvane is indicating a headwind, ie 0 degress relative to vehicle
    AP_Float _dir_analog_deadzone;                  // analog pot deadzone in degrees
    AP_Float _dir_filt_hz;                          // vane Low pass filter frequency
    AP_Int8 _calibration;                           // enter calibration
    AP_Float _dir_speed_cutoff;                     // vane cutoff wind speed
    AP_Int8 _speed_sensor_type;                     // wind speed sensor type
    AP_Int8 _speed_sensor_speed_pin;                // speed sensor analog pin for reading speed
    AP_Float _speed_sensor_temp_pin;                // speed sensor analog pin for reading temp, -1 if disable
    AP_Float _speed_sensor_voltage_offset;          // analog speed zero wind voltage offset
    AP_Float _speed_filt_hz;                        // speed sensor low pass filter frequency
    AP_Float _true_filt_hz;                         // true wind speed and direction low pass filter frequency

    AP_SteeringEncoder_Backend *_direction_driver;
    AP_SteeringEncoder_Backend *_speed_driver;

    // update wind speed sensor
    void update_apparent_wind_speed();

    // update apparent wind direction
    void update_apparent_wind_direction();

    // calculate true wind speed and direction from apparent wind
    void update_true_wind_speed_and_direction();

    // assume true wind has not changed and calculate apparent wind
    void update_apparent_wind_dir_from_true();

    // wind direction variables
    float _direction_apparent_raw;                  // wind's apparent direction in radians (0 = ahead of vehicle) in body frame
    float _direction_apparent;                      // wind's apparent direction in radians (0 = ahead of vehicle) in body frame - filtered
    float _direction_true_raw;                      // wind's true direction in radians (0 = North)
    float _direction_true;                          // wind's true direction in radians (0 = North) - filtered
    float _direction_tack;                          // filtered apparent wind used to determin the current tack
    LowPassFilterFloat _direction_apparent_sin_filt{2.0f};
    LowPassFilterFloat _direction_apparent_cos_filt{2.0f};
    LowPassFilterFloat _direction_true_sin_filt{2.0f};
    LowPassFilterFloat _direction_true_cos_filt{2.0f};
    LowPassFilterFloat _tack_sin_filt{0.1f};
    LowPassFilterFloat _tack_cos_filt{0.1f};

    // wind speed variables
    float _speed_apparent_raw;                      // wind's apparent speed in m/s
    float _speed_apparent;                          // wind's apparent speed in m/s - filtered
    float _speed_true_raw;                          // wind's true estimated speed in m/s
    float _speed_true;                              // wind's true estimated speed in m/s - filtered
    LowPassFilterFloat _speed_apparent_filt{2.0f};
    LowPassFilterFloat _speed_true_filt{2.0f};

    // current tack
    Sailboat_Tack _current_tack;

    // heading in radians recorded when vehicle was armed
    float _home_heading;

    enum SteeringEncoderType {
        STEERINGENCODER_NONE           = 0,
        #if AP_STEERINGENCODER_AS5600I2C_ENABLED
                AP_STEERINGENCODER_AS5600I2C   = 1,
                STEERINGENCODER_AS5600I2C_ADDR     = 0x36,
        #endif
    };


    static AP_SteeringEncoder *_singleton;
};

namespace AP {
    AP_SteeringEncoder *windvane();
};

#endif  // AP_STEERINGENCODER_ENABLED
