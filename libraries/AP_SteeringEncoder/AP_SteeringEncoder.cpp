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

#include "AP_SteeringEncoder_config.h"

#if AP_STEERINGENCODER_ENABLED

#include "AP_SteeringEncoder.h"
#include "AP_SteeringEncoder_AS5600I2C.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>



extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_SteeringEncoder::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Wind Vane Type
    // @Description: Wind Vane type
    // @Values: 0:None,1:Heading when armed,2:RC input offset heading when armed,3:Analog,4:NMEA,10:SITL true,11:SITL apparent
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_SteeringEncoder, _encoder_type, 0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

// constructor
AP_SteeringEncoder::AP_SteeringEncoder()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton) {
        AP_HAL::panic("Too many Wind Vane sensors");
    }
#endif
    _singleton = this;
}

/*
 * Get the AP_SteeringEncoder singleton
 */
AP_SteeringEncoder *AP_SteeringEncoder::get_singleton()
{
    return _singleton;
}

// return true if wind vane is enabled
bool AP_SteeringEncoder::enabled() const
{
    return _encoder_type != STEERINGENCODER_NONE;
}


// Initialize the Wind Vane object and prepare it for use
void AP_SteeringEncoder::init()
{
    // don't construct twice
    if (_direction_driver != nullptr || _speed_driver != nullptr ) {
        return;
    }

    // wind direction
    switch (_encoder_type) {
        case SteeringEncoderType::STEERINGENCODER_NONE:
            // SteeringEncoder disabled
            return;
        
        #if AP_STEERINGENCODER_AS5600I2C_ENABLED
            case SteeringEncoderType::STEERINGENCODER_AS5600I2C_ADDR:
                return; //_direction_driver = NEW_NOTHROW AP_SteeringEncoder_AS5600I2C(*this);
        #endif

    }
}

// update wind vane, expected to be called at 20hz
void AP_SteeringEncoder::update()
{
    //_direction_driver->update();
}

void AP_SteeringEncoder::record_home_heading()
{
    _home_heading = AP::ahrs().get_yaw_rad();
}

// to start direction calibration from mavlink or other
bool AP_SteeringEncoder::start_direction_calibration()
{
    if (enabled() && (_calibration == 0)) {
        _calibration.set(1);
        return true;
    }
    return false;
}

// to start speed calibration from mavlink or other
bool AP_SteeringEncoder::start_speed_calibration()
{
    if (enabled() && (_calibration == 0)) {
        _calibration.set(2);
        return true;
    }
    return false;
}

// send mavlink wind message
void AP_SteeringEncoder::send_wind(mavlink_channel_t chan) const
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // send wind
    mavlink_msg_wind_send(
        chan,
        wrap_360(degrees(get_true_wind_direction_rad())),
        get_true_wind_speed(),
        0);

    // send apparent wind using named floats
    // TODO: create a dedicated MAVLink message
    gcs().send_named_float("AppWndSpd", get_apparent_wind_speed());
    gcs().send_named_float("AppWndDir", degrees(get_apparent_wind_direction_rad()));

}

// calculate true wind speed and direction from apparent wind
// https://en.wikipedia.org/wiki/Apparent_wind
void AP_SteeringEncoder::update_true_wind_speed_and_direction()
{
    // if no vehicle speed, can't do calcs
    Vector3f veh_velocity;
    if (!AP::ahrs().get_velocity_NED(veh_velocity)) {
        // if no vehicle speed use apparent speed and direction directly
        _direction_true_raw = _direction_apparent_raw;
        _speed_true_raw = _speed_apparent;
        return;
    }

    // convert apparent wind speed and direction to 2D vector in same frame as vehicle velocity
    const float wind_dir_180 = _direction_apparent_raw + AP::ahrs().get_yaw_rad() + radians(180);
    const Vector2f wind_apparent_vec(cosf(wind_dir_180) * _speed_apparent, sinf(wind_dir_180) * _speed_apparent);

    // add vehicle velocity
    Vector2f wind_true_vec = Vector2f(wind_apparent_vec.x + veh_velocity.x, wind_apparent_vec.y + veh_velocity.y);

    // calculate true speed and direction
    _direction_true_raw = wrap_PI(atan2f(wind_true_vec.y, wind_true_vec.x) - radians(180));
    _speed_true_raw = wind_true_vec.length();
}

// apparent wind is low, can't trust sensors, assume true wind has not changed and calculate apparent wind
void AP_SteeringEncoder::update_apparent_wind_dir_from_true()
{
    // if no vehicle speed, can't do calcs
    Vector3f veh_velocity;
    if (!AP::ahrs().get_velocity_NED(veh_velocity)) {
        return;
    }

    // convert true wind speed and direction to 2D vector in same frame as vehicle velocity
    const float wind_dir_180 = _direction_true + radians(180);
    const Vector2f wind_true_vec(cosf(wind_dir_180) * _speed_true, sinf(wind_dir_180) * _speed_true);

    // add vehicle velocity
    Vector2f wind_apparent_vec = Vector2f(wind_true_vec.x - veh_velocity.x, wind_true_vec.y - veh_velocity.y);

    // calculate apartment speed and direction
    _direction_apparent_raw = wrap_PI(atan2f(wind_apparent_vec.y, wind_apparent_vec.x) - radians(180) - AP::ahrs().get_yaw_rad());
    _speed_apparent_raw = wind_apparent_vec.length();
}

AP_SteeringEncoder *AP_SteeringEncoder::_singleton = nullptr;

namespace AP {
    AP_SteeringEncoder *windvane()
    {
        return AP_SteeringEncoder::get_singleton();
    }
};

#endif  // AP_STEERINGENCODER_ENABLED
