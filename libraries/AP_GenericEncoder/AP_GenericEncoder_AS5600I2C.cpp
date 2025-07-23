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

#include "AP_GenericEncoder_config.h"
#if AP_GENERICENCODER_ENABLED
#if AP_GENERICENCODER_AS5600I2C_ENABLED

#include "AP_GenericEncoder_AS5600I2C.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_GenericEncoder_AS5600I2C::AP_GenericEncoder_AS5600I2C() { };

/*
   Initialize the AS5600 I2C device. This device only has one I2C address, so the address parameter is ignored.
*/
void AP_GenericEncoder_AS5600I2C::init(float *_ptr_pos, float *_ptr_dt_pos, float *_ptr_ddt_pos, float *_ptr_latest_measurement_time)
{   
    _pos = _ptr_pos;
    _dt_pos = _ptr_dt_pos;
    _ddt_pos = _ptr_ddt_pos;
    _latest_measurement_time = _ptr_latest_measurement_time;
    
    uint8_t recv = 0;
    FOREACH_I2C(i) {
        _dev = hal.i2c_mgr->get_device(i, AS5600_ADDRESS);
        WITH_SEMAPHORE(_dev->get_semaphore());
        if (_dev->read_registers(0, &recv, 1)){
            hal.console->printf("Found AS5600 on bus %li address 0x%02x\n", i, AS5600_ADDRESS);
            break;
        } else {
            hal.console->printf("Could not find AS5600");
        } 
    }

    

    
    // setup();

}

void AP_GenericEncoder_AS5600I2C::setup()
{
    // set the zero position to the current angle
    hal.console->printf("AP_GenericEncoder_AS5600I2C: Setup complete.");
}

void AP_GenericEncoder_AS5600I2C::calibrate()
{
    set_zero_position();

}

void AP_GenericEncoder_AS5600I2C::calibrate(uint32_t angle)
{
    if (angle > 360) {
        hal.console->printf("Angle must be between 0 and 360 degrees.");
        return;
    }
    set_zero_position();

}

/*
Set the zero position of the encoder.
This function reads the current angle from the encoder and sets it as the zero position.
*/
void AP_GenericEncoder_AS5600I2C::set_zero_position()
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->set_retries(10);

    int ret;
    uint8_t msb = 0;
    uint8_t lsb = 0;

    ret = _dev->read_registers(AS5600_RAWANGLE_MSB_REG, &msb, 1);
    if (!ret) { 
        return; 
    }

    ret = _dev->read_registers(AS5600_RAWANGLE_LSB_REG, &lsb, 1);
    if (!ret) { 
        return; 
    }

    ret = _dev->write_register(AS5600_ZPOS_MSB_REG, msb);
    if (!ret) {
        return;
    }
    ret = _dev->write_register(AS5600_ZPOS_LSB_REG, lsb);
    if (!ret) {
        return;
    }

    hal.scheduler->delay_microseconds(1000); // wait for the write to complete
    _dev->set_retries(2);
}

/*
Set the maximum position of the encoder.
This function reads the current angle from the encoder and sets it as the maximum position.
Do not use this if maximum angle is set.
*/
void AP_GenericEncoder_AS5600I2C::set_maximum_position()
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->set_retries(10);
    // set initial position to zero by setting the zero register to zero,
    // reading the measured angle, and setting the register to this angle.
    int ret;
    uint8_t msb = 0;
    uint8_t lsb = 0;

    ret = _dev->read_registers(AS5600_RAWANGLE_MSB_REG, &msb, 1);
    if (!ret) { 
        return; 
    }

    ret = _dev->read_registers(AS5600_RAWANGLE_LSB_REG, &lsb, 1);
    if (!ret) { 
        return; 
    }

    ret = _dev->write_register(AS5600_MPOS_MSB_REG, msb);
    if (!ret) {
        return;
    }
    ret = _dev->write_register(AS5600_MPOS_LSB_REG, lsb);
    if (!ret) {
        return;
    }

    hal.scheduler->delay_microseconds(1000); // wait for the write to complete
    _dev->set_retries(2);
}

/*
Set the maximum angle of the encoder.
This function takes an angle in degrees and sets it as the maximum angle the encoder is expected to hit. 
Do not use this if maximum position is set. 
*/
void AP_GenericEncoder_AS5600I2C::set_maximum_angle(uint32_t angle)
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->set_retries(10);
    int ret;
    uint16_t adc_value = angle/360 * 4096;

    uint8_t msb = (adc_value >> 8) & 0x0F;
    uint8_t lsb = adc_value & 0xFF;

    ret = _dev->write_register(AS5600_MANG_MSB_REG, msb);
    if (!ret) {
        return;
    }
    ret = _dev->write_register(AS5600_MANG_LSB_REG, lsb);
    if (!ret) {
        return;
    }

    hal.scheduler->delay_microseconds(1000); // wait for the write to complete
    _dev->set_retries(2);
}

/*
Update the encoder's position, velocity, and acceleration.
*/
void AP_GenericEncoder_AS5600I2C::update()
{
    update_pos();
    update_velocity();
    update_acceleration();
}

/*
Update the encoder's position.
*/
void AP_GenericEncoder_AS5600I2C::update_pos()
{
    _dev->set_retries(10);
    _prev_measurement_time = *_latest_measurement_time;
    _prev_pos = *_pos;

    WITH_SEMAPHORE(_dev->get_semaphore());

    uint8_t msb = 0;
    int ret = _dev->read_registers(AS5600_ANGLE_MSB_REG, &msb, 1);
    if (!ret) {
        return;
    }
    uint8_t lsb = 0;
    ret = _dev->read_registers(AS5600_ANGLE_LSB_REG, &lsb, 1);
    if (!ret) {
        return;
    }
    
    *_latest_measurement_time = AP_HAL::millis()/1000.0f;
    *_pos = ((msb << 8) | lsb) * AS5600_ADC2RAD;
    _dev->set_retries(2);
}

/*
Update the encoder's velocity.
*/
void AP_GenericEncoder_AS5600I2C::update_velocity()
{
    _prev_dt_pos = *_dt_pos;
    float dpos = calc_abs_rotary_d_pos(*_pos, _prev_pos);
    *_dt_pos = dpos / (*_latest_measurement_time - _prev_measurement_time);
}

/*
Update the encoder's acceleration.
*/
void AP_GenericEncoder_AS5600I2C::update_acceleration()
{
    *_ddt_pos = (*_dt_pos - _prev_dt_pos) / (*_latest_measurement_time - _prev_measurement_time);
}


#endif // AP_GENERICENCODER_AS5600I2C_ENABLED
#endif // AP_GENERIC_ENCODER_ENABLED