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

#include "AP_GenericEncoder_config.h"
#if AP_GENERICENCODER_ENABLED
#if AP_GENERICENCODER_AS5600I2C_ENABLED

#include "AP_GenericEncoder.h"
#include <AP_HAL/I2CDevice.h>

#define I2C_BUS 1
#define AS5600_ADDRESS              0x36
#define AS5600_ADC2RAD              (2 * M_PI / 4096.0f)

#define AS5600_ZMCO_REG             0x00 // ZPOS and MPOS burn count
#define AS5600_ZPOS_MSB_REG         0x01 // Start position MSB
#define AS5600_ZPOS_LSB_REG         0x02 // Start position LSB
#define AS5600_MPOS_MSB_REG         0x03 // Stop position MSB
#define AS5600_MPOS_LSB_REG         0x04 // Stop position LSB
#define AS5600_MANG_MSB_REG         0x05 // Maximum angle MSB
#define AS5600_MANG_LSB_REG         0x06 // Maximum angle LSB
#define AS5600_CONF1_REG            0x07 // 0,0,WD,FTH(2:0),SF(1:0)
#define AS5600_CONF2_REG            0x08 // PWMF(1:0), OUTS(1:0), HYST(1:0), PM(1:0)
#define AS5600_RAWANGLE_MSB_REG     0x0C // Raw angle output (unscaled, unmodified) MSB 
#define AS5600_RAWANGLE_LSB_REG     0x0D // Raw angle output (unscaled, unmodified) LSB
#define AS5600_ANGLE_MSB_REG        0x0E // Angle output MSB
#define AS5600_ANGLE_LSB_REG        0x0F // Angle output LSB
#define AS5600_STATUS_REG           0x0B // Status register: 0,0,MD,ML,MH,0,0,0
#define AS5600_AGC_REG              0x1A // Current automatic gain value
#define AS5600_MAG_MSB_REG          0x1B // MMagnitude in CORDIC MSB
#define AS5600_MAG_LSB_REG          0x1C // MMagnitude in CORDIC LSB
#define AS5600_BURN_REG             0xFF // Burn register, write 0x80 to burn angle (ZPOS, MPOS, MANG) - max 3 times, 0x40 to burn settings (CONF1, CONF2) - max 1 time. 


class AP_GenericEncoder_AS5600I2C: public AP_GenericEncoder{
public:
    // constructor
    AP_GenericEncoder_AS5600I2C();
    ~AP_GenericEncoder_AS5600I2C(void) {};

    // initialization
    void init(float *_ptr_pos, float *_ptr_dt_pos, float *_ptr_ddt_pos, float *_ptr_latest_measurement_time) override;
    void init(float *_ptr_pos, float *_ptr_dt_pos, float *_ptr_ddt_pos);
    void init(float *_ptr_pos, float *_ptr_dt_pos);
    void init(float *_ptr_pos);

    // update state
    void update() override;
    void update_pos() override;
    void update_velocity() override;
    void update_acceleration() override;
    void calibrate() override;
    void calibrate(uint32_t angle);

    void set_zero_position();
    void set_maximum_position();
    void set_maximum_angle(uint32_t angle);

private:
    // latest values read in
    void setup() override;

    // pointer to I2C device
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev = nullptr;
};

#endif // AP_GENERICENCODER_AS5600I2C_ENABLED
#endif // AP_GENERICENCODER_ENABLED