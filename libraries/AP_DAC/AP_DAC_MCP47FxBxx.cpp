/*
  DAC backend for MCP47FxBxx I2C DACs
 */
#include "AP_DAC_config.h"

#if AP_DAC_MCP47FXBXX_ENABLED

#include "AP_DAC.h"
#include "AP_DAC_MCP47FxBxx.h"

#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/I2CDevice.h>

extern const AP_HAL::HAL &hal;

constexpr int AP_DAC_MCP47FxBxx::_dac_bitmask[8] = {
    MCP47FXBXX_MASK_DAC0, 
    MCP47FXBXX_MASK_DAC1,
    MCP47FXBXX_MASK_DAC2,
    MCP47FXBXX_MASK_DAC3,    
    MCP47FXBXX_MASK_DAC4,
    MCP47FXBXX_MASK_DAC5,
    MCP47FXBXX_MASK_DAC6,
    MCP47FXBXX_MASK_DAC7
};

constexpr int AP_DAC_MCP47FxBxx::_dac_vol_reg[8] = {
    MCP47FXBXX_REG_VOL_DAC0, 
    MCP47FXBXX_REG_VOL_DAC1,
    MCP47FXBXX_REG_VOL_DAC2,
    MCP47FXBXX_REG_VOL_DAC3,    
    MCP47FXBXX_REG_VOL_DAC4,
    MCP47FXBXX_REG_VOL_DAC5,
    MCP47FXBXX_REG_VOL_DAC6,
    MCP47FXBXX_REG_VOL_DAC7
};

constexpr int AP_DAC_MCP47FxBxx::_dac_eep_reg[8] = {
    MCP47FXBXX_REG_EEP_DAC0, 
    MCP47FXBXX_REG_EEP_DAC1,
    MCP47FXBXX_REG_EEP_DAC2,
    MCP47FXBXX_REG_EEP_DAC3,    
    MCP47FXBXX_REG_EEP_DAC4,
    MCP47FXBXX_REG_EEP_DAC5,
    MCP47FXBXX_REG_EEP_DAC6,
    MCP47FXBXX_REG_EEP_DAC7
};

constexpr int AP_DAC_MCP47FxBxx::_dac_vol_gain_bitmask[8] = {
    MCP47FXBXX_MASK_VOL_GAIN_DAC0, 
    MCP47FXBXX_MASK_VOL_GAIN_DAC1,
    MCP47FXBXX_MASK_VOL_GAIN_DAC2,
    MCP47FXBXX_MASK_VOL_GAIN_DAC3,    
    MCP47FXBXX_MASK_VOL_GAIN_DAC4,
    MCP47FXBXX_MASK_VOL_GAIN_DAC5,
    MCP47FXBXX_MASK_VOL_GAIN_DAC6,
    MCP47FXBXX_MASK_VOL_GAIN_DAC7
};

constexpr int AP_DAC_MCP47FxBxx::_dac_eep_gain_bitmask[8] = {
    MCP47FXBXX_MASK_EEP_GAIN_DAC0, 
    MCP47FXBXX_MASK_EEP_GAIN_DAC1,
    MCP47FXBXX_MASK_EEP_GAIN_DAC2,
    MCP47FXBXX_MASK_EEP_GAIN_DAC3,    
    MCP47FXBXX_MASK_EEP_GAIN_DAC4,
    MCP47FXBXX_MASK_EEP_GAIN_DAC5,
    MCP47FXBXX_MASK_EEP_GAIN_DAC6,
    MCP47FXBXX_MASK_EEP_GAIN_DAC7
};

constexpr int AP_DAC_MCP47FxBxx::_dac_common_dac_bitmask[8] = {
    MCP47FXBXX_MASK_DAC0, 
    MCP47FXBXX_MASK_DAC1,
    MCP47FXBXX_MASK_DAC2,
    MCP47FXBXX_MASK_DAC3,    
    MCP47FXBXX_MASK_DAC4,
    MCP47FXBXX_MASK_DAC5,
    MCP47FXBXX_MASK_DAC6,
    MCP47FXBXX_MASK_DAC7
};

/**
 * @brief Initialises by detecting the device across its default addresses or using the user specified address. Then determines the type of device within the family (Resolution, channels, eeprom)
 */
void AP_DAC_MCP47FxBxx::init(void)
{
    _general_call_dev = std::move(hal.i2c_mgr->get_device(params.bus, 0x00));
    uint8_t recv = 0;
    if (params.bus_address <= 0) {
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Attempting to use default addresses for MCP47FxBxx");
        hal.console->printf("Attempting to use default addresses for MCP47FxBxx\n");
        for (int addr : {MCP47FXBXX_I2CADDR_DEFAULT1, MCP47FXBXX_I2CADDR_DEFAULT2, MCP47FXBXX_I2CADDR_DEFAULT3, MCP47FXBXX_I2CADDR_DEFAULT4}) {
            _dev = std::move(hal.i2c_mgr->get_device(params.bus, addr));
            WITH_SEMAPHORE(_dev->get_semaphore());
            _dev->set_retries(10);
            if (_dev->read_registers(0, &recv, 1)){
                break;
            } 
            else {
                _dev = nullptr;
            }
        }
    }
    else{
        _dev = std::move(hal.i2c_mgr->get_device(params.bus, params.bus_address));
        WITH_SEMAPHORE(_dev->get_semaphore());
        if (!_dev->read_registers(0, &recv, 1)){
            _dev = nullptr;
        }
        _dev->set_retries(1);
    }

    if (!_dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MCP47FxBxx device is null at %u:%u", unsigned(params.bus), unsigned(params.bus_address));
        return;
    }
    
    _detect_device_type();
    set_voltage(0);
}

/**
 * @brief Detects the device type (resolution, channels and memory type). Only works if voltage stored in eeprom is default (mid-scale output).
 * 
 * @return true if successful
 */
bool AP_DAC_MCP47FxBxx::_detect_device_type(){
    if(!general_wakeup()) return false;

    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->set_retries(10);

    char mem_type = 'V';
    int res = 0;

    uint16_t ret;
    if (_read_register_16(MCP47FXBXX_REG_VOL_DAC0, ret)) {
        switch(ret){
        case 0x07F:
            _resolution = 255;
            res = 0;
            break;
        case 0x1FF:
            _resolution = 1023;
            res = 1;
            break;
        case 0x7FF:
            _resolution = 4095;
            res = 2;
            break;
        }
    } else {
        // Should always have dac0, otherwise device is invalid.
        return false;
    }

    if (_read_register_16(MCP47FXBXX_REG_VOL_DAC4, ret)) {
        _channels = 8;
    } 

    if (_read_register_16(MCP47FXBXX_REG_EEP_DAC0, ret)) {
        mem_type = 'E';
        _eeprom = 1;
    }

    hal.console->printf("Found MCP47F%cB%i%i on 0x%03x\n", mem_type, res, _channels, _dev->get_bus_address());
    _dev->set_retries(1);
    return true;
}

/**
 * @brief Sets the voltage for the channel specified
 * 
 * @param[in]  chan  Channel to set the voltage on
 * @param[in]  voltage   Voltage to set in V
 * 
 * @return true if successful
 */
bool AP_DAC_MCP47FxBxx::set_voltage(uint8_t chan, float voltage) {
    // convert voltage to 
    uint16_t voltage_bits = uint16_t(_resolution * voltage / params.voltage_reference);
    voltage_bits = MIN(voltage_bits, _resolution);
    WITH_SEMAPHORE(_dev->get_semaphore());
    if(_write_register_16(_dac_vol_reg[chan], voltage_bits)){
        return true;
    }
    return false;
}

/**
 * @brief Sets the voltage for all channels
 * 
 * @param[in]  chan         Channel to set the voltage on
 * @param[in]  voltage      Voltage to set in V
 * 
 * @return true if successfully set all channels, false if failed. Some may be not set.
 */
bool AP_DAC_MCP47FxBxx::set_voltage(float voltage) {
    // convert voltage to
    for (int chan = 0; chan < _channels; chan++){
        if(!set_voltage(chan, voltage)){
            return false;
        }
    }
    return true;
    
}

/**
 * @brief Sets the gain for the specified channel.
 * 
 * @param[in]  chan  Channel to configure
 * @param[in]  gain  Gain value to apply
 * 
 * @return true if successful
 */
bool AP_DAC_MCP47FxBxx::set_gain(uint8_t chan, uint8_t gain) 
{
    if (chan >= _channels) return false;
    return _set_gain(chan, gain, 0);
}

/**
 * @brief Sets the non-volatile gain for the specified channel.
 * 
 * @param[in]  chan  Channel to configure
 * @param[in]  gain  Gain value to store in EEPROM
 * 
 * @return true if successful
 */
bool AP_DAC_MCP47FxBxx::set_nonvolatile_gain(uint8_t chan, uint8_t gain)
{
    if (chan >= _channels) return false;
    if (!_eeprom) return false;
    return _set_gain(chan, gain, 1);
}

/**
 * @brief Sets the voltage reference mode for a given channel.
 * 
 * @param[in]  chan  DAC channel to configure
 * @param[in]  mode  Mode is one of the predefined `MCP47FXBXX_CONF_VREF_*`
 * 
 * @return true if successful
 */
bool AP_DAC_MCP47FxBxx::set_voltage_ref(uint8_t chan, int mode) 
{
    if (chan >= _channels) return false;
    return _set_dac_mode(1, chan, mode, 0);
}

/**
 * @brief Sets the non-volatile voltage reference mode for a given channel.
 * 
 * @param[in]  chan  DAC channel to configure
 * @param[in]  mode  Voltage reference mode to store in EEPROM
 * 
 * @return true if successful
 */
bool AP_DAC_MCP47FxBxx::set_nonvolatile_voltage_ref(uint8_t chan, int mode) 
{
    if (chan >= _channels) return false;
    if (!_eeprom) return false;
    return _set_dac_mode(1, chan, mode, 1);
}

/**
 * @brief Sets the power-down mode for a specified channel.
 * 
 * @param[in]  chan  DAC channel to power down
 * @param[in]  mode  Power-down mode to apply
 * 
 * @return true if successful
 */
bool AP_DAC_MCP47FxBxx::set_power_down(uint8_t chan, int mode) 
{
    if (chan >= _channels) return false;
    return _set_dac_mode(0, chan, mode, 0);
}

/**
 * @brief Sets the non-volatile power-down mode for a specified channel.
 * 
 * @param[in]  chan  DAC channel to power down
 * @param[in]  mode  Power-down mode to store in EEPROM
 * 
 * @return true if successful
 */
bool AP_DAC_MCP47FxBxx::set_nonvolatile_power_down(uint8_t chan, int mode) 
{
    if (chan >= _channels) return false;
    if (!_eeprom) return false;
    return _set_dac_mode(0, chan, mode, 1);
}

/**
 * @brief Reads the Power-On Reset (POR) status flag from the device.
 *
 * @param[out] ret  Reference to store the POR status bit (0 or non-zero)
 *
 * @return true if the read was successful
 */
bool AP_DAC_MCP47FxBxx::read_por(uint16_t &ret) 
{
    bool success = _read_register_16(MCP47FXBXX_REG_VOL_GAINSTATUS, ret);
    ret &= MCP47FXBXX_MASK_POR;
    return success;
}

/**
 * @brief Reads the volatile gain/status register and masks out the EEWA (EEPROM Write Active) bit to determine if a non-volatile write is in progress.
 * 
 * @param[out] ret  Reference to store the EEWA bit value (0 if inactive, non-zero if active)
 *
 * @return true if the register was read successfully
 */
bool AP_DAC_MCP47FxBxx::read_eewa(uint16_t &ret) 
{
    bool success = _read_register_16(MCP47FXBXX_REG_VOL_GAINSTATUS, ret);
    ret &= MCP47FXBXX_MASK_EEWA;
    return success;
}

/**
 * @brief Reads the wiper lock status for all DAC channels.
 * 
 * @return 8-bit value indicating lock status of DAC channels
 */
bool AP_DAC_MCP47FxBxx::read_wiperlock_status(uint16_t &ret) 
{
    return _read_register_16(MCP47FXBXX_REG_VOL_WIPERLOCKSTATUS, ret);
}

/**
 * @brief Sends a general call wake-up sequence to all devices on the I2C bus.
 * 
 * @return true if the wake-up was acknowledged
 */
bool AP_DAC_MCP47FxBxx::general_wakeup()
{
    WITH_SEMAPHORE(_general_call_dev->get_semaphore());
    uint8_t cmd = MCP47FXBXX_GENERALCALL_WAKEUP;
    if(!_general_call_dev->transfer(&cmd, 1, nullptr, 0)){
        return false;
    }
    
    cmd = 0x06;
    if(!_general_call_dev->transfer(&cmd, 1, nullptr, 0)){
        return false;
    }
    
    return true;
}

/**
 * @brief Reads a 16-bit register value from the DAC device.
 * 
 * @param[in]  reg  Register address to read
 * @param[out] val  Reference to store the read 16-bit value
 * 
 * @return true if the read was successful
 */
bool AP_DAC_MCP47FxBxx::_read_register_16(uint8_t reg, uint16_t &val)
{
    uint16_t v;
    reg = reg << 3 | 0b110;
    if (!_dev->read_registers(reg, (uint8_t*)&v, sizeof(v))) {
        return false;
    }
    val = be16toh(v);
    return true;
}

/**
 * @brief Writes a 16-bit value to a DAC register.
 * 
 * @param[in] reg  Register address to write to
 * @param[in] val  16-bit value to write
 * 
 * @return true if the write was successful
 */
bool AP_DAC_MCP47FxBxx::_write_register_16(uint8_t reg, uint16_t val)
{   
    reg = reg << 3;
    uint8_t buf[3] { reg, uint8_t(val >> 8), uint8_t(val) };
    return _dev->transfer(buf, sizeof(buf), nullptr, 0);
}

/**
 * @brief Sets the mode (VREF or power-down) for a specified DAC channel.
 *
 * @param[in] vref  True to set VREF mode, false to set power-down mode
 * @param[in] chan  DAC channel index
 * @param[in] mode  Mode value to apply (must be aligned with bitmask)
 * @param[in] eep   True to use EEPROM register, false for volatile
 *
 * @return true if the mode was successfully set
 */
bool AP_DAC_MCP47FxBxx::_set_dac_mode(bool vref, uint8_t chan, int mode, bool eep) 
{
    int reg;
    if (vref){
        reg = (eep) ? MCP47FXBXX_REG_EEP_VREF : MCP47FXBXX_REG_VOL_VREF;
    } else {
        reg = (eep) ? MCP47FXBXX_REG_EEP_PWRDOWN : MCP47FXBXX_REG_VOL_PWRDOWN;
    }
    

    uint16_t val;
    if (!_read_register_16(reg, val)) {
        return false;
    }

    val &= ~_dac_common_dac_bitmask[chan]; // Clear bits
    val |= _dac_common_dac_bitmask[chan] & mode;

    return _write_register_16(reg, val);
}

/**
 * @brief Sets the gain bit for a DAC channel, either volatile or non-volatile.
 *
 * @param[in] chan  DAC channel index
 * @param[in] gain  Gain setting (0 or 1)
 * @param[in] eep   True to use EEPROM register, false for volatile
 *
 * @return true if the gain was successfully set
 */
bool AP_DAC_MCP47FxBxx::_set_gain(uint8_t chan, uint8_t gain, bool eep) 
{
    int reg = (eep) ? MCP47FXBXX_REG_EEP_GAINI2CADDR : MCP47FXBXX_REG_VOL_GAINSTATUS;
    int mask = (eep) ? _dac_eep_gain_bitmask[chan] : _dac_vol_gain_bitmask[chan];

    uint16_t val;
    if (!_read_register_16(reg, val)) return false;
    if (gain) {
        val |= mask;
    } else {
        val &= ~mask;
    }
    return _write_register_16(reg, val);
}


#endif // AP_DAC_MCP47FXBXX_ENABLED