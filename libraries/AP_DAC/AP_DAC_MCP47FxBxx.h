#include "AP_DAC_config.h"

#if AP_DAC_MCP47FXBXX_ENABLED
#include "AP_DAC_Backend.h"
#include "AP_DAC_MCP47FxBxx_registers.h"


class AP_DAC_MCP47FxBxx : public AP_DAC_Backend
{
public:
    using AP_DAC_Backend::AP_DAC_Backend;

    virtual ~AP_DAC_MCP47FxBxx() {}

    void init(void) override;

    // set voltage for a channel
    bool set_voltage(uint8_t chan, float voltage) override;
    bool set_voltage(float voltage);
    // bool set_nonvolatile_voltage(uint8_t chan, float val);
    bool set_gain(uint8_t chan, uint8_t gain);
    bool set_nonvolatile_gain(uint8_t chan, uint8_t gain);
    bool set_voltage_ref(uint8_t chan, int mode);
    bool set_nonvolatile_voltage_ref(uint8_t chan, int mode);
    bool set_power_down(uint8_t chan, int mode);
    bool set_nonvolatile_power_down(uint8_t chan, int mode);
    bool read_por(uint16_t &ret);
    bool read_eewa(uint16_t &ret);
    bool read_wiperlock_status(uint16_t &ret);
    bool general_wakeup();
    
    // Not exposing i2c address set and configuration bit, not likely needed for typical Ardupilot uses. Use a lua script instead.


private:
    static const int _dac_bitmask[8];
    static const int _dac_vol_reg[8];
    static const int _dac_eep_reg[8];
    static const int _dac_vol_gain_bitmask[8];
    static const int _dac_eep_gain_bitmask[8];
    static const int _dac_common_dac_bitmask[8];

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    AP_HAL::OwnPtr<AP_HAL::Device> _general_call_dev;

    bool _detect_device_type();
    bool _read_register_16(uint8_t reg, uint16_t &val);
    bool _write_register_16(uint8_t reg, uint16_t val);
    bool _set_dac_mode(bool vref, uint8_t chan, int mode, bool eep);
    bool _set_gain(uint8_t chan, uint8_t gain, bool eep);
    uint16_t _resolution = 0;
    uint8_t _channels = 4;
    uint8_t _eeprom = 0;


    bool configured[4];
};

#endif // AP_DAC_MCP47FXBXX_ENABLED
