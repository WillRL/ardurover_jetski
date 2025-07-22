//  https://ww1.microchip.com/downloads/aemDocuments/documents/MSLD/ProductDocuments/DataSheets/MCP47FXBX48-Data-Sheet-DS200006368A.pdf

#include "AP_DAC_config.h"

#if AP_DAC_MCP47FXBXX_ENABLED
#include "AP_DAC_Backend.h"

// Default I2C addresses for this family. (Depends on A0 and A1 pin).
// First five bits are fully configurable for non volatile versions of the IC.
#define MCP47FXBXX_I2CADDR_DEFAULT1                     0b1100000
#define MCP47FXBXX_I2CADDR_DEFAULT2                     0b1100001
#define MCP47FXBXX_I2CADDR_DEFAULT3                     0b1100010
#define MCP47FXBXX_I2CADDR_DEFAULT4                     0b1100011

#define MCP47FXBXX_GENERALCALL_WAKEUP                   0x0A


// MCP47FXBXX VOLATILE REGISTERS
#define MCP47FXBXX_REG_VOL_DAC0                         0x00                        // DAC0
#define MCP47FXBXX_REG_VOL_DAC1                         0x01                        // DAC1
#define MCP47FXBXX_REG_VOL_DAC2                         0x02                        // DAC2
#define MCP47FXBXX_REG_VOL_DAC3                         0x03                        // DAC3
#define MCP47FXBXX_REG_VOL_DAC4                         0x04                        // DAC4
#define MCP47FXBXX_REG_VOL_DAC5                         0x05                        // DAC5
#define MCP47FXBXX_REG_VOL_DAC6                         0x06                        // DAC6
#define MCP47FXBXX_REG_VOL_DAC7                         0x07                        // DAC7
#define MCP47FXBXX_REG_VOL_VREF                         0x08                        // VOLTAGE REFERENCE SELECTION
#define MCP47FXBXX_REG_VOL_PWRDOWN                      0x09                        // POWER DOWN
#define MCP47FXBXX_REG_VOL_GAINSTATUS                   0x0A                        // GAIN AND STATUS
#define MCP47FXBXX_REG_VOL_WIPERLOCKSTATUS              0x0B                        // WIPERLOCK TECH STATUS

// MCP47FXBXX NON-VOLATILE REGISTERS
#define MCP47FXBXX_REG_EEP_DAC0                         0x10                        // DAC0
#define MCP47FXBXX_REG_EEP_DAC1                         0x11                        // DAC1
#define MCP47FXBXX_REG_EEP_DAC2                         0x12                        // DAC2
#define MCP47FXBXX_REG_EEP_DAC3                         0x13                        // DAC3
#define MCP47FXBXX_REG_EEP_DAC4                         0x14                        // DAC4
#define MCP47FXBXX_REG_EEP_DAC5                         0x15                        // DAC5
#define MCP47FXBXX_REG_EEP_DAC6                         0x16                        // DAC6
#define MCP47FXBXX_REG_EEP_DAC7                         0x17                        // DAC7
#define MCP47FXBXX_REG_EEP_VREF                         0x18                        // VOLTAGE REFERENCE SELECTION
#define MCP47FXBXX_REG_EEP_PWRDOWN                      0x19                        // POWER DOWN
#define MCP47FXBXX_REG_EEP_GAINI2CADDR                  0x1A                        // GAIN AND I2C ADDRESS REG
#define MCP47FXBXX_REG_EEP_WIPERLOCKSTATUS              0x1B                        // WIPERLOCK TECH STATUS

// MCP47FXBXX COMMON DAC BITMASKS
#define MCP47FXBXX_MASK_DAC0                            0b11
#define MCP47FXBXX_MASK_DAC1                            0b1100
#define MCP47FXBXX_MASK_DAC2                            0b110000
#define MCP47FXBXX_MASK_DAC3                            0b11000000
#define MCP47FXBXX_MASK_DAC4                            0b1100000000
#define MCP47FXBXX_MASK_DAC5                            0b110000000000
#define MCP47FXBXX_MASK_DAC6                            0b11000000000000
#define MCP47FXBXX_MASK_DAC7                            0b1100000000000000

// MCP47FXBXX VOLTAGE REF CONFIGS
#define MCP47FXBXX_CONF_VREF_PIN_BUFFERED               0b1111111111111111
#define MCP47FXBXX_CONF_VREF_PIN_UNBUFFERED             0b1010101010101010
#define MCP47FXBXX_CONF_VREF_1v22                       0b0101010101010101
#define MCP47FXBXX_CONF_VREF_VDD                        0b0000000000000000

// MCP47FXBXX POWER DOWN VOUT MODE CONTROL
#define MCP47FXBXX_CONF_PWRDOWN_OPEN                    0b1111111111111111
#define MCP47FXBXX_CONF_PWRDOWN_125KOHM                 0b1010101010101010
#define MCP47FXBXX_CONF_PWRDOWN_1KOHM                   0b0101010101010101
#define MCP47FXBXX_CONF_PWRDOWN_1OHM                    0b0000000000000000

// VOLATILE GAIN STATUS REG BIT MASKS
#define MCP47FXBXX_MASK_EEWA                            0b0000001                   // Power on reset (brown out reset) status bit.
#define MCP47FXBXX_MASK_POR                             0b00000010                  // EEPROM Write active status bit
#define MCP47FXBXX_MASK_VOL_GAIN_DAC0                   0b000000100
#define MCP47FXBXX_MASK_VOL_GAIN_DAC1                   0b0000001000
#define MCP47FXBXX_MASK_VOL_GAIN_DAC2                   0b00000010000
#define MCP47FXBXX_MASK_VOL_GAIN_DAC3                   0b000000100000
#define MCP47FXBXX_MASK_VOL_GAIN_DAC4                   0b0000001000000
#define MCP47FXBXX_MASK_VOL_GAIN_DAC5                   0b00000010000000
#define MCP47FXBXX_MASK_VOL_GAIN_DAC6                   0b000000100000000
#define MCP47FXBXX_MASK_VOL_GAIN_DAC7                   0b0000001000000000

// NON-VOLATILE GAIN STATUS REG BIT MASKS
#define MCP47FXBXX_MASK_I2CADDR                         0b1111111
#define MCP47FXBXX_MASK_ADLCK                           0b10000000                  // Address reg lock status.
#define MCP47FXBXX_MASK_EEP_GAIN_DAC0                   0b100000000
#define MCP47FXBXX_MASK_EEP_GAIN_DAC1                   0b1000000000
#define MCP47FXBXX_MASK_EEP_GAIN_DAC2                   0b10000000000
#define MCP47FXBXX_MASK_EEP_GAIN_DAC3                   0b100000000000
#define MCP47FXBXX_MASK_EEP_GAIN_DAC4                   0b1000000000000
#define MCP47FXBXX_MASK_EEP_GAIN_DAC5                   0b10000000000000
#define MCP47FXBXX_MASK_EEP_GAIN_DAC6                   0b100000000000000
#define MCP47FXBXX_MASK_EEP_GAIN_DAC7                   0b1000000000000000

#define MCP47FXBXX_FACTORY_DEFAULT_DAC_8                0x7F
#define MCP47FXBXX_FACTORY_DEFAULT_DAC_10               0x1FF
#define MCP47FXBXX_FACTORY_DEFAULT_DAC_12               0x7FF
#define MCP47FXBXX_FACTORY_DEFAULT_VREF                 0x0000
#define MCP47FXBXX_FACTORY_DEFAULT_POWERDOWN            0x0000


#endif // AP_DAC_MCP47FXBXX_ENABLED
