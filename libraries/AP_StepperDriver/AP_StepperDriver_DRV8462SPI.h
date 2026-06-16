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

#include "AP_StepperDriver_config.h"
#if AP_STEPPERDRIVER_ENABLED
#if AP_STEPPERDRIVER_DRV8462SPI_ENABLED

#include "AP_StepperDriver.h"
#include <AP_HAL/SPIDevice.h>
#include "AP_StepperDriver_Backend.h"

// =====================================================================
// DRV8462 Stepper Motor Driver - SPI Register Map
// SPI: 16-bit frames, MSB-first, CPOL=1, CPHA=1
// Bits [15:14] = RW flag (10=read, 00=write), [13:8] = address, [7:0] = data
// =====================================================================

// --- Status Registers (Read-Only) ---
#define DRV8462_FAULT_REG           0x00  // [7]FAULT [6]SPI_ERROR [5]UVLO [4]CPUV [3]OCP [2]STL [1]TF [0]OL
#define DRV8462_DIAG1_REG           0x01  // OCP flags: [7]OCP_LS2_B [6]OCP_HS2_B [5]OCP_LS1_B [4]OCP_HS1_B [3]OCP_LS2_A [2]OCP_HS2_A [1]OCP_LS1_A [0]OCP_HS1_A
#define DRV8462_DIAG2_REG           0x02  // [7]STSL [6]OTW [5]OTS [4]STL_LRN_OK [3]STALL [2]LRN_DONE [1]OL_B [0]OL_A
#define DRV8462_DIAG3_REG           0x03  // [6]NHOME [5]CNT_OFLW [4]CNT_UFLW [2]NPOR

// --- Control Registers (Read/Write) ---
#define DRV8462_CTRL1_REG           0x04  // [7]EN_OUT [6]SR [5]IDX_RST [4:3]TOFF[1:0] [2:0]DECAY[2:0]
#define DRV8462_CTRL2_REG           0x05  // [7]DIR [6]STEP [5]SPI_DIR [4]SPI_STEP [3:0]MICROSTEP_MODE[3:0]
#define DRV8462_CTRL3_REG           0x06  // [7]CLR_FLT [5:3]LOCK[2:0] [2]TOCP [1]OCP_MODE [0]OTSD_MODE -- wait, also OTW_REP
#define DRV8462_CTRL4_REG           0x07  // [7:6]TBLANK_TIME[1:0] [5]STL_LRN [4]EN_STL [3]STL_REP [2]STL_FRQ [1:0]STEP_FRQ_TOL[1:0]
#define DRV8462_CTRL5_REG           0x08  // [7:0]STALL_TH[7:0]
#define DRV8462_CTRL6_REG           0x09  // [7:6]RC_RIPPLE[1:0] [5]DIS_SSC [4]TRQ_SCALE [3:0]STALL_TH[11:8]
#define DRV8462_CTRL7_REG           0x0A  // [7:0]TRQ_COUNT[7:0]  (Read-Only)
#define DRV8462_CTRL8_REG           0x0B  // [3:0]TRQ_COUNT[11:8] (Read-Only)
#define DRV8462_CTRL9_REG           0x0C  // [7]EN_OL [6]OL_MODE [5:4]OL_T[1:0] [3]STEP_EDGE [2:1]RES_AUTO[1:0] [0]EN_AUTO
#define DRV8462_CTRL10_REG          0x0D  // [7:0]ISTSL[7:0]       -- holding current scalar
#define DRV8462_CTRL11_REG          0x0E  // [7:0]TRQ_DAC[7:0]     -- run current scalar
#define DRV8462_CTRL12_REG          0x0F  // [7]EN_STSL [6:3]TSTSL_FALL[3:0]
#define DRV8462_CTRL13_REG          0x10  // [7:2]TSTSL_DLY[5:0] [1]VREF_INT_EN

// --- Indexer Registers (Read-Only) ---
#define DRV8462_INDEX1_REG          0x11  // [7:0]CUR_A_POS[7:0]
#define DRV8462_INDEX2_REG          0x12  // [7]CUR_A_SIGN
#define DRV8462_INDEX3_REG          0x13  // [7:0]CUR_B_POS[7:0]
#define DRV8462_INDEX4_REG          0x14  // [7]CUR_B_SIGN [1:0]CUR_A[1:0]
#define DRV8462_INDEX5_REG          0x15  // [7:0]CUR_A[9:2]

// --- Custom Microstepping Registers (Read/Write) ---
#define DRV8462_CUSTOM_CTRL1_REG    0x16  // [0]EN_CUSTOM
#define DRV8462_CUSTOM_CTRL2_REG    0x17  // [7:0]CUSTOM_CURRENT1
#define DRV8462_CUSTOM_CTRL3_REG    0x18  // [7:0]CUSTOM_CURRENT2
#define DRV8462_CUSTOM_CTRL4_REG    0x19  // [7:0]CUSTOM_CURRENT3
#define DRV8462_CUSTOM_CTRL5_REG    0x1A  // [7:0]CUSTOM_CURRENT4
#define DRV8462_CUSTOM_CTRL6_REG    0x1B  // [7:0]CUSTOM_CURRENT5
#define DRV8462_CUSTOM_CTRL7_REG    0x1C  // [7:0]CUSTOM_CURRENT6
#define DRV8462_CUSTOM_CTRL8_REG    0x1D  // [7:0]CUSTOM_CURRENT7
#define DRV8462_CUSTOM_CTRL9_REG    0x1E  // [7:0]CUSTOM_CURRENT8

// --- Auto-Torque Registers (Read/Write unless noted) ---
#define DRV8462_ATQ_CTRL1_REG       0x1F  // [7:0]ATQ_CNT[7:0]           (Read-Only)
#define DRV8462_ATQ_CTRL2_REG       0x20  // [7:5]ATQ_CNT[10:8] [2:0]ATQ_LRN_CONST1[10:8]
#define DRV8462_ATQ_CTRL3_REG       0x21  // [7:0]ATQ_LRN_CONST1[7:0]
#define DRV8462_ATQ_CTRL4_REG       0x22  // [7:3]ATQ_LRN_MIN_CURRENT[4:0] [2:0]ATQ_LRN_CONST2[10:8]
#define DRV8462_ATQ_CTRL5_REG       0x23  // [7:0]ATQ_LRN_CONST2[7:0]
#define DRV8462_ATQ_CTRL6_REG       0x24  // [7:0]ATQ_UL[7:0]
#define DRV8462_ATQ_CTRL7_REG       0x25  // [7:0]ATQ_LL[7:0]
#define DRV8462_ATQ_CTRL8_REG       0x26  // [7:0]KP[7:0]
#define DRV8462_ATQ_CTRL9_REG       0x27  // [3:0]KD[3:0]
#define DRV8462_ATQ_CTRL10_REG      0x28  // [7]ATQ_EN [6]LRN_START [5:3]ATQ_FRZ[2:0] [2:0]ATQ_AVG[2:0]
#define DRV8462_ATQ_CTRL11_REG      0x29  // [7:0]ATQ_TRQ_MIN[7:0]
#define DRV8462_ATQ_CTRL12_REG      0x2A  // [7:0]ATQ_TRQ_MAX[7:0]
#define DRV8462_ATQ_CTRL13_REG      0x2B  // [7:0]ATQ_D_THR[7:0]
#define DRV8462_ATQ_CTRL14_REG      0x2C  // Reserved
#define DRV8462_ATQ_CTRL15_REG      0x2D  // [7:4]ATQ_ERROR_TRUNCATE[3:0] [3:2]ATQ_LRN_STEP[1:0] [1:0]ATQ_LRN_CYCLE_SELECT[1:0]
#define DRV8462_ATQ_CTRL16_REG      0x2E  // [7:0]ATQ_TRQ_DAC[7:0]       (Read-Only)
#define DRV8462_ATQ_CTRL17_REG      0x2F  // [6]VM_SCALE
#define DRV8462_ATQ_CTRL18_REG      0x30  // Reserved

// --- Silent Step Registers (Read/Write) ---
#define DRV8462_SS_CTRL1_REG        0x31  // [7:6]SS_SMPL_SEL[1:0] [3:2]SS_PWM_FREQ[1:0] [0]EN_SS
#define DRV8462_SS_CTRL2_REG        0x32  // [7:0]SS_KP[7:0]
#define DRV8462_SS_CTRL3_REG        0x33  // [7:0]SS_KI[7:0]
#define DRV8462_SS_CTRL4_REG        0x34  // [6:4]SS_KI_DIV_SEL[2:0] [2:0]SS_KP_DIV_SEL[2:0]
#define DRV8462_SS_CTRL5_REG        0x35  // [7:0]SS_THR[7:0]

// --- Misc ---
#define DRV8462_CTRL14_REG          0x3C  // [6:2]VM_ADC[4:0]  -- VM voltage monitor
#define KV                          0.33
#define IVREF                       3.3
// --- SPI Frame Helpers ---
// SPI frame: 16-bit. Bit15=0 for write, Bit15=1 for read. Bits[13:8]=address. Bits[7:0]=data.
#define DRV8462_SPI_READ(addr)      (0x8000 | ((addr) << 8))
#define DRV8462_SPI_WRITE(addr, data) (((addr) << 8) | ((data) & 0xFF))

// --- Common CTRL1 bit masks ---
#define DRV8462_EN_OUT              (1 << 7)  // Enable output drivers
#define DRV8462_IDX_RST             (1 << 5)  // Reset indexer to 45° electrical
#define DRV8462_DECAY_SLOW          0x00
#define DRV8462_DECAY_MIXED_30      0x04
#define DRV8462_DECAY_MIXED_60      0x05
#define DRV8462_DECAY_SMART_DYNAMIC 0x06
#define DRV8462_DECAY_SMART_RIPPLE  0x07  // Default

// --- Common CTRL3 bit masks ---
#define DRV8462_CLR_FLT             (1 << 7)  // Clear faults (auto-clears)



class AP_StepperDriver_DRV8462SPI: public AP_StepperDriver_Backend{
public:
    // constructor
    AP_StepperDriver_DRV8462SPI(AP_StepperDriver &frontend);
    AP_StepperDriver_DRV8462SPI();
    ~AP_StepperDriver_DRV8462SPI(void) {};
    
    struct config {
        // CTRL1 (0x04)
        int8_t en_out = -1;       // -1 = not set
        int8_t sr = -1;
        int8_t idx_rst = -1;
        int8_t toff = -1;         // 2 bits
        int8_t decay = -1;        // 3 bits

        // CTRL2 (0x05)
        int8_t dir = -1;
        int8_t step = -1;
        int8_t spi_dir = -1;
        int8_t spi_step = -1;
        int8_t microstep_mode = -1; // 4 bits

        // CTRL3 (0x06)
        int8_t clr_flt = -1;
        int8_t lock = -1;         // 3 bits
        int8_t tocp = -1;
        int8_t ocp_mode = -1;
        int8_t otsd_mode = -1;
        int8_t otw_rep = -1;

        // CTRL4 (0x07)
        int8_t tblank_time = -1;  // 2 bits
        int8_t stl_lrn = -1;
        int8_t en_stl = -1;
        int8_t stl_rep = -1;
        int8_t frq_chg = -1;
        int8_t step_frq_tol = -1; // 2 bits

        // CTRL5 (0x08)
        int16_t stall_th = -1;    // 12 bits, split across CTRL5 and CTRL6

        // CTRL6 (0x09)
        int8_t rc_ripple = -1;    // 2 bits
        int8_t dis_ssc = -1;
        int8_t trq_scale = -1;

        // CTRL9 (0x0C)
        int8_t en_ol = -1;
        int8_t ol_mode = -1;
        int8_t ol_t = -1;         // 2 bits
        int8_t step_edge = -1;
        int8_t res_auto = -1;     // 2 bits
        int8_t en_auto = -1;

        // CTRL10 (0x0D)
        int16_t istsl = -1;       // 8 bits

        // CTRL11 (0x0E)
        int16_t trq_dac = -1;     // 8 bits

        // CTRL12 (0x0F)
        int8_t en_stsl = -1;
        int8_t tstsl_fall = -1;   // 4 bits

        // CTRL13 (0x10)
        int8_t tstsl_dly = -1;    // 6 bits
        int8_t vref_int_en = -1;
    };

    static AP_StepperDriver_DRV8462SPI *get_singleton() { return _singleton; }

    // initialization
    void init() override;
    bool write_config(config &cfg);
    bool read_reg(uint8_t addr, uint8_t *recv) override;
    bool write_reg(uint8_t addr, uint8_t data) override;
    bool step() override;
    bool dir(bool dir) override;
    bool enable() override;
    bool set_run_I(float I) override;
    bool set_hold_I(float I) override;
    bool set_microstep_mode(AP_StepperDriver::MicrostepMode mode) override;
    bool use_SPI(bool use_spi) override;
    bool use_IVREF(bool use_ivref) override;
    
    private:
    static AP_StepperDriver_DRV8462SPI *_singleton;
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev = nullptr;
    bool _drv8462_write_reg(uint8_t addr, uint8_t data);
};

#endif // AP_STEPPERDRIVER_DRV8462SPI_ENABLED
#endif // AP_STEPPERDRIVER_ENABLED