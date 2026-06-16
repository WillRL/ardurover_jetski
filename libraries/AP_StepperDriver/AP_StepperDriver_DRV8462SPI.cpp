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

#include "AP_StepperDriver_config.h"
#if AP_STEPPERDRIVER_ENABLED
#if AP_STEPPERDRIVER_DRV8462SPI_ENABLED

#include "AP_StepperDriver_DRV8462SPI.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL& hal;

AP_StepperDriver_DRV8462SPI *AP_StepperDriver_DRV8462SPI::_singleton = nullptr;

AP_StepperDriver_DRV8462SPI::AP_StepperDriver_DRV8462SPI(AP_StepperDriver &frontend) :
  AP_StepperDriver_Backend(frontend)
{
    _singleton = this;
}


/*
   Initialize the DRV846 SPI device. This device only has one SPI address, so the address parameter is ignored.
*/
void AP_StepperDriver_DRV8462SPI::init()
{   
    _dev = hal.spi->get_device(HAL_STR_DRV8462_NAME);
    if (!_dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AP_StepperDriver_DRV8462SPI: Failed to initialize SPI device.");
        return;
    }

    _dev->set_speed(AP_HAL::SPIDevice::SPEED_LOW);
    _dev->set_slowdown(10);

    uint8_t recv = 0;
    WITH_SEMAPHORE(_dev->get_semaphore());
        if (read_reg(DRV8462_FAULT_REG, &recv)){
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "DRV8462SPI initialized successfully");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "DRV8462SPI failed to initialise");
        }
} 

bool AP_StepperDriver_DRV8462SPI::read_reg(uint8_t addr, uint8_t *recv) {
    WITH_SEMAPHORE(_dev->get_semaphore());
    uint8_t buf[2] = { (uint8_t)(0x40 | (addr & 0x3F)), 0x00 };
    bool result = _dev->transfer_fullduplex(buf, 2);
    *recv = buf[1];
    return result;
}


bool AP_StepperDriver_DRV8462SPI::_drv8462_write_reg(uint8_t addr, uint8_t data) {
    WITH_SEMAPHORE(_dev->get_semaphore());
    uint8_t buf[2] = { (uint8_t)(addr & 0x3F), data };
    bool result = _dev->transfer_fullduplex(buf, 2);
    return result;
}

bool AP_StepperDriver_DRV8462SPI::write_reg(uint8_t addr, uint8_t data) {
    if (!_drv8462_write_reg(addr, data)) {
        return false;
    }
    uint8_t recv;
    read_reg(addr, &recv);
    return recv == data;
}


bool AP_StepperDriver_DRV8462SPI::write_config(config &cfg) {
    bool ok = true;

    // CTRL1 (0x04): EN_OUT, SR, IDX_RST, TOFF[1:0], DECAY[2:0]
    if (cfg.en_out != -1 || cfg.sr != -1 || cfg.idx_rst != -1 || cfg.toff != -1 || cfg.decay != -1) {
        uint8_t reg = 0;
        read_reg(DRV8462_CTRL1_REG, &reg);
        if (cfg.en_out   != -1) reg = (reg & ~(1 << 7))   | ((cfg.en_out   & 0x1) << 7);
        if (cfg.sr       != -1) reg = (reg & ~(1 << 6))   | ((cfg.sr       & 0x1) << 6);
        if (cfg.idx_rst  != -1) reg = (reg & ~(1 << 5))   | ((cfg.idx_rst  & 0x1) << 5);
        if (cfg.toff     != -1) reg = (reg & ~(0x3 << 3)) | ((cfg.toff     & 0x3) << 3);
        if (cfg.decay    != -1) reg = (reg & ~(0x7 << 0)) | ((cfg.decay    & 0x7) << 0);
        ok &= _drv8462_write_reg(DRV8462_CTRL1_REG, reg);
    }

    // CTRL2 (0x05): DIR, STEP, SPI_DIR, SPI_STEP, MICROSTEP_MODE[3:0]
    if (cfg.dir != -1 || cfg.step != -1 || cfg.spi_dir != -1 || cfg.spi_step != -1 || cfg.microstep_mode != -1) {
        uint8_t reg = 0;
        read_reg(DRV8462_CTRL2_REG, &reg);
        if (cfg.dir            != -1) reg = (reg & ~(1 << 7))   | ((cfg.dir            & 0x1) << 7);
        if (cfg.step           != -1) reg = (reg & ~(1 << 6))   | ((cfg.step           & 0x1) << 6);
        if (cfg.spi_dir        != -1) reg = (reg & ~(1 << 5))   | ((cfg.spi_dir        & 0x1) << 5);
        if (cfg.spi_step       != -1) reg = (reg & ~(1 << 4))   | ((cfg.spi_step       & 0x1) << 4);
        if (cfg.microstep_mode != -1) reg = (reg & ~(0xF << 0)) | ((cfg.microstep_mode & 0xF) << 0);
        ok &= _drv8462_write_reg(DRV8462_CTRL2_REG, reg);
        hal.console->printf("CTRL2 reg: %u\n", reg);
    }

    // CTRL3 (0x06): CLR_FLT, LOCK[2:0], TOCP, OCP_MODE, OTSD_MODE, OTW_REP
    if (cfg.clr_flt != -1 || cfg.lock != -1 || cfg.tocp != -1 || cfg.ocp_mode != -1 || cfg.otsd_mode != -1 || cfg.otw_rep != -1) {
        uint8_t reg = 0;
        read_reg(DRV8462_CTRL3_REG, &reg);
        if (cfg.clr_flt   != -1) reg = (reg & ~(1 << 7))   | ((cfg.clr_flt   & 0x1) << 7);
        if (cfg.lock      != -1) reg = (reg & ~(0x7 << 4)) | ((cfg.lock      & 0x7) << 4);
        if (cfg.tocp      != -1) reg = (reg & ~(1 << 3))   | ((cfg.tocp      & 0x1) << 3);
        if (cfg.ocp_mode  != -1) reg = (reg & ~(1 << 2))   | ((cfg.ocp_mode  & 0x1) << 2);
        if (cfg.otsd_mode != -1) reg = (reg & ~(1 << 1))   | ((cfg.otsd_mode & 0x1) << 1);
        if (cfg.otw_rep   != -1) reg = (reg & ~(1 << 0))   | ((cfg.otw_rep   & 0x1) << 0);
        ok &= _drv8462_write_reg(DRV8462_CTRL3_REG, reg);
    }

    // CTRL4 (0x07): TBLANK_TIME[1:0], STL_LRN, EN_STL, STL_REP, STL_FRCTRL3 Q, STEP_FRQ_TOL[1:0]
    if (cfg.tblank_time != -1 || cfg.stl_lrn != -1 || cfg.en_stl != -1 || cfg.stl_rep != -1 || cfg.frq_chg != -1 || cfg.step_frq_tol != -1) {
        uint8_t reg = 0;
        read_reg(DRV8462_CTRL4_REG, &reg);
        if (cfg.tblank_time  != -1) reg = (reg & ~(0x3 << 6)) | ((cfg.tblank_time  & 0x3) << 6);
        if (cfg.stl_lrn      != -1) reg = (reg & ~(1 << 5))   | ((cfg.stl_lrn      & 0x1) << 5);
        if (cfg.en_stl       != -1) reg = (reg & ~(1 << 4))   | ((cfg.en_stl       & 0x1) << 4);
        if (cfg.stl_rep      != -1) reg = (reg & ~(1 << 3))   | ((cfg.stl_rep      & 0x1) << 3);
        if (cfg.frq_chg      != -1) reg = (reg & ~(1 << 2))   | ((cfg.frq_chg      & 0x1) << 2);
        if (cfg.step_frq_tol != -1) reg = (reg & ~(0x3 << 0)) | ((cfg.step_frq_tol & 0x3) << 0);
        ok &= _drv8462_write_reg(DRV8462_CTRL4_REG, reg);
    }

    // CTRL5 + CTRL6 (0x08, 0x09): STALL_TH[11:0] spans both, plus RC_RIPPLE, DIS_SSC, TRQ_SCALE
    if (cfg.stall_th != -1 || cfg.rc_ripple != -1 || cfg.dis_ssc != -1 || cfg.trq_scale != -1) {
        uint8_t reg5 = 0, reg6 = 0;
        read_reg(DRV8462_CTRL5_REG, &reg5);
        read_reg(DRV8462_CTRL6_REG, &reg6);
        if (cfg.stall_th != -1) {
            reg5 = (uint8_t)(cfg.stall_th & 0xFF);
            reg6 = (reg6 & ~0xF) | (uint8_t)((cfg.stall_th >> 8) & 0xF);
        }
        if (cfg.rc_ripple  != -1) reg6 = (reg6 & ~(0x3 << 6)) | ((cfg.rc_ripple  & 0x3) << 6);
        if (cfg.dis_ssc    != -1) reg6 = (reg6 & ~(1 << 5))   | ((cfg.dis_ssc    & 0x1) << 5);
        if (cfg.trq_scale  != -1) reg6 = (reg6 & ~(1 << 4))   | ((cfg.trq_scale  & 0x1) << 4);
        ok &= _drv8462_write_reg(DRV8462_CTRL5_REG, reg5);
        ok &= _drv8462_write_reg(DRV8462_CTRL6_REG, reg6);
    }


    // CTRL9 (0x0C): EN_OL, OL_MODE, OL_T[1:0], STEP_EDGE, RES_AUTO[1:0], EN_AUTO
    if (cfg.en_ol != -1 || cfg.ol_mode != -1 || cfg.ol_t != -1 || cfg.step_edge != -1 || cfg.res_auto != -1 || cfg.en_auto != -1) {
        uint8_t reg = 0;
        read_reg(DRV8462_CTRL9_REG, &reg);
        if (cfg.en_ol     != -1) reg = (reg & ~(1 << 7))   | ((cfg.en_ol     & 0x1) << 7);
        if (cfg.ol_mode   != -1) reg = (reg & ~(1 << 6))   | ((cfg.ol_mode   & 0x1) << 6);
        if (cfg.ol_t      != -1) reg = (reg & ~(0x3 << 4)) | ((cfg.ol_t      & 0x3) << 4);
        if (cfg.step_edge != -1) reg = (reg & ~(1 << 3))   | ((cfg.step_edge & 0x1) << 3);
        if (cfg.res_auto  != -1) reg = (reg & ~(0x3 << 1)) | ((cfg.res_auto  & 0x3) << 1);
        if (cfg.en_auto   != -1) reg = (reg & ~(1 << 0))   | ((cfg.en_auto   & 0x1) << 0);
        ok &= _drv8462_write_reg(DRV8462_CTRL9_REG, reg);
    }

    // CTRL10 (0x0D): ISTSL[7:0]
    if (cfg.istsl != -1) {
        ok &= _drv8462_write_reg(DRV8462_CTRL10_REG, (uint8_t)(cfg.istsl & 0xFF));
    }

    // CTRL11 (0x0E): TRQ_DAC[7:0]
    if (cfg.trq_dac != -1) {
        ok &= _drv8462_write_reg(DRV8462_CTRL11_REG, (uint8_t)(cfg.trq_dac & 0xFF));
    }

    // CTRL12 (0x0F): EN_STSL, TSTSL_FALL[3:0]
    if (cfg.en_stsl != -1 || cfg.tstsl_fall != -1) {
        uint8_t reg = 0;
        read_reg(DRV8462_CTRL12_REG, &reg);
        if (cfg.en_stsl    != -1) reg = (reg & ~(1 << 7))   | ((cfg.en_stsl    & 0x1) << 7);
        if (cfg.tstsl_fall != -1) reg = (reg & ~(0xF << 3)) | ((cfg.tstsl_fall & 0xF) << 3);
        ok &= _drv8462_write_reg(DRV8462_CTRL12_REG, reg);
    }

    // CTRL13 (0x10): TSTSL_DLY[5:0], VREF_INT_EN
    if (cfg.tstsl_dly != -1 || cfg.vref_int_en != -1) {
        uint8_t reg = 0;
        read_reg(DRV8462_CTRL13_REG, &reg);
        if (cfg.tstsl_dly   != -1) reg = (reg & ~(0x3F << 2)) | ((cfg.tstsl_dly   & 0x3F) << 2);
        if (cfg.vref_int_en != -1) reg = (reg & ~(1 << 1))    | ((cfg.vref_int_en & 0x1)  << 1);
        ok &= _drv8462_write_reg(DRV8462_CTRL13_REG, reg);
    }
    return ok;
}

bool AP_StepperDriver_DRV8462SPI::step() {
    config cfg;
    cfg.spi_step = 1;
    cfg.step = 1;
    return write_config(cfg);
}

bool AP_StepperDriver_DRV8462SPI::dir(bool dir) {
    config cfg;
    cfg.dir = dir;
    return write_config(cfg);
}

bool AP_StepperDriver_DRV8462SPI::enable() {
    config cfg;
    cfg.en_out = 1;
    return write_config(cfg);
}

bool AP_StepperDriver_DRV8462SPI::set_hold_I(float I) {
    float istsl_f = (I * KV) / IVREF * 255.0f;
    istsl_f = constrain_float(istsl_f, 0.0f, 255.0f);

    config cfg;
    cfg.istsl = (int16_t)istsl_f;
    hal.console->printf("Setting hold current to %fA (istsl: %u)\n", I, (uint16_t)cfg.istsl);
    return write_config(cfg);
}

bool AP_StepperDriver_DRV8462SPI::set_run_I(float I) {
    float trq_dac_f = (I * KV) / IVREF * 255.0f;
    trq_dac_f = constrain_float(trq_dac_f, 0.0f, 255.0f);

    config cfg;
    cfg.trq_dac = (int16_t)trq_dac_f;
    return write_config(cfg);
}

bool AP_StepperDriver_DRV8462SPI::set_microstep_mode(AP_StepperDriver::MicrostepMode mode) {
    static const uint8_t mode_LUT[] = {
        0b0001,  // FULL
        0b0011,  // HALF
        0b0100,  // QUARTER
        0b0101,  // EIGHTH
        0b0110,  // SIXTEENTH
        0b0111,  // THIRTY_SECOND
        0b1000,  // SIXTY_FOURTH
        0b1001,  // ONE28TH
        0b1010,  // TWO56TH
    };

    if (mode >= ARRAY_SIZE(mode_LUT)) {
        return false;
    }

    config cfg;
    cfg.microstep_mode = mode_LUT[mode];
    return write_config(cfg);
}

bool AP_StepperDriver_DRV8462SPI::use_SPI(bool use_spi) {
    config cfg;
    cfg.spi_dir = use_spi;
    cfg.spi_step = use_spi;
    return write_config(cfg);
}

bool AP_StepperDriver_DRV8462SPI::use_IVREF(bool use_ivref) {
    config cfg;
    cfg.vref_int_en = use_ivref;
    return write_config(cfg);
}



#endif // AP_STEPPERDRIVER_DRV8462SPI_ENABLED
#endif // AP_GENERIC_ENCODER_ENABLED