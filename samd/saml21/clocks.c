/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Scott Shawcroft for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "hal_atomic.h"
#include "samd/clocks.h"

bool gclk_enabled(uint8_t gclk) {
    return GCLK->GENCTRL[gclk].bit.GENEN;
}

void disable_gclk(uint8_t gclk) {
    while ((GCLK->SYNCBUSY.vec.GENCTRL & (1 << gclk)) != 0) {}
    GCLK->GENCTRL[gclk].bit.GENEN = false;
    while ((GCLK->SYNCBUSY.vec.GENCTRL & (1 << gclk)) != 0) {}
}

void connect_gclk_to_peripheral(uint8_t gclk, uint8_t peripheral) {
    GCLK->PCHCTRL[peripheral].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN(gclk);
    while(GCLK->SYNCBUSY.reg != 0) {}
}

void disconnect_gclk_from_peripheral(uint8_t gclk, uint8_t peripheral) {
    GCLK->PCHCTRL[peripheral].reg = 0;
}

static void enable_clock_generator_sync(uint8_t gclk, uint32_t source, uint16_t divisor, bool sync) {
    uint32_t divsel = 0;
    // The datasheet says 8 bits and max value of 512, how is that possible?
    if (divisor > 255) { // Generator 1 has 16 bits
        divsel = GCLK_GENCTRL_DIVSEL;
        for (int i = 15; i > 0; i--) {
            if (divisor & (1 << i)) {
                divisor = i - 1;
                break;
            }
        }
    }

    GCLK->GENCTRL[gclk].reg = GCLK_GENCTRL_SRC(source) | GCLK_GENCTRL_DIV(divisor) | divsel | GCLK_GENCTRL_OE | GCLK_GENCTRL_GENEN;
    if (sync)
        while ((GCLK->SYNCBUSY.vec.GENCTRL & (1 << gclk)) != 0) {}
}

void enable_clock_generator(uint8_t gclk, uint32_t source, uint16_t divisor) {
    enable_clock_generator_sync(gclk, source, divisor, true);
}

void disable_clock_generator(uint8_t gclk) {
    GCLK->GENCTRL[gclk].reg = 0;
    while ((GCLK->SYNCBUSY.vec.GENCTRL & (1 << gclk)) != 0) {}
}

static void init_clock_source_osculp32k(void) {
    //for SAML21???
    // Calibration value is loaded at startup
    //OSC32KCTRL->OSCULP32K.bit.EN1K = 0;
    //OSC32KCTRL->OSCULP32K.bit.EN32K = 1;
}

static void init_clock_source_xosc32k(void) {
    OSC32KCTRL->XOSC32K.reg = OSC32KCTRL_XOSC32K_ONDEMAND |
                              OSC32KCTRL_XOSC32K_EN32K |
                              OSC32KCTRL_XOSC32K_XTALEN |
                              OSC32KCTRL_XOSC32K_ENABLE;
}

static void init_clock_source_dpll0(void)
{
    GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN(5);
    OSCCTRL->DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0) | OSCCTRL_DPLLRATIO_LDR(59);
    OSCCTRL->DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK(0);
    OSCCTRL->DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;

    while (!(OSCCTRL->DPLLSTATUS.bit.LOCK || OSCCTRL->DPLLSTATUS.bit.CLKRDY)) {}
}

void clock_init(bool has_crystal, uint32_t dfll48m_fine_calibration) {
    // DFLL48M is enabled by default
    // TODO: handle fine calibration data.

    init_clock_source_osculp32k();

    if (has_crystal) {
        init_clock_source_xosc32k();
        OSC32KCTRL->RTCCTRL.bit.RTCSEL = OSC32KCTRL_RTCCTRL_RTCSEL_XOSC32K_Val;
    } else {
        OSC32KCTRL->RTCCTRL.bit.RTCSEL = OSC32KCTRL_RTCCTRL_RTCSEL_ULP32K_Val;
    }

    MCLK->CPUDIV.reg = MCLK_CPUDIV_CPUDIV(1);

    enable_clock_generator_sync(0, GCLK_GENCTRL_SRC_DPLL96M_Val, 1, false);
    enable_clock_generator_sync(1, GCLK_GENCTRL_SRC_DFLL48M_Val, 1, false);
    enable_clock_generator_sync(4, GCLK_GENCTRL_SRC_DPLL96M_Val, 1, false);
    enable_clock_generator_sync(5, GCLK_GENCTRL_SRC_DFLL48M_Val, 24, false);
    enable_clock_generator_sync(6, GCLK_GENCTRL_SRC_DFLL48M_Val, 4, false);

    init_clock_source_dpll0();

    // Do this after all static clock init so that they aren't used dynamically.
    init_dynamic_clocks();
}

static bool clk_enabled(uint8_t clk) {
    return GCLK->PCHCTRL[clk].bit.CHEN;
}

static uint8_t clk_get_generator(uint8_t clk) {
    return GCLK->PCHCTRL[clk].bit.GEN;
}

static uint8_t generator_get_source(uint8_t gen) {
    return GCLK->GENCTRL[gen].bit.SRC;
}

static bool osc_enabled(uint8_t index) {
    switch (index) {
        case GCLK_SOURCE_XOSC:
            return OSCCTRL->XOSCCTRL.bit.ENABLE;
        case GCLK_SOURCE_OSCULP32K:
            return true;
        case GCLK_SOURCE_XOSC32K:
            return OSC32KCTRL->XOSC32K.bit.ENABLE;
        case GCLK_SOURCE_DFLL48M:
            return OSCCTRL->DFLLCTRL.bit.ENABLE;
        case GCLK_SOURCE_DPLL96M:
            return OSCCTRL->DPLLCTRLA.bit.ENABLE;
    };
    return false;
}

static uint32_t osc_get_source(uint8_t index) {
    uint8_t dpll_index = index - GCLK_SOURCE_DPLL96M;
    uint32_t refclk = OSCCTRL->DPLLCTRLB.bit.REFCLK;
    switch (refclk) {
        case 0x0:
            return GCLK_SOURCE_XOSC32K;
        case 0x1:
            return GCLK_SOURCE_XOSC;
        case 0x2:
            return generator_get_source(GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL + dpll_index].bit.GEN);
    }
    return 0;
}

static uint32_t osc_get_frequency(uint8_t index);

static uint32_t generator_get_frequency(uint8_t gen) {
        uint8_t src = GCLK->GENCTRL[gen].bit.SRC;
        uint32_t div;
        if (GCLK->GENCTRL[gen].bit.DIVSEL) {
            div = 1 << (GCLK->GENCTRL[gen].bit.DIV + 1);
        } else {
            div = GCLK->GENCTRL[gen].bit.DIV;
            if (!div)
                div = 1;
        }

        return osc_get_frequency(src) / div;
}

static uint32_t dpll_get_frequency(uint8_t index) {
    uint8_t dpll_index = index - GCLK_SOURCE_DPLL96M;
    uint32_t refclk = OSCCTRL->DPLLCTRLB.bit.REFCLK;
    uint32_t freq;

    switch (refclk) {
        case 0x0: // GCLK
            freq = generator_get_frequency(GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL + dpll_index].bit.GEN);
            break;
        case 0x1: // XOSC32
            freq = 32768;
            break;
        case 0x2: // XOSC0
        case 0x3: // XOSC1
        default:
            return 0; // unknown
    }

    return (freq * (OSCCTRL->DPLLRATIO.bit.LDR + 1)) +
           (freq * OSCCTRL->DPLLRATIO.bit.LDRFRAC / 32);
}

static uint32_t osc_get_frequency(uint8_t index) {
    switch (index) {
        case GCLK_SOURCE_XOSC:
            return 0; // unknown
        case GCLK_SOURCE_OSCULP32K:
        case GCLK_SOURCE_XOSC32K:
            return 32768;
        case GCLK_SOURCE_DFLL48M:
            return 48000000;
        case GCLK_SOURCE_DPLL96M:
            return dpll_get_frequency(index);
    }
    return 0;
}

bool clock_get_enabled(uint8_t type, uint8_t index) {
    if (type == 0)
        return osc_enabled(index);
    if (type == 1)
        return clk_enabled(index);
    if (type == 2)
        return SysTick->CTRL & SysTick_CTRL_ENABLE_Msk;
    return false;
}

bool clock_get_parent(uint8_t type, uint8_t index, uint8_t *p_type, uint8_t *p_index) {
    if (type == 0 && osc_enabled(index)) {
         if (index == GCLK_SOURCE_DPLL96M) {
            *p_type = 0;
            *p_index = osc_get_source(index);
            return true;
         }
         return false;
    }
    if (type == 1 && index <= 47 && clk_enabled(index)) {
        *p_type = 0;
        *p_index = generator_get_source(clk_get_generator(index));
        return true;
    }
    if (type == 2) {
        switch (index) {
            case 0:
            case 1:
                *p_type = 0;
                *p_index = generator_get_source(0);
                return true;
            case 2:
                *p_type = 0;
                switch (OSC32KCTRL->RTCCTRL.bit.RTCSEL) {
                    case 0:
                    case 1:
                        *p_index = GCLK_SOURCE_OSCULP32K;
                        return true;
                    case 4:
                    case 5:
                        *p_index = GCLK_SOURCE_XOSC32K;
                        return true;
                }
                return false;
        }
    }
    return false;
}

uint32_t clock_get_frequency(uint8_t type, uint8_t index) {
    if (type == 0) {
        return osc_get_frequency(index);
    }
    if (type == 1 && index <= 47 && clk_enabled(index)) {
        return generator_get_frequency(clk_get_generator(index));
    }
    if (type == 2) {
        switch (index) {
            case 0:
                return clock_get_frequency(0, generator_get_source(0)) / SysTick->LOAD;
            case 1:
                return clock_get_frequency(0, generator_get_source(0)) / MCLK->CPUDIV.bit.CPUDIV;
            case 2:
                switch (OSC32KCTRL->RTCCTRL.bit.RTCSEL) {
                    case 0:
                    case 4:
                        return 1024;
                    case 1:
                    case 5:
                        return 32768;
                }
        }
    }
    return 0;
}

uint32_t clock_get_calibration(uint8_t type, uint8_t index) {
    if (type == 0) {
        switch (index) {
            case GCLK_SOURCE_OSCULP32K:
                return OSC32KCTRL->OSCULP32K.bit.CALIB;
        };
    }
    if (type == 2 && index == 0) {
        return SysTick->LOAD + 1;
    }
    return 0;
}

int clock_set_calibration(uint8_t type, uint8_t index, uint32_t val) {
    if (type == 0) {
        switch (index) {
            case GCLK_SOURCE_OSCULP32K:
                if (val > 0x3f)
                    return -1;
                OSC32KCTRL->OSCULP32K.bit.CALIB = val;
                return 0;
        };
    }
    if (type == 2 && index == 0) {
        if (val < 0x1000 || val > 0x1000000)
            return -1;
        SysTick->LOAD = val - 1;
        return 0;
    }
    return -2;
}
