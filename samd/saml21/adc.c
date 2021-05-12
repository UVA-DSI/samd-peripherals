/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Dan Halbert for Adafruit Industries
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

#include "hal/include/hal_adc_sync.h"
#include "hpl/gclk/hpl_gclk_base.h"
#include "hri_mclk.h"

// Do initialization and calibration setup needed for any use of the ADC.
// The reference and resolution should be set by the caller.
void samd_peripherals_adc_setup(struct adc_sync_descriptor *adc, Adc *instance) {
    // Turn the clocks on.
    hri_mclk_set_APBDMASK_ADC_bit(MCLK);
    hri_gclk_write_PCHCTRL_reg(GCLK, ADC_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));

    adc_sync_init(adc, instance, (void *)NULL);

    // SAML21 has a CALIB register but doesn't have documented fuses for them.
    uint8_t biasrefbuf;
    uint8_t biascomp;
        biasrefbuf = ((*(uint32_t*) ADC_FUSES_BIASREFBUF_ADDR) & ADC_FUSES_BIASREFBUF_Msk) >> ADC_FUSES_BIASREFBUF_Pos;
        biascomp = ((*(uint32_t*) ADC_FUSES_BIASCOMP_ADDR) & ADC_FUSES_BIASCOMP_Msk) >> ADC_FUSES_BIASCOMP_Pos;
    hri_adc_write_CALIB_BIASREFBUF_bf(instance, biasrefbuf);
    hri_adc_write_CALIB_BIASCOMP_bf(instance, biascomp);
}
