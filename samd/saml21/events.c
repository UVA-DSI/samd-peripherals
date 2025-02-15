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

#include "samd/events.h"

#include "samd/clocks.h"

#include "py/runtime.h"

#include "hpl/pm/hpl_pm_base.h"

void turn_on_event_system(void) {
    hri_mclk_set_APBDMASK_EVSYS_bit(MCLK);
}

void reset_event_system(void) {
    EVSYS->CTRLA.bit.SWRST = true;
    hri_mclk_clear_APBDMASK_EVSYS_bit(MCLK);
}

bool event_channel_free(uint8_t channel) {
    return 0;
}

void disable_event_channel(uint8_t channel_number) {
}

void disable_event_user(uint8_t user_number) {
}

void connect_event_user_to_channel(uint8_t user, uint8_t channel) {
}

void init_async_event_channel(uint8_t channel, uint8_t generator) {
}

void init_event_channel_interrupt(uint8_t channel, uint8_t gclk, uint8_t generator) {
}

bool event_interrupt_active(uint8_t channel) {
    return 0;
}

bool event_interrupt_overflow(uint8_t channel) {
    return 0;
}
