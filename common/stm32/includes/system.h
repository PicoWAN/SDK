/*! \file
 * \brief System related stuff
 *
 * Copyright (c) 2018, Archos S.A.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of Archos nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
 * AND EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ARCHOS S.A. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include <stdint.h>
#include <arch.h>
#include "gpio.h"

#define PIN_END				255

typedef struct io_pin {
	gpio_port_t port;
	uint8_t pin;
	gpio_pupd_t pupd;
} io_pin_t;

typedef enum rf_path {
	PATH_RX,
	PATH_TX_RFO,
	PATH_TX_PA,
	PATH_NONE,
} rf_path_t;


/*!
 * \brief   Switches the DC-DC converter to high-power mode.
 *
 * \details If the DC-DC converter of the board supports different power modes, this function allows to switch
 *          from low-power to high-power mode.
 *
 * \param   en 1 to enable the high power mode, 0 to disable it.
 */
void system_set_dcdc_high_power(uint8_t en);

/*!
 * \brief   Powers up/down the radio IC.
 *
 * \param   en 1 to power up the radio IC, 0 to power it down.
 */
void system_enable_sx_power(uint8_t en);

/*!
 * \brief   Enables/disables the radio IC's clock.
 *
 * \param   en 1 enable the clock, 0 to disable it.
 */
void system_enable_sx_clock(uint8_t en);

/*!
 * \brief   Selects the requested RF path.
 *
 * \note    What can actually be done depends on the RF switches available on the board.
 *
 * \param   path The path to select. This can be PATH_RX, PATH_TX_RFO, PATH_TX_PA, or PATH_NONE.
 */
void system_try_select_rf_path(rf_path_t path);

/*!
 * \brief   Sets to HIGH or LOW the radio nSS (inverted slave select) pin (for SPI communication).
 *
 * \param   state 1 for HIGH the pin (deselects the radio IC), 0 for LOW (selects the radio IC).
 */
void system_set_sx_nss(uint8_t state);

/*!
 * \brief   Sets to HIGH, LOW, or floating the radio reset pin (for SPI communication).
 *
 * \param   state 1 for HIGH the pin, 0 for LOW (selects the radio IC), or 2 for floating.
 */
void system_set_sx_rst(uint8_t state);

/*!
 * \brief   Disables all IRQs.
 */
void system_disable_irqs(void);

/*!
 * \brief   Enables all IRQs.
 */
void system_enable_irqs(void);

/*!
 * \brief   Registers additional pins whose configuration and state must be kept when the board/device is in low-power sleep mode.
 *
 * \note    Any pin whose function cannot dynamically change should be declared in boards.h instead.
 *
 * \param   pins The pins to register.
 */
void system_register_lowpower_pins(io_pin_t *pins);

/*!
 * \brief   Enters sleep mode.
 *
 * \note    This function is used by the scheduler and should not be called anywhere else.
 */
void system_sleep(void);

/*!
 * \brief   Enters low-power sleep mode.
 *
 * \note    This function is used by the scheduler and should not be called anywhere else.
 */
void system_sleep_low_power(void);

/*!
 * \brief   Reboots the system.
 */
void system_reboot(void);

/*!
 * \brief   Initializes the system (IOs, clocks, ...).
 *
 * \note    This function is used by the scheduler and should not be called anywhere else.
 */
void system_init(void);

#endif /* _SYSTEM_H_ */
