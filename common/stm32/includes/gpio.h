/*! \file
 * \brief GPIO driver
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

#ifndef _GPIO_H_
#define _GPIO_H_

#include <stdint.h>
#include <arch.h>

typedef enum {
	GPIO_PMODE_INPUT,
	GPIO_PMODE_OUTPUT,
	GPIO_PMODE_ANALOG,
	GPIO_PMODE_ALT_FUNC,
} gpio_pin_mode_t;

typedef enum {
	GPIO_OMODE_OPEN_DRAIN,
	GPIO_OMODE_PUSH_PULL,
} gpio_output_mode_t;

typedef enum {
	GPIO_PUPD_NONE,
	GPIO_PUPD_UP,
	GPIO_PUPD_DOWN,
} gpio_pupd_t;

typedef enum {
	GPIO_OSPEED_LOW,
	GPIO_OSPEED_MED,
	GPIO_OSPEED_HIGH,
	GPIO_OSPEED_VHIGH,
} gpio_output_speed_t;

typedef enum {
	GPIO_IRQ_MODE_NONE,
	GPIO_IRQ_MODE_FALLING,
	GPIO_IRQ_MODE_RISING,
	GPIO_IRQ_MODE_BOTH,
} gpio_irq_mode_t;


/*!
 * \brief   Configures the behavior of a particular pin.
 *
 * \param   port The port of the pin (GPIO_PORT_A, GPIO_PORT_B, ...).
 * \param   pin The number of the pin.
 * \param   mode The mode of the pin. This can be GPIO_PMODE_INPUT for input, GPIO_PMODE_OUTPUT for output, GPIO_PMODE_ALT_FUNC
 *               for alternate-function, or GPIO_PMODE_ANALOG for analog.
 * \param   type The type of the pin. This can be GPIO_OMODE_PUSH_PULL for push-pull or GPIO_OMODE_OPEN_DRAIN for open-drain.
 * \param   pull The internal pull-up/down of the pin. This can be GPIO_PUPD_UP, GPIO_PUPD_DOWN or GPIO_PUPD_NONE.
 * \param   speed The speed of the pin. This can be GPIO_OSPEED_LOW, GPIO_OSPEED_MED, GPIO_OSPEED_HIGH, or GPIO_OSPEED_VHIGH.
 * \param   alt The alternate function number of the pin (used only if if its mode is GPIO_PMODE_ALT_FUNC).
 */
void gpio_cfg_pin(gpio_port_t port, uint8_t pin, gpio_pin_mode_t mode, gpio_output_mode_t omode, gpio_pupd_t pull, gpio_output_speed_t speed, uint8_t alt);

/*!
 * \brief   Sets the state of a particular pin configured in output mode.
 *
 * \param   port The port of the pin (GPIO_PORT_A, GPIO_PORT_B, ...).
 * \param   pin The number of the pin.
 * \param   state The new state of the pin (1 for High, 0 for Low).
 */
void gpio_set_pin(gpio_port_t port, uint8_t pin, uint8_t state);

/*!
 * \brief   Toggles the state of a particular pin configured in output mode.
 *
 * \param   port The port of the pin (GPIO_PORT_A, GPIO_PORT_B, ...).
 * \param   pin The number of the pin.
 */
void gpio_toggle_pin(gpio_port_t port, uint8_t pin);

/*!
 * \brief   Gets the state of a particular pin.
 *
 * \param   port The port of the pin (GPIO_PORT_A, GPIO_PORT_B, ...).
 * \param   pin The number of the pin.
 *
 * \retval  uint8_t 1 for High, 0 for Low.
 */
uint8_t gpio_get_pin(gpio_port_t port, uint8_t pin);

/*!
 * \brief   Configures the IRQ mode of a pin.
 *
 * \param   port The port of the pin (GPIO_PORT_A, GPIO_PORT_B, ...).
 * \param   pin The number of the pin.
 * \param   mode The IRQ mode of the pin. This can be GPIO_IRQ_MODE_NONE, GPIO_IRQ_MODE_FALLING, GPIO_IRQ_MODE_RISING or GPIO_IRQ_MODE_BOTH.
 */
void gpio_config_irq(gpio_port_t port, uint8_t pin, gpio_irq_mode_t mode);

#endif /* _GPIO_H_ */
