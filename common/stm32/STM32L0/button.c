/*
 * button - Button driver with debounce
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

#include <os.h>
#include <button.h>
#include "boards.h"
#include "gpio.h"
#include "system.h"
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"

#define LONG_PRESS_DURATION			1000 // ms
#define DEBOUNCE_DURATION			100 // ms

enum button_state {
	BUTTON_RELEASED = 0,
	BUTTON_PRESSED = 1,
	BUTTON_LONG_PRESSED = 2
};

struct button_cfg {
	gpio_port_t port;
	uint8_t pin;
	enum button_mode mode;
	enum button_state state;
	os_job_t handler_job;
	os_job_t long_press_job;
	uint8_t registered;
};

static struct button_cfg buttons[MAX_BUTTONS] = {{0}};

static void (*button_app_cb)(uint8_t num, enum button_press_duration duration) = NULL;


static void button_register(uint8_t num, gpio_port_t port, uint8_t pin, enum button_mode mode)
{
	if (num >= MAX_BUTTONS) {
		return;
	}

	buttons[num].port = port;
	buttons[num].pin = pin;
	buttons[num].mode = mode;
	buttons[num].state = BUTTON_RELEASED;
	buttons[num].registered = 1;

	if (button_app_cb != NULL) {
		gpio_config_irq(port, pin, (mode == BUTTON_ACTIVE_HIGH) ? GPIO_IRQ_MODE_RISING : GPIO_IRQ_MODE_FALLING);
	}
}

void button_init(void (*cb)(uint8_t num, enum button_press_duration duration))
{
	button_app_cb = cb;

#if defined(BUTTON0_PORT) && defined(BUTTON0_PIN) && defined(BUTTON0_MODE)
	button_register(0, BUTTON0_PORT, BUTTON0_PIN, BUTTON0_MODE);
#endif
#if defined(BUTTON1_PORT) && defined(BUTTON1_PIN) && defined(BUTTON1_MODE)
	button_register(1, BUTTON1_PORT, BUTTON1_PIN, BUTTON1_MODE);
#endif
#if defined(BUTTON2_PORT) && defined(BUTTON2_PIN) && defined(BUTTON2_MODE)
	button_register(2, BUTTON2_PORT, BUTTON2_PIN, BUTTON2_MODE);
#endif
#if defined(BUTTON3_PORT) && defined(BUTTON3_PIN) && defined(BUTTON3_MODE)
	button_register(3, BUTTON3_PORT, BUTTON3_PIN, BUTTON3_MODE);
#endif
}

uint8_t button_is_pressed(uint8_t num)
{
	uint8_t res;

	if (num >= MAX_BUTTONS || buttons[num].registered == 0) {
		return 0;
	}

	res = gpio_get_pin(buttons[num].port, buttons[num].pin);

	return ((buttons[num].mode == BUTTON_ACTIVE_HIGH) ? res : !res);
}

static void long_press_task(os_job_t *j)
{
	uint8_t button_num;

	/* Look-up the button number using the job address */
	if (j == &(buttons[0].long_press_job)) {
		button_num = 0;
	} else if (j == &(buttons[1].long_press_job)) {
		button_num = 1;
	} else if (j == &(buttons[2].long_press_job)) {
		button_num = 2;
	} else if (j == &(buttons[3].long_press_job)) {
		button_num = 3;
	} else {
		return;
	}

	buttons[button_num].state = BUTTON_LONG_PRESSED;

	if (button_app_cb != NULL) {
		button_app_cb(button_num, BUTTON_LONG_PRESS);
	}
}

static void button_debounced_handler_task(os_job_t *j)
{
	uint8_t button_num;

	/* Look-up the button number using the job address */
	if (j == &(buttons[0].handler_job)) {
		button_num = 0;
	} else if (j == &(buttons[1].handler_job)) {
		button_num = 1;
	} else if (j == &(buttons[2].handler_job)) {
		button_num = 2;
	} else if (j == &(buttons[3].handler_job)) {
		button_num = 3;
	} else {
		return;
	}

	/* In order to be sure the requested IRQ edge stays aligned with the current
	 * GPIO state, that state is checked before changing the detection edge.
	 */
	if (buttons[button_num].state == BUTTON_PRESSED || buttons[button_num].state == BUTTON_LONG_PRESSED) {
		/* Release detected */
		os_cancel_job(&(buttons[button_num].long_press_job));

		if (!button_is_pressed(button_num)) {
			/* Invert the IRQ detection edge */
			gpio_config_irq(buttons[button_num].port, buttons[button_num].pin,
				      (buttons[button_num].mode == BUTTON_ACTIVE_HIGH) ? GPIO_IRQ_MODE_RISING : GPIO_IRQ_MODE_FALLING);
		} else {
			/* The button has been pressed again within the debounce period,
			 * trigger the handler once again to be sure the event will
			 * be reported.
			 */
			os_post_job(&(buttons[button_num].handler_job), button_debounced_handler_task);
		}

		/* Report a short-press if no long-press has been reported yet */
		if (button_app_cb != NULL && buttons[button_num].state == BUTTON_PRESSED) {
			button_app_cb(button_num, BUTTON_SHORT_PRESS);
		}

		buttons[button_num].state = BUTTON_RELEASED;
	} else {
		/* Press detected */
		if (button_is_pressed(button_num)) {
			/* Invert the IRQ detection edge */
			gpio_config_irq(buttons[button_num].port, buttons[button_num].pin,
				      (buttons[button_num].mode == BUTTON_ACTIVE_HIGH) ? GPIO_IRQ_MODE_FALLING : GPIO_IRQ_MODE_RISING);
		} else {
			/* The button has been released within the debounce period,
			 * trigger the handler once again to be sure the event will
			 * be reported.
			 */
			os_post_job(&(buttons[button_num].handler_job), button_debounced_handler_task);
		}

		buttons[button_num].state = BUTTON_PRESSED;

		/* Queue the long-press job */
		os_post_delayed_job(&(buttons[button_num].long_press_job), os_get_time() + ms2ostime(LONG_PRESS_DURATION), long_press_task);
	}
}

void button_irq_handler(void)
{
#if defined(BUTTON0_PIN)
	if (__HAL_GPIO_EXTI_GET_IT(1 << BUTTON0_PIN) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(1 << BUTTON0_PIN);
		/* Delay the actual handler (debounce) */
		os_post_delayed_job(&(buttons[0].handler_job), os_get_time() + ms2ostime(DEBOUNCE_DURATION), button_debounced_handler_task);
	}
#endif
#if defined(BUTTON1_PIN)
	if (__HAL_GPIO_EXTI_GET_IT(1 << BUTTON1_PIN) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(1 << BUTTON1_PIN);
		/* Delay the actual handler (debounce) */
		os_post_delayed_job(&(buttons[1].handler_job), os_get_time() + ms2ostime(DEBOUNCE_DURATION), button_debounced_handler_task);
	}
#endif
#if defined(BUTTON2_PIN)
	if (__HAL_GPIO_EXTI_GET_IT(1 << BUTTON2_PIN) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(1 << BUTTON2_PIN);

		/* Delay the actual handler (debounce) */
		os_post_delayed_job(&(buttons[2].handler_job), os_get_time() + ms2ostime(DEBOUNCE_DURATION), button_debounced_handler_task);
	}
#endif
#if defined(BUTTON3_PIN)
	if (__HAL_GPIO_EXTI_GET_IT(1 << BUTTON3_PIN) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(1 << BUTTON3_PIN);

		/* Delay the actual handler (debounce) */
		os_post_delayed_job(&(buttons[3].handler_job), os_get_time() + ms2ostime(DEBOUNCE_DURATION), button_debounced_handler_task);
	}
#endif
}
