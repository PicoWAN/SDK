/*
 * led - LED driver
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
#include <led.h>
#include <gpio.h>
#include "boards.h"
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"

#define BLINKING_PERIOD		1200 // ms
#define REPEAT_FOREVER	-1

struct led_cfg {
	gpio_port_t port;
	uint8_t pin;
	enum led_mode mode;
	enum led_state state;
	uint16_t slots_mask;
	int repeat;
	uint8_t registered;
};

static struct led_cfg leds[MAX_LEDS] = {{0}};

static int current_slot = 0;

static TIM_HandleTypeDef htim;


static void led_register(uint8_t num, gpio_port_t port, uint8_t pin, enum led_mode mode)
{
	if (num >= MAX_LEDS) {
		return;
	}

	leds[num].port = port;
	leds[num].pin = pin;
	leds[num].mode = mode;
	leds[num].slots_mask = 0;
	leds[num].repeat = 0;
	leds[num].state = LED_OFF;
	leds[num].registered = 1;

	/* Turn the LED off */
	gpio_set_pin(leds[num].port, leds[num].pin, ((uint8_t) leds[num].state == (uint8_t) leds[num].mode) ? 1 : 0);
}

void led_init(void)
{
#if defined(LED0_PORT) && defined(LED0_PIN) && defined(LED0_MODE)
	led_register(0, LED0_PORT, LED0_PIN, LED0_MODE);
#endif
#if defined(LED1_PORT) && defined(LED1_PIN) && defined(LED1_MODE)
	led_register(1, LED1_PORT, LED1_PIN, LED1_MODE);
#endif
#if defined(LED2_PORT) && defined(LED2_PIN) && defined(LED2_MODE)
	led_register(2, LED2_PORT, LED2_PIN, LED2_MODE);
#endif
#if defined(LED3_PORT) && defined(LED3_PIN) && defined(LED3_MODE)
	led_register(3, LED3_PORT, LED3_PIN, LED3_MODE);
#endif

	/* Enable and set TIMER 21 INT to the highest priority */
	HAL_NVIC_SetPriority(TIM21_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM21_IRQn);
}

static void enable_blinking(void)
{
	TIM_ClockConfigTypeDef cctim;

	current_slot = 0;

	__HAL_RCC_TIM21_CLK_ENABLE();
	__HAL_RCC_TIM21_FORCE_RESET();
	__HAL_RCC_TIM21_RELEASE_RESET();

	htim.Instance			= TIM21;
	htim.Init.Prescaler		= 0;
	htim.Init.CounterMode		= TIM_COUNTERMODE_UP;
	htim.Init.Period		= ms2ostime(BLINKING_PERIOD/MAX_SLOTS);
	htim.Init.ClockDivision		= TIM_CLOCKDIVISION_DIV1;
	cctim.ClockSource		= TIM_CLOCKSOURCE_ETRMODE2;
	cctim.ClockPolarity		= TIM_CLOCKPOLARITY_NONINVERTED;
	cctim.ClockPrescaler		= TIM_CLOCKPRESCALER_DIV4;
	cctim.ClockFilter		= 0;

	/* Initialize TIM21 peripheral according to the passed parameters */
	HAL_TIM_ConfigClockSource(&htim, &cctim);
	HAL_TIM_Base_Init(&htim);
	HAL_TIMEx_RemapConfig(&htim, TIM21_ETR_LSE);
	HAL_TIM_Base_Start_IT(&htim);
}

static void disable_blinking(void)
{
	HAL_TIM_Base_Stop_IT(&htim);
	HAL_TIM_Base_DeInit(&htim);
	__HAL_RCC_TIM21_CLK_DISABLE();
}

static uint8_t is_blinking_enabled(void)
{
	return ((TIM21->CR1 & TIM_CR1_CEN) != 0);
}

static int set_led_state(uint8_t num, enum led_state state, uint16_t led_on_mask, int repeat)
{
	uint8_t i;

	if (num >= MAX_LEDS || leds[num].registered == 0) {
		return -1;
	}

	if (leds[num].state == state && leds[num].slots_mask == led_on_mask && leds[num].repeat == repeat) {
		return 0;
	}

	leds[num].state = state;
	if (leds[num].state == LED_BLINK) {
		leds[num].slots_mask = led_on_mask;
		leds[num].repeat = repeat;
	} else {
		leds[num].slots_mask = 0;
		leds[num].repeat = 0;
	}

	/* Check if blinking needs to be enabled or disabled */
	for (i = 0; i < MAX_LEDS; i++) {
		if (leds[i].state == LED_BLINK) {
			break;
		}
	}

	if (i < MAX_LEDS) {
		if (!is_blinking_enabled()) {
			enable_blinking();
		}
	} else {
		if (is_blinking_enabled()) {
			disable_blinking();
		}
	}

	/* Set the proper state */
	if (state == LED_BLINK) {
		if (((1 << current_slot) & leds[num].slots_mask) != 0) {
			state = LED_ON;
		} else {
			state = LED_OFF;
		}
	}

	gpio_set_pin(leds[num].port, leds[num].pin, ((uint8_t) state == (uint8_t) leds[num].mode) ? 1 : 0);
	return 0;
}

void led_set(uint8_t num, enum led_state state)
{
	if (state == LED_BLINK) {
		set_led_state(num, state, ON_OFF_SLOTS_PERIOD_16_DUTY_50, REPEAT_FOREVER);
	} else {
		set_led_state(num, state, 0, 0);
	}
}

void led_blink(uint8_t num, uint16_t led_on_mask, uint8_t repeat)
{
	int repeat_internal = repeat;

	if (repeat_internal == 0) {
		repeat_internal = REPEAT_FOREVER;
	}
	set_led_state(num, LED_BLINK, led_on_mask, repeat_internal);
}

enum led_state led_get(uint8_t num)
{
	if (num >= MAX_LEDS) {
		return LED_OFF;
	}

	return leds[num].state;
}

void led_toggle(uint8_t num)
{
	if (num >= MAX_LEDS) {
		return;
	}

	led_set(num, (leds[num].state == LED_OFF) ? LED_ON : LED_OFF);
}

void TIM21_IRQHandler()
{
	uint8_t i;

	if (TIM21->SR & TIM_SR_UIF) { // overflow
		current_slot++;
		if (current_slot >= MAX_SLOTS) {
			current_slot = 0;
		}

		for (i = 0; i < MAX_LEDS; i++) {
			if ((leds[i].state == LED_BLINK) && (leds[i].slots_mask != 0)) {
				if ((leds[i].repeat > 0) || (leds[i].repeat == REPEAT_FOREVER)) {
					int previous_slot = current_slot - 1;
					if (previous_slot < 0) {
						previous_slot = MAX_SLOTS - 1;
					}
					int previous_state = ((1 << previous_slot) & leds[i].slots_mask) != 0 ? 1 : 0;
					int current_state = ((1 << current_slot) & leds[i].slots_mask) != 0 ? 1 : 0;
					if (previous_state != current_state) {
						gpio_toggle_pin(leds[i].port, leds[i].pin);
					}
					if (leds[i].repeat > 0) {
						leds[i].repeat--;
						if (leds[i].repeat == 0) {
							leds[i].state = LED_OFF;
							leds[i].slots_mask = 0;
							gpio_set_pin(leds[i].port, leds[i].pin, ((uint8_t) LED_OFF == (uint8_t) leds[i].mode) ? 1 : 0);
						}
					}

				}
			}
		}
	}

	TIM21->SR = 0; // clear IRQ flags
}
