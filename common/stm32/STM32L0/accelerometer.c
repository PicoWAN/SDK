/*
 * accelerometer - Accelerometer stuff related to the STM32 platform
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
#include <accelerometer.h>
#include "boards.h"
#include "gpio.h"
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"

#define DEBOUNCE_DURATION			100 // ms

#if defined(ACC1_PIN) || defined(ACC2_PIN)
static os_job_t int1_handler_job;
static os_job_t int2_handler_job;
#endif

static void (*accel_app_cb)(uint8_t num) = NULL;


void accelerometer_init(void (*irq_cb)(uint8_t num))
{
	accel_app_cb = irq_cb;

	if (accel_app_cb == NULL) {
		return;
	}

#if defined(ACC1_PORT) && defined(ACC1_PIN)
	gpio_config_irq(ACC1_PORT, ACC1_PIN, GPIO_IRQ_MODE_RISING);
#endif
#if defined(ACC2_PORT) && defined(ACC2_PIN)
	gpio_config_irq(ACC2_PORT, ACC2_PIN, GPIO_IRQ_MODE_RISING);
#endif
}

#if defined(ACC1_PIN) || defined(ACC2_PIN)
static void debounced_handler_task(os_job_t *j)
{
	uint8_t int_num;

	/* Look-up the INT number using the job address */
	if (j == &int1_handler_job) {
		int_num = ACC1_INT;
	} else if (j == &int2_handler_job) {
		int_num = ACC2_INT;
	} else {
		return;
	}

	if (accel_app_cb != NULL) {
		accel_app_cb(int_num);
	}
}
#endif

void accelerometer_irq_handler(void)
{
#if defined(ACC1_PIN)
	if (__HAL_GPIO_EXTI_GET_IT(1 << ACC1_PIN) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(1 << ACC1_PIN);

		/* Delay the actual handler (debounce) */
		os_post_delayed_job(&int1_handler_job, os_get_time() + ms2ostime(DEBOUNCE_DURATION), debounced_handler_task);
	}
#endif
#if defined(ACC2_PIN)
	if (__HAL_GPIO_EXTI_GET_IT(1 << ACC2_PIN) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(1 << ACC2_PIN);

		/* Delay the actual handler (debounce) */
		os_post_delayed_job(&int2_handler_job, os_get_time() + ms2ostime(DEBOUNCE_DURATION), debounced_handler_task);
	}
#endif
}
