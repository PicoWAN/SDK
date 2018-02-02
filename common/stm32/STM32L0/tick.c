/*
 * tick - Tick driver for STM32L0 using the LPTIMER 1
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
#include <tick.h>
#include <system.h>
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"

#define WAKEUP_NOW_THRESHOLD	5 // in ticks (the minimal time the CPU can spend in regular Sleep mode)

static uint64_t ticks = 0;

static LPTIM_HandleTypeDef hlptim;


void tick_init(void)
{
	/* Enable LPTIM clock */
	__HAL_RCC_LPTIM1_CLK_ENABLE();

	/* Reset LPTIM interface */
	__HAL_RCC_LPTIM1_FORCE_RESET();
	__HAL_RCC_LPTIM1_RELEASE_RESET();

	/* Internal clock source (LSE) with /4 prescaler and no filter (8,192 kHz) */
	hlptim.Instance                = LPTIM1;
	hlptim.Init.Clock.Source       = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
	hlptim.Init.Clock.Prescaler    = LPTIM_PRESCALER_DIV4;
	hlptim.Init.CounterSource      = LPTIM_COUNTERSOURCE_INTERNAL;
	hlptim.Init.Trigger.Source     = LPTIM_TRIGSOURCE_SOFTWARE;
	hlptim.Init.Trigger.ActiveEdge = LPTIM_ACTIVEEDGE_RISING;

	/* Initialize LPTIM peripheral according to the passed parameters */
	HAL_LPTIM_Init(&hlptim);

	/* Start counting */
	HAL_LPTIM_Counter_Start_IT(&hlptim, 0xFFFF);

	/* Enable and set LPTIM Interrupt to the highest priority */
	HAL_NVIC_SetPriority(LPTIM1_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
}

uint64_t tick_get_count(void)
{
	uint64_t t;
	uint16_t cnt;

	system_disable_irqs();

	t = ticks;
	cnt = LPTIM1->CNT;

	/* Check if an overflow is not already pending */
	if (__HAL_LPTIM_GET_FLAG(&hlptim, LPTIM_FLAG_ARRM) != RESET) {
		t++;
		cnt = LPTIM1->CNT;
	}

	system_enable_irqs();

	return (cnt | (t << 16));
}

latency_t tick_handle_next_wakeup(uint64_t time, uint8_t lp_blocked)
{
	uint64_t t = tick_get_count();
	uint16_t cnt = t & 0xFFFF;
	int64_t delta = time - t;

	/*
	 * When a job is scheduled, the CPU will first go in LP Sleep mode for as long
	 * as possible, then it will go in sleep mode for a very short time (the smallest,
	 * the better), and finally, the cpu will wait in Run mode for the exact time of
	 * the job (theoretically no time at all, since it is better to wait in Sleep mode
	 * than in Run mode).
	 *
	 *                                      HAL_CPU_LP_WAKEUP_LATENCY
	 *                                   |<---------------------------->|
	 * LP Sleep Mode: ...xxxxxxxxxxxxxxxx|                              |
	 * Sleep Mode   :                    |                        |xxxxx|
	 * Run Mode     :                    |xxxxxxxxxxxxxxxxxxxxxxxx|     |xxxxx...
	 *                                   |<---------------------->|     |<----...
	 *                                     Actual wake-up from LP |     | Job
	 *                                                            |<--->|
	 *                                                    > WAKEUP_NOW_THRESHOLD
	 */

	/* Clear any pending interrupt */
	__HAL_LPTIM_CLEAR_FLAG(&hlptim, LPTIM_FLAG_CMPM);

	if (delta < WAKEUP_NOW_THRESHOLD) {
		/* The deadline is almost now (no time sleep), disable the comparator and its interrupt */
		__HAL_LPTIM_DISABLE_IT(&hlptim, LPTIM_IT_CMPM);
		return NO_LATENCY;
	} else if (delta < (HAL_CPU_LP_SLEEP_LATENCY + HAL_CPU_LP_WAKEUP_LATENCY) || lp_blocked) {
		if (delta + cnt <= 0xFFFF) {
			/* The deadline is soon enough, configure the comparator to its exact time */
			__HAL_LPTIM_COMPARE_SET(&hlptim, (uint16_t) (delta + cnt));
			/* Enable comparator interrupt */
			__HAL_LPTIM_ENABLE_IT(&hlptim, LPTIM_IT_CMPM);
		}

		/* Notify the scheduler that we have no time to go to low power sleep mode */
		return LOW_LATENCY;
	} else if ((delta + cnt - HAL_CPU_LP_WAKEUP_LATENCY) <= 0xFFFF) {
		/* The deadline is soon enough, configure the comparator in order compensate the CPU wakeup latency */
		__HAL_LPTIM_COMPARE_SET(&hlptim, (uint16_t) (delta + cnt - HAL_CPU_LP_WAKEUP_LATENCY));
		/* Enable comparator interrupt */
		__HAL_LPTIM_ENABLE_IT(&hlptim, LPTIM_IT_CMPM);
	}

	/* Notify the scheduler that we have enough time to go to low power sleep mode */
	return HIGH_LATENCY;
}

void LPTIM1_IRQHandler(void)
{
	if (__HAL_LPTIM_GET_FLAG(&hlptim, LPTIM_FLAG_ARROK) != RESET) {
		if (__HAL_LPTIM_GET_IT_SOURCE(&hlptim, LPTIM_IT_ARROK) != RESET) {
			__HAL_LPTIM_CLEAR_FLAG(&hlptim, LPTIM_FLAG_ARROK);
		}
	}

	if (__HAL_LPTIM_GET_FLAG(&hlptim, LPTIM_FLAG_CMPM) != RESET) {
		if (__HAL_LPTIM_GET_IT_SOURCE(&hlptim, LPTIM_IT_CMPM) != RESET) {
			__HAL_LPTIM_CLEAR_FLAG(&hlptim, LPTIM_FLAG_CMPM);
			/* Just wake the CPU */
		}
	}

	/* Autoreload match interrupt */
	if (__HAL_LPTIM_GET_FLAG(&hlptim, LPTIM_FLAG_ARRM) != RESET) {
		if (__HAL_LPTIM_GET_IT_SOURCE(&hlptim, LPTIM_IT_ARRM) != RESET) {
			__HAL_LPTIM_CLEAR_FLAG(&hlptim, LPTIM_FLAG_ARRM);
			ticks++;
		}
	}
}
