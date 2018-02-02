/*
 * tick - Tick driver for STM32L1 using the TIMER 9
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
#include "stm32l1xx.h"

#define WAKEUP_NOW_THRESHOLD	5 // in ticks (the minimal time the CPU can spend in regular Sleep mode)

static uint64_t ticks = 0;


void tick_init(void)
{
	/* Enable write access */
	PWR->CR |= PWR_CR_DBP;

	/* Enable LSE (32,768 kHz), and wait until it is ready */
	RCC->CSR |= RCC_CSR_LSEON;
	while (!(RCC->CSR & RCC_CSR_LSERDY));

	/* Enable TIM9 clock */
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
	/* Enable TIM9 clock in low-power mode */
	RCC->APB2LPENR |= RCC_APB2LPENR_TIM9LPEN;

	/* Reset TIM9 interface */
	RCC->APB2RSTR |= RCC_APB2RSTR_TIM9RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM9RST;

	/* External clock enable (source clock mode 2) with /4 prescaler and no filter (8,192 kHz) */
	TIM9->SMCR = TIM_SMCR_ECE | TIM_SMCR_ETPS_1;

	/* Configure NVIC (set priority and enable IRQ) */
	NVIC->IP[TIM9_IRQn] = 0x50;
	NVIC->ISER[TIM9_IRQn >> 5] = 1 << (TIM9_IRQn & 0x1F);

	/* Enable interrupt on update (and overflow) event */
	TIM9->DIER |= TIM_DIER_UIE;

	/* Start counting */
	TIM9->CR1 = TIM_CR1_CEN;
}

uint64_t tick_get_count(void)
{
	uint64_t t;
	uint16_t cnt;

	system_disable_irqs();

	t = ticks;
	cnt = TIM9->CNT;

	/* Check if an overflow is not already pending */
	if (TIM9->SR & TIM_SR_UIF) {
		t++;
		cnt = TIM9->CNT;
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
	TIM9->SR &= ~TIM_SR_CC2IF;

	if (delta < WAKEUP_NOW_THRESHOLD) {
		/* The deadline is almost now (no time sleep), disable the comparator and its interrupt */
		TIM9->DIER &= ~TIM_DIER_CC2IE;
		TIM9->CCER &= ~TIM_CCER_CC2E;
		return NO_LATENCY;
	} else if (delta < (HAL_CPU_LP_SLEEP_LATENCY + HAL_CPU_LP_WAKEUP_LATENCY) || lp_blocked) {
		if (delta + cnt <= 0xFFFF) {
			/* The deadline is soon enough, configure the comparator to its exact time */
			TIM9->CCR2 = (uint16_t) (delta + cnt);
			/* Enable comparator interrupt */
			TIM9->DIER |= TIM_DIER_CC2IE;
			/* Enable comparator */
			TIM9->CCER |= TIM_CCER_CC2E;
		}

		/* Notify the scheduler that we have no time to go to low power sleep mode */
		return LOW_LATENCY;
	} else if ((delta + cnt - HAL_CPU_LP_WAKEUP_LATENCY) <= 0xFFFF) {
		/* The deadline is soon enough, configure the comparator in order compensate the CPU wakeup latency */
		TIM9->CCR2 = (uint16_t) (delta + cnt - HAL_CPU_LP_WAKEUP_LATENCY);
		/* Enable comparator interrupt */
		TIM9->DIER |= TIM_DIER_CC2IE;
		/* Enable comparator */
		TIM9->CCER |= TIM_CCER_CC2E;
	}

	/* Notify the scheduler that we have enough time to go to low power sleep mode */
	return HIGH_LATENCY;
}

void TIM9_IRQHandler(void)
{
	if (TIM9->SR & TIM_SR_UIF) {
		/* Overflow of TIM9 */
		ticks++;
	}

	/* Clear all interrupts */
	TIM9->SR = 0;
}
