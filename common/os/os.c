/*
 * os - Implementation of a Low-Power scheduler
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

#include <system.h>
#include <tick.h>
#include "os.h"

static os_job_t *job_scheduled = NULL;
static os_job_t *delayed_job_scheduled = NULL;

#ifndef CFG_DISABLE_LOW_POWER_MODE
static uint8_t powersave_blocked = 0;
#else
static uint8_t powersave_blocked = 1;
#endif


void os_init(void)
{
	system_init();
	tick_init();
}

os_time_t os_get_time(void)
{
	return (os_time_t) tick_get_count();
}

void os_wait_until(os_time_t time)
{
	while (tick_get_count() < time);
}

void os_delay(os_time_t time)
{
	os_wait_until(os_get_time() + time);
}

os_time_t os_get_elapsed_time(os_time_t time)
{
	os_time_t elapsed_time = 0;

	elapsed_time = os_get_time();

	return (elapsed_time - time);
}

static uint8_t delete_job_from_queue(os_job_t *job, os_job_t **queue)
{
	while (*queue != NULL) {
		if (*queue == job) {
			*queue = (*queue)->next;
			return 1;
		}

		queue = &((*queue)->next);
	}

	return 0;
}

uint8_t os_cancel_job(os_job_t *job)
{
	uint8_t res;

	system_disable_irqs();

	res = delete_job_from_queue(job, &job_scheduled) | delete_job_from_queue(job, &delayed_job_scheduled);

	system_enable_irqs();

	return res;
}

void os_post_job(os_job_t *job, void (*cb)(os_job_t *))
{
	os_job_t **j = &job_scheduled;

	system_disable_irqs();

	/* Make sure the job is not already queued */
	os_cancel_job(job);

	while (*j != NULL) {
		j = &((*j)->next);
	}

	job->callback = cb;
	job->next = NULL;
	*j = job;

	system_enable_irqs();
}

void os_post_delayed_job(os_job_t *job, os_time_t time, void (*cb)(os_job_t *))
{
	os_job_t **j = &delayed_job_scheduled;

	system_disable_irqs();

	/* Make sure the job is not already queued */
	os_cancel_job(job);

	while (*j != NULL) {
		if ((*j)->time > time) {
			/* That's the right place to insert the job in order to keep a sorted list */
			break;
		}

		j = &((*j)->next);
	}

	job->callback = cb;
	job->time = time;
	job->next = *j;
	*j = job;

	system_enable_irqs();
}

void os_unblock_powersave(void)
{
#ifndef CFG_DISABLE_LOW_POWER_MODE
	system_disable_irqs();

	if (powersave_blocked) {
		powersave_blocked--;
	}

	system_enable_irqs();
#endif
}

void os_block_powersave(void)
{
#ifndef CFG_DISABLE_LOW_POWER_MODE
	system_disable_irqs();

	if (powersave_blocked < 0xFF) {
		powersave_blocked++;
	}

	system_enable_irqs();
#endif
}

void os_mainloop(void)
{
	os_job_t *job;
	latency_t latency = HIGH_LATENCY;

	while (1) {
		system_disable_irqs();

		if (powersave_blocked) {
			latency = LOW_LATENCY;
		} else {
			latency = HIGH_LATENCY;
		}

		if (job_scheduled != NULL) {
			/* A job is waiting to be started */
			job = job_scheduled;
			job_scheduled = job_scheduled->next;
		} else if (delayed_job_scheduled != NULL &&
			   (latency = tick_handle_next_wakeup(delayed_job_scheduled->time, powersave_blocked)) == NO_LATENCY) {
			/* The next delayed job is scheduled for now */
			job = delayed_job_scheduled;
			delayed_job_scheduled = delayed_job_scheduled->next;
			os_wait_until(job->time);
		} else {
			job = NULL;
			if (latency == LOW_LATENCY) {
				/* Just wait for the next IRQ (low latency) */
				system_sleep();
			} else {
				/* Go in lowpower mode, and wait for the next IRQ (high latency) */
				system_sleep_low_power();
			}
		}

		system_enable_irqs();

		if (job != NULL) {
			job->callback(job);
		}
	}
}
