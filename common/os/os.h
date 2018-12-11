/*! \file
 * \brief Implementation of a Low-Power scheduler
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

#ifndef _OS_H_
#define _OS_H_

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <tick.h>

typedef int64_t os_time_t;

#ifndef TICK_FREQ
#error "TICK_FREQ has not been defined for the current MCU !"
#elif ((TICK_FREQ % 64) != 0)
#error "TICK_FREQ must be a multiple of 64 !"
#elif (TICK_FREQ < 8192)
#error "TICK_FREQ too low (should be >= 8192) !"
#endif

#define s2ostime(s)		((os_time_t) ((int64_t) (s) * TICK_FREQ))
#define ms2ostime(ms)		((os_time_t) (((int64_t) (ms) * TICK_FREQ) / 1000))
#define us2ostime(us)		((os_time_t) (((int64_t) (us) * (TICK_FREQ / 64)) / 15625))

#define ostime2s(t)		((int64_t) ((t) / TICK_FREQ))
#define ostime2ms(t)		((int64_t) (((t) * (int64_t) 1000) / TICK_FREQ))
#define ostime2us(t)		((int64_t) (((t) * (int64_t) 15625) / (TICK_FREQ / 64)))

struct os_job {
	void (*callback)(struct os_job *);
	os_time_t time;
	struct os_job *next;
};

typedef struct os_job os_job_t;


/*!
 * \brief   Initializes the Operating System.
 *
 * \details This function initializes the Low-Level system dependencies.
 */
void os_init(void);

/*!
 * \brief   Gets the current time.
 *
 * \details This function returns the current time in ticks.
 *
 * \retval  os_time_t
 */
os_time_t os_get_time(void);

/*!
 * \brief   Waits until a given time.
 *
 * \details This function blocks until the given time is reached.
 *
 * \param   time The timestamp to reach.
 */
void os_wait_until(os_time_t time);

/*!
 * \brief   Waits for a given time.
 *
 * \details This function waits for a given time.
 *
 * \param   time How much time to wait.
 */
void os_delay(os_time_t time);

/*!
 * \brief   Returns how much time passed since a given timestamp.
 *
 * \details This function returns how much time passed since a given timestamp.
 *
 * \param   time The timestamp used as origin.
 *
 * \retval  os_time_t
 */
os_time_t os_get_elapsed_time(os_time_t time);

/*!
 * \brief   Cancels a scheduled job.
 *
 * \details This function cancels a job that has been previously scheduled (delayed or not).
 *
 * \param   job The job to cancel.
 *
 * \retval  uint8_t 1 if the job has been canceled, 0 otherwise.
 */
uint8_t os_cancel_job(os_job_t *job);

/*!
 * \brief   Posts a job to be executed as soon as possible.
 *
 * \details This function posts a job that will be executed as soon as possible. If the job is already scheduled, it will be canceled first.
 *
 * \param   job The job handle.
 * \param   cb The callback that will be executed.
 */
void os_post_job(os_job_t *job, void (*cb)(os_job_t *));

/*!
 * \brief   Posts a job to be executed at a given time.
 *
 * \details This function posts a job that will be executed at a given time. If the job is already scheduled, it will be canceled first.
 *
 * \param   job The job handle.
 * \param   time The timestamp at which the job needs to be executed.
 * \param   cb The callback that will be executed.
 */
void os_post_delayed_job(os_job_t *job, os_time_t time, void (*cb)(os_job_t *));

/*!
 * \brief   Allows the system to go to sleep mode.
 *
 * \details This function allows the system to go to sleep mode. Any calls to os_block_powersave() must be released
 *          at a point by a call to os_unblock_powersave().
 */
void os_unblock_powersave(void);

/*!
 * \brief   Prevents the system from going to sleep mode.
 *
 * \details This function prevents the system from going to sleep, thus allowing to get the lowest possible latency.
 *          It uses a reference counter, so as many calls to os_unblock_powersave() as calls to os_block_powersave()
 *          will be needed to re-enable the sleep mode.
 */
void os_block_powersave(void);

/*!
 * \brief   The Operating System mainloop.
 *
 * \details This is the Operating System mainloop. It must be called only once, as soon as basic system initialization has been performed.
 */
void os_mainloop(void);

#endif /* _OS_H_ */
