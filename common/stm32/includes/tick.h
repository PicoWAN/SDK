/*! \file
 * \brief Tick driver
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

#ifndef _TICK_H_
#define _TICK_H_

#include <stdint.h>
#include <arch.h>

typedef enum latency {
	NO_LATENCY,
	LOW_LATENCY,
	HIGH_LATENCY,
} latency_t;


/*!
 * \brief   Initializes the system tick.
 *
 * \note    This function is used by the scheduler and should not be called anywhere else in normal conditions.
 */
void tick_init(void);

/*!
 * \brief   Gets the number of elapsed ticks since the device has been powered up.
 *
 * \note    This function is used by the scheduler and should not be called anywhere else in normal conditions.
 *
 * \retval  uint64_t The number of elapsed ticks.
 */
uint64_t tick_get_count(void);

/*!
 * \brief   Handles the next scheduled event.
 *
 * \details This function decides in which latency mode the device must be put until the next scheduled event, and configures
 *          a timer to wake it up accordingly.
 *
 * \note    This function is used by the scheduler and should not be called anywhere else in normal conditions.
 *
 * \param   time The deadline for the next scheduled event.
 * \param   lp_blocked 0 if the the high latency mode is allowed, 1 otherwise.
 *
 * \retval  latency_t The latency mode decided.
 */
latency_t tick_handle_next_wakeup(uint64_t time, uint8_t lp_blocked);

#endif /* _TICK_H_ */
