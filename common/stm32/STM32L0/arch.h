/*! \file
 * \brief Architecture dependent parameters
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

#ifndef _ARCH_H_
#define _ARCH_H_

/* Tick system is LSE/4 = 8,192 kHz */
#define TICK_FREQ          8192 // Hz

/* CPU Low-Power wakeup and sleep latency */
#define HAL_CPU_LP_WAKEUP_LATENCY	24 // in ticks (~3 ms)
#define HAL_CPU_LP_SLEEP_LATENCY	24 // in ticks (~3 ms)

/* GPIO mapping */
typedef enum {
	GPIO_PORT_A = 0,
	GPIO_PORT_B = 1,
	GPIO_PORT_C = 2,
	GPIO_PORT_D = 3,
	GPIO_PORT_E = 4,
	GPIO_PORT_H = 7,
} gpio_port_t;

/* List of available USART ports */
typedef enum {
	USART_PORT_1,
	USART_PORT_2,
	USART_PORT_4,
	USART_PORT_5,
	LPUART_PORT_1,
} usart_port_t;

/* List of available I2C ports */
typedef enum {
	I2C_PORT_1,
	I2C_PORT_2,
} i2c_port_t;

#endif /* _ARCH_H_ */
