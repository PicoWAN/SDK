/*! \file
 * \brief UART/USART driver
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

#ifndef _USART_H_
#define _USART_H_

#include <stdint.h>
#include <arch.h>

/*!
 * \brief   Writes a byte to a given USART port (blocking).
 *
 * \param   usart The USART port number (USART_PORT_n).
 * \param   c The byte to write.
 */
void usart_putc(usart_port_t usart, uint8_t c);

/*!
 * \brief   Reads a byte from a given USART port (blocking).
 *
 * \param   usart The USART port number (USART_PORT_n).
 *
 * \retval  uint8_t The byte read.
 */
uint8_t usart_getc(usart_port_t usart);


/*!
 * \brief   Initializes a given USART (RX disabled by default).
 *
 * \param   usart The USART port number (USART_PORT_n).
 * \param   baudrate The requested baud-rate in bps.
 */
void usart_init(usart_port_t usart, uint32_t baudrate);

/*!
 * \brief   De-initializes a given USART.
 *
 * \param   usart The USART port number (USART_PORT_n).
 */
void usart_deinit(usart_port_t usart);

/*!
 * \brief   Enables the reception of data (RX).
 *
 * \param   usart The USART port number (USART_PORT_n).
 * \param   irq_cb The callback that will be executed in IRQ context when data will be available.
 */
void usart_enable_rx(usart_port_t usart, void (*irq_cb)(void));

/*!
 * \brief   Disables the reception of data (RX).
 *
 * \param   usart The USART port number (USART_PORT_n).
 */
void usart_disable_rx(usart_port_t usart);

/*!
 * \brief   Synchronizes a given USART.
 *
 * \details This function waits until the ongoing transmission in finished.
 *
 * \param   usart The USART port number (USART_PORT_n).
 */
void usart_sync(usart_port_t usart);

/*!
 * \brief   Checks is some data are available to read.
 *
 * \param   usart The USART port number (USART_PORT_n).
 *
 * \retval  uint8_t 1 if some data are available, 0 otherwise.
 */
uint8_t usart_is_rx_not_empty(usart_port_t usart);

#endif /* _USART_H_ */
