/*
 * usart - UART/USART driver
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
#include <usart.h>
#include "stm32l1xx.h"

static void (*usart1_irq_cb)(void) = NULL;
static void (*usart2_irq_cb)(void) = NULL;
static void (*usart3_irq_cb)(void) = NULL;


static USART_TypeDef * usart_lookup(usart_port_t usart)
{
	switch (usart) {
		case USART_PORT_1:
			return USART1;

		case USART_PORT_2:
			return USART2;

		case USART_PORT_3:
			return USART3;
	}

	return NULL;
}

void usart_putc(usart_port_t usart, uint8_t c)
{
	USART_TypeDef *usart_ptr = usart_lookup(usart);

	while (!(usart_ptr->SR & USART_SR_TXE));
	usart_ptr->DR = c;
}

uint8_t usart_getc(usart_port_t usart)
{
	USART_TypeDef *usart_ptr = usart_lookup(usart);

	return (uint8_t) (usart_ptr->DR);
}

void usart_init(usart_port_t usart, uint32_t baudrate)
{
	USART_TypeDef *usart_ptr = usart_lookup(usart);

	// configure USART (115200/8N1)
	if (usart_ptr == USART1) {
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	} else if (usart_ptr == USART2) {
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	} else if (usart_ptr == USART3) {
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	}

	usart_ptr->BRR = (uint16_t) (32000000 / baudrate); // (16*HCLK)/(8*(2-OVER8)*baudrate)
	usart_ptr->CR1 = USART_CR1_UE | USART_CR1_TE;     // usart+transmitter enable
}

void usart_deinit(usart_port_t usart)
{
	USART_TypeDef *usart_ptr = usart_lookup(usart);

	usart_ptr->CR1 = 0;

	// disable USART
	if (usart_ptr == USART1) {
		RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
	} else if (usart_ptr == USART2) {
		RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
	} else if (usart_ptr == USART3) {
		RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
	}
}

void usart_enable_rx(usart_port_t usart, void (*irq_cb)(void))
{
	USART_TypeDef *usart_ptr = usart_lookup(usart);
	uint8_t irq_n;

	if (usart_ptr == USART1) {
		irq_n = USART1_IRQn;
		usart1_irq_cb = irq_cb;
	} else if (usart_ptr == USART2) {
		irq_n = USART2_IRQn;
		usart2_irq_cb = irq_cb;
	} else if (usart_ptr == USART3) {
		irq_n = USART3_IRQn;
		usart3_irq_cb = irq_cb;
	} else {
		return;
	}

	usart_ptr->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE; // receiver+int enable

	// configure the NVIC
	NVIC->IP[irq_n] = 0x70;			      // RX interrupt priority
	NVIC->ISER[irq_n >> 5] = 1 << (irq_n & 0x1F); // set enable IRQ
}

void usart_disable_rx(usart_port_t usart)
{
	USART_TypeDef *usart_ptr = usart_lookup(usart);
	uint8_t irq_n;

	if (usart_ptr == USART1) {
		irq_n = USART1_IRQn;
		usart1_irq_cb = NULL;
	} else if (usart_ptr == USART2) {
		irq_n = USART2_IRQn;
		usart2_irq_cb = NULL;
	} else if (usart_ptr == USART3) {
		irq_n = USART3_IRQn;
		usart3_irq_cb = NULL;
	} else {
		return;
	}

	usart_ptr->CR1 &= ~(USART_CR1_RE | USART_CR1_RXNEIE); // receiver+int disable
	NVIC->ISER[irq_n >> 5] = 0 << (irq_n & 0x1F);	 // set disable IRQ
}

void usart_sync(usart_port_t usart)
{
	USART_TypeDef *usart_ptr = usart_lookup(usart);

	if (usart_ptr->CR1 & USART_CR1_TE) {
		while (!(usart_ptr->SR & USART_SR_TC));
	}
}

uint8_t usart_is_rx_not_empty(usart_port_t usart)
{
	USART_TypeDef *usart_ptr = usart_lookup(usart);

	return ((usart_ptr->SR & USART_SR_RXNE) != 0);
}

void USART1_IRQHandler()
{
	if (usart1_irq_cb != NULL) {
		usart1_irq_cb();
	}
}

void USART2_IRQHandler()
{
	if (usart2_irq_cb != NULL) {
		usart2_irq_cb();
	}
}

void USART3_IRQHandler()
{
	if (usart3_irq_cb != NULL) {
		usart3_irq_cb();
	}
}
