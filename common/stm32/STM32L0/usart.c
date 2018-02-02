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
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"

#define RX_BUFFER_SIZE				64 // in bytes

static void (*usart1_irq_cb)(void) = NULL;
static void (*usart2_irq_cb)(void) = NULL;
static void (*usart4_irq_cb)(void) = NULL;
static void (*usart5_irq_cb)(void) = NULL;
static void (*lpuart1_irq_cb)(void) = NULL;

/* Software FIFO using DMA for LPUART1 only */
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t rx_out_pos = 0;
static DMA_HandleTypeDef hdma_rx = {0};


static USART_TypeDef * usart_lookup(usart_port_t usart)
{
	switch (usart) {
		case USART_PORT_1:
			return USART1;

		case USART_PORT_2:
			return USART2;

		case USART_PORT_4:
			return USART4;

		case USART_PORT_5:
			return USART5;

		case LPUART_PORT_1:
			return LPUART1;
	}

	return NULL;
}

void usart_putc(usart_port_t usart, uint8_t c)
{
	USART_TypeDef *usart_ptr = usart_lookup(usart);

	while (!(usart_ptr->ISR & USART_FLAG_TXE));
	usart_ptr->TDR = c;
}

uint8_t usart_getc(usart_port_t usart)
{
	USART_TypeDef *usart_ptr = usart_lookup(usart);
	uint8_t c = 0;

	if (usart_ptr == LPUART1) {
		/* FIFO */
		if ((RX_BUFFER_SIZE - hdma_rx.Instance->CNDTR) != rx_out_pos) {
			c = rx_buffer[rx_out_pos];
			rx_out_pos = (rx_out_pos + 1) % RX_BUFFER_SIZE;
		}

		return c;
	} else {
		return (uint8_t) (usart_ptr->RDR);
	}
}

void usart_init(usart_port_t usart, uint32_t baudrate)
{
	USART_TypeDef *usart_ptr = usart_lookup(usart);
	UART_HandleTypeDef huart = {0};

	if (usart_ptr == USART1) {
		__HAL_RCC_USART1_CLK_ENABLE();
	} else if (usart_ptr == USART2) {
		__HAL_RCC_USART2_CLK_ENABLE();
	} else if (usart_ptr == USART4) {
		__HAL_RCC_USART4_CLK_ENABLE();
	} else if (usart_ptr == USART5) {
		__HAL_RCC_USART5_CLK_ENABLE();
	} else if (usart_ptr == LPUART1) {
		__HAL_RCC_LPUART1_CLK_ENABLE();
#if defined(CFG_CONSOLE_RX_ENABLED) && !defined(CFG_DISABLE_LOW_POWER_MODE)
		/* The maximum allowed baudrate for LPUART1 is 9600 bps
		 * when the LSE (32 kHz) is used as peripheral clock
		 */
		if (baudrate > 9600) {
			baudrate = 9600;
		}
#endif
	}

	huart.Instance	      = usart_ptr;
	huart.Init.BaudRate   = baudrate;
	huart.Init.WordLength = USART_WORDLENGTH_8B;
	huart.Init.StopBits   = USART_STOPBITS_1;
	huart.Init.Parity     = USART_PARITY_NONE;
	huart.Init.Mode	      = USART_MODE_TX;

	HAL_UART_Init(&huart);
}

void usart_deinit(usart_port_t usart)
{
	USART_TypeDef *usart_ptr = usart_lookup(usart);

	usart_ptr->CR1 = 0x0;
	usart_ptr->CR2 = 0x0;
	usart_ptr->CR3 = 0x0;

	if (usart_ptr == USART1) {
		__HAL_RCC_USART1_CLK_DISABLE();
	} else if (usart_ptr == USART2) {
		__HAL_RCC_USART2_CLK_DISABLE();
	} else if (usart_ptr == USART4) {
		__HAL_RCC_USART4_CLK_DISABLE();
	} else if (usart_ptr == USART5) {
		__HAL_RCC_USART5_CLK_DISABLE();
	} else if (usart_ptr == LPUART1) {
		__HAL_RCC_LPUART1_CLK_DISABLE();
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
	} else if (usart_ptr == USART4) {
		irq_n = USART4_5_IRQn;
		usart4_irq_cb = irq_cb;
	} else if (usart_ptr == USART5) {
		irq_n = USART4_5_IRQn;
		usart5_irq_cb = irq_cb;
	} else if (usart_ptr == LPUART1) {
		irq_n = LPUART1_IRQn;
		lpuart1_irq_cb = irq_cb;

		/* Enable DMA1 clock */
		__HAL_RCC_DMA1_CLK_ENABLE();

		/* Configure the DMA handler for Transmission process */
		hdma_rx.Instance                 = DMA1_Channel3;
		hdma_rx.Init.Request             = DMA_REQUEST_5;
		hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
		hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
		hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		hdma_rx.Init.Mode                = DMA_CIRCULAR;
		hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
		HAL_DMA_Init(&hdma_rx);

		/* NVIC configuration for DMA transfer */
		HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

		/* No DMA callback (RXNE will be used to trigger the data
		 * processing after each byte received, or after a wakeup)
		 */
		hdma_rx.XferCpltCallback = NULL;
		hdma_rx.XferHalfCpltCallback = NULL;
		hdma_rx.XferErrorCallback = NULL;

		/* Enable the DMA Stream */
		HAL_DMA_Start_IT(&hdma_rx, (uint32_t) &usart_ptr->RDR, (uint32_t) &rx_buffer, RX_BUFFER_SIZE);

		/* Enable the DMA transfer for the USART receiver */
		usart_ptr->CR3 |= USART_CR3_DMAR;
	} else {
		return;
	}

	usart_ptr->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE; // receiver+int enable

	// configure the NVIC
	HAL_NVIC_SetPriority(irq_n, 3, 0);
	HAL_NVIC_EnableIRQ(irq_n);
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
	} else if (usart_ptr == USART4) {
		irq_n = USART4_5_IRQn;
		usart4_irq_cb = NULL;
	} else if (usart_ptr == USART5) {
		irq_n = USART4_5_IRQn;
		usart5_irq_cb = NULL;
	} else if (usart_ptr == LPUART1) {
		irq_n = LPUART1_IRQn;
		lpuart1_irq_cb = NULL;

		/* Stop the DMA Stream and disable the DMA */
		usart_ptr->CR3 &= ~USART_CR3_DMAR;
		HAL_DMA_Abort(&hdma_rx);
		HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
		HAL_DMA_DeInit(&hdma_rx);
		__HAL_RCC_DMA1_CLK_DISABLE();
	} else {
		return;
	}

	usart_ptr->CR1 &= ~(USART_CR1_RE | USART_CR1_RXNEIE); // receiver+int disable
	HAL_NVIC_DisableIRQ(irq_n);
}

uint8_t usart_is_rx_not_empty(usart_port_t usart)
{
	USART_TypeDef *usart_ptr = usart_lookup(usart);

	if (usart_ptr == LPUART1) {
		/* FIFO of RX_BUFFER_SIZE - 1 bytes maximum */
		return ((RX_BUFFER_SIZE - hdma_rx.Instance->CNDTR) != rx_out_pos);
	} else {
		return ((usart_ptr->ISR & USART_FLAG_RXNE) != 0);
	}
}

void usart_sync(usart_port_t usart)
{
	USART_TypeDef *usart_ptr = usart_lookup(usart);

	if (usart_ptr->CR1 & USART_CR1_TE) {
		while (!(usart_ptr->ISR & USART_FLAG_TC));
	}
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

void USART4_5_IRQHandler()
{
	if ((usart4_irq_cb != NULL) && (USART4->ISR & USART_FLAG_RXNE)) {
		usart4_irq_cb();
	}

	if ((usart5_irq_cb != NULL) && (USART5->ISR & USART_FLAG_RXNE)) {
		usart5_irq_cb();
	}
}

void lpuart1_irq_handler()
{
	if (lpuart1_irq_cb != NULL) {
		lpuart1_irq_cb();
	}
}

void DMA1_Channel2_3_IRQHandler()
{
	HAL_DMA_IRQHandler(&hdma_rx);
}
