/*
 * gpio - GPIO driver
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
#include <gpio.h>
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"

// GPIO by port number (A=0, B=1, ..)
#define GPIOx(no) ((GPIO_TypeDef *) (GPIOA_BASE + (no) * (GPIOB_BASE - GPIOA_BASE)))

#define GPIO_MODE             ((uint32_t)0x00000003)
#define EXTI_MODE             ((uint32_t)0x10000000)
#define GPIO_MODE_IT          ((uint32_t)0x00010000)
#define GPIO_MODE_EVT         ((uint32_t)0x00020000)
#define RISING_EDGE           ((uint32_t)0x00100000)
#define FALLING_EDGE          ((uint32_t)0x00200000)
#define GPIO_OUTPUT_TYPE      ((uint32_t)0x00000010)

#define GPIO_NUMBER           ((uint32_t)16)


void gpio_cfg_pin(gpio_port_t port, uint8_t pin, gpio_pin_mode_t mode, gpio_output_mode_t omode, gpio_pupd_t pull, gpio_output_speed_t speed, uint8_t alt)
{
	uint8_t m, pd, s;
	GPIO_TypeDef *p = GPIOx(port);
	GPIO_InitTypeDef GPIO_InitStruct;

	switch (mode) {
		case GPIO_PMODE_INPUT:
			m = GPIO_MODE_INPUT;
			break;
		case GPIO_PMODE_OUTPUT:
			m = GPIO_MODE_OUTPUT_PP;
			break;
		case GPIO_PMODE_ANALOG:
			m = GPIO_MODE_ANALOG;
			break;
		case GPIO_PMODE_ALT_FUNC:
			m = GPIO_MODE_AF_PP;
			break;
		default:
			return;
	}

	switch (omode) {
		case GPIO_OMODE_OPEN_DRAIN:
			m |= GPIO_OUTPUT_TYPE;
			break;
		case GPIO_OMODE_PUSH_PULL:
			break;
		default:
			return;
	}

	switch (pull) {
		case GPIO_PUPD_NONE:
			pd = GPIO_NOPULL;
			break;
		case GPIO_PUPD_UP:
			pd = GPIO_PULLUP;
			break;
		case GPIO_PUPD_DOWN:
			pd = GPIO_PULLDOWN;
			break;
		default:
			return;
	}

	switch (speed) {
		case GPIO_OSPEED_LOW:
			s = GPIO_SPEED_FREQ_LOW;
			break;
		case GPIO_OSPEED_MED:
			s = GPIO_SPEED_FREQ_MEDIUM;
			break;
		case GPIO_OSPEED_HIGH:
			s = GPIO_SPEED_FREQ_HIGH;
			break;
		case GPIO_OSPEED_VHIGH:
			s = GPIO_SPEED_FREQ_VERY_HIGH;
			break;
		default:
			return;
	}

	GPIO_InitStruct.Pin	  = (1 << pin);
	GPIO_InitStruct.Mode	  = m;
	GPIO_InitStruct.Pull	  = pd;
	GPIO_InitStruct.Speed	  = s;
 	GPIO_InitStruct.Alternate = alt;

	HAL_GPIO_Init(p, &GPIO_InitStruct);
}

void gpio_set_pin(gpio_port_t port, uint8_t pin, uint8_t state)
{
	GPIO_TypeDef *p = GPIOx(port);

	HAL_GPIO_WritePin(p, (1 << pin), state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void gpio_toggle_pin(gpio_port_t port, uint8_t pin)
{
	GPIO_TypeDef *p = GPIOx(port);

	HAL_GPIO_TogglePin(p, (1 << pin));
}

uint8_t gpio_get_pin(gpio_port_t port, uint8_t pin)
{
	GPIO_TypeDef *p = GPIOx(port);

	return HAL_GPIO_ReadPin(p, (1 << pin));
}

void gpio_config_irq(gpio_port_t port, uint8_t pin, gpio_irq_mode_t mode)
{
	uint32_t temp = 0x00;

	/* Enable SYSCFG Clock */
	__HAL_RCC_SYSCFG_CLK_ENABLE();

	/* Enable the requested port for the requested line */
	temp = SYSCFG->EXTICR[pin >> 2];
	temp &= ~(((uint32_t) 0xF) << ((pin & 0x3) << 2));
	temp |= ((uint32_t) (port) << ((pin & 0x3) << 2));
	SYSCFG->EXTICR[pin >> 2] = temp;

	/* Clear EXTI line configuration */
	temp = EXTI->IMR;
	temp &= ~((uint32_t) (1 << pin));
	if (mode != GPIO_IRQ_MODE_NONE) {
		temp |= (1 << pin);
	}
	EXTI->IMR = temp;

	temp = EXTI->EMR;
	temp &= ~((uint32_t) (1 << pin));
	EXTI->EMR = temp;

	/* Configure Rising/Falling edge trigger */
	temp = EXTI->RTSR;
	temp &= ~((uint32_t) (1 << pin));
	if (mode == GPIO_IRQ_MODE_RISING || mode == GPIO_IRQ_MODE_BOTH) {
		temp |= (1 << pin);
	}
	EXTI->RTSR = temp;

	temp = EXTI->FTSR;
	temp &= ~((uint32_t) (1 << pin));
	if (mode == GPIO_IRQ_MODE_FALLING || mode == GPIO_IRQ_MODE_BOTH) {
		temp |= (1 << pin);
	}
	EXTI->FTSR = temp;

	/* Configure NVIC (set priority and enable IRQ) */
	uint8_t exti = (pin < 2) ? EXTI0_1_IRQn : ((pin < 4) ? EXTI2_3_IRQn : EXTI4_15_IRQn);
	HAL_NVIC_SetPriority(exti, 15, 1);
	HAL_NVIC_EnableIRQ(exti);
}
