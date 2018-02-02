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
#include "stm32l1xx.h"

// GPIO by port number (A=0, B=1, ..)
#define GPIOx(no) ((GPIO_TypeDef *) (GPIOA_BASE + (no) * (GPIOB_BASE - GPIOA_BASE)))

#define ST_GPIO_MODE_IN			0x00
#define ST_GPIO_MODE_OUT		0x01
#define ST_GPIO_MODE_AF			0x02
#define ST_GPIO_MODE_AN			0x03

#define ST_GPIO_OTYPE_PP		0x00
#define ST_GPIO_OTYPE_OD		0x01

#define ST_GPIO_SPEED_400KHZ		0x00
#define ST_GPIO_SPEED_2MHZ		0x01
#define ST_GPIO_SPEED_10MHZ		0x02
#define ST_GPIO_SPEED_40MHZ		0x03

#define ST_GPIO_PUPD_NOPULL		0x00
#define ST_GPIO_PUPD_UP			0x01
#define ST_GPIO_PUPD_DOWN		0x02


void gpio_cfg_pin(gpio_port_t port, uint8_t pin, gpio_pin_mode_t mode, gpio_output_mode_t omode, gpio_pupd_t pull, gpio_output_speed_t speed, uint8_t alt)
{
	uint8_t m, t, pd, s;
	GPIO_TypeDef *p = GPIOx(port);

	switch (mode) {
		case GPIO_PMODE_INPUT:
			m = ST_GPIO_MODE_IN;
			break;
		case GPIO_PMODE_OUTPUT:
			m = ST_GPIO_MODE_OUT;
			break;
		case GPIO_PMODE_ANALOG:
			m = ST_GPIO_MODE_AN;
			break;
		case GPIO_PMODE_ALT_FUNC:
			m = ST_GPIO_MODE_AF;
			break;
		default:
			return;
	}

	switch (omode) {
		case GPIO_OMODE_OPEN_DRAIN:
			t = ST_GPIO_OTYPE_OD;
			break;
		case GPIO_OMODE_PUSH_PULL:
			t = ST_GPIO_OTYPE_PP;
			break;
		default:
			return;
	}

	switch (pull) {
		case GPIO_PUPD_NONE:
			pd = ST_GPIO_PUPD_NOPULL;
			break;
		case GPIO_PUPD_UP:
			pd = ST_GPIO_PUPD_UP;
			break;
		case GPIO_PUPD_DOWN:
			pd = ST_GPIO_PUPD_DOWN;
			break;
		default:
			return;
	}

	switch (speed) {
		case GPIO_OSPEED_LOW:
			s = ST_GPIO_SPEED_400KHZ;
			break;
		case GPIO_OSPEED_MED:
			s = ST_GPIO_SPEED_2MHZ;
			break;
		case GPIO_OSPEED_HIGH:
			s = ST_GPIO_SPEED_10MHZ;
			break;
		case GPIO_OSPEED_VHIGH:
			s = ST_GPIO_SPEED_40MHZ;
			break;
		default:
			return;
	}

	p->MODER = (p->MODER & ~(3 << (pin << 1))) | ((m & 0x3) << (pin << 1));
	p->OTYPER = (p->OTYPER & ~(1 << pin)) | ((t & 1) << pin);
	p->OSPEEDR = (p->OSPEEDR & ~(3 << (pin << 1))) | ((s & 0x3) << (pin << 1));
	p->PUPDR = (p->PUPDR & ~(3 << (pin << 1))) | ((pd & 0x3) << (pin << 1));

	p->AFR[pin >> 3] = (p->AFR[pin >> 3] & ~(0xF << ((pin & 0x7) << 2))) | (alt << ((pin & 0x7) << 2));
}

void gpio_set_pin(gpio_port_t port, uint8_t pin, uint8_t state)
{
	GPIO_TypeDef *p = GPIOx(port);

	p->ODR = (p->ODR & ~(1 << pin)) | ((state & 1) << pin);
}

void gpio_toggle_pin(gpio_port_t port, uint8_t pin)
{
	GPIO_TypeDef *p = GPIOx(port);

	p->ODR ^= (1 << pin);
}

uint8_t gpio_get_pin(gpio_port_t port, uint8_t pin)
{
	GPIO_TypeDef *p = GPIOx(port);

	return ((p->IDR >> pin) & 1);
}

void gpio_config_irq(gpio_port_t port, uint8_t pin, gpio_irq_mode_t mode)
{
	uint32_t temp;

	/* Turn ON the module */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

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
	uint8_t channel = (pin < 5) ? (EXTI0_IRQn + pin) : ((pin < 10) ? EXTI9_5_IRQn : EXTI15_10_IRQn);
	NVIC->IP[channel] = 0x70;
	NVIC->ISER[channel >> 5] = 1 << (channel & 0x1F);
}
