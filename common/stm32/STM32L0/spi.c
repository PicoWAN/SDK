/*
 * spi - SPI driver
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
#include <spi.h>
#include <system.h>
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"

static uint32_t get_prescaler_for(uint32_t freq)
{
	uint32_t d = SystemCoreClock/freq;

	if (d > 128) {
		return SPI_BAUDRATEPRESCALER_256;
	} else if (d > 64) {
		return SPI_BAUDRATEPRESCALER_128;
	} else if (d > 32) {
		return SPI_BAUDRATEPRESCALER_64;
	} else if (d > 16) {
		return SPI_BAUDRATEPRESCALER_32;
	} else if (d > 8) {
		return SPI_BAUDRATEPRESCALER_16;
	} else if (d > 4) {
		return SPI_BAUDRATEPRESCALER_8;
	} else if (d > 2) {
		return SPI_BAUDRATEPRESCALER_4;
	} else {
		return SPI_BAUDRATEPRESCALER_2;
	}
}

void spi_init(void)
{
	SPI_HandleTypeDef hspi1;

	/* Be sure NSS is high before anything else */
	system_set_sx_nss(1);

	/* Enable SPI1 clock */
	__SPI1_CLK_ENABLE();

	/* Configure SPI1 (8-bit, 2-wire, no crc, MSBF, PCLK/2, CPOL0, CPHA0) */
	hspi1.Instance               = SPI1;
	hspi1.Init.Mode              = SPI_MODE_MASTER;
	hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
	hspi1.Init.NSS               = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = get_prescaler_for(10000000);
	hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode            = SPI_TIMODE_DISABLED;
	hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
	hspi1.Init.CRCPolynomial     = 7;

	HAL_SPI_Init(&hspi1);

	/* Enable SPI1 */
	__HAL_SPI_ENABLE(&hspi1);
}

uint8_t spi_rw(uint8_t data)
{
	SPI1->DR = data;
	while (!(SPI1->SR & SPI_SR_RXNE));
	return SPI1->DR;
}
