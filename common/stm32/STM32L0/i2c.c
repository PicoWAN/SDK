/*
 * i2c - I2C driver
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
#include <i2c.h>
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"

#define I2C_TIMING_100KHZ		0x10A13E56 /* 100 kHz with analog Filter ON, Rise Time 400ns, Fall Time 100ns */
#define I2C_TIMING_400KHZ		0x00B1112E /* 400 kHz with analog Filter ON, Rise Time 250ns, Fall Time 100ns */

static I2C_HandleTypeDef hi2c1 = {
	.Init =
		{
			.Timing = 0,
			.OwnAddress1 = 0x00,
			.AddressingMode = I2C_ADDRESSINGMODE_7BIT,
			.DualAddressMode = I2C_DUALADDRESS_DISABLE,
			.OwnAddress2 = 0x00,
			.OwnAddress2Masks = I2C_OA2_NOMASK,
			.GeneralCallMode = I2C_GENERALCALL_DISABLE,
			.NoStretchMode = I2C_NOSTRETCH_DISABLE,
		},
	.Instance = I2C1,
};

static I2C_HandleTypeDef hi2c2 = {
	.Init =
		{
			.Timing = 0,
			.OwnAddress1 = 0x00,
			.AddressingMode = I2C_ADDRESSINGMODE_7BIT,
			.DualAddressMode = I2C_DUALADDRESS_DISABLE,
			.OwnAddress2 = 0x00,
			.OwnAddress2Masks = I2C_OA2_NOMASK,
			.GeneralCallMode = I2C_GENERALCALL_DISABLE,
			.NoStretchMode = I2C_NOSTRETCH_DISABLE,
		},
	.Instance = I2C2,
};


static I2C_HandleTypeDef * i2c_lookup(i2c_port_t i2c_port)
{
	switch (i2c_port) {
		case I2C_PORT_1:
			return &hi2c1;

		case I2C_PORT_2:
			return &hi2c2;
	}

	/* Default is I2C1 */
	return &hi2c1;
}

void i2c_init(i2c_port_t i2c_port, uint32_t speed)
{
	I2C_HandleTypeDef *hi2c = i2c_lookup(i2c_port);

	switch (speed) {
		case 100000:
			hi2c->Init.Timing = I2C_TIMING_100KHZ;
			break;

		case 400000:
			hi2c->Init.Timing = I2C_TIMING_400KHZ;
			break;

		default:
			/* Unsupported speed */
			return;
	}

	HAL_I2C_Init(hi2c),

	/* Enable the Analog I2C Filter */
	HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE);
}

void i2c_deinit(i2c_port_t i2c_port)
{
	I2C_HandleTypeDef *hi2c = i2c_lookup(i2c_port);

	HAL_I2C_DeInit(hi2c);
}

void i2c_power_up(i2c_port_t i2c_port)
{
	switch (i2c_port) {
		case I2C_PORT_1:
			__HAL_RCC_I2C1_CLK_ENABLE();
			break;

		case I2C_PORT_2:
			__HAL_RCC_I2C2_CLK_ENABLE();
			break;
	}
}

void i2c_power_down(i2c_port_t i2c_port)
{
	switch (i2c_port) {
		case I2C_PORT_1:
			__HAL_RCC_I2C1_CLK_DISABLE();
			break;

		case I2C_PORT_2:
			__HAL_RCC_I2C2_CLK_DISABLE();
			break;
	}
}

void i2c_block_write(i2c_port_t i2c_port, uint8_t addr, uint8_t *data, uint8_t len)
{
	I2C_HandleTypeDef *hi2c = i2c_lookup(i2c_port);

	HAL_I2C_Master_Transmit(hi2c, addr, data, len, HAL_MAX_DELAY);
}

void i2c_block_read(i2c_port_t i2c_port, uint8_t addr, uint8_t *data, uint8_t len)
{
	I2C_HandleTypeDef *hi2c = i2c_lookup(i2c_port);

	HAL_I2C_Master_Receive(hi2c, addr, data, len, HAL_MAX_DELAY);
}
