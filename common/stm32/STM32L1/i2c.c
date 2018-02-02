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
#include "i2c-internal.h"
#include "stm32l1xx.h"

#define HELPER_I2C_FLAG_TIMEOUT		((uint32_t)0x1000)
#define HELPER_I2C_LONG_TIMEOUT		((uint32_t)0x10000)


static I2C_TypeDef * i2c_lookup(i2c_port_t i2c_port)
{
	switch (i2c_port) {
		case I2C_PORT_1:
			return I2C1;

		case I2C_PORT_2:
			return I2C2;
	}

	/* Default is I2C1 */
	return I2C1;
}

void i2c_init(i2c_port_t i2c_port, uint32_t speed)
{
	I2C_InitTypeDef I2C_InitStructure;
	I2C_TypeDef *i2c = i2c_lookup(i2c_port);

	I2C_DeInit(i2c);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = speed;

	I2C_Cmd(i2c, ENABLE);

	I2C_Init(i2c, &I2C_InitStructure);
}

void i2c_deinit(i2c_port_t i2c_port)
{
	I2C_TypeDef *i2c = i2c_lookup(i2c_port);

	I2C_Cmd(i2c, DISABLE);
	I2C_DeInit(i2c);
}

void i2c_power_up(i2c_port_t i2c_port)
{
	I2C_TypeDef *i2c = i2c_lookup(i2c_port);

	I2C_PowerUP(i2c);
}

void i2c_power_down(i2c_port_t i2c_port)
{
	I2C_TypeDef *i2c = i2c_lookup(i2c_port);

	I2C_PowerDOWN(i2c);
}

void i2c_write(i2c_port_t i2c_port, uint8_t addr, uint8_t *data, uint8_t len)
{
	uint32_t timeout;
	I2C_TypeDef *i2c = i2c_lookup(i2c_port);

	/* Test on BUSY Flag */
	timeout = HELPER_I2C_LONG_TIMEOUT;
	while (I2C_GetFlagStatus(i2c, I2C_FLAG_BUSY)) {
		if (timeout-- == 0) {
			return;
		}
	}

	/* Enable the I2C peripheral */
	I2C_GenerateSTART(i2c, ENABLE);

	timeout = HELPER_I2C_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (timeout-- == 0) {
			return;
		}
	}

	/* Send device address for write */
	I2C_Send7bitAddress(i2c, addr, I2C_Direction_Transmitter);

	/* Test on ADDR Flag */
	timeout = HELPER_I2C_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if (timeout-- == 0) {
			return;
		}
	}

	while (len--) {
		/* Send the data */
		I2C_SendData(i2c, *(data++));

		/* Test on TXE FLag (data sent) */
		timeout = HELPER_I2C_FLAG_TIMEOUT;
		while ((!I2C_GetFlagStatus(i2c, I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(i2c, I2C_FLAG_BTF))) {
			if (timeout-- == 0) {
				return;
			}
		}
	}

	/* Send STOP Condition */
	I2C_GenerateSTOP(i2c, ENABLE);
}

void i2c_read(i2c_port_t i2c_port, uint8_t addr, uint8_t *data, uint8_t len)
{
	uint32_t timeout;
	I2C_TypeDef *i2c = i2c_lookup(i2c_port);

	/* Test on BUSY Flag */
	timeout = HELPER_I2C_LONG_TIMEOUT;
	while (I2C_GetFlagStatus(i2c, I2C_FLAG_BUSY)) {
		if (timeout-- == 0) {
			return;
		}
	}

	/* Enable ACK */
	I2C_AcknowledgeConfig(i2c, ENABLE);

	/* Send START condition */
	I2C_GenerateSTART(i2c, ENABLE);

	timeout = HELPER_I2C_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (timeout-- == 0) {
			return;
		}
	}

	/* Send device address for read */
	I2C_Send7bitAddress(i2c, addr, I2C_Direction_Receiver);

	if (len == 1) {
		/* Acknoledge needs to be disabled now to be sure the
		 * sensor will read a NACK once the last byte read
		 */
		I2C_AcknowledgeConfig(i2c, DISABLE);
	}

	/* Test on ADDR Flag */
	timeout = HELPER_I2C_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
		if (timeout-- == 0) {
			return;
		}
	}

	while (len--) {
		if (len == 0) {
			/* Acknoledge needs to be disabled now (if not already)
			 * to be sure the sensor will read a NACK once the last
			 * byte read
			 */
			I2C_AcknowledgeConfig(i2c, DISABLE);
		}

		/* Test on TXE FLag (data sent) */
		timeout = HELPER_I2C_FLAG_TIMEOUT;
		while ((!I2C_GetFlagStatus(i2c, I2C_FLAG_RXNE)) && (!I2C_GetFlagStatus(i2c, I2C_FLAG_BTF))) {
			if (timeout-- == 0) {
				return;
			}
		}

		*(data++) = I2C_ReceiveData(i2c);
	}

	/* Send STOP Condition */
	I2C_GenerateSTOP(i2c, ENABLE);
}
