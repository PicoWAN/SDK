/*! \file
 * \brief ST LIS2DE12 accelerometer driver
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

#ifndef _LIS2DE12_H_
#define _LIS2DE12_H_

#include "i2c.h"

#define LIS2DE12_INT1			0x01
#define LIS2DE12_INT2			0x02

struct lis2de12_accel_data {
	int16_t x;
	int16_t y;
	int16_t z;
};


/*!
 * \brief   Checks if the accelerometer is enabled.
 *
 * \retval  uint8_t 1 if the accelerometer is enabled, 0 otherwise.
 */
uint8_t lis2de12_is_enabled(void);

/*!
 * \brief   Gets the raw acceleration values.
 *
 * \param   data A pointer to the structure that will hold the raw values.
 */
void lis2de12_get_accel(struct lis2de12_accel_data *data);

/*!
 * \brief   Handles the accelerometer IRQs.
 *
 * \details The function handles the accelerometer IRQs, and must be called to clear it on the accelerometer side.
 *
 * \param   pin The accelerometer pin that triggered the IRQ. This can be LIS2DE12_INT1 or LIS2DE12_INT2
 */
void lis2de12_handle_int(uint8_t pin);

/*!
 * \brief   Initializes the accelerometer driver.
 *
 * \param   i2c_port The I2C port to which the accelerometer is attached. This should be I2C_PORT_n where n is the I2C port number.
 *
 * \retval  uint8_t 0 if the initialization is successful, 1 otherwise.
 */
uint8_t lis2de12_init(uint8_t i2c_port);

/*!
 * \brief   Deinitializes the accelerometer driver.
 */
void lis2de12_deinit(void);

#endif /* _LIS2DE12_H_ */
