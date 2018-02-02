/*! \file
 * \brief I2C driver
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

#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>
#include <arch.h>


/*!
 * \brief   Initializes an I2C port.
 *
 * \param   i2c_port The I2C port to initialize (I2C_PORT_n).
 * \param   speed The I2C speed in Hz.
 */
void i2c_init(i2c_port_t i2c_port, uint32_t speed);

/*!
 * \brief   De-initializes an I2C port.
 *
 * \param   i2c_port The I2C port (I2C_PORT_n).
 */
void i2c_deinit(i2c_port_t i2c_port);

/*!
 * \brief   Powers up an I2C port.
 *
 * \note    This should be done before initializing it.
 *
 * \param   i2c_port The I2C port (I2C_PORT_n).
 */
void i2c_power_up(i2c_port_t i2c_port);

/*!
 * \brief   Powers down an I2C port.
 *
 * \note    This should be done after de-initializing it.
 *
 * \param   i2c_port The I2C port (I2C_PORT_n).
 */
void i2c_power_down(i2c_port_t i2c_port);

/*!
 * \brief   Writes/sends I2C data to the given address.
 *
 * \param   i2c_port The I2C port (I2C_PORT_n).
 * \param   addr The address of the device to write to.
 * \param   data A pointer to the buffer holding the data to write.
 * \param   len The length of the data to write in bytes.
 */
void i2c_write(i2c_port_t i2c_port, uint8_t addr, uint8_t *data, uint8_t len);

/*!
 * \brief   Reads/receives I2C data from the given address.
 *
 * \param   i2c_port The I2C port (I2C_PORT_n).
 * \param   addr The address of the device to read from.
 * \param   data A pointer to the buffer that will hold the data read.
 * \param   len The length of the data to read in bytes.
 */
void i2c_read(i2c_port_t i2c_port, uint8_t addr, uint8_t *data, uint8_t len);


#endif /*_I2C_H_ */
