/*! \file
 * \brief Various handy functions
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

#ifndef _UTILS_H_
#define _UTILS_H_

#include <stdint.h>


/*!
 * \brief   Reads data as little-endian half-word.
 *
 * \param   buf pointer to the data to read.
 *
 * \retval  uint16_t read half-word.
 */
uint16_t utils_read16_le(const uint8_t *buf);

/*!
 * \brief   Reads data as little-endian word.
 *
 * \param   buf pointer to the data to read.
 *
 * \retval  uint32_t read word.
 */
uint32_t utils_read32_le(const uint8_t *buf);

/*!
 * \brief   Reads data as big-endian word.
 *
 * \param   buf pointer to the data to read.
 *
 * \retval  uint32_t read word.
 */
uint32_t utils_read32_be(const uint8_t *buf);

/*!
 * \brief   Writes data as little-endian half-word.
 *
 * \param   buf pointer to the memory area where the data will be written.
 * \param   v half-word to write.
 */
void utils_write16_le(uint8_t *buf, uint16_t v);

/*!
 * \brief   Writes data as little-endian word.
 *
 * \param   buf pointer to the memory area where the data will be written.
 * \param   v word to write.
 */
void utils_write32_le(uint8_t *buf, uint32_t v);

/*!
 * \brief   Writes data as big-endian word.
 *
 * \param   buf pointer to the memory area where the data will be written.
 * \param   v word to write.
 */
void utils_write32_be(uint8_t *buf, uint32_t v);

/*!
 * \brief   Copies a memory area in reverse order.
 *
 * \details The function copies a memory area in reverse order (last byte of the source area
 *          copied to the first byte of the destination area).
 *
 * \param   dst destination pointer.
 * \param   src source pointer.
 * \param   size size of the memory area to copy.
 */
void utils_memcpy_r(uint8_t *dst, const uint8_t *src, uint16_t size);

#endif /* _UTILS_H_ */
