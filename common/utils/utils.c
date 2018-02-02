/*
 * utils - Various handy functions
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

#include "utils.h"

uint16_t utils_read16_le(const uint8_t *buf)
{
	return (uint16_t) (buf[0] | (buf[1] << 8));
}

uint32_t utils_read32_le(const uint8_t *buf)
{
	return (uint32_t) (buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
}

uint32_t utils_read32_be(const uint8_t *buf)
{
	return (uint32_t) (buf[3] | (buf[2] << 8) | (buf[1] << 16) | (buf[0] << 24));
}

void utils_write16_le(uint8_t *buf, uint16_t v)
{
	buf[0] = v;
	buf[1] = v >> 8;
}

void utils_write32_le(uint8_t *buf, uint32_t v)
{
	buf[0] = v;
	buf[1] = v >> 8;
	buf[2] = v >> 16;
	buf[3] = v >> 24;
}

void utils_write32_be(uint8_t *buf, uint32_t v)
{
	buf[0] = v >> 24;
	buf[1] = v >> 16;
	buf[2] = v >> 8;
	buf[3] = v;
}

void utils_memcpy_r(uint8_t *dst, const uint8_t *src, uint16_t size)
{
	src += size - 1;
	while (size--) {
		*(dst++) = *(src--);
	}
}
