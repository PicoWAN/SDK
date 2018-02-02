/*
 * mic - Fonctions allowing to compute the MIC of a buffer
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

#include <stdint.h>
#include <string.h>
#include "cmac.h"
#include "osal.h"

#define MIC_BLOCK_B0_SIZE				16

static uint8_t mic_block_B0[MIC_BLOCK_B0_SIZE] = {0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static AES_CMAC_CTX aes_context;


uint32_t mic_compute(const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t seq)
{
	uint8_t mic[16];

	mic_block_B0[5] = dir;

	osal_write32_le(mic_block_B0 + 6, address);
	osal_write32_le(mic_block_B0 + 10, seq);

	mic_block_B0[15] = size & 0xFF;

	AES_CMAC_Init(&aes_context);
	AES_CMAC_SetKey(&aes_context, key);
	AES_CMAC_Update(&aes_context, mic_block_B0, MIC_BLOCK_B0_SIZE);
	AES_CMAC_Update(&aes_context, buffer, size & 0xFF);
	AES_CMAC_Final(mic, &aes_context);

	return (uint32_t) ((uint32_t) mic[0] << 24 | (uint32_t) mic[1] << 16 | (uint32_t) mic[2] << 8 | (uint32_t) mic[3]);
}

uint32_t mic_compute_join(const uint8_t *buffer, uint16_t size, const uint8_t *key)
{
	uint8_t mic[16];

	AES_CMAC_Init(&aes_context);
	AES_CMAC_SetKey(&aes_context, key);
	AES_CMAC_Update(&aes_context, buffer, size & 0xFF);
	AES_CMAC_Final(mic, &aes_context);

	return (uint32_t) ((uint32_t) mic[0] << 24 | (uint32_t) mic[1] << 16 | (uint32_t) mic[2] << 8 | (uint32_t) mic[3]);
}
