/*
 * message_picowan - The PicoWAN message layer
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

#ifndef _MESSAGE_PICOWAN_H_
#define _MESSAGE_PICOWAN_H_

#include <message_common.h>
#include <message_interface.h>


// MHDR + address + fcnt

#define MSG_PICOWAN_MHDR_LEN		(sizeof(uint8_t))
#define MSG_PICOWAN_ADDR_LEN		(sizeof(uint32_t))
#define MSG_PICOWAN_FCNT_LEN		(sizeof(uint8_t))

#define MSG_PICOWAN_MIC_LEN		(sizeof(uint32_t))

#define MSG_PICOWAN_HEADER_MIN_LEN	(MSG_PICOWAN_MHDR_LEN + MSG_PICOWAN_ADDR_LEN + MSG_PICOWAN_FCNT_LEN)

enum {
	// Data frame format
	MSG_PICOWAN_OFF_DAT_MHDR	= 0,
	MSG_PICOWAN_OFF_DAT_ADDR	= 1,
	MSG_PICOWAN_OFF_DAT_FCNT	= 5,
	MSG_PICOWAN_OFF_DAT_PAYLOAD	= 6
};


enum { MSG_PICOWAN_VERSION_V1 = 0x00,
};

typedef union {
	uint8_t value;
	struct {
		uint8_t direction	: 1;
		uint8_t ack		: 1;
		uint8_t mac		: 1;
		uint8_t join		: 1;
		uint8_t rfu		: 2;
		uint8_t version		: 2;
	} bits;
} message_picowan_mhdr_t;


typedef struct {
	message_picowan_mhdr_t mhdr;
} message_picowan_private_t;

extern message_interface_t message_picowan;

#endif /* _MESSAGE_PICOWAN_H_ */
