/*
 * message_common - The message layer private common code
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

#ifndef _MESSAGE__COMMON_H_
#define _MESSAGE__COMMON_H_

#include <stdint.h>

#define MSG_MAX_FRAME_LEN	64

typedef enum {
	MSG_FORMAT_EASY		= 0,
	MSG_FORMAT_LORA		= 1,
	MSG_FORMAT_PICOWAN	= 2,
} message_format_t;

enum {
	MSG_TYPE_JOIN_REQ		= 0x00,
	MSG_TYPE_JOIN_ACCEPT		= 0x01,
	MSG_TYPE_DATA_UNCONFIRMED_UP	= 0x02,
	MSG_TYPE_DATA_UNCONFIRMED_DOWN	= 0x03,
	MSG_TYPE_DATA_CONFIRMED_UP	= 0x04,
	MSG_TYPE_DATA_CONFIRMED_DOWN	= 0x05,
	MSG_TYPE_RFU			= 0x06,
	MSG_TYPE_PROPRIETARY		= 0x07,
};

enum {
	MSG_UPLINK   = 0,
	MSG_DOWNLINK = 1
};

typedef struct {
	uint8_t valid;

	/*
	 * message type:
	 * 0 for easy format
	 * 1 for lora format
	 * 2 for picowan format
	 */
	uint8_t format;

	/*
	 * format version:
	 */
	uint8_t version;

	/*
	 * message type:
	 */
	uint8_t type;

	/*
	 * port:
	 * 0 is for mac commands else it is application port
	 */
	uint8_t port;

	/*
	 * network_id :
	 */
	uint32_t network_id;

	/*
	 * devaddr :
	 * accessory network address
	 * Source for messages from accessory
	 * Destination for messages from Gateway
	 * Optional. Depends on type
	 */
	uint32_t devaddr;

	/*
	 * seqno :
	 * Sequence number (framecounter)
	 */
	uint16_t seqno;

	/*
	 * acknowledge bit:
	 */
	uint8_t ack;

	/*
	 * adaptive data rate bit:
	 */
	uint8_t adr;

	/*
	 * ADR acknowledgement request bit
	 */
	uint8_t adr_req;

	/*
	 * beacon_seed:
	 */
	uint8_t beacon_seed;

	/*
	 * message length:
	 * length of the payload + headers + MIC
	 */
	uint8_t msg_len;

	/*
	 * Payload length:
	 * length of the payload to follow in bytes _NOT_ including MIC
	 */
	uint8_t payload_len;

	/*
	 * Payload:
	 * uint8_t* to the data which were received
	 */
	uint8_t *payload;

	/*
	 * Fopts length:
	 * length of MAC commands
	 */
	uint8_t fopts_len;

	/*
	 * Fopts: contain MAC commands
	 * uint8_t* to commands which were received
	 */
	uint8_t *fopts;

	/*
	 * Raw data
	 */
	uint8_t raw[MSG_MAX_FRAME_LEN];
} message_t;


#endif
