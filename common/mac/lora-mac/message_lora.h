/*
 * message_lora - The LoRaWAN message layer
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

#ifndef _MESSAGE_LORA_H_
#define _MESSAGE_LORA_H_

#include <message_common.h>
#include <message_interface.h>

/*!
 * Maximum allowed gap for the FCNT field
 */
#define MAX_FCNT_GAP 	16384

#define MSG_LORA_MAX_FRAME_LEN 256

// MHDR + address + FCTRL + fcnt + fopts + fport

#define MSG_LORA_MHDR_LEN		(sizeof(uint8_t))
#define MSG_LORA_MIC_LEN		(sizeof(uint32_t))
//MAC Payload
#define MSG_LORA_ADDR_LEN		(sizeof(uint32_t))
#define MSG_LORA_FCTRL_LEN		(sizeof(uint8_t))
#define MSG_LORA_FCNT_LEN		(sizeof(uint16_t))
#define MSG_LORA_FPORT_LEN		(sizeof(uint8_t))
//Join Request
#define MSG_LORA_APPEUI_LEN		(sizeof(uint64_t))
#define MSG_LORA_DEVEUI_LEN		(sizeof(uint64_t))
#define MSG_LORA_DEVNONCE_LEN		(sizeof(uint16_t))
//Join Accept
#define MSG_LORA_APPNONCE_LEN		(3*sizeof(uint8_t))
#define MSG_LORA_NETID_LEN		(3*sizeof(uint8_t))
#define MSG_LORA_DLSETTINGS_LEN		(sizeof(uint8_t))
#define MSG_LORA_RXDELAY_LEN		(sizeof(uint8_t))
#define MSG_LORA_CFLIST_LEN		(16*sizeof(uint8_t))

#define MSG_LORA_MIN_MESSAGESIZE 	(MSG_LORA_MHDR_LEN + MSG_LORA_ADDR_LEN + MSG_LORA_FCTRL_LEN + MSG_LORA_FCNT_LEN + MSG_LORA_MIC_LEN)
#define MSG_LORA_REQ_MESSAGESIZE	(MSG_LORA_MHDR_LEN + MSG_LORA_APPEUI_LEN + MSG_LORA_DEVEUI_LEN + MSG_LORA_DEVNONCE_LEN + MSG_LORA_MIC_LEN)
#define MSG_LORA_ACC_MAX_MESSAGESIZE	(MSG_LORA_MHDR_LEN + MSG_LORA_APPNONCE_LEN + MSG_LORA_NETID_LEN + MSG_LORA_ADDR_LEN + MSG_LORA_DLSETTINGS_LEN + MSG_LORA_RXDELAY_LEN + MSG_LORA_CFLIST_LEN + MSG_LORA_MIC_LEN)

#define MSG_LORA_FHDR_MIN_LEN		(MSG_LORA_ADDR_LEN + MSG_LORA_FCTRL_LEN + MSG_LORA_FCNT_LEN)


enum {
	// Data frame format
	MSG_LORA_OFF_DAT_MHDR		= 0,
	MSG_LORA_OFF_DAT_FHDR_ADDR	= 1,
	MSG_LORA_OFF_DAT_FHDR_FCTRL	= 5,
	MSG_LORA_OFF_DAT_FHDR_FCNT	= 6,
	MSG_LORA_OFF_DAT_FHDR_FOPTS	= 8,
};

enum {
	// Data frame format
	MSG_LORA_OFF_REQ_MHDR		= 0,
	MSG_LORA_OFF_REQ_APPEUI		= 1,
	MSG_LORA_OFF_REQ_DEVEUI		= 9,
	MSG_LORA_OFF_REQ_DEVNONCE	= 17,
// 	MSG_LORA_OFF_REQ_MIC		= 19,
};

enum { MSG_LORA_MAX_PAYLOAD_LEN = MSG_LORA_MAX_FRAME_LEN - (int) MSG_LORA_OFF_DAT_FHDR_FOPTS - 4 };

enum { MSG_LORA_MHDR_MAJOR_V1 = 0x00,
};

typedef union {
	uint8_t value;
	struct {
		uint8_t major		: 2;
		uint8_t rfu		: 3;
		uint8_t mtype		: 3;
	} bits;
} message_lora_mhdr_t;


typedef union {
	uint8_t value;
	struct {
		uint8_t foptslen	: 4;
		uint8_t fpending	: 1;
		uint8_t ack		: 1;
		uint8_t adrackreq	: 1;
		uint8_t adr		: 1;
	} bits;
} message_lora_fctrl_t;

typedef struct {
	message_lora_mhdr_t mhdr;
	message_lora_fctrl_t fctrl;
	uint8_t fopts[16];
} message_lora_private_t;

extern message_interface_t message_lora;

#endif /* _MESSAGE_LORA_H_ */
