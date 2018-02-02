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

#include <stdlib.h>
#include <message.h>
#include "lora_mac_internal.h"
#include "mac.h"
#include "message_lora.h"
#include "mic.h"
#include "osal.h"

uint16_t message_lora_max_payload_length(void);


/*!
 * Encryption aBlock and sBlock
 */
static uint8_t aBlock[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t sBlock[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static int message_lora_verify_mic(uint8_t *key, int len, uint8_t dir, message_t *msg)
{
	return (mic_compute(msg->raw, len, key, msg->devaddr, dir, msg->seqno) == osal_read32_be(msg->raw + len));
}

static void message_lora_append_mic(uint8_t *key, int len, uint8_t dir, message_t *msg)
{
	uint32_t mic;
	mic = mic_compute(msg->raw, len, key, msg->devaddr, dir, msg->seqno);
	osal_write32_be(msg->raw + len, mic);
}

static void message_lora_join_append_mic(uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t *mic)
{
	*mic = mic_compute_join(buffer, size, key);
	osal_write32_be(buffer + size, *mic);
}

static void message_lora_encrypt(const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t seqno,
				 uint8_t *enc_buffer)
{
	uint16_t i;
	uint8_t buffer_index = 0;
	uint16_t ctr = 1;

	osal_aes_setkey(key);

	aBlock[5] = dir;

	aBlock[6] = (address) &0xFF;
	aBlock[7] = (address >> 8) & 0xFF;
	aBlock[8] = (address >> 16) & 0xFF;
	aBlock[9] = (address >> 24) & 0xFF;

	aBlock[10] = (seqno) &0xFF;
	aBlock[11] = (seqno >> 8) & 0xFF;
	aBlock[12] = (seqno >> 16) & 0xFF;
	aBlock[13] = (seqno >> 24) & 0xFF;

	while (size >= 16) {
		aBlock[15] = ((ctr) &0xFF);
		ctr++;
		osal_aes_encrypt(aBlock, sBlock, 16);
		for (i = 0; i < 16; i++) {
			enc_buffer[buffer_index + i] = buffer[buffer_index + i] ^ sBlock[i];
		}
		size -= 16;
		buffer_index += 16;
	}

	if (size > 0) {
		aBlock[15] = ((ctr) &0xFF);
		osal_aes_encrypt(aBlock, sBlock, 16);
		for (i = 0; i < size; i++) {
			enc_buffer[buffer_index + i] = buffer[buffer_index + i] ^ sBlock[i];
		}
	}
}

static void message_lora_decrypt(const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t seqno,
				 uint8_t *dec_buffer)
{
	message_lora_encrypt(buffer, size, key, address, dir, seqno, dec_buffer);
}

/*!
 * Computes the LoRaMAC join frame decryption
 *
 * \param [IN]  buffer          - Data buffer
 * \param [IN]  size            - Data buffer size
 * \param [IN]  key             - AES key to be used
 * \param [OUT] decBuffer       - Decrypted buffer
 */
static void message_lora_join_decrypt(const uint8_t *buffer, uint16_t size, const uint8_t *key, uint8_t *decBuffer)
{
	osal_aes_setkey(key);

	osal_memcpy(decBuffer, buffer, size);

	osal_aes_encrypt(decBuffer, decBuffer, 16);

	// Check if optional CFList is included
	if (size >= 16) {
		osal_aes_encrypt(decBuffer + 16, decBuffer + 16, 16);
	}
}

static void message_lora_join_compute_session_keys(const uint8_t *key, const uint8_t *app_nonce, uint16_t dev_nonce, uint8_t *nwkskey,
						   uint8_t *appskey)
{
	uint8_t nonce[16] = {0};
	uint8_t i = 0;

	osal_aes_setkey(key);

	for (i = 0; i < 16; i++) {
		nonce[i] = 0;
	}

	nonce[0] = 0x01;
	osal_memcpy(nonce + 1, app_nonce, 6);
	osal_memcpy(nonce + 7, &dev_nonce, 2);
	osal_aes_encrypt(nonce, nonce, 16);
	osal_memcpy(nwkskey, nonce, 16);

	for (i = 0; i < 16; i++) {
		nonce[i] = 0;
	}

	nonce[0] = 0x02;
	osal_memcpy(nonce + 1, app_nonce, 6);
	osal_memcpy(nonce + 7, &dev_nonce, 2);
	osal_aes_encrypt(nonce, nonce, 16);
	osal_memcpy(appskey, nonce, 16);
}

static int read_header(message_lora_private_t *priv, message_t *msg)
{
	msg->type = priv->mhdr.bits.mtype;
	msg->version = priv->mhdr.bits.major;
	msg->ack = priv->fctrl.bits.ack;

	switch (priv->mhdr.bits.mtype) {
		case MSG_TYPE_JOIN_REQ:
		case MSG_TYPE_JOIN_ACCEPT:
		case MSG_TYPE_DATA_UNCONFIRMED_UP:
		case MSG_TYPE_DATA_UNCONFIRMED_DOWN:
		case MSG_TYPE_DATA_CONFIRMED_UP:
		case MSG_TYPE_DATA_CONFIRMED_DOWN:
		case MSG_TYPE_PROPRIETARY:
			break;

		case MSG_TYPE_RFU:
			osal_printf("lora read_header rfu must not be used\n");
			return -1;

		default:
			osal_printf("lora read_header unkown type=%02X\n", priv->mhdr.bits.mtype);
			return -1;
	}
	return 0;
}

static int build_header(uint8_t type, uint8_t version, message_lora_private_t *priv)
{
	priv->mhdr.bits.major = version;
	priv->mhdr.bits.rfu = 0;
	priv->fctrl.value = 0;
	priv->mhdr.bits.mtype = type;

	switch (type) {
		case MSG_TYPE_JOIN_REQ:
		case MSG_TYPE_JOIN_ACCEPT:
		case MSG_TYPE_DATA_UNCONFIRMED_UP:
		case MSG_TYPE_DATA_UNCONFIRMED_DOWN:
		case MSG_TYPE_DATA_CONFIRMED_UP:
		case MSG_TYPE_DATA_CONFIRMED_DOWN:
		case MSG_TYPE_PROPRIETARY:
			break;

		case MSG_TYPE_RFU:
			osal_printf("lora build_header rfu must not be used\n");
			return -1;

		default:
			osal_printf("lora read_header unkown type=%02X\n", type);
			return -1;
	}
	return 0;
}

/**
  * @brief  compose a message according to some inputs
  * @param  appskey: Pointer to the application session AES128 key array ( 16 bytes )
  * @param  nwkskey: Pointer to the network session AES128 key array ( 16 bytes )
  * @param  netid: 24 bits network identifier ( provided by network operator )
  * @param  type: message type as defined in message_common.h
  * @param  devaddr: 32 bits device address on the network (must be unique to the network)
  * @param  port: payload port (0 for mac specific message else must be > 0 for application data)
  * @param  payload: A pointer to the data to send
  * @param  payload_len: The length of the data to send
  * @param  seqno: frame counter
  * @param  msg:  A pointer to composed message
  * @return message_status
  */
message_status_t message_lora_compose(uint8_t *appskey, uint8_t *nwkskey, uint32_t netid, uint8_t type, uint32_t devaddr, uint8_t port,
				      const uint8_t *payload, uint8_t payload_len, uint8_t *fopts, uint8_t fopts_len, uint16_t seqno, message_t *msg)
{
	uint8_t *d;
	uint8_t i;
	message_lora_private_t private_data;
	uint8_t direction;
	uint8_t *encode_key;
	uint16_t max_lora_message_length = message_lora_max_payload_length();

	msg->valid = 0;

	if ((fopts_len + payload_len) > max_lora_message_length) {
		osal_printf("message_lorawan_compose bad len=%u\n", fopts_len + payload_len);
		return MSG_STATUS_LENGTH_ERROR;
	}
	if (build_header(type, MSG_LORA_MHDR_MAJOR_V1, &private_data) < 0) {
		return MSG_STATUS_HEADER_ERROR;
	}
	private_data.fctrl.bits.ack = (msg->ack == 1) ? 1 : 0;
	private_data.fctrl.bits.adr = (msg->adr == 1) ? 1 : 0;
	private_data.fctrl.bits.adrackreq = (msg->adr_req == 1) ? 1 : 0;

	msg->format = MSG_FORMAT_LORA;
	msg->version = MSG_LORA_MHDR_MAJOR_V1;
	msg->type = type;
	msg->port = port;
	msg->network_id = netid;
	msg->devaddr = devaddr;
	msg->seqno = seqno;
	msg->payload_len = payload_len;
	msg->fopts_len = fopts_len;
	direction = message_get_direction(type);

	d = msg->raw;
	d[MSG_LORA_OFF_DAT_MHDR] = private_data.mhdr.value;

	osal_write32_le(d + MSG_LORA_OFF_DAT_FHDR_ADDR, devaddr);

	// Integration of MAC Commands
	if (fopts_len != 0) {
		private_data.fctrl.bits.foptslen = fopts_len;
		msg->fopts = msg->raw + MSG_LORA_OFF_DAT_FHDR_FOPTS;
		osal_memcpy((void *) msg->fopts, (void *) fopts, fopts_len);
	}

	d[MSG_LORA_OFF_DAT_FHDR_FCTRL] = private_data.fctrl.value;
	osal_write16_le(d + MSG_LORA_OFF_DAT_FHDR_FCNT, seqno);

	d[MSG_LORA_OFF_DAT_FHDR_FOPTS + fopts_len] = port;

	msg->payload = msg->raw + MSG_LORA_OFF_DAT_FHDR_FOPTS + fopts_len + MSG_LORA_FPORT_LEN;

	osal_memcpy((void *) msg->payload, (void *) payload, payload_len);

	msg->msg_len = msg->payload_len + MSG_LORA_MIN_MESSAGESIZE + fopts_len + MSG_LORA_FPORT_LEN;

	osal_printf("Raw Len  = 0x%02X, ", msg->msg_len - MSG_LORA_MIC_LEN);
	for (i = 0; i < (msg->msg_len - MSG_LORA_MIC_LEN); i++) {
		osal_printf("%02X ", msg->raw[i]);
	}
	osal_printf("\n");

	if (msg->port == MAC_PORT_MAC_COMMAND) {
		encode_key = nwkskey;
	} else {
		encode_key = appskey;
	}

	message_lora_encrypt(payload, payload_len, (const uint8_t *) encode_key, devaddr, direction, seqno, msg->payload);
	message_lora_append_mic(nwkskey, msg->msg_len - MSG_LORA_MIC_LEN, direction, msg);

	osal_printf("encrypted raw Len  = 0x%02X, ", msg->msg_len);
	for (i = 0; i < msg->msg_len; i++) {
		osal_printf("%02X ", msg->raw[i]);
	}
	osal_printf("\n");
	msg->valid = 1;
	return MSG_STATUS_NO_ERROR;
}


/**
 * @brief  compose a special message for OTA
 * @param  type: message type
 * @param  dev_eui: pointer to the device EUI array (8 bytes)
 * @param  app_eui: pointer to the application EUI array (8 bytes)
 * @param  app_key: pointer to the application AES128 key array (16 bytes)
 * @return message_status
 */
message_status_t message_lora_compose_join_OTA(uint8_t type, uint8_t *dev_eui, uint8_t *app_eui, uint16_t *dev_nonce, uint8_t *app_key,
					       message_t *msg)
{
	uint8_t *d;
	message_lora_private_t private_data;
	// uint8_t direction;
	uint8_t i = 0;
	uint32_t mic = 0;

	msg->valid = 0;

	if (build_header(type, MSG_LORA_MHDR_MAJOR_V1, &private_data) < 0) {
		return MSG_STATUS_HEADER_ERROR;
	}

	msg->format = MSG_FORMAT_LORA;
	msg->version = MSG_LORA_MHDR_MAJOR_V1;
	msg->type = type;

	// direction = message_get_direction(type);

	d = msg->raw;

	d[MSG_LORA_OFF_REQ_MHDR] = private_data.mhdr.value;

	osal_memcpy((void *) (d + MSG_LORA_OFF_REQ_APPEUI), (void *) app_eui, 8);
	osal_memcpy((void *) (d + MSG_LORA_OFF_REQ_DEVEUI), (void *) dev_eui, 8);
	*dev_nonce = osal_rand16();

	osal_memcpy((void *) (d + MSG_LORA_OFF_REQ_DEVNONCE), dev_nonce, 2);

	msg->msg_len = MSG_LORA_REQ_MESSAGESIZE;

	message_lora_join_append_mic(d, msg->msg_len - MSG_LORA_MIC_LEN, app_key, &mic);

	osal_printf("Join REQ Frame\nraw Len  = %d, ", msg->msg_len);
	for (i = 0; i < msg->msg_len; i++) {
		osal_printf("%02X ", msg->raw[i]);
	}
	osal_printf("\n");
	msg->valid = 1;
	return MSG_STATUS_NO_ERROR;
}


/**
  * @brief  decode a message according to format choosen
  * @param  appskey: Pointer to the application session AES128 key array ( 16 bytes )
  * @param  nwkskey: Pointer to the network session AES128 key array ( 16 bytes )
  * @param  buf: A pointer to the data to decode
  * @param  buf_len: The length of the data to decode
  * @param  msg:  A pointer to decoded message
  * @return message_status
  */
message_status_t message_lora_decode(uint8_t *appskey, uint8_t *nwkskey, uint8_t *buf, uint8_t buf_len, message_t *msg)
{
	uint8_t *d;
	uint8_t i;
	message_lora_private_t private_data;
	uint8_t *decode_key;
	uint8_t direction;
	int payload_start = 0;

	msg->format = MSG_FORMAT_LORA;
	msg->valid = 0;

	if (buf_len > MSG_LORA_MAX_FRAME_LEN || buf_len < MSG_LORA_MIN_MESSAGESIZE) {
		osal_printf("message_lora_decode bad len\n");
		return MSG_STATUS_LENGTH_ERROR;
	}

	private_data.mhdr.value = buf[MSG_LORA_OFF_DAT_MHDR];
	private_data.fctrl.value = buf[MSG_LORA_OFF_DAT_FHDR_FCTRL];

	if (read_header(&private_data, msg) < 0) {
		return MSG_STATUS_HEADER_ERROR;
	}

	osal_memcpy((void *) &msg->raw, (void *) buf, buf_len);
	d = msg->raw;

	msg->devaddr = osal_read32_le(d + MSG_LORA_OFF_DAT_FHDR_ADDR);

	msg->seqno = osal_read16_le(d + MSG_LORA_OFF_DAT_FHDR_FCNT);

	msg->fopts_len = private_data.fctrl.bits.foptslen;
	if (private_data.fctrl.bits.foptslen != 0) {
		msg->fopts = d + MSG_LORA_OFF_DAT_FHDR_FOPTS;
	}
	msg->port = d[MSG_LORA_OFF_DAT_FHDR_FOPTS + private_data.fctrl.bits.foptslen];
	payload_start = MSG_LORA_OFF_DAT_FHDR_FOPTS + private_data.fctrl.bits.foptslen + MSG_LORA_FPORT_LEN;

	msg->payload = d + payload_start;
	msg->payload_len = buf_len - MSG_LORA_MHDR_LEN - MSG_LORA_FHDR_MIN_LEN - MSG_LORA_MIC_LEN - private_data.fctrl.bits.foptslen;
	if (msg->payload_len > MSG_LORA_FPORT_LEN) { // When payload is empty, FPORT is empty too
		msg->payload_len -= MSG_LORA_FPORT_LEN;
	}

	direction = message_get_direction(msg->type);

	if (message_lora_verify_mic(nwkskey, buf_len - MSG_LORA_MIC_LEN, direction, msg) == 0) {
		osal_printf("message_lora_decode mic failed\n");
		return MSG_STATUS_MIC_ERROR;
	}
	if (msg->port == MAC_PORT_MAC_COMMAND) {
		decode_key = nwkskey;
	} else {
		decode_key = appskey;
	}
	message_lora_decrypt(&buf[payload_start], msg->payload_len, decode_key, msg->devaddr, direction, msg->seqno, msg->payload);

	osal_printf("payload Len  = 0x%02X, ", msg->payload_len);
	for (i = 0; i < msg->payload_len; i++) {
		osal_printf("%02X ", msg->payload[i]);
	}
	osal_printf("\n");
	msg->msg_len = buf_len;
	msg->valid = 1;
	return MSG_STATUS_NO_ERROR;
}

/**
 * @brief  decode a message from OTA
 * @param  buf: A pointer to the data to decode
 * @param  buf_len: The length of the data to decode
 * @param  app_key: pointer to the application AES128 key array (16 bytes)
 * @param  appskey: Pointer to the application session AES128 key array ( 16 bytes )
 * @param  nwkskey: Pointer to the network session AES128 key array ( 16 bytes )
 * @param  dev_nonce: ramdom nonce to create session keys
 * @param  netid: 24 bits network identifier ( provided by network operator )
 * @param  devaddr: 32 bits device address on the network (must be unique to the network)
 * @param  rx1_dr_offset: datarate offset between uplink and downlink on first window
 * @param  rx2_dr: datarate on the second window
 * @param  rx_delay: delay to wait before starting the first windows
 * @param  cf_list: channels available
 * @return message_status
 */
message_status_t message_lora_decode_join_OTA(uint8_t *buf, uint8_t buf_len, uint8_t *app_key, uint8_t *appskey, uint8_t *nwkskey,
					      uint16_t *dev_nonce, uint32_t *netid, uint32_t *devaddr, uint8_t *rx1_dr_offset, uint8_t *rx2_dr,
					      uint8_t *rx_delay, uint8_t *cf_list)
{
	uint8_t buf_temp[MSG_LORA_ACC_MAX_MESSAGESIZE];
	uint32_t mic = 0;
	uint32_t mic_rx = 0;
	uint8_t i = 3;

	message_lora_join_decrypt(buf + 1, buf_len - 1, app_key, buf_temp + 1); // Decryption without MHDR
	buf_temp[0] = buf[0];							// Add MHDR

	mic = mic_compute_join(buf_temp, buf_len - MSG_LORA_MIC_LEN, app_key);

	mic_rx = osal_read32_be(buf_temp + buf_len - MSG_LORA_MIC_LEN);

	if (mic_rx == mic) {
		osal_printf("Join accept is correct\n");

		message_lora_join_compute_session_keys(app_key, buf_temp + 1, *dev_nonce, nwkskey, appskey);

		*netid = (uint32_t) buf_temp[4];
		*netid |= (uint32_t) buf_temp[5] << 8;
		*netid |= (uint32_t) buf_temp[6] << 16;

		*devaddr = osal_read32_le(buf_temp + 7);

		// DLSettings
		*rx1_dr_offset = (buf_temp[11] >> 4) & 0x07;
		*rx2_dr = buf_temp[11] & 0x0F;

		// RX delay
		*rx_delay = buf_temp[12] & 0x0F;

		// CFList
		if ((buf_len - 1) > 16) {
			for (i = 13; i < buf_len - MSG_LORA_MIC_LEN; i++) {
				cf_list[i - 13] = buf_temp[i];
			}
		}
		return MSG_STATUS_NO_ERROR;
	}
	return MSG_STATUS_MIC_ERROR;
}
/**
  * @brief  Return maximun allowed bytes for user payload
  * @return max payload length
  */
uint16_t message_lora_max_payload_length(void)
{
	return lora_mac_max_message_length() - MSG_LORA_MIN_MESSAGESIZE;
}

/**
 * @brief  Get the address of the frame
 * @param  buf: A pointer to the data to decode
 * @param  buf_len: The length of the data to decode
 * @param  msg:  A pointer to save address message
 * @return message_status
 */
message_status_t message_lora_get_address(uint8_t *buf, uint8_t buf_len, message_t *msg)
{
	if (buf_len > MSG_LORA_MAX_FRAME_LEN || buf_len < MSG_LORA_MIN_MESSAGESIZE) {
		osal_printf("message_lora_get_address bad len\n");
		return MSG_STATUS_LENGTH_ERROR;
	}
	msg->devaddr = osal_read32_le(buf + MSG_LORA_OFF_DAT_FHDR_ADDR);
	return MSG_STATUS_NO_ERROR;
}

/**
 * @brief  Get frame header
 * @param  buf: A pointer to the data to decode
 * @param  buf_len: The length of the data to decode
 * @param  msg:  A pointer to save header message
 * @return message_status
 */
message_status_t message_lora_get_header(uint8_t *buf, uint8_t buf_len, message_t *msg)
{
	message_lora_private_t priv_temp;
	if (buf_len > MSG_LORA_MAX_FRAME_LEN || buf_len < MSG_LORA_MIN_MESSAGESIZE) {
		osal_printf("message_lora_get_header bad len\n");
		return MSG_STATUS_LENGTH_ERROR;
	}

	msg->format = MSG_FORMAT_LORA;
	priv_temp.mhdr.value = buf[MSG_LORA_OFF_DAT_MHDR];
	read_header(&priv_temp, msg);
	return MSG_STATUS_NO_ERROR;
}

message_interface_t message_lora = {
	.message_compose = &message_lora_compose,
	.message_compose_join_OTA = &message_lora_compose_join_OTA,
	.message_decode = &message_lora_decode,
	.message_decode_join_OTA = &message_lora_decode_join_OTA,
	.message_authentication = NULL, // not yet implemented
	.message_max_payload_length = &message_lora_max_payload_length,
	.message_get_address = &message_lora_get_address,
	.message_get_header = &message_lora_get_header,
};
