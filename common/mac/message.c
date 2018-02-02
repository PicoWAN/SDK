/*
 * message - The message layer public common code
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
#include "message.h"
#include "message_interface.h"
#include "mac.h"
#ifdef CFG_ENABLE_LORAMAC
#include <message_lora.h>
#endif
#ifdef CFG_ENABLE_PICOMAC
#include <message_picowan.h>
#endif
#include "osal.h"

message_format_t format = MSG_FORMAT_PICOWAN;
message_interface_t *current_if = NULL;

/*!
 * AES encryption/decryption cipher network session key
 */
static uint8_t aes_nwkskey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*!
 * AES encryption/decryption cipher application session key
 */
static uint8_t aes_appskey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint32_t network_id;
static uint32_t device_address;


/**
 * @brief  Initializes the frame's type.
 * @param  frame_format: type of the MAC
 */
void message_init_format(message_format_t frame_format)
{
	format = frame_format;
	switch (format) {
#ifdef CFG_ENABLE_LORAMAC
		case MSG_FORMAT_LORA:
			current_if = &message_lora;
			break;
#endif
#ifdef CFG_ENABLE_PICOMAC
		case MSG_FORMAT_PICOWAN:
			current_if = &message_picowan;
			break;
#endif
		default:
			current_if = NULL;
	}
}

/**
 * @brief  Initializes the session keys. Network session AES128 key and application session AES128 key.
 * @param  nwkskey: Pointer to the network session AES128 key array ( 16 bytes )
 * @param  appskey: Pointer to the application session AES128 key array ( 16 bytes )
 */
void message_init_session_keys(uint8_t *nwkskey, uint8_t *appskey)
{
	osal_memcpy(aes_nwkskey, nwkskey, 16);
	osal_memcpy(aes_appskey, appskey, 16);
}

/**
  * @brief  Initializes the network IDs. Device address, network session AES128 key and application session AES128 key.
  * @param  netid: 24 bits network identifier ( provided by network operator )
  * @param  devaddr: 32 bits device address on the network (must be unique to the network)
  * @param  nwkskey: Pointer to the network session AES128 key array ( 16 bytes )
  * @param  appskey: Pointer to the application session AES128 key array ( 16 bytes )
  */
void message_init_ids(uint32_t netid, uint32_t devaddr, uint8_t *nwkskey, uint8_t *appskey)
{
	network_id = netid;
	device_address = devaddr;
	message_init_session_keys(nwkskey, appskey);
}


/**
  * @brief  compose a message according to some inputs
  * @param  type: message type
  * @param  port: payload port (0 for mac specific message else must be > 0 for application data)
  * @param  payload: A pointer to the data to send
  * @param  payload_len: The length of the data to send
  * @param  seqno: frame counter
  * @param  msg:  A pointer to composed message
  * @return message_status
  */
message_status_t message_compose(uint8_t type, uint8_t port, const uint8_t *payload, uint8_t payload_len, uint8_t *fopts, uint8_t fopts_len,
				 uint16_t seqno, message_t *msg)
{
	uint8_t *encode_key;

	if (port == MAC_PORT_MAC_COMMAND) {
		encode_key = aes_nwkskey;
	} else {
		encode_key = aes_appskey;
	}

	if (current_if != NULL && current_if->message_compose != NULL) {
		return current_if->message_compose(encode_key, aes_nwkskey, network_id, type, device_address, port, payload, payload_len, fopts,
					           fopts_len, seqno, msg);
	} else {
		osal_printf("message_compose unsupported format:%02X\n", format);
		return MSG_STATUS_FORMAT_ERROR;
	}
}


/**
 * @brief  compose a special message for OTA
 * @param  type: message type
 * @param  dev_eui: pointer to the device EUI array (8 bytes)
 * @param  app_eui: pointer to the application EUI array (8 bytes)
 * @param  app_key: pointer to the application AES128 key array (16 bytes)
 * @return message_status
 */
message_status_t message_compose_join_OTA(uint8_t type, uint8_t *dev_eui, uint8_t *app_eui, uint16_t *dev_nonce, uint8_t *app_key, message_t *msg)
{
	if (current_if != NULL && current_if->message_compose_join_OTA != NULL) {
		return current_if->message_compose_join_OTA(type, dev_eui, app_eui, dev_nonce, app_key, msg);
	} else {
		osal_printf("message_compose_join_OTA unsupported format:%02X\n", format);
		return MSG_STATUS_FORMAT_ERROR;
	}
}


/**
  * @brief  decode a message according to format choosen
  * @param  buf: A pointer to the data to decode
  * @param  buf_len: The length of the data to decode
  * @param  msg:  A pointer to decoded message
  * @return message_status
  */
message_status_t message_decode(uint8_t *buf, uint8_t buf_len, message_t *msg)
{
	if (current_if != NULL && current_if->message_decode != NULL) {
		return current_if->message_decode(aes_appskey, aes_nwkskey, buf, buf_len, msg);
	} else {
		osal_printf("message_decode unsupported format:%02X\n", format);
		return MSG_STATUS_FORMAT_ERROR;
	}
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
message_status_t message_decode_join_OTA(uint8_t *buf, uint8_t buf_len, uint8_t *app_key, uint8_t *appskey, uint8_t *nwkskey, uint16_t *dev_nonce,
					 uint32_t *netid, uint32_t *devaddr, uint8_t *rx1_dr_offset, uint8_t *rx2_dr, uint8_t *rx_delay,
					 uint8_t *cf_list)
{
	if (current_if != NULL && current_if->message_decode_join_OTA != NULL) {
		return current_if->message_decode_join_OTA(buf, buf_len, app_key, appskey, nwkskey, dev_nonce, netid, devaddr, rx1_dr_offset, rx2_dr,
							   rx_delay, cf_list);
	} else {
		osal_printf("message_decode_join_OTA unsupported format:%02X\n", format);
		return MSG_STATUS_FORMAT_ERROR;
	}
}


/**
  * @brief  check if message type needs an acknowledgement
  * @param  msg:  A pointer to a given message
  * @return 0 ack_request enum, NO_ACK in case of error
  */
enum ack_request message_ack_request(message_t *msg)
{
	if (msg->valid != 0) {
		switch (msg->type) {
			case MSG_TYPE_DATA_CONFIRMED_UP:
			case MSG_TYPE_DATA_CONFIRMED_DOWN:
			case MSG_TYPE_JOIN_REQ:
				return WITH_ACK;

			default:
				return WITHOUT_ACK;
		}
	}
	return WITHOUT_ACK;
}


message_status_t message_authentication(message_t *msg)
{
	if (current_if != NULL && current_if->message_authentication != NULL) {
		return current_if->message_authentication(aes_nwkskey, msg);
	} else {
		return MSG_STATUS_FORMAT_ERROR;
	}
}

/**
  * @brief  return message direction
  * @param  type:  message type
  * @return 1 if downlink, 0 if uplink or error
  */
uint8_t message_get_direction(uint8_t type)
{
	switch (type) {

		case MSG_TYPE_PROPRIETARY:
			return MSG_DOWNLINK; // TODO must check payload to know direction

		case MSG_TYPE_JOIN_ACCEPT:
		case MSG_TYPE_DATA_UNCONFIRMED_DOWN:
		case MSG_TYPE_DATA_CONFIRMED_DOWN:
			return MSG_DOWNLINK;

		case MSG_TYPE_JOIN_REQ:
		case MSG_TYPE_DATA_UNCONFIRMED_UP:
		case MSG_TYPE_DATA_CONFIRMED_UP:
		case MSG_TYPE_RFU:
		default:
			return MSG_UPLINK;
	}
	return MSG_UPLINK;
}

/**
  * @brief  Return maximun allowed bytes for user payload
  * @return max payload length or 0
  */
uint16_t message_max_payload_length()
{
	if (current_if != NULL && current_if->message_max_payload_length != NULL) {
		return current_if->message_max_payload_length();
	} else {
		return 0;
	}
}

/**
 * @brief  Get the address of the frame
 * @param  buf: A pointer to the data to decode
 * @param  buf_len: The length of the data to decode
 * @param  msg:  A pointer to save address message
 * @return message_status
 */
message_status_t message_get_address(uint8_t *buf, uint8_t buf_len, message_t *msg)
{
	if (current_if != NULL && current_if->message_get_address != NULL) {
		return current_if->message_get_address(buf, buf_len, msg);
	} else {
		osal_printf("message_get_address unsupported format:%02X\n", format);
		return MSG_STATUS_FORMAT_ERROR;
	}
}

/**
 * @brief  Get frame header
 * @param  buf: A pointer to the data to decode
 * @param  buf_len: The length of the data to decode
 * @param  msg:  A pointer to save header message
 * @return message_status
 */
message_status_t message_get_header(uint8_t *buf, uint8_t buf_len, message_t *msg)
{
	if (current_if != NULL && current_if->message_get_header != NULL) {
		return current_if->message_get_header(buf, buf_len, msg);
	} else {
		osal_printf("message_get_header unsupported format:%02X\n", format);
		return MSG_STATUS_FORMAT_ERROR;
	}
}
