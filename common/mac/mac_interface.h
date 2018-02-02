/*
 * mac_interface - The interface that any MAC needs to implement
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

#ifndef _MAC_INTERFACE_H_
#define _MAC_INTERFACE_H_

#include <stdint.h>
#include "mac.h"

typedef struct {
	/*!
	 * \brief Initializes the MAC.
	 */
	void (*mac_init)(mac_message_callbacks_t *cb, mac_battery_callback_t *battery_cb, mac_flash_callback_t *flash_cbs);

	/*!
	 * \brief Deinitializes the MAC.
	 */
	void (*mac_deinit)(void);

	/*!
	 * \brief Sets end-device class. (class A, B or C for LoRaWAN, A or C for PicoWAN)
	 *
	 * \param[in] End-device class to set. [CLASS_A, CLASS_B, CLASS_C]
	 */
	void (*mac_set_device_class)(device_class_t device_class);

	/*!
	 * \brief Initializes the network IDs. The sessions keys must be provided.
	 */
	void (*mac_init_activation_personalization)(uint32_t netid, uint32_t devaddr, uint8_t *nwkskey, uint8_t *appskey);

	/*!
	 * \brief Initiates the Over-the-Air activation. Sends a request with device and application EUI, and waits a response to compute sessions
	 * keys.
	 *
	 * \retval mac_status_t
	 */
	mac_status_t (*mac_init_activation_on_air)(uint8_t *dev_eui, uint8_t *app_eui, uint8_t *app_key);

	/*!
	 * \brief Sends data without acknowledge.
	 *
	 * \retval mac_status_t
	 */
	mac_status_t (*mac_send_when_possible)(uint8_t port, uint8_t *payload, uint8_t payload_len, downlink_mode_t dl_mode);

	/*!
	 * \brief Sends data with acknowledge.
	 *
	 * \retval mac_status_t
	 */
	mac_status_t (*mac_send_when_possible_confirmed)(uint8_t port, uint8_t *payload, uint8_t payload_len, uint8_t nb_retries,
							 downlink_mode_t dl_mode);

	/*!
	 * \brief  Reports network availability using callback. It corresponds to the lora mac command Link Check Req.
	 *
	 * \retval mac_status_t
	 */
	mac_status_t (*mac_network_available)(void);

	/*!
	 * \brief Gets device address
	 *
	 * \details This function returns the device address of the current MAC. 0x00000000 is the default device address, before joining a network
	 *
	 * \retval uint32_t Current device address
	 */
	uint32_t (*mac_get_device_address)(void);
} mac_interface_t;

#endif /* _MAC_INTERFACE_H_ */
