/*
 * mac - MAC unified API for PicoWAN and LoRaWAN network access
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

#ifdef CFG_ENABLE_LORAMAC
#include <lora_mac_internal.h>
#endif
#ifdef CFG_ENABLE_PICOMAC
#include <pico_mac_internal.h>
#endif
#include <stdlib.h>
#include "mac.h"
#include "mac_interface.h"
#include "message.h"

mac_interface_t *current_mac = NULL;


mac_status_t mac_init(mac_type_t mac, mac_message_callbacks_t *cb, mac_battery_callback_t *battery_cb, mac_flash_callback_t *flash_cbs)
{
	/* Be sure no MAC is running */
	if (current_mac != NULL) {
		mac_deinit();
	}

	switch (mac) {
#ifdef CFG_ENABLE_PICOMAC
		case PICOMAC:
			current_mac = &picomac;
			message_init_format(MSG_FORMAT_PICOWAN);
			break;
#endif
#ifdef CFG_ENABLE_LORAMAC
		case LORAMAC:
			current_mac = &loramac;
			message_init_format(MSG_FORMAT_LORA);
			break;
#endif
		default:
			return MAC_STATUS_NOT_INITIALIZED;
	}

	if (current_mac != NULL) {
		current_mac->mac_init(cb, battery_cb, flash_cbs);
		return MAC_STATUS_NO_ERROR;
	} else {
		return MAC_STATUS_NOT_INITIALIZED;
	}
}

mac_status_t mac_deinit(void)
{
	if (current_mac != NULL) {
		if (current_mac->mac_deinit != NULL) {
			current_mac->mac_deinit();
			return MAC_STATUS_NO_ERROR;
		} else {
			return MAC_STATUS_FUNCTION_NOT_FOUND;
		}

		current_mac = NULL;
	} else {
		return MAC_STATUS_NOT_INITIALIZED;
	}
}

mac_status_t mac_init_activation_personalization(uint32_t netid, uint32_t devaddr, uint8_t *nwkskey, uint8_t *appskey)
{
	if (current_mac != NULL) {
		if (current_mac->mac_init_activation_personalization != NULL) {
			current_mac->mac_init_activation_personalization(netid, devaddr, nwkskey, appskey);
			return MAC_STATUS_NO_ERROR;
		} else {
			return MAC_STATUS_FUNCTION_NOT_FOUND;
		}
	} else {
		return MAC_STATUS_NOT_INITIALIZED;
	}
}

mac_status_t mac_init_activation_on_air(uint8_t *dev_eui, uint8_t *app_eui, uint8_t *app_key)
{
	if (current_mac != NULL) {
		if (current_mac->mac_init_activation_on_air != NULL) {
			return current_mac->mac_init_activation_on_air(dev_eui, app_eui, app_key);
		} else {
			return MAC_STATUS_FUNCTION_NOT_FOUND;
		}
	} else {
		return MAC_STATUS_NOT_INITIALIZED;
	}
}

mac_status_t mac_send_when_possible(uint8_t port, uint8_t *payload, uint8_t payload_len, downlink_mode_t dl_mode)
{
	if (current_mac != NULL) {
		if (current_mac->mac_send_when_possible != NULL) {
			return current_mac->mac_send_when_possible(port, payload, payload_len, dl_mode);
		} else {
			return MAC_STATUS_FUNCTION_NOT_FOUND;
		}
	} else {
		return MAC_STATUS_NOT_INITIALIZED;
	}
}

mac_status_t mac_send_when_possible_confirmed(uint8_t port, uint8_t *payload, uint8_t payload_len, uint8_t nb_retries, downlink_mode_t dl_mode)
{
	if (current_mac != NULL) {
		if (current_mac->mac_send_when_possible_confirmed != NULL) {
			return current_mac->mac_send_when_possible_confirmed(port, payload, payload_len, nb_retries, dl_mode);
		} else {
			return MAC_STATUS_FUNCTION_NOT_FOUND;
		}
	} else {
		return MAC_STATUS_NOT_INITIALIZED;
	}
}

mac_status_t mac_network_available(void)
{
	if (current_mac != NULL) {
		if (current_mac->mac_network_available != NULL) {
			return current_mac->mac_network_available();
		} else {
			return MAC_STATUS_FUNCTION_NOT_FOUND;
		}
	} else {
		return MAC_STATUS_NOT_INITIALIZED;
	}
}

mac_status_t mac_set_device_class(device_class_t device_class)
{
	if (current_mac != NULL) {
		if (current_mac->mac_set_device_class != NULL) {
			current_mac->mac_set_device_class(device_class);
			return MAC_STATUS_NO_ERROR;
		} else {
			return MAC_STATUS_FUNCTION_NOT_FOUND;
		}
	} else {
		return MAC_STATUS_NOT_INITIALIZED;
	}
}

mac_status_t mac_get_device_address(uint32_t *device_address)
{
	if (current_mac != NULL) {
		if (current_mac->mac_get_device_address != NULL) {
			*device_address = current_mac->mac_get_device_address();
			return MAC_STATUS_NO_ERROR;
		} else {
			return MAC_STATUS_FUNCTION_NOT_FOUND;
		}
	} else {
		return MAC_STATUS_NOT_INITIALIZED;
	}
}

uint16_t mac_message_max_payload_length(void)
{
	return message_max_payload_length();
}

mac_type_t mac_get_stack(void)
{
#ifdef CFG_ENABLE_PICOMAC
	if (current_mac == &picomac) {
		return PICOMAC;
	}
#endif
#ifdef CFG_ENABLE_LORAMAC
	if (current_mac == &loramac) {
		return LORAMAC;
	}
#endif
	return UNDEF_MAC;
}
