/*
 * ATModem - A PicoWAN/LoRaWAN modem firmware using AT commands
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

#include <boards.h>
#include <console.h>
#include <crc8.h>
#include <flash.h>
#include <led.h>
#include <lora_mac.h>
#include <mac.h>
#include <os.h>
#include <radio.h>
#include <spi.h>
#include <stdlib.h>
#include <string.h>
#include <system.h>
#include <usart.h>
#include <utils.h>
#include <xprintf.h>

#define PAYLOAD_BUFFER_SIZE		250

#define PAYLOAD_S_BUFFER_LEN		5
#define TIMEOUT_S_BUFFER_LEN		5
#define NB_RETRIES_S_BUFFER_LEN		3

#define LED0				0
#define LED1				1

#ifdef CFG_picotag_board
/* These informations are read by the PicoSmartTAG bootloader */
typedef struct {
	uint8_t version;
	uint8_t type; /* must be 253 */
} fw_info_t;

/* Firmware info (will be read by the bootloader) */
static __attribute__((used, section(".FWVersion"))) fw_info_t firmware_info = {
	.version = 19,
	.type = 253,
};
#endif

/* Device Address */
static uint32_t DEVADDR = 0;

/* Device specific Application session key */
static __attribute__((aligned (4))) uint8_t APPSKEY[16] = {0};

/* Device specific Network session key */
static __attribute__((aligned (4))) uint8_t NWKSKEY[16] = {0};

/* Application EUI */
static __attribute__((aligned (4))) uint8_t APPEUI[8] = {0};

/* Device EUI */
static __attribute__((aligned (4))) uint8_t DEVEUI[8] = {0};

/* Application key */
static __attribute__((aligned (4))) uint8_t APPKEY[16] = {0};

static mac_type_t current_mac = PICOMAC;
static device_class_t current_class = CLASS_A;

static os_job_t timeout_job;
static os_job_t turn_off_leds_job;

static uint8_t port = 1;
static uint8_t rx2_dr = 0;
static uint8_t mode_otaa = 0;
static uint16_t payload_len = 0;
static uint8_t tx_confirmed = 0;
static uint8_t is_joining = 0;
static uint16_t nb_retries = 2;

static uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE];
static uint16_t bytes_in_payload_buffer = 0;

static void rxdone_cb(mac_rx_info_t *info);
static void txdone_cb(mac_tx_info_t *info);
static void network_cb(mac_network_info_t *info);
static void mac_state_cb(mac_state_t state);

static uint32_t get_uplink_counter_cb(void);
static void save_uplink_counter_cb(uint32_t uplink_counter);
static uint32_t get_downlink_counter_cb(void);
static void save_downlink_counter_cb(uint32_t downlink_counter);

static void main_init(os_job_t *j);

static mac_message_callbacks_t message_cbs = {
	.mac_tx_done = txdone_cb,
	.mac_rx_done = rxdone_cb,
	.mac_network_state = network_cb,
	.mac_state = mac_state_cb,
};

static mac_flash_callback_t flash_cbs = {
	.get_uplink_counter = get_uplink_counter_cb,
	.save_uplink_counter = save_uplink_counter_cb,
	.get_downlink_counter = get_downlink_counter_cb,
	.save_downlink_counter = save_downlink_counter_cb,
};



static void turn_off_leds_task(os_job_t *j)
{
	led_set(LED0, LED_OFF);
	led_set(LED1, LED_OFF);
}

static void rxdone_cb(mac_rx_info_t *info)
{
	uint8_t i;

	if (is_joining) {
		if (info->status == MAC_INFO_STATUS_JOIN_ACCEPT_OK) {
			console_return_ok();
		} else {
			console_return_error();
		}
		is_joining = 0;
	}

	if (info->status == MAC_INFO_STATUS_OK) {
		xprintf("+DOWNLINK=%d,%d,%u,%u,", info->snr, info->rssi, info->port, info->payload_len);
		for (i = 0; i < info->payload_len; i++) {
			xprintf("%02X", info->payload[i]);
		}
		xprintf("\n");
	}
}

static void txdone_cb(mac_tx_info_t *info)
{
	if (info->status < 0) {
		led_set(LED1, LED_OFF);
		led_set(LED0, LED_ON);
		os_post_delayed_job(&turn_off_leds_job, os_get_time() + ms2ostime(5000), turn_off_leds_task);
		console_return_error();
	} else {
		led_set(LED1, LED_ON);
		led_set(LED0, LED_OFF);
		os_post_delayed_job(&turn_off_leds_job, os_get_time() + ms2ostime(5000), turn_off_leds_task);
		console_return_ok();
	}
}

static void network_cb(mac_network_info_t *info)
{
	if (info->status == NETWORK_AVAILABLE) {
		xprintf("+CHECKNET: AVAILABLE\n");
	} else if (info->status == NETWORK_UNAVAILABLE) {
		xprintf("+CHECKNET: UNAVAILABLE\n");
	} else {
		xprintf("+CHECKNET: UNKNOWN\n");
	}
}

static void mac_state_cb(mac_state_t state)
{
	switch (state) {
		case MAC_STATE_SEARCHING_NETWORK:
			led_blink(LED1, ON_OFF_SLOTS_PERIOD_2_DUTY_50, 0);
			break;

		case MAC_STATE_SENDING_DATA:
			led_blink(LED1, ON_OFF_SLOTS_PERIOD_4_DUTY_50, 0);
			break;

		case MAC_STATE_WAITING_ACK:
			led_blink(LED1, ON_OFF_SLOTS_PERIOD_8_DUTY_50, 0);
			break;

		case MAC_STATE_WAITING_RETRY:
			led_blink(LED1, ON_OFF_SLOTS_PERIOD_16_DUTY_50, 0);
			break;

		default:
			break;
	}
}

static uint32_t get_uplink_counter_cb(void)
{
	uint32_t uplink_counter;

	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_LORA_UPLINK_COUNTER_OFFSET, &uplink_counter, NV_COMMON_LORA_UPLINK_COUNTER_SIZE);

	return uplink_counter;
}

static void save_uplink_counter_cb(uint32_t uplink_counter)
{
	flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_LORA_UPLINK_COUNTER_OFFSET, &uplink_counter, NV_COMMON_LORA_UPLINK_COUNTER_SIZE);
}

static uint32_t get_downlink_counter_cb(void)
{
	uint32_t downlink_counter;

	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_LORA_DOWNLINK_COUNTER_OFFSET, &downlink_counter, NV_COMMON_LORA_DOWNLINK_COUNTER_SIZE);

	return downlink_counter;
}

static void save_downlink_counter_cb(uint32_t downlink_counter)
{
	flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_LORA_DOWNLINK_COUNTER_OFFSET, &downlink_counter, NV_COMMON_LORA_DOWNLINK_COUNTER_SIZE);
}


static void timeout_task(os_job_t *j)
{
	xprintf("Timeout reached\n");
	bytes_in_payload_buffer = 0;
	console_register_raw_callback(NULL);
	console_enable_commands(1);
	console_return_error();
}

static void del_spaces(char *s)
{
	uint8_t i, j = 0;

	for (i = 0; s[i] != '\0'; i++) {
		if (s[i] != ' ') {
			s[j++] = s[i];
		}
	}
	s[j] = '\0';
}

static cmd_ret_val_t reboot_cb(char *args)
{
	/* Wait for any UART transmission to complete */
	usart_sync(CONSOLE_USART);

	system_reboot();

	return CMD_OK;
}

static cmd_ret_val_t mac_cb(char *args)
{
	uint8_t len = strlen(args);

	if (len != 0) {
		if (args[0] == '?') { // query
			xprintf("+MAC: ");
			(current_mac == PICOMAC) ? xprintf("PICO\n") : xprintf("LORA\n");
		} else {
			if (!(strncmp(args, "PICO", len) && strncmp(args, "0", len))) {
				xprintf("+MAC: PICO\n");
				current_mac = PICOMAC;

				flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_DEVICE_EUI_OFFSET, (uint32_t *) DEVEUI, NV_COMMON_DEVICE_EUI_SIZE);
				flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_APPLICATION_EUI_OFFSET, (uint32_t *) APPEUI,
						  NV_COMMON_APPLICATION_EUI_SIZE);
				flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_PICOWAN_DEVICE_ADDRESS_OFFSET, &DEVADDR,
						  NV_COMMON_PICOWAN_DEVICE_ADDRESS_SIZE);
				flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_OFFSET, (uint32_t *) NWKSKEY,
						  NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_SIZE);
				flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_OFFSET, (uint32_t *) APPSKEY,
						  NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_SIZE);
				flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_APPLICATION_KEY_OFFSET, (uint32_t *) APPKEY,
						  NV_COMMON_APPLICATION_KEY_SIZE);
			} else if (!(strncmp(args, "LORA", len) && strncmp(args, "1", len))) {
				xprintf("+MAC: LORA\n");
				current_mac = LORAMAC;

				flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_DEVICE_EUI_OFFSET, (uint32_t *) DEVEUI, NV_COMMON_DEVICE_EUI_SIZE);
				flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_APPLICATION_EUI_OFFSET, (uint32_t *) APPEUI,
						  NV_COMMON_APPLICATION_EUI_SIZE);
				flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_LORA_DEVICE_ADDRESS_OFFSET, &DEVADDR,
						  NV_COMMON_LORA_DEVICE_ADDRESS_SIZE);
				flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_LORA_NETWORK_SESSION_KEY_OFFSET, (uint32_t *) NWKSKEY,
						  NV_COMMON_LORA_NETWORK_SESSION_KEY_SIZE);
				flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_LORA_APPLICATION_SESSION_KEY_OFFSET, (uint32_t *) APPSKEY,
						  NV_COMMON_LORA_APPLICATION_SESSION_KEY_SIZE);
				flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_APPLICATION_KEY_OFFSET, (uint32_t *) APPKEY,
						  NV_COMMON_APPLICATION_KEY_SIZE);
			} else {
				xprintf("Invalid parameter\n");
				return CMD_ERROR;
			}
		}
	} else {
		xprintf("Invalid parameter\n");
		return CMD_ERROR;
	}

	mac_init(current_mac, &message_cbs, NULL, &flash_cbs);

	return CMD_OK;
}

static cmd_ret_val_t mode_cb(char *args)
{
	uint8_t len = strlen(args);

	if (len != 0) {
		if (args[0] == '?') { // query
			xprintf("+MODE: ");
			mode_otaa ? xprintf("OTAA\n") : xprintf("ABP\n");
		} else {
			if (!(strncmp(args, "ABP", len) && strncmp(args, "0", len))) {
				mode_otaa = 0;
				xprintf("+MODE: ABP\n");
			} else if (!(strncmp(args, "OTAA", len) && strncmp(args, "1", len))) {
				mode_otaa = 1;
				xprintf("+MODE: OTAA\n");
			} else {
				xprintf("Invalid parameter\n");
				return CMD_ERROR;
			}
		}
	} else {
		xprintf("Invalid parameter\n");
		return CMD_ERROR;
	}

	return CMD_OK;
}

static cmd_ret_val_t join_cb(char *args)
{
	mac_status_t error;

	if (!mode_otaa) {
		error = mac_init_activation_personalization(0, DEVADDR, NWKSKEY, APPSKEY);
		if (error < 0) {
			return CMD_ERROR;
		} else {
			return CMD_OK;
		}
	} else {
		is_joining = 1;
		error = mac_init_activation_on_air(DEVEUI, APPEUI, APPKEY);
		if (error < 0) {
			return CMD_ERROR;
		} else {
			return CMD_DELAYED;
		}
	}
}

static cmd_ret_val_t check_net_cb(char *args)
{
	if (mac_network_available() < 0) {
		return CMD_ERROR;
	}

	return CMD_OK;
}

static cmd_ret_val_t devaddr_cb(char *args)
{
	uint8_t len = strlen(args);
	char *endptr = NULL;
	uint32_t dev_addr_tmp = 0;

	if (len != 0) {
		del_spaces(args);
		len = strlen(args);
		if (args[0] == '?') { // query
			if (mode_otaa) {
				mac_get_device_address(&dev_addr_tmp);
				xprintf("+DEVADDR: %08X\n", dev_addr_tmp);
			} else {
				xprintf("+DEVADDR: %08X\n", DEVADDR);
			}
		} else if (len == 8) { // without "0x" or "x"
			if (!mode_otaa) {
				dev_addr_tmp = (uint32_t) strtoul(args, &endptr, 16);
				if (endptr == (args + 8)) {
					DEVADDR = dev_addr_tmp;
					xprintf("+DEVADDR: %08X\n", DEVADDR);
				} else {
					xprintf("Parse error: unexpected character\n");
					return CMD_ERROR;
				}
			} else {
				xprintf("Invalid mode to set a value\n");
				return CMD_ERROR;
			}
		} else {
			xprintf("Invalid parameter\n");
			return CMD_ERROR;
		}
	} else {
		xprintf("Invalid parameter\n");
		return CMD_ERROR;
	}

	if (current_mac == PICOMAC) {
		flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_PICOWAN_DEVICE_ADDRESS_OFFSET, &DEVADDR, NV_COMMON_PICOWAN_DEVICE_ADDRESS_SIZE);
	} else {
		flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_LORA_DEVICE_ADDRESS_OFFSET, &DEVADDR, NV_COMMON_LORA_DEVICE_ADDRESS_SIZE);
	}

	return CMD_OK;
}

static cmd_ret_val_t deveui_cb(char *args)
{
	uint8_t len = strlen(args);
	char *endptr = NULL;
	uint8_t deveui_tmp[8];
	char args_double[2];
	uint8_t i = 0;

	if (len != 0) {
		del_spaces(args);
		len = strlen(args);
		if (args[0] == '?') { // query
			xprintf("+DEVEUI: ");
			for (i = 0; i < 8; i++) {
				xprintf("%02X", DEVEUI[7 - i]);
			}
			xprintf("\n");
		} else if (len == 16) {
			for (i = 0; i < 8; i++) {
				args_double[0] = args[i * 2];
				args_double[1] = args[i * 2 + 1];
				deveui_tmp[i] = (uint8_t) strtoul(args_double, &endptr, 16);
				if (endptr != (args_double + 2)) {
					xprintf("Parse error: unexpected character\n");
					return CMD_ERROR;
				}
			}

			utils_memcpy_r(DEVEUI, deveui_tmp, 8);
			flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_DEVICE_EUI_OFFSET, (uint32_t *) DEVEUI, NV_COMMON_DEVICE_EUI_SIZE);

			xprintf("+DEVEUI: ");
			for (i = 0; i < 8; i++) {
				xprintf("%02X", DEVEUI[7 - i]);
			}
			xprintf("\n");
		} else {
			xprintf("Invalid parameter\n");
			return CMD_ERROR;
		}
	} else {
		xprintf("Invalid parameter\n");
		return CMD_ERROR;
	}

	return CMD_OK;
}

static cmd_ret_val_t appeui_cb(char *args)
{
	uint8_t len = strlen(args);
	char *endptr = NULL;
	uint8_t appeui_tmp[8];
	char args_double[2];
	uint8_t i = 0;

	if (len != 0) {
		del_spaces(args);
		len = strlen(args);
		if (args[0] == '?') { // query
			xprintf("+APPEUI: ");
			for (i = 0; i < 8; i++) {
				xprintf("%02X", APPEUI[7 - i]);
			}
			xprintf("\n");
		} else if (len == 16) {
			for (i = 0; i < 8; i++) {
				args_double[0] = args[i * 2];
				args_double[1] = args[i * 2 + 1];
				appeui_tmp[i] = (uint8_t) strtoul(args_double, &endptr, 16);
				if (endptr != (args_double + 2)) {
					xprintf("Parse error: unexpected character\n");
					return CMD_ERROR;
				}
			}

			utils_memcpy_r(APPEUI, appeui_tmp, 8);
			flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_APPLICATION_EUI_OFFSET, (uint32_t *) APPEUI, NV_COMMON_APPLICATION_EUI_SIZE);

			xprintf("+APPEUI: ");
			for (i = 0; i < 8; i++) {
				xprintf("%02X", APPEUI[7 - i]);
			}
			xprintf("\n");
		} else {
			xprintf("Invalid parameter\n");
			return CMD_ERROR;
		}
	} else {
		xprintf("Invalid parameter\n");
		return CMD_ERROR;
	}

	return CMD_OK;
}

static cmd_ret_val_t nwkskey_cb(char *args)
{
	uint8_t len = strlen(args);
	char *endptr = NULL;
	uint8_t nwkskey_tmp[16];
	char args_double[2];
	uint8_t i = 0;

	if (len != 0) {
		if (!mode_otaa) {
			del_spaces(args);
			len = strlen(args);
			if (len == 32) {
				for (i = 0; i < 16; i++) {
					args_double[0] = args[i * 2];
					args_double[1] = args[i * 2 + 1];
					nwkskey_tmp[i] = (uint8_t) strtoul(args_double, &endptr, 16);
					if (endptr != (args_double + 2)) {
						xprintf("Parse error: unexpected character\n");
						return CMD_ERROR;
					}
				}

				memcpy(NWKSKEY, nwkskey_tmp, 16);

				xprintf("+NWKSKEY: ");
				for (i = 0; i < 16; i++) {
					xprintf("%02X", NWKSKEY[i]);
				}
				xprintf("\n");
			} else {
				xprintf("Invalid parameter\n");
				return CMD_ERROR;
			}
		} else {
			xprintf("Invalid mode\n");
			return CMD_ERROR;
		}
	} else {
		xprintf("Invalid parameter\n");
		return CMD_ERROR;
	}

	if (current_mac == PICOMAC) {
		flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_OFFSET, (uint32_t *) NWKSKEY,
				   NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_SIZE);
	} else {
		flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_LORA_NETWORK_SESSION_KEY_OFFSET, (uint32_t *) NWKSKEY,
				   NV_COMMON_LORA_NETWORK_SESSION_KEY_SIZE);
	}

	return CMD_OK;
}

static cmd_ret_val_t appskey_cb(char *args)
{
	uint8_t len = strlen(args);
	char *endptr = NULL;
	uint8_t appskey_tmp[16];
	char args_double[2];
	uint8_t i = 0;

	if (len != 0) {
		if (!mode_otaa) {
			del_spaces(args);
			len = strlen(args);
			if (len == 32) {
				for (i = 0; i < 16; i++) {
					args_double[0] = args[i * 2];
					args_double[1] = args[i * 2 + 1];
					appskey_tmp[i] = (uint8_t) strtoul(args_double, &endptr, 16);
					if (endptr != (args_double + 2)) {
						xprintf("Parse error: unexpected character\n");
						return CMD_ERROR;
					}
				}

				memcpy(APPSKEY, appskey_tmp, 16);

				xprintf("+APPSKEY: ");
				for (i = 0; i < 16; i++) {
					xprintf("%02X", APPSKEY[i]);
				}
				xprintf("\n");
			} else {
				xprintf("Invalid parameter\n");
				return CMD_ERROR;
			}
		} else {
			xprintf("Invalid mode\n");
			return CMD_ERROR;
		}
	} else {
		xprintf("Invalid parameter\n");
		return CMD_ERROR;
	}

	if (current_mac == PICOMAC) {
		flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_OFFSET, (uint32_t *) APPSKEY,
				   NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_SIZE);
	} else {
		flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_LORA_APPLICATION_SESSION_KEY_OFFSET, (uint32_t *) APPSKEY,
				   NV_COMMON_LORA_APPLICATION_SESSION_KEY_SIZE);
	}

	return CMD_OK;
}

static cmd_ret_val_t appkey_cb(char *args)
{
	uint8_t len = strlen(args);
	char *endptr = NULL;
	uint8_t appkey_tmp[16];
	char args_double[2];
	uint8_t i = 0;

	if (len != 0) {
		del_spaces(args);
		len = strlen(args);
		if (len == 32) {
			for (i = 0; i < 16; i++) {
				args_double[0] = args[i * 2];
				args_double[1] = args[i * 2 + 1];
				appkey_tmp[i] = (uint8_t) strtoul(args_double, &endptr, 16);
				if (endptr != (args_double + 2)) {
					xprintf("Parse error: unexpected character\n");
					return CMD_ERROR;
				}
			}

			memcpy(APPKEY, appkey_tmp, 16);

			flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_APPLICATION_KEY_OFFSET, (uint32_t *) APPKEY, NV_COMMON_APPLICATION_KEY_SIZE);

			xprintf("+APPKEY: ");
			for (i = 0; i < 16; i++) {
				xprintf("%02X", APPKEY[i]);
			}
			xprintf("\n");
		} else {
			xprintf("Invalid parameter\n");
			return CMD_ERROR;
		}
	} else {
		xprintf("Invalid parameter\n");
		return CMD_ERROR;
	}

	return CMD_OK;
}

static cmd_ret_val_t store_pico_infos_cb(char *args)
{
	uint32_t dev_addr_tmp = 0;
	uint8_t deveui_tmp[8];
	uint8_t appskey_tmp[16];
	uint8_t nwkskey_tmp[16];
	char *endptr = NULL;
	char args_double[2];
	uint8_t i = 0;
	char byte_str[3] = {0};
	uint8_t crc = 0;

	del_spaces(args);

	if ((args[8] == ',') && (args[25] == ',') && (args[58] == ',') && (args[91] == ',')) { // check good length for every parameters
		/* CRC8 check */
		byte_str[0] = args[strlen(args) - 2];
		byte_str[1] = args[strlen(args) - 1];
		crc = (uint8_t) strtol(byte_str, NULL, 16);
		if (crc8(0, (const uint8_t *) args, strlen(args) - 2) != crc) {
			return CMD_ERROR;
		}

		/* DEVICE ADDRESS */
		dev_addr_tmp = (uint32_t) strtoul(args, &endptr, 16);
		if (endptr == (args + 8)) {
			DEVADDR = dev_addr_tmp;
		} else {
			xprintf("Parse error: unexpected character\n");
			return CMD_ERROR;
		}

		/* DEVEUI */
		for (i = 0; i < 8; i++) {
			args_double[0] = args[9 + i * 2];
			args_double[1] = args[9 + i * 2 + 1];
			deveui_tmp[i] = (uint8_t) strtoul(args_double, &endptr, 16);
			if (endptr != (args_double + 2)) {
				xprintf("Parse error: unexpected character\n");
				return CMD_ERROR;
			}
		}
		utils_memcpy_r(DEVEUI, deveui_tmp, 8);

		/* APPSKEY */
		for (i = 0; i < 16; i++) {
			args_double[0] = args[26 + i * 2];
			args_double[1] = args[26 + i * 2 + 1];
			appskey_tmp[i] = (uint8_t) strtoul(args_double, &endptr, 16);
			if (endptr != (args_double + 2)) {
				xprintf("Parse error: unexpected character\n");
				return CMD_ERROR;
			}
		}
		memcpy(APPSKEY, appskey_tmp, 16);

		/* NWKSKEY */
		for (i = 0; i < 16; i++) {
			args_double[0] = args[59 + i * 2];
			args_double[1] = args[59 + i * 2 + 1];
			nwkskey_tmp[i] = (uint8_t) strtoul(args_double, &endptr, 16);
			if (endptr != (args_double + 2)) {
				xprintf("Parse error: unexpected character\n");
				return CMD_ERROR;
			}
		}
		memcpy(NWKSKEY, nwkskey_tmp, 16);

		/* Store informations */
		flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_DEVICE_EUI_OFFSET, (uint32_t *) DEVEUI, NV_COMMON_DEVICE_EUI_SIZE);
		flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_PICOWAN_DEVICE_ADDRESS_OFFSET, &DEVADDR,
				   NV_COMMON_PICOWAN_DEVICE_ADDRESS_SIZE);
		flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_OFFSET, (uint32_t *) APPSKEY,
				   NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_SIZE);
		flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_OFFSET, (uint32_t *) NWKSKEY,
				   NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_SIZE);

		/* Display informations */
		xprintf("+DEVADDR: %08X\n", DEVADDR);

		xprintf("+DEVEUI: ");
		for (i = 0; i < 8; i++) {
			xprintf("%02X", DEVEUI[7 - i]);
		}
		xprintf("\n");

		xprintf("+APPSKEY: ");
		for (i = 0; i < 16; i++) {
			xprintf("%02X", APPSKEY[i]);
		}
		xprintf("\n");

		xprintf("+NWKSKEY: ");
		for (i = 0; i < 16; i++) {
			xprintf("%02X", NWKSKEY[i]);
		}
		xprintf("\n");
	} else {
		xprintf("Invalid parameter\n");
		return CMD_ERROR;
	}

	return CMD_OK;
}

static cmd_ret_val_t class_cb(char *args)
{
	uint8_t len = strlen(args);

	if (len != 0) {
		if (args[0] == '?') {
			switch (current_class) {
				case CLASS_A:
					xprintf("+CLASS: %c\n", 'A');
					return CMD_OK;
				case CLASS_C:
					xprintf("+CLASS: %c\n", 'C');
					return CMD_OK;
				default:
					return CMD_ERROR;
			}
		} else {
			switch (args[0]) {
				case 'A':
					current_class = CLASS_A;
					mac_set_device_class(CLASS_A);
					xprintf("+CLASS: %c\n", 'A');
					return CMD_OK;
				case 'C':
					current_class = CLASS_C;
					mac_set_device_class(CLASS_C);
					xprintf("+CLASS: %c\n", 'C');
					return CMD_OK;
				default :
					xprintf("Invalid parameter\n");
					return CMD_ERROR;
			}
		}
	} else {
		xprintf("Invalid parameter\n");
		return CMD_ERROR;
	}

	return CMD_OK;
}

static cmd_ret_val_t port_cb(char *args)
{
	uint8_t len = strlen(args);
	uint32_t port_temp = 0;
	char *endptr = NULL;

	if (len != 0) {
		if (args[0] == '?') {
			xprintf("+PORT: %u\n", port);
		} else {
			port_temp = (uint32_t) strtoul(args, &endptr, 10);
			if (endptr == args) {
				xprintf("Parse error: unexpected character\n");
				return CMD_ERROR;
			}

			if (port_temp < 0) {
				port_temp = 1; // don't send a MAC command by default
			} else if (port_temp > 223) {
				port_temp = 223;
			}

			port = (uint8_t) port_temp;
			xprintf("+PORT: %u\n", port);
		}
	} else {
		xprintf("Invalid parameter\n");
		return CMD_ERROR;
	}

	return CMD_OK;
}

static cmd_ret_val_t rx2_dr_cb(char *args)
{
	uint8_t len = strlen(args);
	uint8_t rx2_dr_temp = 0;
	char *endptr = NULL;

	if (len != 0) {
		if (args[0] == '?') {
			xprintf("+RX2DR: %u\n", rx2_dr);
		} else {
			rx2_dr_temp = (uint8_t) strtoul(args, &endptr, 10);
			if (endptr == args) {
				xprintf("Parse error: unexpected character\n");
				return CMD_ERROR;
			}

			if (rx2_dr_temp > 6) {
				rx2_dr_temp = 6;
			}

			rx2_dr = rx2_dr_temp;
			lora_mac_set_rx2_datarate(rx2_dr);
			xprintf("+RX2DR: %u\n", rx2_dr);
		}
	} else {
		xprintf("Invalid parameter\n");
		return CMD_ERROR;
	}

	return CMD_OK;
}

static void raw_cb(uint8_t *args, uint16_t length)
{
	uint16_t tmp = 0;

	while (length > 0) {
		if (length > PAYLOAD_BUFFER_SIZE - bytes_in_payload_buffer) {
			tmp = PAYLOAD_BUFFER_SIZE - bytes_in_payload_buffer;
		} else {
			tmp = length;
		}

		if ((bytes_in_payload_buffer + tmp) > payload_len) {
			tmp = payload_len - bytes_in_payload_buffer;
		}

		memcpy(&payload_buffer[bytes_in_payload_buffer], args, tmp);
		bytes_in_payload_buffer += tmp;
		args += tmp;
		length -= tmp;

		if (bytes_in_payload_buffer == PAYLOAD_BUFFER_SIZE || bytes_in_payload_buffer == payload_len) {
			if (bytes_in_payload_buffer == payload_len) {
				os_cancel_job(&timeout_job);
				payload_buffer[bytes_in_payload_buffer] = '\0';

				xprintf("Sending payload\n");

				if (tx_confirmed) {
					mac_status_t error = mac_send_when_possible_confirmed(port, (uint8_t *) payload_buffer,
											      bytes_in_payload_buffer, (uint8_t) nb_retries, DL_ENABLED);
					if (error < 0) {
						xprintf("CTX error: %d\n", error);
						console_return_error();
					}
				} else {
					mac_status_t error =
						mac_send_when_possible(port, (uint8_t *) payload_buffer, bytes_in_payload_buffer, DL_ENABLED);
					if (error < 0) {
						xprintf("UTX error: %d\n", error);
						console_return_error();
					}
				}

				bytes_in_payload_buffer = 0;
				console_register_raw_callback(NULL);
				console_enable_commands(1);
				return;
			}
		}
	}
}

static cmd_ret_val_t utx_cb(char *args)
{
	char payload_len_s[PAYLOAD_S_BUFFER_LEN] = {0};
	char timeout_s[TIMEOUT_S_BUFFER_LEN] = {0};
	uint32_t timeout = 1000;
	int32_t delim = 0;
	char *endptr = NULL;

	if (args == NULL) {
		xprintf("Syntax error\n");
		return CMD_ERROR;
	}

	del_spaces(args);
	delim = strchr(args, ',') - args;

	if (delim <= 0) { // no coma in args
		delim = strlen(args);
		if (delim > PAYLOAD_S_BUFFER_LEN) { // payload length buffer too small
			xprintf("Payload length too big\n");
			return CMD_ERROR;
		}
	} else if (delim > PAYLOAD_S_BUFFER_LEN) { // payload length buffer too small
		xprintf("Payload length too big\n");
		return CMD_ERROR;
	} else if ((strlen(args) - (delim + 1)) > TIMEOUT_S_BUFFER_LEN) { // timeout buffer too small
		xprintf("Timeout too big\n");
		return CMD_ERROR;
	}

	strncpy(payload_len_s, args, delim);
	payload_len = (uint16_t) strtoul(payload_len_s, &endptr, 10);
	if (endptr == payload_len_s) {
		xprintf("No payload\n");
		return CMD_ERROR;
	}

	if (delim != strlen(args)) {
		strcpy(timeout_s, args + delim + 1);
		timeout = (uint32_t) strtoul(timeout_s, &endptr, 10);
		if (endptr == timeout_s) {
			xprintf("No timeout\n");
			return CMD_ERROR;
		}
	}

	if (payload_len <= 250) {
		tx_confirmed = 0;
		os_post_delayed_job(&timeout_job, os_get_time() + ms2ostime(timeout), timeout_task);
		console_register_raw_callback(&raw_cb);
		console_enable_commands(0);
	} else {
		xprintf("The maximum payload length is 250\n");
		return CMD_ERROR;
	}

	return CMD_DELAYED;
}

static cmd_ret_val_t ctx_cb(char *args)
{
	char payload_len_s[PAYLOAD_S_BUFFER_LEN] = {0};
	char timeout_s[TIMEOUT_S_BUFFER_LEN] = {0};
	char nb_retries_s[NB_RETRIES_S_BUFFER_LEN] = {0};
	uint32_t timeout = 1000;
	int32_t delim = 0, delim2 = 0;
	char *endptr = NULL;

	nb_retries = 2; // Default retries number

	if (args == NULL) {
		xprintf("Syntax error\n");
		return CMD_ERROR;
	}

	del_spaces(args);
	delim = strchr(args, ',') - args;
	delim2 = strchr(args + delim + 1, ',') - args;

	if (delim <= 0) { // no coma in args
		delim = strlen(args);
		if (delim > PAYLOAD_S_BUFFER_LEN) { // payload length buffer too small
			xprintf("Payload length too big\n");
			return CMD_ERROR;
		}
	} else if (delim > PAYLOAD_S_BUFFER_LEN) { // payload length buffer too small
		xprintf("Payload length too big\n");
		return CMD_ERROR;
	} else {
		if (delim2 <= 0) {
			delim2 = strlen(args);
			if (((delim2 - 1) - delim) > NB_RETRIES_S_BUFFER_LEN) { // retries number buffer too small
				xprintf("Number of retries too big\n");
				return CMD_ERROR;
			}
		} else if (((delim2 - 1) - delim) > NB_RETRIES_S_BUFFER_LEN) { // retries number buffer too small
			xprintf("Number of retries too big\n");
			return CMD_ERROR;
		} else if ((strlen(args) - (delim2 + 1)) > TIMEOUT_S_BUFFER_LEN) { // timeout buffer too small
			xprintf("Timeout too big\n");
			return CMD_ERROR;
		}
	}

	strncpy(payload_len_s, args, delim);
	payload_len = (uint16_t) strtoul(payload_len_s, &endptr, 10);
	if (endptr == payload_len_s) {
		xprintf("No payload\n");
		return CMD_ERROR;
	}

	if (delim != strlen(args)) {
		strcpy(nb_retries_s, args + delim + 1);
		nb_retries = (uint16_t) strtoul(nb_retries_s, &endptr, 10);
		if (endptr == nb_retries_s) {
			xprintf("No retries number\n");
			return CMD_ERROR;
		}

		if (delim2 != strlen(args)) {
			strcpy(timeout_s, args + delim2 + 1);
			timeout = (uint32_t) strtoul(timeout_s, &endptr, 10);
			if (endptr == timeout_s) {
				xprintf("No timeout\n");
				return CMD_ERROR;
			}
		}
	}

	if (payload_len <= 250) {
		if (nb_retries <= 255) {
			tx_confirmed = 1;
			os_post_delayed_job(&timeout_job, os_get_time() + ms2ostime(timeout), timeout_task);
			console_register_raw_callback(&raw_cb);
			console_enable_commands(0);
		} else {
			xprintf("The maximum number of retries is 255\n");
			return CMD_ERROR;
		}
	} else {
		xprintf("The maximum payload length is 250\n");
		return CMD_ERROR;
	}

	return CMD_DELAYED;
}

static struct command cmd_list[] = {
	{"HELP", &console_commands_help},
	{"REBOOT", &reboot_cb},
	{"MAC", &mac_cb},		//Pico, LoRa
	{"MODE", &mode_cb},		//ABP, OTAA
	{"JOIN", &join_cb},
	{"CHECKNET", &check_net_cb},
	{"DEVADDR", &devaddr_cb},
	{"DEVEUI", &deveui_cb},
	{"APPEUI", &appeui_cb},
	{"NWKSKEY", &nwkskey_cb},
	{"APPSKEY", &appskey_cb},
	{"APPKEY", &appkey_cb},
	{"PICOCRED", &store_pico_infos_cb},
	{"CLASS", &class_cb},
	{"PORT", &port_cb},
	{"LORARX2DR", &rx2_dr_cb},
	{"UTX", &utx_cb},
	{"CTX", &ctx_cb},

	{NULL, NULL}
};

static void main_init(os_job_t *j)
{
	/* Initialize the LEDs */
	led_init();

	/* Initialization and sanity check of the Eeprom data (no private section defined) */
	flash_data_nvcheck(0, 0);

	xprintf("PicoWAN/LoRaWAN AT Modem\n\n");

	mac_cb("PICO");
	mode_cb("ABP");
	class_cb("A");

	console_register_cmds(cmd_list);
	console_enable_commands(1);

	xprintf("+READY\n");
}

int main(void)
{
	os_job_t initjob;

	/* Initialize the system */
	os_init();
	spi_init();
	usart_init(CONSOLE_USART, 9600);
	console_init(CONSOLE_USART);
	radio_init();

	/* Initial job to run once in the mainloop */
	os_post_job(&initjob, main_init);

	/* Start the mainloop */
	os_mainloop();

	/* Never reached */
	return 0;
}
