/*
 * FlashKeys - A tool to flash PicoWAN and LoRaWAN keys/IDs
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
#include <flash.h>
#include <led.h>
#include <mac.h>
#include <os.h>
#include <radio.h>
#include <spi.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <usart.h>
#include <utils.h>
#include <crc8.h>
#include <xprintf.h>

#define LED0				0
#define LED1				1

/* If you are using IAR or TrueSTUDIO, the PicoWAN credentials must be defined below. Otherwise,
 * they can be defined on the "make" command-line using the CREDENTIALS variable.
 */
//#define CREDENTIALS			""

#ifndef CREDENTIALS
#error "CREDENTIALS must be set with the proper PicoWAN credentials"
#endif


#ifdef CFG_picotag_board
/* These informations are read by the PicoSmartTAG bootloader */
typedef struct {
	uint8_t version;
	uint8_t type; /* must be 253 */
} fw_info_t;

/* Firmware info (will be read by the bootloader) */
static __attribute__((used, section(".FWVersion"))) fw_info_t firmware_info = {
	.version = 1,
	.type = 253,
};
#endif

/* Device Address */
static uint32_t DEVADDR = 0;

/* Device specific Application session key */
static __attribute__((aligned (4))) uint8_t APPSKEY[16] = {0};

/* Device specific Network session key */
static __attribute__((aligned (4))) uint8_t NWKSKEY[16] = {0};

/* Device EUI */
static __attribute__((aligned (4))) uint8_t DEVEUI[8] = {0};


static void main_init(os_job_t *j);

static void flash_keys(char *args)
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

	if ((args[8] == ',') && (args[25] == ',') && (args[58] == ',') && (args[91] == ',')) { // check good length for every parameters
		/* CRC8 check */
		byte_str[0] = args[strlen(args) - 2];
		byte_str[1] = args[strlen(args) - 1];
		crc = (uint8_t) strtol(byte_str, NULL, 16);
		if (crc8(0, (const uint8_t *) args, strlen(args) - 2) != crc) {
			return;
		}

		/* DEVICE ADDRESS */
		dev_addr_tmp = (uint32_t) strtoul(args, &endptr, 16);
		if (endptr == (args + 8)) {
			DEVADDR = dev_addr_tmp;
		} else {
			xprintf("Parse error: unexpected character\n");
			return;
		}

		/* DEVEUI */
		for (i = 0; i < 8; i++) {
			args_double[0] = args[9 + i * 2];
			args_double[1] = args[9 + i * 2 + 1];
			deveui_tmp[i] = (uint8_t) strtoul(args_double, &endptr, 16);
			if (endptr != (args_double + 2)) {
				xprintf("Parse error: unexpected character\n");
				return;
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
				return;
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
				return;
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
		xprintf("Device address: %08X\n", DEVADDR);

		xprintf("Device EUI: ");
		for (i = 0; i < 8; i++) {
			xprintf("%02X", DEVEUI[7 - i]);
		}
		xprintf("\n");

		xprintf("Application session key: ");
		for (i = 0; i < 16; i++) {
			xprintf("%02X", APPSKEY[i]);
		}
		xprintf("\n");

		xprintf("Network session key: ");
		for (i = 0; i < 16; i++) {
			xprintf("%02X", NWKSKEY[i]);
		}
		xprintf("\n");
	} else {
		xprintf("Invalid parameter\n");
		return;
	}
}

static void main_init(os_job_t *j)
{
	/* Initialize the LEDs */
	led_init();

	/* Initialization and sanity check of the Eeprom data (no private section defined) */
	flash_data_nvcheck(0, 0);

	xprintf("Flashing PicoWAN credentials ...\n\n");

	flash_keys(CREDENTIALS);

	xprintf("\nDone !\n");
}

int main(void)
{
	os_job_t initjob;

	/* Initialize the system */
	os_init();
	spi_init();
	usart_init(CONSOLE_USART, 115200);
	radio_init();

	/* Initial job to run once in the mainloop */
	os_post_job(&initjob, main_init);

	/* Start the mainloop */
	os_mainloop();

	/* Never reached */
	return 0;
}
