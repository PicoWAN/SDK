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
#include <usart.h>
#include <utils.h>
#include <xprintf.h>

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
	.version = 1,
	.type = 253,
};
#endif

/* Device EUI in big-endian (provided by Archos) */
static __attribute__((aligned (4))) uint8_t DEVEUI[8] = {0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42};

/* Application EUI in big-endian */
static __attribute__((aligned (4))) uint8_t APPEUI[8] = {0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42};

/* Application key */
static __attribute__((aligned (4))) uint8_t APPKEY[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

/* Device Address in big-endian for PicoWAN (provided by Archos) */
static __attribute__((aligned (4))) uint8_t PICO_DEVADDR[4] = {0x42, 0x42, 0x42, 0x42};

/* Device specific Application session key for PicoWAN */
static __attribute__((aligned (4))) uint8_t PICO_APPSKEY[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

/* Device specific Network session key for PicoWAN (provided by Archos) */
static __attribute__((aligned (4))) uint8_t PICO_NWKSKEY[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

/* Device Address in big-endian for LoRaWAN */
static __attribute__((aligned (4))) uint8_t LORA_DEVADDR[4] = {0x42, 0x42, 0x42, 0x42};

/* Device specific Application session key for LoRaWAN */
static __attribute__((aligned (4))) uint8_t LORA_APPSKEY[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

/* Device specific Network session key for LoRaWAN */
static __attribute__((aligned (4))) uint8_t LORA_NWKSKEY[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};


static void main_init(os_job_t *j)
{
	uint8_t i;
	uint8_t tmp_buffer[8];

	/* Initialize the LEDs */
	led_init();

	led_blink(LED0, ON_OFF_SLOTS_PERIOD_2_DUTY_50, 0);
	led_blink(LED1, ON_OFF_SLOTS_PERIOD_2_DUTY_50, 0);

	/* Initialization and sanity check of the Eeprom data (no private section defined) */
	flash_data_nvcheck(0, 0);

	/* Write the keys/IDs in the flash (IDs are stored in little-endian) */
	utils_memcpy_r(tmp_buffer, DEVEUI, 8);
	flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_DEVICE_EUI_OFFSET, (uint32_t *) tmp_buffer, NV_COMMON_DEVICE_EUI_SIZE);
	utils_memcpy_r(tmp_buffer, APPEUI, 8);
	flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_APPLICATION_EUI_OFFSET, (uint32_t *) tmp_buffer, NV_COMMON_APPLICATION_EUI_SIZE);
	flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_APPLICATION_KEY_OFFSET, (uint32_t *) APPKEY, NV_COMMON_APPLICATION_KEY_SIZE);

	utils_memcpy_r(tmp_buffer, PICO_DEVADDR, 4);
	flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_PICOWAN_DEVICE_ADDRESS_OFFSET, (uint32_t *) tmp_buffer, NV_COMMON_PICOWAN_DEVICE_ADDRESS_SIZE);
	flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_OFFSET, (uint32_t *) PICO_NWKSKEY,
			   NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_SIZE);
	flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_OFFSET, (uint32_t *) PICO_APPSKEY,
			   NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_SIZE);

	utils_memcpy_r(tmp_buffer, LORA_DEVADDR, 4);
	flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_LORA_DEVICE_ADDRESS_OFFSET, (uint32_t *) tmp_buffer, NV_COMMON_LORA_DEVICE_ADDRESS_SIZE);
	flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_LORA_NETWORK_SESSION_KEY_OFFSET, (uint32_t *) LORA_NWKSKEY,
			   NV_COMMON_LORA_NETWORK_SESSION_KEY_SIZE);
	flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_LORA_APPLICATION_SESSION_KEY_OFFSET, (uint32_t *) LORA_APPSKEY,
			   NV_COMMON_LORA_APPLICATION_SESSION_KEY_SIZE);

	/* Get the keys/IDs from the flash */
	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_DEVICE_EUI_OFFSET, (uint32_t *) DEVEUI, NV_COMMON_DEVICE_EUI_SIZE);
	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_APPLICATION_EUI_OFFSET, (uint32_t *) APPEUI, NV_COMMON_APPLICATION_EUI_SIZE);
	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_APPLICATION_KEY_OFFSET, (uint32_t *) APPKEY, NV_COMMON_APPLICATION_KEY_SIZE);

	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_PICOWAN_DEVICE_ADDRESS_OFFSET, (uint32_t *) PICO_DEVADDR, NV_COMMON_PICOWAN_DEVICE_ADDRESS_SIZE);
	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_OFFSET, (uint32_t *) PICO_NWKSKEY,
			  NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_SIZE);
	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_OFFSET, (uint32_t *) PICO_APPSKEY,
			  NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_SIZE);

	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_LORA_DEVICE_ADDRESS_OFFSET, (uint32_t *) LORA_DEVADDR, NV_COMMON_LORA_DEVICE_ADDRESS_SIZE);
	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_LORA_NETWORK_SESSION_KEY_OFFSET, (uint32_t *) LORA_NWKSKEY,
			  NV_COMMON_LORA_NETWORK_SESSION_KEY_SIZE);
	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_LORA_APPLICATION_SESSION_KEY_OFFSET, (uint32_t *) LORA_APPSKEY,
			  NV_COMMON_LORA_APPLICATION_SESSION_KEY_SIZE);

	xprintf("Values read from flash are now:\n");
	xprintf("   Device EUI:");
	for (i = 0; i < 8; i++) {
		xprintf(" %02X", DEVEUI[7 - i]);
	}
	xprintf("\n");

	xprintf("   Application EUI:");
	for (i = 0; i < 8; i++) {
		xprintf(" %02X", APPEUI[7 - i]);
	}
	xprintf("\n");

	xprintf("   Application Key:");
	for (i = 0; i < 16; i++) {
		xprintf(" %02X", APPKEY[i]);
	}
	xprintf("\n\n");

	xprintf("   PicoWAN Device Address: ");
	for (i = 0; i < 4; i++) {
		xprintf("%02X", PICO_DEVADDR[3 - i]);
	}
	xprintf("\n");

	xprintf("   PicoWAN Application Session Key:");
	for (i = 0; i < 16; i++) {
		xprintf(" %02X", PICO_APPSKEY[i]);
	}
	xprintf("\n");

	xprintf("   PicoWAN Network Session Key:");
	for (i = 0; i < 16; i++) {
		xprintf(" %02X", PICO_NWKSKEY[i]);
	}
	xprintf("\n\n");

	xprintf("   LoRaWAN Device Address: ");
	for (i = 0; i < 4; i++) {
		xprintf("%02X", LORA_DEVADDR[3 - i]);
	}
	xprintf("\n");

	xprintf("   LoRaWAN Application Session Key:");
	for (i = 0; i < 16; i++) {
		xprintf(" %02X", LORA_APPSKEY[i]);
	}
	xprintf("\n");

	xprintf("   LoRaWAN Network Session Key:");
	for (i = 0; i < 16; i++) {
		xprintf(" %02X", LORA_NWKSKEY[i]);
	}
	xprintf("\n");

	led_set(LED0, LED_OFF);
	led_set(LED1, LED_OFF);
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
