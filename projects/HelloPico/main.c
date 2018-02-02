/*
 * HelloPico - A very simple application using the PicoWAN SDK
 *
 * NOTE: When the user presses a button, HelloPico sends a message using the
 *       PicoWAN network. It should be used as an example/starting point
 *       by anyone wishing to develop an application based on the PicoWAN/LoRaWAN
 *       dual stack SDK.
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
#include <button.h>
#include <flash.h>
#include <led.h>
#include <lora_certif.h>
#include <os.h>
#include <radio.h>
#include <spi.h>
#include <stdint.h>
#include <usart.h>
#ifdef HAS_LIS2DE12_ACCELEROMETER
#include <accelerometer.h>
#include <lis2de12.h>
#endif
#include <mac.h>
#include <xprintf.h>

#define BUTTON0				0

#define LED0				0
#define LED1				1

/* Define this only if you plan to do a LoRaWAN application that needs to be certified */
#define LORA_CERTIFICATION_ENABLED


#ifdef CFG_picotag_board
/* These informations are read by the PicoSmartTAG bootloader */
typedef struct {
	uint8_t version;
	uint8_t type; /* must be 253 */
} fw_info_t;

/* Firmware info (will be read by the bootloader) */
static __attribute__((used, section(".FWVersion"))) fw_info_t firmware_info = {
	.version = 42,
	.type = 253,
};
#endif

/* Device EUI (provided by Archos) */
static __attribute__((aligned (4))) uint8_t DEVEUI[8];

/* Application EUI */
static __attribute__((aligned (4))) uint8_t APPEUI[8];

/* Application key */
static __attribute__((aligned (4))) uint8_t APPKEY[16];

/* Device Address for PicoWAN (provided by Archos) */
static uint32_t PICO_DEVADDR;

/* Device specific Application session key for PicoWAN */
static __attribute__((aligned (4))) uint8_t PICO_APPSKEY[16];

/* Device specific Network session key for PicoWAN (provided by Archos) */
static __attribute__((aligned (4))) uint8_t PICO_NWKSKEY[16];

/* Device Address for LoRaWAN */
static uint32_t LORA_DEVADDR;

/* Device specific Application session key for LoRaWAN */
static __attribute__((aligned (4))) uint8_t LORA_APPSKEY[16];

/* Device specific Network session key for LoRaWAN */
static __attribute__((aligned (4))) uint8_t LORA_NWKSKEY[16];

static uint8_t data[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};

static uint8_t is_usecase_started = 0;

static os_job_t turn_off_leds_job;
static os_job_t send_job;
static os_job_t usecase_job;

static void rxdone_cb(mac_rx_info_t *info);
static void txdone_cb(mac_tx_info_t *info);
static void network_cb(mac_network_info_t *info);
static void mac_state_cb(mac_state_t state);

static uint8_t batt_measure_value_cb(void);

static uint32_t get_uplink_counter_cb(void);
static void save_uplink_counter_cb(uint32_t uplink_counter);
static uint32_t get_downlink_counter_cb(void);
static void save_downlink_counter_cb(uint32_t downlink_counter);

static void start_usecase_cb(os_job_t* j);
static void stop_usecase_cb(os_job_t* j);

static mac_message_callbacks_t message_cbs = {
	.mac_tx_done = txdone_cb,
	.mac_rx_done = rxdone_cb,
	.mac_network_state = network_cb,
	.mac_state = mac_state_cb,
};

static mac_battery_callback_t battery_cb = {
	.get_battery_level = batt_measure_value_cb,
};

static mac_flash_callback_t flash_cb = {
	.get_uplink_counter = get_uplink_counter_cb,
	.save_uplink_counter = save_uplink_counter_cb,
	.get_downlink_counter = get_downlink_counter_cb,
	.save_downlink_counter = save_downlink_counter_cb,
};

#ifdef LORA_CERTIFICATION_ENABLED
static lora_certif_usecase_callbacks_t usecase_cbs = {
	.lora_certif_stop_usecase = stop_usecase_cb,
	.lora_certif_start_usecase = start_usecase_cb,
};
#endif


static void rxdone_cb(mac_rx_info_t *info)
{
	uint8_t i = 0;

#ifdef LORA_CERTIFICATION_ENABLED
	mac_type_t current_mac = mac_get_stack();

	/* Handover the payload to the LoRa certification code if needed */
	if (current_mac == LORAMAC && info->port == MAC_PORT_LORA_CERTIFICATION) {
			lora_certif_handle_payload(info, DEVEUI, APPEUI, APPKEY);
			return;
	}
#endif

	if (info->status == MAC_INFO_STATUS_OK) {
		if (is_usecase_started) {
			xprintf("RX done:");
			for (i = 0; i < info->payload_len; i++) {
				xprintf(" %02X", info->payload[i]);
			}
			xprintf("\n");
		}
	} else if (info->status < 0) {
		xprintf("RX error: %d\n", info->status);
	}
}

static void turn_off_leds_task(os_job_t *j)
{
	led_set(LED0, LED_OFF);
	led_set(LED1, LED_OFF);
}

static void txdone_cb(mac_tx_info_t *info)
{
	if (!is_usecase_started) {
		return;
	}

	if (info->status == MAC_INFO_STATUS_OK) {
		xprintf("Frame %u sent\n", data[0]);
		led_set(LED1, LED_ON);
		led_set(LED0, LED_OFF);
		os_post_delayed_job(&turn_off_leds_job, os_get_time() + ms2ostime(5000), turn_off_leds_task);
	} else {
		xprintf("Frame %u not sent\n", data[0]);
		led_set(LED1, LED_OFF);
		led_set(LED0, LED_ON);
		os_post_delayed_job(&turn_off_leds_job, os_get_time() + ms2ostime(5000), turn_off_leds_task);
	}
}

static void network_cb(mac_network_info_t *info)
{
	if (!is_usecase_started) {
		return;
	}

	switch (info->status) {
		case NETWORK_UNAVAILABLE:
			xprintf("NETWORK_UNAVAILABLE\n");
			break;

		case NETWORK_AVAILABLE:
			xprintf("NETWORK_AVAILABLE\n");
			break;

		case NETWORK_UNKNOWN:
			xprintf("NETWORK_UNKNOWN\n");
			break;

		default:
			break;
	}
}

static void mac_state_cb(mac_state_t state)
{
	if (!is_usecase_started) {
		return;
	}

	switch (state) {
		case MAC_STATE_IDLE:
			xprintf("MAC_STATE_IDLE\n");
			break;

		case MAC_STATE_SEARCHING_NETWORK:
			xprintf("MAC_STATE_SEARCHING_NETWORK\n");
			led_blink(LED1, ON_OFF_SLOTS_PERIOD_2_DUTY_50, 0);
			break;

		case MAC_STATE_SENDING_DATA:
			xprintf("MAC_STATE_SENDING_DATA\n");
			led_blink(LED1, ON_OFF_SLOTS_PERIOD_4_DUTY_50, 0);
			break;

		case MAC_STATE_WAITING_ACK:
			xprintf("MAC_STATE_WAITING_ACK\n");
			led_blink(LED1, ON_OFF_SLOTS_PERIOD_8_DUTY_50, 0);
			break;

		case MAC_STATE_WAITING_RETRY:
			xprintf("MAC_STATE_WAITING_RETRY\n");
			led_blink(LED1, ON_OFF_SLOTS_PERIOD_16_DUTY_50, 0);
			break;

		case MAC_STATE_WAITING_DATA:
			xprintf("MAC_STATE_WAITING_DATA\n");
			break;

		default:
			break;
	}
}

static uint8_t batt_measure_value_cb(void)
{
	return 0;
}

static uint32_t get_uplink_counter_cb(void)
{
	/* We do not store the counter, so start at 1 */
	return 1;
}

static void save_uplink_counter_cb(uint32_t uplink_counter)
{
	xprintf("Save uplink counter %u\n", uplink_counter);
}

static uint32_t get_downlink_counter_cb(void)
{
	/* We do not store the counter, so start at 1 */
	return 1;
}

static void save_downlink_counter_cb(uint32_t downlink_counter)
{
	xprintf("Save downlink counter %u\n", downlink_counter);
}

static void send_task(os_job_t *j)
{
	/* First byte is a counter */
	data[0]++;

	/* Cancel any pending turn_off_leds task, and turn off the LEDs right away */
	os_cancel_job(&turn_off_leds_job);
	turn_off_leds_task(NULL);

	if (mac_send_when_possible_confirmed(MAC_PORT_FIRST_APP, data, sizeof(data), 2, DL_DISABLED) < 0) {
		xprintf("The frame cannot be sent\n");
	}
}

void button_handler(uint8_t num, enum button_press_duration duration)
{
	xprintf("Button %d detected !\n", num);

	if (!is_usecase_started) {
		return;
	}

	if (num == BUTTON0 && duration == BUTTON_SHORT_PRESS) {
		/* Send a message */
		os_post_job(&send_job, send_task);
	}
}

#ifdef HAS_LIS2DE12_ACCELEROMETER
void accelerometer_handler(uint8_t num)
{
	if (num == ACC1_INT) {
		xprintf("Movement detected !\n");
		lis2de12_handle_int(LIS2DE12_INT1);
	} else if (num == ACC2_INT) {
		lis2de12_handle_int(LIS2DE12_INT2);
	}
}
#endif

static void start_usecase_cb(os_job_t* j)
{
#ifdef HAS_LIS2DE12_ACCELEROMETER
	/* Initialize the accelerometer */
	i2c_power_up(ACC_I2C_PORT);
	lis2de12_init(ACC_I2C_PORT);

	accelerometer_init(&accelerometer_handler);
#endif

	is_usecase_started = 1;
}

static void stop_usecase_cb(os_job_t* j)
{
	is_usecase_started = 0;

#ifdef HAS_LIS2DE12_ACCELEROMETER
	/* Disable the accelerometer */
	accelerometer_init(NULL);

	lis2de12_deinit();
	i2c_power_down(ACC_I2C_PORT);

#endif
	/* Cancel all possible pending jobs */
	os_cancel_job(&send_job);
}

static void main_init(os_job_t *j)
{
	/* Initialize the buttons */
	button_init(&button_handler);

	/* Initialize the LEDs */
	led_init();

	/* Sanity check of the Eeprom data (no private section defined) */
	flash_data_nvcheck(0, 0);

	/* Get the keys/IDs from the flash */
	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_DEVICE_EUI_OFFSET, (uint32_t *) DEVEUI, NV_COMMON_DEVICE_EUI_SIZE);
	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_APPLICATION_EUI_OFFSET, (uint32_t *) APPEUI, NV_COMMON_APPLICATION_EUI_SIZE);
	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_APPLICATION_KEY_OFFSET, (uint32_t *) APPKEY, NV_COMMON_APPLICATION_KEY_SIZE);

	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_PICOWAN_DEVICE_ADDRESS_OFFSET, &PICO_DEVADDR, NV_COMMON_PICOWAN_DEVICE_ADDRESS_SIZE);
	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_OFFSET, (uint32_t *) PICO_NWKSKEY,
			  NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_SIZE);
	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_OFFSET, (uint32_t *) PICO_APPSKEY,
			  NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_SIZE);

	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_LORA_DEVICE_ADDRESS_OFFSET, (uint32_t *) &LORA_DEVADDR, NV_COMMON_LORA_DEVICE_ADDRESS_SIZE);
	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_LORA_NETWORK_SESSION_KEY_OFFSET, (uint32_t *) LORA_NWKSKEY,
			  NV_COMMON_LORA_NETWORK_SESSION_KEY_SIZE);
	flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_LORA_APPLICATION_SESSION_KEY_OFFSET, (uint32_t *) LORA_APPSKEY,
			  NV_COMMON_LORA_APPLICATION_SESSION_KEY_SIZE);

	/* Initialize the MAC */
	mac_init(PICOMAC, &message_cbs, &battery_cb, &flash_cb);
	mac_set_device_class(CLASS_A);
	mac_init_activation_personalization(0, PICO_DEVADDR, PICO_NWKSKEY, PICO_APPSKEY);

#ifdef LORA_CERTIFICATION_ENABLED
	/* LoRaWAN certification initilization (used only if you switch to LORAMAC) */
	lora_certif_init(&usecase_cbs);
#endif

	/* Start the usecase */
	os_post_job(&usecase_job, start_usecase_cb);

	xprintf("Device initialized\n");
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
