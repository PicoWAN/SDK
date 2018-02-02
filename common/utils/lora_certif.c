/*
 * lora_certif - Functions to use for the LoRaWAN certification
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

#include "lora_certif.h"
#include <lora_mac.h>
#include <os.h>
#include <xprintf.h>


#define APP_DATA_MAX_SIZE		64
/*!
 * LoRaWAN compliance tests support data
 */
struct compliance_test_s
{
	uint8_t running;
	uint8_t state;
	uint8_t is_tx_confirmed;
	uint8_t app_port;
	uint8_t app_data_size;
	uint8_t app_data_buffer[APP_DATA_MAX_SIZE];
	uint16_t downlink_counter;
	uint8_t link_check;
	uint8_t demod_margin;
	uint8_t nb_gateways;
} compliance_test;

static os_job_t txjob;
static os_job_t timeoutjob;

static uint8_t tx_confirmed = 1;
static uint8_t frames_test_without_downlink = 0;

static lora_certif_usecase_callbacks_t *usecase_callbacks = NULL;

static void tx_test_task(os_job_t* j);
static void timeout_test_task(os_job_t* j);


static void deactivation_test_mode(void)
{
	os_cancel_job(&timeoutjob);
	os_cancel_job(&txjob);
	compliance_test.running = 0;
	compliance_test.downlink_counter = 0;

	lora_mac_test_set_duty_cycle(1);
	if (usecase_callbacks->lora_certif_start_usecase) {
		usecase_callbacks->lora_certif_start_usecase();
	}
	xprintf("Test mode deactivated\n");
}

void lora_certif_handle_payload(mac_rx_info_t *info, uint8_t *DEVEUI, uint8_t *APPEUI, uint8_t *APPKEY)
{
	uint8_t i = 0;

	if (info->status < 0) {
		xprintf("TEST MODE RX error: %d\n", info->status);
		return;
	}

	if (compliance_test.running == 0) {
		if (info->payload_len == 4) {
			if ((info->payload[0] == 0x01) && (info->payload[1] == 0x01) &&
			    (info->payload[2] == 0x01) && (info->payload[3] == 0x01)) {
				compliance_test.running = 1;
				compliance_test.app_port = 224;
				compliance_test.app_data_size = 2;
				compliance_test.downlink_counter = 0;
				compliance_test.demod_margin = 0;
				compliance_test.nb_gateways = 0;
				compliance_test.link_check = 0;
				tx_confirmed = 1;
				frames_test_without_downlink = 0;
				compliance_test.app_data_buffer[0] = compliance_test.downlink_counter >> 8;
				compliance_test.app_data_buffer[1] = compliance_test.downlink_counter;
				lora_mac_test_set_duty_cycle(0);
				if (usecase_callbacks->lora_certif_stop_usecase) {
					usecase_callbacks->lora_certif_stop_usecase();
				}

				xprintf("Test mode activated\n");
				os_post_job(&txjob, tx_test_task);
			}
		}
	} else {
		// Test mode is activated
		compliance_test.downlink_counter++;
		compliance_test.app_data_size = 2;
		compliance_test.app_data_buffer[0] = compliance_test.downlink_counter >> 8;
		compliance_test.app_data_buffer[1] = compliance_test.downlink_counter;
		if (info->payload_len > 0) {
			// Compute the payload
			switch (info->payload[0]) {
				case 0:		// Deactivation test mode, back to normal application
					deactivation_test_mode();
					return;

				case 1:		// Send frame on port 224 with downlink_counter in the payload
					compliance_test.app_data_size = 2;
					compliance_test.app_data_buffer[0] = compliance_test.downlink_counter >> 8;
					compliance_test.app_data_buffer[1] = compliance_test.downlink_counter;
					break;

				case 2:		// Set to confirmed frames
					xprintf("Set to confirmed frames\n");
					tx_confirmed = 1;
					break;

				case 3:		// Set to unconfirmed frames
					xprintf("Set to unconfirmed frames\n");
					tx_confirmed = 0;
					break;

				case 4:		// Cryptography test
					xprintf("Cryptography test\n");
					compliance_test.app_data_size = info->payload_len;
					compliance_test.app_data_buffer[0] = 4;
					for (i = 1; i < info->payload_len; i++) {
						compliance_test.app_data_buffer[i] = info->payload[i] + 1;
					}
					break;

				case 5:		// Link Check Request
					xprintf("Checking the network\n");
					mac_network_available();
					break;

				case 6:		// Join Request
					xprintf("Join Request\n");
					compliance_test.downlink_counter = 0;
					compliance_test.running = 0;
					lora_mac_test_set_duty_cycle(1);
					mac_init_activation_on_air(DEVEUI, APPEUI, APPKEY);
					break;
			}
		}
	}
	// Stop test mode after 30 minutes without server response
	os_post_delayed_job(&timeoutjob, os_get_time()+ms2ostime(1800000), timeout_test_task);
	frames_test_without_downlink = 0;
}

static void tx_test_task(os_job_t* j)
{
	int8_t error = 0;

	// Do not send anymore if 192 frames are sent without server response
	if (frames_test_without_downlink <= 192) {
		os_post_delayed_job(&txjob, os_get_time()+ms2ostime(5000), tx_test_task);
	} else {
		deactivation_test_mode();
		return;
	}
	frames_test_without_downlink++;

	if (tx_confirmed) {
		error = mac_send_when_possible_confirmed(compliance_test.app_port, compliance_test.app_data_buffer, compliance_test.app_data_size, 8, DL_ENABLED);
		if (error < 0) {
			xprintf("TEST MODE: The frame cannot be sent: %d\n", error);
		}
	} else {
		error = mac_send_when_possible(compliance_test.app_port, compliance_test.app_data_buffer, compliance_test.app_data_size, DL_ENABLED);
		if (error < 0) {
			xprintf("TEST MODE: The frame cannot be sent: %d\n", error);
		}
	}
	compliance_test.app_data_size = 2;
	compliance_test.app_data_buffer[0] = compliance_test.downlink_counter >> 8;
	compliance_test.app_data_buffer[1] = compliance_test.downlink_counter;
}

static void timeout_test_task(os_job_t* j)
{
	deactivation_test_mode();
}

int8_t lora_certif_init(lora_certif_usecase_callbacks_t *cb)
{
	if (cb == NULL) {
		return -1;
	} else {
		usecase_callbacks = cb;
		return 0;
	}
}
