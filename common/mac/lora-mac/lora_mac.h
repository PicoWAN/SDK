/*! \file
 * \brief Functions that are specific to LoRaWAN
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

#ifndef _LORA_MAC_H_
#define _LORA_MAC_H_

#include <stdint.h>


/*!
 * \brief   Enables/disables the ADR (Adaptive Data Rate).
 *
 * \param   enable 1 to enable ADR, 0 otherwise.
 */
void lora_mac_set_adr(uint8_t enable);

/*!
 * \brief   Sets the channels TX output power.
 *
 * \param   tx_power The TX output power. Allowed values are TX_POWER_16_DBM, TX_POWER_14_DBM,
 *                   TX_POWER_12_DBM, TX_POWER_10_DBM, TX_POWER_08_DBM, TX_POWER_06_DBM,
 *                   TX_POWER_04_DBM, TX_POWER_02_DBM.
 */
void lora_mac_set_channels_tx_power(int8_t tx_power);

/*!
 * \brief   Sets the channels datarate.
 *
 * \param   datarate The new datarate. Allowed values are:
 *                   * EU868 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_5, DR_6]
 */
void lora_mac_set_channels_datarate(int8_t datarate);

/*!
 * \brief   Enables/disables repeater support.
 *
 * \param   enable 1 to enable repeater support, 0 to disable it.
 */
void lora_mac_set_repeater_support(uint8_t enable);

/*!
 * \brief   Link margin reported after a LinkCheckReq command.
 *
 * \retval  uint8_t The margin in dB between 0 and 255.
 */
uint8_t lora_mac_get_last_demod_margin(void);

/*!
 * \brief   Number of gateways reported after a LinkCheckReq command.
 *
 * \retval  uint8_t The number of gateways.
 */
uint8_t lora_mac_get_last_nb_gateways(void);

/*!
 * \brief   Sets the datarate of the RX2 window.
 *
 * \param   dr The datarate to set (DR_n where n can be 0-6).
 */
void lora_mac_set_rx2_datarate(uint8_t dr);

/*!
 * \brief   Enables/disables the duty cycle limitation (for test purpose).
 *
 * \param   enable 1 to enable the limitation, 0 to disable it.
 */
void lora_mac_test_set_duty_cycle(uint8_t enable);

#endif /* _LORA_MAC_H_ */
