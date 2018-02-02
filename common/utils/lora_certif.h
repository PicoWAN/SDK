/*! \file
 * \brief Functions to use for the LoRaWAN certification
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

#ifndef _LORACERTIF_H_
#define _LORACERTIF_H_

#include <stdint.h>
#include <mac.h>

#define MAC_PORT_LORA_CERTIFICATION	224


/*!
 * \brief   LoRa certif structure.
 *
 * \details This structure allows the certification layer to start and stop the main application usecase.
 */
typedef struct {
	/*!
	 * \brief   LoRa Certification layer: stop usecase prototype.
	 */
	void (*lora_certif_stop_usecase)();

	/*!
	 * \brief   LoRa Certification layer: start usecase prototype.
	 */
	void (*lora_certif_start_usecase)();
} lora_certif_usecase_callbacks_t;

/*!
 * \brief   Handles the payload regarding certification.
 *
 * \details This function handles all certification related payload. It should be called only if the received data
 *          used the certification port.
 */
void lora_certif_handle_payload(mac_rx_info_t *info, uint8_t *DEVEUI, uint8_t *APPEUI, uint8_t *APPKEY);

/*!
 * \brief   Initializes the LoRaWAN Certification.
 *
 * \param   cb Callback to start or stop the main application usecase.
 *
 * \retval  int8_t 0: Success, -1: No callbacks set.
 */
int8_t lora_certif_init(lora_certif_usecase_callbacks_t *cb);


#endif /* _LORACERTIF_H_ */