/*! \file
 * \brief Eeprom/Flash access routines
 *
 * NOTE: The eeprom is divided in 2 sections:
 * - one private for the usecase/application (will be erased if a different application is flashed)
 * - one common for the system (will be kept/migrated if a different application is flashed)
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

#ifndef _FLASH_H_
#define _FLASH_H_

#include <stdint.h>

/*
 * 2 logical sections: one private for the usecase/application (will be erased if a different application is flashed)
 *                     one common for the system (will be kept/migrated if a different application is flashed)
 */
enum nv_section {
	NV_SECTION_PRIVATE,
	NV_SECTION_COMMON,
};

/*
 * Definition of the common section structure
 * A field has a minimum size of 1 block = sizeof(uint32_t) bytes
 * Each time the structure is extended, NV_COMMON_SECTION_LENGTH must be updated
 */
#define NV_COMMON_MAGIC						0x442A6B6E

#define NV_COMMON_MAGIC_OFFSET					0
#define NV_COMMON_MAGIC_SIZE					1 /* in blocks */
#define NV_COMMON_SECTION_LENGTH_OFFSET				(NV_COMMON_MAGIC_OFFSET + NV_COMMON_MAGIC_SIZE * sizeof(uint32_t))
#define NV_COMMON_SECTION_LENGTH_SIZE				1 /* in blocks */
#define NV_COMMON_FIRST_FIELD_OFFSET				(NV_COMMON_SECTION_LENGTH_OFFSET + NV_COMMON_SECTION_LENGTH_SIZE * sizeof(uint32_t))

/* The list of available fields starts here (the offsets must be multiples of sizeof(uint32_t)) */
#define NV_COMMON_PICOWAN_DEVICE_ADDRESS_OFFSET			NV_COMMON_FIRST_FIELD_OFFSET
#define NV_COMMON_PICOWAN_DEVICE_ADDRESS_SIZE			1 /* in blocks */
#define NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_OFFSET		(NV_COMMON_PICOWAN_DEVICE_ADDRESS_OFFSET + NV_COMMON_PICOWAN_DEVICE_ADDRESS_SIZE * sizeof(uint32_t))
#define NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_SIZE		4 /* in blocks */
#define NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_OFFSET	(NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_OFFSET + NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_SIZE * sizeof(uint32_t))
#define NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_SIZE		4 /* in blocks */
#define NV_COMMON_DEVICE_EUI_OFFSET				(NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_OFFSET + NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_SIZE * sizeof(uint32_t))
#define NV_COMMON_DEVICE_EUI_SIZE				2 /* in blocks */
#define NV_COMMON_APPLICATION_EUI_OFFSET			(NV_COMMON_DEVICE_EUI_OFFSET + NV_COMMON_DEVICE_EUI_SIZE * sizeof(uint32_t))
#define NV_COMMON_APPLICATION_EUI_SIZE				2 /* in blocks */
#define NV_COMMON_APPLICATION_KEY_OFFSET			(NV_COMMON_APPLICATION_EUI_OFFSET + NV_COMMON_APPLICATION_EUI_SIZE * sizeof(uint32_t))
#define NV_COMMON_APPLICATION_KEY_SIZE				4 /* in blocks */
#define NV_COMMON_LORA_DEVICE_ADDRESS_OFFSET			(NV_COMMON_APPLICATION_KEY_OFFSET + NV_COMMON_APPLICATION_KEY_SIZE * sizeof(uint32_t))
#define NV_COMMON_LORA_DEVICE_ADDRESS_SIZE			1 /* in blocks */
#define NV_COMMON_LORA_NETWORK_SESSION_KEY_OFFSET		(NV_COMMON_LORA_DEVICE_ADDRESS_OFFSET + NV_COMMON_LORA_DEVICE_ADDRESS_SIZE * sizeof(uint32_t))
#define NV_COMMON_LORA_NETWORK_SESSION_KEY_SIZE			4 /* in blocks */
#define NV_COMMON_LORA_APPLICATION_SESSION_KEY_OFFSET		(NV_COMMON_LORA_NETWORK_SESSION_KEY_OFFSET + NV_COMMON_LORA_NETWORK_SESSION_KEY_SIZE * sizeof(uint32_t))
#define NV_COMMON_LORA_APPLICATION_SESSION_KEY_SIZE		4 /* in blocks */
#define NV_COMMON_LORA_UPLINK_COUNTER_OFFSET			(NV_COMMON_LORA_APPLICATION_SESSION_KEY_OFFSET + NV_COMMON_LORA_APPLICATION_SESSION_KEY_SIZE * sizeof(uint32_t))
#define NV_COMMON_LORA_UPLINK_COUNTER_SIZE			1 /* in blocks */
#define NV_COMMON_LORA_DOWNLINK_COUNTER_OFFSET			(NV_COMMON_LORA_UPLINK_COUNTER_OFFSET + NV_COMMON_LORA_UPLINK_COUNTER_SIZE * sizeof(uint32_t))
#define NV_COMMON_LORA_DOWNLINK_COUNTER_SIZE			1 /* in blocks */

/* Size of the section in blocks, starting from FIRST_FIELD */
#define NV_COMMON_SECTION_LENGTH				NV_COMMON_PICOWAN_DEVICE_ADDRESS_SIZE + \
								NV_COMMON_PICOWAN_NETWORK_SESSION_KEY_SIZE + \
								NV_COMMON_PICOWAN_APPLICATION_SESSION_KEY_SIZE + \
								NV_COMMON_DEVICE_EUI_SIZE + \
								NV_COMMON_APPLICATION_EUI_SIZE + \
								NV_COMMON_APPLICATION_KEY_SIZE + \
								NV_COMMON_LORA_DEVICE_ADDRESS_SIZE + \
								NV_COMMON_LORA_NETWORK_SESSION_KEY_SIZE + \
								NV_COMMON_LORA_APPLICATION_SESSION_KEY_SIZE + \
								NV_COMMON_LORA_UPLINK_COUNTER_SIZE + \
								NV_COMMON_LORA_DOWNLINK_COUNTER_SIZE

/*
 * Definition of the private section structure
 * A field has a minimum size of 1 block = sizeof(uint32_t) bytes
 * Each time the structure is extended, NV_PRIVATE_SECTION_LENGTH must be updated
 */

#define NV_PRIVATE_MAGIC_OFFSET					0
#define NV_PRIVATE_MAGIC_SIZE					1 /* in blocks */
#define NV_PRIVATE_SECTION_LENGTH_OFFSET			(NV_PRIVATE_MAGIC_OFFSET + NV_PRIVATE_MAGIC_SIZE * sizeof(uint32_t))
#define NV_PRIVATE_SECTION_LENGTH_SIZE				1 /* in blocks */
#define NV_PRIVATE_FIRST_FIELD_OFFSET				(NV_PRIVATE_SECTION_LENGTH_OFFSET + NV_PRIVATE_SECTION_LENGTH_SIZE * sizeof(uint32_t))


/*!
 * \brief   Reads 32-bit long blocks from the Eeprom using PicoWAN-SDK's Eeprom layout.
 *
 * \note    Data from NV_SECTION_COMMON are kept when different applications are flashed into the board, while data from
 *          NV_SECTION_PRIVATE are lost everytime a new application is flashed.
 *
 * \param   section The section to read from. This can be NV_SECTION_COMMON for general data related to MACs, or NV_SECTION_PRIVATE
 *                  for data related to the main application.
 * \param   offset The offset of the field to read. This must come from the list of available fields for the section, ex NV_COMMON_APPLICATION_EUI_OFFSET.
 * \param   data A pointer to the buffer that will hold the data.
 * \param   length The length of the data to read in 32-bit long blocks, ex NV_COMMON_APPLICATION_EUI_SIZE.
 *
 * \retval  int8_t -1 if it failed, 0 otherwise.
 */
int8_t flash_data_nvread(enum nv_section section, uint32_t offset, uint32_t *data, uint32_t length);

/*!
 * \brief   Writes 32-bit long blocks to the Eeprom using PicoWAN-SDK's Eeprom layout.
 *
 * \note    Data from NV_SECTION_COMMON are kept when different applications are flashed into the board, while data from
 *          NV_SECTION_PRIVATE are lost everytime a new application is flashed.
 *
 * \param   section The section to write to. This can be NV_SECTION_COMMON for general data related to MACs, or NV_SECTION_PRIVATE
 *                  for data related to the main application.
 * \param   offset The offset of the field to write. This must come from the list of available fields for the section, ex NV_COMMON_APPLICATION_EUI_OFFSET.
 * \param   data A pointer to the buffer that holds the data to write.
 * \param   length The length of the data to write in 32-bit long blocks, ex NV_COMMON_APPLICATION_EUI_SIZE.
 *
 * \retval  int8_t -1 if it failed, 0 otherwise.
 */
int8_t flash_data_nvwrite(enum nv_section section, uint32_t offset, uint32_t *data, uint32_t length);

/*!
 * \brief   Initializes and checks the Eeprom format.
 *
 * \details This functions performs a sanity check on the Eeprom sections, initializing or migrating them if necessary. Keep in mind that
 *          different applications must have different private magics to be sure that flashing a new application will re-initialize the
 *          private section.
 *
 * \param   private_magic The magic of the private section (must be unique per application)
 * \param   private_size The total size in 32-bit long blocks of the private section. This size can be increases in case new fields are added to the private section, however decreasing it is not allowed.
 */
void flash_data_nvcheck(uint32_t private_magic, uint32_t private_size);

#endif
