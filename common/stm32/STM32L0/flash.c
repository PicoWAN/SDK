/*
 * flash - Eeprom/Flash access routines
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

#include <os.h>
#include <flash.h>
#include <xprintf.h>
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"

#if !defined(SECTION_PRIVATE_START) || !defined(SECTION_PRIVATE_LENGTH)
#error Missing SECTION_PRIVATE definition in Makefile !
#endif

#if !defined(SECTION_COMMON_START) || !defined(SECTION_COMMON_LENGTH)
#error Missing SECTION_COMMON definition in Makefile !
#endif

/* Dual bank eeprom (2 x 0xC00) */
#define EEPROM_BASE_ADDR			0x08080000
#define SECTION_PRIVATE_BASE_ADDR		(EEPROM_BASE_ADDR + SECTION_PRIVATE_START)
#define SECTION_PRIVATE_END_ADDR		(EEPROM_BASE_ADDR + SECTION_PRIVATE_START + SECTION_PRIVATE_LENGTH)
#define SECTION_COMMON_BASE_ADDR		(EEPROM_BASE_ADDR + SECTION_COMMON_START)
#define SECTION_COMMON_END_ADDR			(EEPROM_BASE_ADDR + SECTION_COMMON_START + SECTION_COMMON_LENGTH)


static HAL_StatusTypeDef generic_read(uint32_t addr, uint32_t *data, uint32_t length)
{
	__IO uint32_t *ptr = (__IO uint32_t *) addr;

	while (length-- > 0) {
		*(data++) = *(ptr++);
	}

	return HAL_OK;
}

static HAL_StatusTypeDef generic_write(uint32_t addr, uint32_t *data, uint32_t length)
{
	__IO uint32_t *ptr = (__IO uint32_t *) addr;
	HAL_StatusTypeDef status;

	while (length-- > 0) {
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) (ptr++), (uint32_t) *(data++));
		if (status != HAL_OK) {
			return status;
		}
	}

	return HAL_OK;
}

static uint32_t get_base_address(enum nv_section section)
{
	switch (section) {
		case NV_SECTION_PRIVATE:
			return SECTION_PRIVATE_BASE_ADDR;
		case NV_SECTION_COMMON:
			return SECTION_COMMON_BASE_ADDR;

		default:
			return 0;
	}
}

static uint32_t get_end_address(enum nv_section section)
{
	switch (section) {
		case NV_SECTION_PRIVATE:
			return SECTION_PRIVATE_END_ADDR;
		case NV_SECTION_COMMON:
			return SECTION_COMMON_END_ADDR;

		default:
			return 0;
	}
}

int8_t flash_data_nvread(enum nv_section section, uint32_t offset, uint32_t *data, uint32_t length)
{
	uint32_t base_addr;

	if (length == 0) {
		/* Nothing to do */
		return 0;
	}

	base_addr = get_base_address(section);
	if (base_addr == 0) {
		return -1;
	}

	if (!IS_FLASH_DATA_ADDRESS(base_addr + offset)) {
		return -1;
	}

	if (base_addr + offset + length * sizeof(uint32_t) > get_end_address(section)) {
		return -1;
	}

	if (!IS_FLASH_DATA_ADDRESS(base_addr + offset + (length - 1) * sizeof(uint32_t))) {
		return -1;
	}

	return ((generic_read(base_addr + offset, data, length) == HAL_OK) ? 0 : -1);

}

int8_t flash_data_nvwrite(enum nv_section section, uint32_t offset, uint32_t *data, uint32_t length)
{
	uint32_t base_addr;
	HAL_StatusTypeDef status = HAL_OK;

	if (length == 0) {
		/* Nothing to do */
		return 0;
	}

	base_addr = get_base_address(section);
	if (base_addr == 0) {
		return -1;
	}

	if (!IS_FLASH_DATA_ADDRESS(base_addr + offset)) {
		return -1;
	}

	if (base_addr + offset + length * sizeof(uint32_t) > get_end_address(section)) {
		return -1;
	}

	if (!IS_FLASH_DATA_ADDRESS(base_addr + offset + (length - 1) * sizeof(uint32_t))) {
		return -1;
	}

	HAL_FLASHEx_DATAEEPROM_Unlock();
	status = generic_write(base_addr + offset, data, length);
	HAL_FLASHEx_DATAEEPROM_Lock();

	return ((status == HAL_OK) ? 0 : -1);
}

void flash_data_nvcheck(uint32_t private_magic, uint32_t private_size)
{
	uint32_t recorded_section_size = 0;
	uint32_t magic;
	uint32_t zero = 0;
	int8_t status;
	uint32_t i;

	/* Check the common section */
	status = flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_MAGIC_OFFSET, &magic, NV_COMMON_MAGIC_SIZE);

	if (status != 0) {
		/* Failed to read the eeprom */
		return;
	}

	if (magic == NV_COMMON_MAGIC) {
		/* A migration could be needed */
		status = flash_data_nvread(NV_SECTION_COMMON, NV_COMMON_SECTION_LENGTH_OFFSET, &recorded_section_size, NV_COMMON_SECTION_LENGTH_SIZE);

		if (status != 0) {
			/* Failed to read the eeprom */
			return;
		}
	}

	if (recorded_section_size < NV_COMMON_SECTION_LENGTH) {
		/* Migration needed */
		xprintf("NV common data migration from %u to %u blocks\n", recorded_section_size, NV_COMMON_SECTION_LENGTH);

		/* Initialize the new fields to 0 */
		for (i = recorded_section_size; i < NV_COMMON_SECTION_LENGTH; i++) {
			flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_FIRST_FIELD_OFFSET + i * sizeof(uint32_t), &zero, 1);
		}

		recorded_section_size = NV_COMMON_SECTION_LENGTH;
		flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_SECTION_LENGTH_OFFSET, &recorded_section_size, NV_COMMON_SECTION_LENGTH_SIZE);
	}

	if (magic != NV_COMMON_MAGIC) {
		/* Set the proper magic */
		magic = NV_COMMON_MAGIC;
		flash_data_nvwrite(NV_SECTION_COMMON, NV_COMMON_MAGIC_OFFSET, &magic, NV_COMMON_MAGIC_SIZE);
	}

	/* Check the private section */
	recorded_section_size = 0;

	status = flash_data_nvread(NV_SECTION_PRIVATE, NV_PRIVATE_MAGIC_OFFSET, &magic, NV_PRIVATE_MAGIC_SIZE);

	if (status != 0) {
		/* Failed to read the eeprom */
		return;
	}

	if (magic == private_magic) {
		/* A migration could be needed */
		status = flash_data_nvread(NV_SECTION_PRIVATE, NV_PRIVATE_SECTION_LENGTH_OFFSET, &recorded_section_size,
					   NV_PRIVATE_SECTION_LENGTH_SIZE);

		if (status != 0) {
			/* Failed to read the eeprom */
			return;
		}
	}

	if (recorded_section_size < private_size) {
		/* Migration needed */
		xprintf("NV private data migration from %u to %u blocks\n", recorded_section_size, private_size);

		/* Initialize the new fields to 0 */
		for (i = recorded_section_size; i < private_size; i++) {
			flash_data_nvwrite(NV_SECTION_PRIVATE, NV_PRIVATE_FIRST_FIELD_OFFSET + i * sizeof(uint32_t), &zero, 1);
		}

		recorded_section_size = private_size;
		flash_data_nvwrite(NV_SECTION_PRIVATE, NV_PRIVATE_SECTION_LENGTH_OFFSET, &recorded_section_size, NV_PRIVATE_SECTION_LENGTH_SIZE);
	}

	if (magic != private_magic) {
		/* Set the proper magic */
		magic = private_magic;
		flash_data_nvwrite(NV_SECTION_PRIVATE, NV_PRIVATE_MAGIC_OFFSET, &magic, NV_PRIVATE_MAGIC_SIZE);
	}
}