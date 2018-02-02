/*
 * osal - OS abstraction layer for Libmultimac
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

#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include "osal.h"

#include <aes.h>
#include <boards.h>
#include <system.h>
#include <os.h>
#include <radio.h>
#include <utils.h>
#include <xprintf.h>

/*
 * Maximum STM32 and SX1276 Wake-up Latency
 */
#define RADIO_WAKEUP_LATENCY	3000	// us


#define container_of(ptr, type, member) ((type *)((char *)(ptr)-(char *)(&((type *)0)->member)))

/*
 * AES computation context variable
 */
static aes_context AesContext;

/*
 * Callback that will be called by the radio after an INT
 */
static void (*radio_callback)(void) = NULL;


/*!
 * \brief  Returns the elapsed time since the device has been powered-up.
 * \retval osal_time_t: current time in us.
 */
osal_time_t osal_get_time(void)
{
	return (osal_time_t) ostime2us(os_get_time());
}

/*!
 * \brief  Returns the maximum time the radio can take before being actually
 *         receiving (usually sleep to RX transition).
 * \note   This is around 3000 us for the SX1276.
 * \retval osal_time_t: latency in us.
 */
osal_time_t osal_get_radio_wakeup_latency(void)
{
	return (osal_time_t) RADIO_WAKEUP_LATENCY;
}

/*!
 * \brief  Returns an offset that will be removed to the deadlines of timing
 *         critical tasks in case the OS scheduler is not able to start the
 *         tasks on-time (for instance, in case of wakeup latency from a low
 *         power state).
 * \note   Ideally, the OS scheduler should handle any CPU latency, meaning
 *         this should be set to zero.
 * \retval osal_time_t: latency in us.
 */
osal_time_t osal_get_additional_cpu_wakeup_latency(void)
{
	return (osal_time_t) 0;
}

static void osal_job_handler(os_job_t *j)
{
	osal_job_t *job = container_of((const uint8_t(*)[32]) j, osal_job_t, data);
	job->cb(job);
}

/*!
 * \brief  Queue a job for immediate execution (but after any other job
 *         already in the queue).
 * \note
 *         - The immediate queue has priority over the scheduled queue.
 *         - All jobs are statically allocated, so their pointers can be used
 *           to identify them.
 *         - If the job is already queued, this job must be moved to the end
 *           of the queue.
 *         - The osal_job_t object has a data field of 32 bytes that can be
 *           used to store any relevant information.
 * \param  job: pointer to the job to queue.
 * \param  cb: pointer to the function to call once the job must be executed.
 * \retval None
 */
void osal_post_job(osal_job_t *job, void (*cb)(osal_job_t *))
{
	job->cb = cb;
	os_post_job((os_job_t *) &job->data, osal_job_handler);
}

/*!
 * \brief  Queue a job for scheduled execution.
 * \note
 *         - The immediate queue has priority over the scheduled queue.
 *         - All jobs are statically allocated, so their pointers can be used
 *           to identify them.
 *         - If the job is already queued, its deadline must be updated to
 *           the new value.
 *         - The osal_job_t object has a data field of 32 bytes that can be
 *           used to store any relevant information.
 * \param  job: pointer to the job to queue.
 * \param  time: timestamp at which the job must be executed.
 * \param  cb: pointer to the function to call once the job must be executed.
 * \retval None
 */
void osal_post_delayed_job(osal_job_t *job, osal_time_t time, void (*cb)(osal_job_t *))
{
	job->cb = cb;
	os_post_delayed_job((os_job_t *) &job->data, (os_time_t) us2ostime(time), osal_job_handler);
}

/*!
 * \brief  Cancel a queued job.
 * \note
 *         - All jobs are statically allocated, so their pointers can be used
 *           to identify them.
 *         - The osal_job_t object has a data field of 32 bytes that can be
 *           used to store any relevant information.
 * \param  job: pointer to the job to cancel.
 * \retval None
 */
void osal_cancel_job(osal_job_t *job)
{
	os_cancel_job((os_job_t *) &job->data);
}


/*!
 * \brief  Prevents the device from going to lowpower and/or sleep mode (if the
 *         device supports a lowpower and/or a sleep mode inducing a latency).
 * \note   Once called, the device must work as fast as possible until the
 *         osal_lowpower_sleep_unlock() counterpart function is called.
 * \retval None
 */
void osal_lowpower_sleep_lock(void)
{
	os_block_powersave();
}

/*!
 * \brief  Allows the device to go to lowpower and/or sleep mode (if the
 *         device supports a lowpower and/or a sleep mode inducing a latency).
 * \note   Once called, the device has no more constraint related to speed or
 *         latency.
 * \retval None
 */
void osal_lowpower_sleep_unlock(void)
{
	os_unblock_powersave();
}


/*!
 * \brief  Copies a memory area.
 * \param  dst: destination pointer.
 * \param  src: source pointer.
 * \param  size: size of the memory area to copy.
 * \retval None
 */
void osal_memcpy(void *dst, const void *src, uint16_t size)
{
	memcpy(dst, src, size);
}

/*!
 * \brief  Copies a memory area in reverse order (last byte of the source area
 *         copied to the first byte of the destination area).
 * \param  dst: destination pointer.
 * \param  src: source pointer.
 * \param  size: size of the memory area to copy.
 * \retval None
 */
void osal_memcpy_reverse(void *dst, const void *src, uint16_t size)
{
	utils_memcpy_r((uint8_t *) dst, (uint8_t *) src, size);
}

/*!
 * \brief  Fills a memory area with a constant value.
 * \param  dst: destination pointer.
 * \param  byte: value used to fill the area.
 * \param  size: size of the memory area to fill.
 * \retval None
 */
void osal_memset(void *dst, uint8_t byte, uint16_t size)
{
	memset(dst, byte, size);
}


/*!
 * \brief  Returns a random byte.
 * \retval uint8_t: random byte.
 */
uint8_t osal_rand8(void)
{
	return radio_rand1();
}

/*!
 * \brief  Returns a random half-word.
 * \retval uint16_t: random half-word.
 */
uint16_t osal_rand16(void)
{
	return ((radio_rand1() << 8) | radio_rand1());
}


/*!
 * \brief  Sets the AES key to use for encryption and decryption.
 * \note
 *         - This key stays valid for any call to osal_aes_encrypt() or
 *           osal_aes_decrypt() until a new key is set.
 *         - The key will always be 16 bytes long.
 * \param  key: AES key to set.
 * \retval None
 */
void osal_aes_setkey(const uint8_t *key)
{
	aes_set_key(key, 16, &AesContext);
}

/*!
 * \brief  Encrypts a buffer using the AES algorithm (ECB mode).
 * \note
 *         - The last key set with osal_aes_setkey() must be used.
 *         - The size of the in and out buffers will always be a multiple of
 *           16 bytes.
 * \param  in: buffer to encrypt.
 * \param  out: pointer to the memory area where the encrypted buffer will be
 *              stored.
 * \param  len: length of the in and out buffers.
 * \retval None
 */
void osal_aes_encrypt(const uint8_t *in, uint8_t *out, uint32_t len)
{
	aes_encrypt_all(in, out, len, &AesContext);
}

/*!
 * \brief  Decrypts a buffer using the AES algorithm (ECB mode).
 * \note
 *         - The last key set with osal_aes_setkey() must be used.
 *         - The size of the in and out buffers will always be a multiple of
 *           16 bytes.
 * \param  in: buffer to encrypt.
 * \param  out: pointer to the memory area where the decrypted buffer will be
 *              stored.
 * \param  len: length of the in and out buffers.
 * \retval None
 */
void osal_aes_decrypt(const uint8_t *in, uint8_t *out, uint32_t len)
{
	aes_decrypt_all(in, out, len, &AesContext);
}


/*!
 * \brief  Returns the maximum available power.
 * \note   Without additional PA, the SX1276 maximum power is 20 dBm using the
 *         PA_BOOST pin, and 14 dBm using the RFO pin.
 * \retval int8_t: maximum available power.
 */
int8_t osal_get_max_power(void)
{
	return MAX_RF_POWER;
}

/*!
 * \brief  Returns the minimum available power.
 * \note   Without additional PA, the SX1276 minimum power is 2 dBm using the
 *         PA_BOOST pin, and 0 dBm using the RFO pin.
 * \retval int8_t: minimum available power.
 */
int8_t osal_get_min_power(void)
{
	return MIN_RF_POWER;
}

/*!
 * \brief  Returns the antenna gain in dBi.
 * \note   Antenna gain rounded down.
 * \retval int8_t: antenna gain in dBi.
 */
int8_t osal_get_antenna_gain(void)
{
	return ANTENNA_GAIN;
}

/*!
 * \brief  Checks if a given SF is supported.
 * \note   SF12 at lower bandwidth requires good frequency stability, so usually a TCXO is needed.
 * \retval int8_t: 1 if supported, 0 if not.
 */
int8_t osal_radio_SF_supported(enum osal_sf_t sf)
{
	switch (sf) {
		case OSAL_SF6:
			return 1;
		case OSAL_SF7:
			return 1;
		case OSAL_SF8:
			return 1;
		case OSAL_SF9:
			return 1;
		case OSAL_SF10:
			return 1;
		case OSAL_SF11:
			return 1;
		case OSAL_SF12:
#ifdef HAS_TCXO
			return 1;
#else
			// first boards without tcxo have a clock drift issue using SF12 and 20dbm power
			// so we do not use SF12
			return 0;
#endif
		default:
			return 0;
	}
}

/*!
 * \brief  Tells if frequency calibration should be used.
 * \note   This will detect and correct any frequency offset introduced by
 *         the XTAL used for RF. It should not be needed if the board has a
 *         TCXO.
 * \retval int8_t: 1 if enabled, 0 if not.
 */
int8_t osal_is_freq_calibration_enabled(void)
{
#ifdef HAS_TCXO
	return 0;
#else
	return 1;
#endif
}

/*!
 * \brief  Sets the radio parameters.
 * \note
 *         - The osal_radio_params_t structure is explained in osal.h.
 *         - These parameters must stay valid until new ones are set.
 * \param  params: parameters to set.
 * \retval None
 */
void osal_set_radio_params(osal_radio_params_t *params)
{
	enum _cr_t cr = CR_4_5;
	enum _sf_t sf = SF7;
	enum _bw_t bw = BW125;

	switch (params->cr) {
		case OSAL_CR4_5:
			cr = CR_4_5;
			break;

		case OSAL_CR4_6:
			cr = CR_4_6;
			break;

		case OSAL_CR4_7:
			cr = CR_4_7;
			break;

		case OSAL_CR4_8:
			cr = CR_4_8;
			break;
	}

	switch (params->sf) {
		case OSAL_SF6:
			sf = SF6;
			break;

		case OSAL_SF7:
			sf = SF7;
			break;

		case OSAL_SF8:
			sf = SF8;
			break;

		case OSAL_SF9:
			sf = SF9;
			break;

		case OSAL_SF10:
			sf = SF10;
			break;

		case OSAL_SF11:
			sf = SF11;
			break;

		case OSAL_SF12:
			sf = SF12;
			break;
	}

	switch (params->bw) {
		case OSAL_BW62_5:
			bw = BW62_5;
			break;

		case OSAL_BW125:
			bw = BW125;
			break;

		case OSAL_BW250:
			bw = BW250;
			break;

		case OSAL_BW500:
			bw = BW500;
			break;
	}

	RADIO.freq = params->freq;
	RADIO.rxtime = us2ostime(params->rxtime);
	RADIO.rxsyms = params->rxsyms;
	RADIO.txpow = params->txpow;
	RADIO.preamble_len = params->preamble_len;
	RADIO.sync_word = params->sync_word;
	RADIO.invert_iq = ((params->iq == OSAL_IQ_NORMAL) ? 0 : 1);
	RADIO.ppm_offset = params->ppm_offset;
	RADIO.rps = MAKERPS(sf, bw, cr, params->ih, ((params->crc == OSAL_CRC_OFF) ? 1 : 0));
}

static void osal_radio_handler(os_job_t *j)
{
	if (radio_callback != NULL) {
		radio_callback();
	}
}

/*!
 * \brief  Sets a radio callback that will be called once the radio has
 *         received or transmitted a buffer.
 * \param  cd: pointer to the callback.
 * \retval None
 */
void osal_set_radio_callback(void (*cb)(void))
{
	radio_callback = cb;
	RADIO.osjob.callback = osal_radio_handler;
}

/*!
 * \brief  Clears the radio callback.
 * \note   This function must ensure that no further call to the radio
 *         callback will be made, including any call that could have been
 *         already scheduled.
 * \retval None
 */
void osal_clear_radio_callback()
{
	os_cancel_job(&RADIO.osjob);
	radio_callback = NULL;
}

/*!
 * \brief  Retrieves the radio status related to the last reception or
 *         transmission.
 * \note   The osal_radio_status_t structure is explained in osal.h.
 * \param  status: pointer to the osal_radio_status_t structure that will hold
 *                 the result.
 * \retval None
 */
void osal_get_radio_status(osal_radio_status_t *status)
{
	status->rssi = RADIO.rssi;
	status->snr = RADIO.snr;
	status->txend = (osal_time_t) ostime2us(RADIO.txend);
	status->freq_delta = RADIO.freq_delta;
	if (status->freq_delta & (1 << 19)) {
		status->freq_delta |= 0xFFF00000; // Fill up 20-Bit twos complement to 32-Bit
	}
	switch (getBw(RADIO.rps)) {
		// Fosc/(2^24)Hz * 500kHz/BW  = 954kHz/BW = approx 1000kHz/BW
		case BW500:
			status->freq_delta = status->freq_delta / 2;
			break;
		case BW250:
			status->freq_delta = status->freq_delta / 4;
			break;
		case BW125:
			status->freq_delta = status->freq_delta / 8;
			break;
		case BW62_5:
			status->freq_delta = status->freq_delta / 15;
			break;
	}

	status->crc_ok = ((RADIO.crc_ok == 0) ? OSAL_CRC_NOK : OSAL_CRC_OK);
}

/*!
 * \brief  Retrieves the pointer to the radio buffer.
 * \note   Data can be read from or written to it.
 * \retval uint8_t* : pointer to the radio buffer.
 */
uint8_t * osal_get_radio_buffer(void)
{
	return RADIO.frame;
}

/*!
 * \brief  Retrieves the length of the data inside the radio buffer.
 * \note   This is the length of the received or transmitted data.
 * \retval uint8_t: length of the data.
 */
uint8_t osal_get_radio_buffer_length(void)
{
	return RADIO.dataLen;
}

/*!
 * \brief  Fills the radio buffer with data to transmit.
 * \note   This function does not send anything, but must copies the data to
 *         the radio buffer and sets its length.
 * \param  src: pointer to the data to send.
 * \param  length: length of the data to send.
 * \retval None
 */
void osal_write_radio_buffer(uint8_t *src, uint8_t length)
{
	RADIO.dataLen = length;
	memcpy((void *) RADIO.frame, (void *) src, length);
}

/*!
 * \brief  Returns the RSSI currently perceived by the radio.
 * \note   This function will be called only if the radio is in RX mode.
 * \retval int16_t: perceived RSSI.
 */
int16_t osal_read_radio_rssi(void)
{
	return ((int16_t) radio_rssi() - 157);
}

/*!
 * \brief  Sets the new state of the radio.
 * \note   This function changes the hardware state of the radio (TX/RX/IDLE).
 * \param  state: state to set.
 * \retval None
 */
void osal_radio(enum osal_radio_state_t state)
{
	switch (state) {
		case OSAL_RADIO_STOP:
			os_radio(RADIO_RST);
			break;

		case OSAL_RADIO_START_TX:
			os_radio(RADIO_TX);
			break;

		case OSAL_RADIO_START_RX_SINGLE:
			os_radio(RADIO_RX);
			break;

		case OSAL_RADIO_START_RX:
			os_radio(RADIO_RXON);
			break;
	}
}


/*!
 * \brief  Formats and prints data
 * \note   This optional function is expected to behave like printf().
 * \param  fmt: formatted string.
 * \param  ...: list of arguments.
 * \retval None
 */
void osal_printf(const char *fmt, ...)
{
#ifndef CFG_CONSOLE_DEBUG_MAC_DISABLED
	va_list arp;

	va_start(arp, fmt);
	xvprintf(fmt, arp);
	va_end(arp);
#endif
}


/*!
 * \brief  Reads data as little-endian half-word.
 * \param  buf: pointer to the data to read.
 * \retval uint16_t: read half-word
 */
uint16_t osal_read16_le(const uint8_t *buf)
{
	return utils_read16_le(buf);
}

/*!
 * \brief  Reads data as little-endian word.
 * \param  buf: pointer to the data to read.
 * \retval uint32_t: read word
 */
uint32_t osal_read32_le(const uint8_t *buf)
{
	return utils_read32_le( buf);
}

/*!
 * \brief  Reads data as big-endian word.
 * \param  buf: pointer to the data to read.
 * \retval uint32_t: read word
 */
uint32_t osal_read32_be(const uint8_t *buf)
{
	return utils_read32_be(buf);
}

/*!
 * \brief  Writes data as little-endian half-word.
 * \param  buf: pointer to the memory area where the data will be written.
 * \param  v: half-word to write.
 * \retval none
 */
void osal_write16_le(uint8_t *buf, uint16_t v)
{
	utils_write16_le(buf, v);
}

/*!
 * \brief  Writes data as little-endian word.
 * \param  buf: pointer to the memory area where the data will be written.
 * \param  v: word to write.
 * \retval none
 */
void osal_write32_le(uint8_t *buf, uint32_t v)
{
	utils_write32_le(buf, v);
}

/*!
 * \brief  Writes data as big-endian word.
 * \param  buf: pointer to the memory area where the data will be written.
 * \param  v: word to write.
 * \retval none
 */
void osal_write32_be(uint8_t *buf, uint32_t v)
{
	utils_write32_be(buf, v);
}
