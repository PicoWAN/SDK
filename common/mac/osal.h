/*! \file
 * \brief OS abstraction layer for Libmultimac
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

#ifndef _OSAL_H_
#define _OSAL_H_

#include <stdint.h>

typedef int64_t osal_time_t;

#define ms2us(ms)		((osal_time_t) (((osal_time_t) (ms))*((osal_time_t) 1000)))
#define us2ms(us)		((osal_time_t) (((osal_time_t) (us))/((osal_time_t) 1000)))
#define us2ms_round(us)		((osal_time_t) (((osal_time_t) (us) + ((osal_time_t) 500))/((osal_time_t) 1000)))

#define OSAL_TIME_MAX		((osal_time_t) 0x7FFFFFFFFFFFFFFFLL)

enum osal_cr_t {
	OSAL_CR4_5 = 0,
	OSAL_CR4_6 = 1,
	OSAL_CR4_7 = 2,
	OSAL_CR4_8 = 3,
};

enum osal_sf_t {
	OSAL_SF6 = 0,
	OSAL_SF7 = 1,
	OSAL_SF8 = 2,
	OSAL_SF9 = 3,
	OSAL_SF10 = 4,
	OSAL_SF11 = 5,
	OSAL_SF12 = 6,
};

enum osal_bw_t {
	OSAL_BW62_5 = 0,
	OSAL_BW125 = 1,
	OSAL_BW250 = 2,
	OSAL_BW500 = 3,
};

enum osal_crc_t {
	OSAL_CRC_OFF = 0,
	OSAL_CRC_ON = 1,
};

enum osal_iq_t {
	OSAL_IQ_NORMAL = 0,
	OSAL_IQ_INVERTED = 1,
};

typedef struct {
	uint32_t		freq;		///< Frequency in Hz
	enum osal_cr_t		cr;		///< Coding rate
	enum osal_sf_t		sf;		///< Spreading factor
	enum osal_bw_t		bw;		///< Bandwidth
	enum osal_crc_t		crc;		///< CRC enabled/disabled
	int8_t			ih;		///< Implicit header size
	uint16_t		rxsyms;		///< RX timeout in symbols
	int8_t			txpow;		///< TX power in dBm
	uint16_t		preamble_len;	///< Length of the preamble
	uint8_t			payload_length;	///< Length of the payload to receive
	uint8_t			sync_word;	///< Sync word
	enum osal_iq_t		iq;		///< I/Q inverted or not
	int8_t			ppm_offset;	///< Offset in ppm to apply
	osal_time_t		rxtime;		///< Exact timestamp when the receiving will start
} osal_radio_params_t;

enum osal_crc_ok_t {
	OSAL_CRC_NOK = 0,
	OSAL_CRC_OK = 1,
};

typedef struct {
	int16_t			rssi;		///< RSSI
	int8_t			snr;		///< SNR
	osal_time_t		txend;		///< Time at which the radio finished transmitting the frame
	int32_t			freq_delta;	///< Estimated frequency error during the reception of the frame
	enum osal_crc_ok_t	crc_ok;		///< Status of the CRC in the received frame
} osal_radio_status_t;

enum osal_radio_state_t {
	OSAL_RADIO_STOP = 0,			///< Stop the radio
	OSAL_RADIO_START_TX = 1,		///< Start the transmission
	OSAL_RADIO_START_RX_SINGLE = 2,		///< Start the reception of one packet
	OSAL_RADIO_START_RX = 3,		///< Start the reception
};

typedef struct osal_job {
	void (*cb)(struct osal_job *);
	uint8_t data[32];
} osal_job_t;


/*!
 * \brief  Returns the elapsed time since the device has been powered-up.
 * \retval osal_time_t: current time in us.
 */
osal_time_t osal_get_time(void);

/*!
 * \brief  Returns the maximum time the radio can take before being actually
 *         receiving (usually sleep to RX transition).
 * \note   This is around 3000 us for the SX1276.
 * \retval osal_time_t: latency in us.
 */
osal_time_t osal_get_radio_wakeup_latency(void);

/*!
 * \brief  Returns an offset that will be removed to the deadlines of timing
 *         critical tasks in case the OS scheduler is not able to start the
 *         tasks on-time (for instance, in case of wakeup latency from a low
 *         power state).
 * \note   Ideally, the OS scheduler should handle any CPU latency, meaning
 *         this should be set to zero.
 * \retval osal_time_t: latency in us.
 */
osal_time_t osal_get_additional_cpu_wakeup_latency(void);


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
void osal_post_job(osal_job_t *job, void (*cb)(osal_job_t *));

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
void osal_post_delayed_job(osal_job_t *job, osal_time_t time, void (*cb)(osal_job_t *));

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
void osal_cancel_job(osal_job_t *job);


/*!
 * \brief  Prevents the device from going to lowpower and/or sleep mode (if the
 *         device supports a lowpower and/or a sleep mode inducing a latency).
 * \note   Once called, the device must work as fast as possible until the
 *         osal_lowpower_sleep_unlock() counterpart function is called.
 * \retval None
 */
void osal_lowpower_sleep_lock(void);

/*!
 * \brief  Allows the device to go to lowpower and/or sleep mode (if the
 *         device supports a lowpower and/or a sleep mode inducing a latency).
 * \note   Once called, the device has no more constraint related to speed or
 *         latency.
 * \retval None
 */
void osal_lowpower_sleep_unlock(void);


/*!
 * \brief  Copies a memory area.
 * \param  dst: destination pointer.
 * \param  src: source pointer.
 * \param  size: size of the memory area to copy.
 * \retval None
 */
void osal_memcpy(void *dst, const void *src, uint16_t size);

/*!
 * \brief  Copies a memory area in reverse order (last byte of the source area
 *         copied to the first byte of the destination area).
 * \param  dst: destination pointer.
 * \param  src: source pointer.
 * \param  size: size of the memory area to copy.
 * \retval None
 */
void osal_memcpy_reverse(void *dst, const void *src, uint16_t size);

/*!
 * \brief  Fills a memory area with a constant value.
 * \param  dst: destination pointer.
 * \param  byte: value used to fill the area.
 * \param  size: size of the memory area to fill.
 * \retval None
 */
void osal_memset(void *dst, uint8_t byte, uint16_t size);


/*!
 * \brief  Returns a random byte.
 * \retval uint8_t: random byte.
 */
uint8_t osal_rand8(void);

/*!
 * \brief  Returns a random half-word.
 * \retval uint16_t: random half-word.
 */
uint16_t osal_rand16(void);


/*!
 * \brief  Sets the AES key to use for encryption and decryption.
 * \note
 *         - This key stays valid for any call to osal_aes_encrypt() or
 *           osal_aes_decrypt() until a new key is set.
 *         - The key will always be 16 bytes long.
 * \param  key: AES key to set.
 * \retval None
 */
void osal_aes_setkey(const uint8_t *key);

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
void osal_aes_encrypt(const uint8_t *in, uint8_t *out, uint32_t len);

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
void osal_aes_decrypt(const uint8_t *in, uint8_t *out, uint32_t len);


/*!
 * \brief  Sets the radio parameters.
 * \note
 *         - The osal_radio_params_t structure is explained in osal.h.
 *         - These parameters must stay valid until new ones are set.
 * \param  params: parameters to set.
 * \retval None
 */
void osal_set_radio_params(osal_radio_params_t *params);

/*!
 * \brief  Sets a radio callback that will be called once the radio has
 *         received or transmitted a buffer.
 * \param  cd: pointer to the callback.
 * \retval None
 */
void osal_set_radio_callback(void (*cb)(void));

/*!
 * \brief  Clears the radio callback.
 * \note   This function must ensure that no further call to the radio
 *         callback will be made, including any call that could have been
 *         already scheduled.
 * \retval None
 */
void osal_clear_radio_callback(void);

/*!
 * \brief  Returns the maximum available power.
 * \note   Without additional PA, the SX1276 maximum power is 20 dBm using the
 *         PA_BOOST pin, and 14 dBm using the RFO pin.
 * \retval int8_t: maximum available power.
 */
int8_t osal_get_max_power(void);

/*!
 * \brief  Returns the minimum available power.
 * \note   Without additional PA, the SX1276 minimum power is 2 dBm using the
 *         PA_BOOST pin, and 0 dBm using the RFO pin.
 * \retval int8_t: minimum available power.
 */
int8_t osal_get_min_power(void);

/*!
 * \brief  Returns the antenna gain in dBi.
 * \note   Antenna gain rounded down.
 * \retval int8_t: antenna gain in dBi.
 */
int8_t osal_get_antenna_gain(void);

/*!
 * \brief  Checks if a given SF is supported.
 * \note   SF12 at lower bandwidth requires good frequency stability, so usually a TCXO is needed.
 * \retval int8_t: 1 if supported, 0 if not.
 */
int8_t osal_radio_SF_supported(enum osal_sf_t sf);

/*!
 * \brief  Tells if frequency calibration should be used.
 * \note   This will detect and correct any frequency offset introduced by
 *         the XTAL used for RF. It should not be needed if the board has a
 *         TCXO.
 * \retval int8_t: 1 if enabled, 0 if not.
 */
int8_t osal_is_freq_calibration_enabled(void);

/*!
 * \brief  Retrieves the radio status related to the last reception or
 *         transmission.
 * \note   The osal_radio_status_t structure is explained in osal.h.
 * \param  status: pointer to the osal_radio_status_t structure that will hold
 *                 the result.
 * \retval None
 */
void osal_get_radio_status(osal_radio_status_t *status);

/*!
 * \brief  Retrieves the pointer to the radio buffer.
 * \note   Data can be read from or written to it.
 * \retval uint8_t* : pointer to the radio buffer.
 */
uint8_t * osal_get_radio_buffer(void);

/*!
 * \brief  Retrieves the length of the data inside the radio buffer.
 * \note   This is the length of the received or transmitted data.
 * \retval uint8_t: length of the data.
 */
uint8_t osal_get_radio_buffer_length(void);

/*!
 * \brief  Fills the radio buffer with data to transmit.
 * \note   This function does not send anything, but must copies the data to
 *         the radio buffer and sets its length.
 * \param  src: pointer to the data to send.
 * \param  length: length of the data to send.
 * \retval None
 */
void osal_write_radio_buffer(uint8_t *src, uint8_t length);

/*!
 * \brief  Returns the RSSI currently perceived by the radio.
 * \note   This function will be called only if the radio is in RX mode.
 * \retval int16_t: perceived RSSI.
 */
int16_t osal_read_radio_rssi(void);

/*!
 * \brief  Sets the new state of the radio.
 * \note   This function changes the hardware state of the radio (TX/RX/IDLE).
 * \param  state: state to set.
 * \retval None
 */
void osal_radio(enum osal_radio_state_t state);


/*!
 * \brief  Formats and prints data
 * \note   This optional function is expected to behave like printf().
 * \param  fmt: formatted string.
 * \param  ...: list of arguments.
 * \retval None
 */
void osal_printf(const char *fmt, ...)
#ifdef  __GNUC__
__attribute__ ((format (printf, 1, 2)))
#endif
;


/*!
 * \brief  Reads data as little-endian half-word.
 * \param  buf: pointer to the data to read.
 * \retval uint16_t: read half-word
 */
uint16_t osal_read16_le(const uint8_t *buf);

/*!
 * \brief  Reads data as little-endian word.
 * \param  buf: pointer to the data to read.
 * \retval uint32_t: read word
 */
uint32_t osal_read32_le(const uint8_t *buf);

/*!
 * \brief  Reads data as big-endian word.
 * \param  buf: pointer to the data to read.
 * \retval uint32_t: read word
 */
uint32_t osal_read32_be(const uint8_t *buf);

/*!
 * \brief  Writes data as little-endian half-word.
 * \param  buf: pointer to the memory area where the data will be written.
 * \param  v: half-word to write.
 * \retval none
 */
void osal_write16_le(uint8_t *buf, uint16_t v);

/*!
 * \brief  Writes data as little-endian word.
 * \param  buf: pointer to the memory area where the data will be written.
 * \param  v: word to write.
 * \retval none
 */
void osal_write32_le(uint8_t *buf, uint32_t v);

/*!
 * \brief  Writes data as big-endian word.
 * \param  buf: pointer to the memory area where the data will be written.
 * \param  v: word to write.
 * \retval none
 */
void osal_write32_be(uint8_t *buf, uint32_t v);

#endif /* _OSAL_H_ */
