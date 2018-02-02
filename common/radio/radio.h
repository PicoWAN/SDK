/*! \file
 * \brief SX1272/6 radio driver
 */
/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * Copyright (c) 2018 Archos S.A.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *    Archos S.A.
 *******************************************************************************/

#ifndef _radio_h_
#define _radio_h_

/*!
 * Radio wakeup time from SLEEP mode
 */
#define RADIO_OSC_STARTUP		1 // [ms]

/*!
 * Radio PLL lock and Mode Ready delay which can vary with the temperature
 */
#define RADIO_SLEEP_TO_RX		2 // [ms]

/*!
 * Radio complete Wake-up Time with margin for temperature compensation
 */
#define RADIO_WAKEUP_TIME		(RADIO_OSC_STARTUP + RADIO_SLEEP_TO_RX)

// purpose of receive window - lmic_t.rxState
enum { RADIO_RST=0, RADIO_TX=1, RADIO_RX=2, RADIO_RXON=3 };

enum _cr_t { CR_4_5 = 0, CR_4_6, CR_4_7, CR_4_8 };
enum _sf_t { FSK = 0, SF6, SF7, SF8, SF9, SF10, SF11, SF12 };
enum _bw_t { BW7_8 = 0, BW10_4, BW15_6, BW20_8, BW31_25, BW41_7, BW62_5, BW125, BW250, BW500 };
typedef uint8_t cr_t;
typedef uint8_t sf_t;
typedef uint8_t bw_t;
typedef uint8_t dr_t;

// Radio parameter set (encodes SF/BW/CR/IH/NOCRC)
typedef uint32_t rps_t;

// Global maximum frame length
enum { STD_PREAMBLE_LEN  =  8 };
enum { MAX_LEN_FRAME     = 64 };

struct radio_t {
	// Radio settings TX/RX
	os_time_t	txend;
	os_time_t	rxtime;
	uint32_t	freq;
	int16_t		rssi;
	int8_t		snr;
	rps_t		rps;
	uint16_t	rxsyms;
	uint8_t		dndr;
	int8_t		txpow;		// dBm
	uint16_t	preamble_len;
	uint8_t		payload_length;
	uint8_t		sync_word;
	uint8_t		invert_iq;	// '0' -> normal mode, '1' -> inverted
	uint32_t	freq_delta;
	int8_t		ppm_offset;
	uint8_t		crc_ok;

	os_job_t	osjob;

	// Public part of RADIO state
	uint8_t		dataLen;    // 0 no data or zero length data, >0 byte count of data
	uint8_t		frame[MAX_LEN_FRAME];
};

extern struct radio_t RADIO;


#define MAKERPS(sf, bw, cr, ih, nocrc) ((rps_t) ((sf) | ((bw) << 3) | ((cr) << 7) | ((nocrc) ? (1 << 9) : 0) | ((ih & 0xFF) << 10)))

static inline sf_t getSf(rps_t params)
{
	return (sf_t) (params & 0x7);
}

static inline rps_t setSf(rps_t params, sf_t sf)
{
	return (rps_t) ((params & ~0x7) | sf);
}

static inline bw_t getBw(rps_t params)
{
	return (bw_t) ((params >> 3) & 0xf);
}

static inline rps_t setBw(rps_t params, bw_t bw)
{
	return (rps_t) ((params & ~(0xf << 3)) | (bw << 3));
}

static inline cr_t getCr(rps_t params)
{
	return (cr_t) ((params >> 7) & 0x3);
}

static inline rps_t setCr(rps_t params, cr_t cr)
{
	return (rps_t) ((params & ~(0x3 << 7)) | (cr << 7));
}

static inline int getNocrc(rps_t params)
{
	return ((params >> 9) & 0x1);
}

static inline rps_t setNocrc(rps_t params, int nocrc)
{
	return (rps_t) ((params & ~(0x1 << 9)) | (nocrc << 9));
}

static inline int getIh(rps_t params)
{
	return ((params >> 10) & 0xFF);
}

static inline rps_t setIh(rps_t params, int ih)
{
	return (rps_t) ((params & ~(0xff << 10)) | ((ih & 0xff) << 10));
}

static inline rps_t makeRps(sf_t sf, bw_t bw, cr_t cr, int ih, int nocrc)
{
	return sf | (bw << 3) | (cr << 7) | (nocrc ? (1 << 9) : 0) | ((ih & 0xFF) << 10);
}


/*!
 * \brief  Returns a random byte.
 *
 * \retval uint8_t A random byte.
 */
uint8_t radio_rand1(void);

/*!
 * \brief  Initializes the radio driver.
 */
void radio_init(void);

/*!
 * \brief  Resets the SX127x module.
 *
 * \param  val 1 to put the radio in reset, 0 to release the reset.
 */
void radio_reset(uint8_t val);

/*!
 * \brief  The radio driver IRQ handler.
 *
 * \param  dio The INT number to handle.
 */
void radio_irq_handler(uint8_t dio);

/*!
 * \brief   Returns the current RSSI.
 *
 * \details The function returns the current RSSI in dBm + 157 dBm. The SX127x has to be in RX mode.
 *
 * \retval  uint8_t The RSSI read.
 */
uint8_t radio_rssi(void);

/*!
 * \brief  Changes the radio mode.
 *
 * \param  mode The mode in which the radio will be put. This can be RADIO_RST, RADIO_TX, RADIO_RX, or RADIO_RXON.
 */
void os_radio(uint8_t mode);

/*!
 * \brief   Reads a radio register.
 *
 * \details The function reads the value of a specific SX127x register.
 *
 * \param   reg The register to read from.
 *
 * \retval  uint8_t The value read.
 */
uint8_t radio_read(uint8_t reg);
/*!
 * \brief   Writes a radio register.
 *
 * \details The function writes a value to a specific SX127x register.
 *
 * \param   addr The register to write to.
 * \param   data The value to write.
 */
void radio_write(uint8_t addr, uint8_t data);

#endif
