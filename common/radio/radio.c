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

#include <aes.h>
#include <boards.h>
#include <system.h>
#include <os.h>
#include <spi.h>
#include "radio.h"

// ----------------------------------------
// Registers Mapping
#define RegFifo                                    0x00 // common
#define RegOpMode                                  0x01 // common
#define FSKRegBitrateMsb                           0x02
#define FSKRegBitrateLsb                           0x03
#define FSKRegFdevMsb                              0x04
#define FSKRegFdevLsb                              0x05
#define RegFrfMsb                                  0x06 // common
#define RegFrfMid                                  0x07 // common
#define RegFrfLsb                                  0x08 // common
#define RegPaConfig                                0x09 // common
#define RegPaRamp                                  0x0A // common
#define RegOcp                                     0x0B // common
#define RegLna                                     0x0C // common
#define FSKRegRxConfig                             0x0D
#define LORARegFifoAddrPtr                         0x0D
#define FSKRegRssiConfig                           0x0E
#define LORARegFifoTxBaseAddr                      0x0E
#define FSKRegRssiCollision                        0x0F
#define LORARegFifoRxBaseAddr                      0x0F
#define FSKRegRssiThresh                           0x10
#define LORARegFifoRxCurrentAddr                   0x10
#define FSKRegRssiValue                            0x11
#define LORARegIrqFlagsMask                        0x11
#define FSKRegRxBw                                 0x12
#define LORARegIrqFlags                            0x12
#define FSKRegAfcBw                                0x13
#define LORARegRxNbBytes                           0x13
#define FSKRegOokPeak                              0x14
#define LORARegRxHeaderCntValueMsb                 0x14
#define FSKRegOokFix                               0x15
#define LORARegRxHeaderCntValueLsb                 0x15
#define FSKRegOokAvg                               0x16
#define LORARegRxPacketCntValueMsb                 0x16
#define LORARegRxpacketCntValueLsb                 0x17
#define LORARegModemStat                           0x18
#define LORARegPktSnrValue                         0x19
#define FSKRegAfcFei                               0x1A
#define LORARegPktRssiValue                        0x1A
#define FSKRegAfcMsb                               0x1B
#define LORARegRssiValue                           0x1B
#define FSKRegAfcLsb                               0x1C
#define LORARegHopChannel                          0x1C
#define FSKRegFeiMsb                               0x1D
#define LORARegModemConfig1                        0x1D
#define FSKRegFeiLsb                               0x1E
#define LORARegModemConfig2                        0x1E
#define FSKRegPreambleDetect                       0x1F
#define LORARegSymbTimeoutLsb                      0x1F
#define FSKRegRxTimeout1                           0x20
#define LORARegPreambleMsb                         0x20
#define FSKRegRxTimeout2                           0x21
#define LORARegPreambleLsb                         0x21
#define FSKRegRxTimeout3                           0x22
#define LORARegPayloadLength                       0x22
#define FSKRegRxDelay                              0x23
#define LORARegPayloadMaxLength                    0x23
#define FSKRegOsc                                  0x24
#define LORARegHopPeriod                           0x24
#define FSKRegPreambleMsb                          0x25
#define LORARegFifoRxByteAddr                      0x25
#define LORARegModemConfig3                        0x26
#define FSKRegPreambleLsb                          0x26
#define LORARegPpmCorrection                       0x27
#define FSKRegSyncConfig                           0x27
#define LORARegFeiMsb                              0x28
#define FSKRegSyncValue1                           0x28
#define LORARegFeiMib                              0x29
#define FSKRegSyncValue2                           0x29
#define LORARegFeiLsb                              0x2A
#define FSKRegSyncValue3                           0x2A
#define FSKRegSyncValue4                           0x2B
#define LORARegRssiWideband                        0x2C
#define FSKRegSyncValue5                           0x2C
#define FSKRegSyncValue6                           0x2D
#define FSKRegSyncValue7                           0x2E
#define LORAReserved2F                             0x2F
#define FSKRegSyncValue8                           0x2F
#define LORAReserved30                             0x30
#define FSKRegPacketConfig1                        0x30
#define FSKRegPacketConfig2                        0x31
#define LORARegDetectOptimize                      0x31
#define FSKRegPayloadLength                        0x32
#define FSKRegNodeAdrs                             0x33
#define LORARegInvertIQ                            0x33
#define FSKRegBroadcastAdrs                        0x34
#define FSKRegFifoThresh                           0x35
#define LORAReserved36                             0x36
#define FSKRegSeqConfig1                           0x36
#define FSKRegSeqConfig2                           0x37
#define LORARegDetectionThreshold                  0x37
#define FSKRegTimerResol                           0x38
#define FSKRegTimer1Coef                           0x39
#define LORARegSyncWord                            0x39
#define LORAReserved3A                             0x3A
#define FSKRegTimer2Coef                           0x3A
#define FSKRegImageCal                             0x3B
#define LORARegInvertIQ2                           0x3B
#define FSKRegTemp                                 0x3C
#define FSKRegLowBat                               0x3D
#define FSKRegIrqFlags1                            0x3E
#define FSKRegIrqFlags2                            0x3F
#define RegDioMapping1                             0x40 // common
#define RegDioMapping2                             0x41 // common
#define RegVersion                                 0x42 // common
// #define RegAgcRef                                  0x43 // common
// #define RegAgcThresh1                              0x44 // common
// #define RegAgcThresh2                              0x45 // common
// #define RegAgcThresh3                              0x46 // common
// #define RegPllHop                                  0x4B // common
// #define RegTcxo                                    0x58 // common
#define RegTcxo                                    0x4B // SX1276
#define RegPaDac                                   0x4D // common
// #define RegPll                                     0x5C // common
// #define RegPllLowPn                                0x5E // common
// #define RegFormerTemp                              0x6C // common
// #define RegBitRateFrac                             0x70 // common

// ----------------------------------------
// spread factors and mode for RegModemConfig2
#define SX1272_MC2_FSK  0x00
#define SX1272_MC2_SF6  0x60
#define SX1272_MC2_SF7  0x70
#define SX1272_MC2_SF8  0x80
#define SX1272_MC2_SF9  0x90
#define SX1272_MC2_SF10 0xA0
#define SX1272_MC2_SF11 0xB0
#define SX1272_MC2_SF12 0xC0
// bandwidth for RegModemConfig1
#define SX1272_MC1_BW_125  0x00
#define SX1272_MC1_BW_250  0x40
#define SX1272_MC1_BW_500  0x80
// coding rate for RegModemConfig1
#define SX1272_MC1_CR_4_5 0x08
#define SX1272_MC1_CR_4_6 0x10
#define SX1272_MC1_CR_4_7 0x18
#define SX1272_MC1_CR_4_8 0x20
#define SX1272_MC1_IMPLICIT_HEADER_MODE_ON 0x04 // required for receive
#define SX1272_MC1_RX_PAYLOAD_CRCON        0x02
#define SX1272_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12
// transmit power configuration for RegPaConfig
#define SX1272_PAC_PA_SELECT_PA_BOOST 0x80
#define SX1272_PAC_PA_SELECT_RFIO_PIN 0x00


// sx1276 RegModemConfig1
#define SX1276_MC1_BW_7_8                0x00
#define SX1276_MC1_BW_10_4               0x10
#define SX1276_MC1_BW_15_6               0x20
#define SX1276_MC1_BW_20_8               0x30
#define SX1276_MC1_BW_31_25              0x40
#define SX1276_MC1_BW_41_7               0x50
#define SX1276_MC1_BW_62_5               0x60
#define SX1276_MC1_BW_125                0x70
#define SX1276_MC1_BW_250                0x80
#define SX1276_MC1_BW_500                0x90
#define SX1276_MC1_CR_4_5            0x02
#define SX1276_MC1_CR_4_6            0x04
#define SX1276_MC1_CR_4_7            0x06
#define SX1276_MC1_CR_4_8            0x08

#define SX1276_MC1_IMPLICIT_HEADER_MODE_ON    0x01

// sx1276 RegModemConfig2
#define SX1276_MC2_RX_PAYLOAD_CRCON        0x04
#define SX1276_MC2_RX_SYMBTIMEOUTMSB_MASK  0xFC

// sx1276 RegModemConfig3
#define SX1276_MC3_LOW_DATA_RATE_OPTIMIZE  0x08
#define SX1276_MC3_AGCAUTO                 0x04

#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 0x0A
#ifdef CFG_sx1276_radio
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x70
#elif CFG_sx1272_radio
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x74
#endif



// ----------------------------------------
// Constants for radio registers
#define OPMODE_LORA      0x80
#define OPMODE_MASK      0x07
#define OPMODE_SLEEP     0x00
#define OPMODE_STANDBY   0x01
#define OPMODE_FSTX      0x02
#define OPMODE_TX        0x03
#define OPMODE_FSRX      0x04
#define OPMODE_RX        0x05
#define OPMODE_RX_SINGLE 0x06
#define OPMODE_CAD       0x07

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK 0x80
#define IRQ_LORA_RXDONE_MASK 0x40
#define IRQ_LORA_CRCERR_MASK 0x20
#define IRQ_LORA_HEADER_MASK 0x10
#define IRQ_LORA_TXDONE_MASK 0x08
#define IRQ_LORA_CDDONE_MASK 0x04
#define IRQ_LORA_FHSSCH_MASK 0x02
#define IRQ_LORA_CDDETD_MASK 0x01

#define IRQ_FSK1_MODEREADY_MASK         0x80
#define IRQ_FSK1_RXREADY_MASK           0x40
#define IRQ_FSK1_TXREADY_MASK           0x20
#define IRQ_FSK1_PLLLOCK_MASK           0x10
#define IRQ_FSK1_RSSI_MASK              0x08
#define IRQ_FSK1_TIMEOUT_MASK           0x04
#define IRQ_FSK1_PREAMBLEDETECT_MASK    0x02
#define IRQ_FSK1_SYNCADDRESSMATCH_MASK  0x01
#define IRQ_FSK2_FIFOFULL_MASK          0x80
#define IRQ_FSK2_FIFOEMPTY_MASK         0x40
#define IRQ_FSK2_FIFOLEVEL_MASK         0x20
#define IRQ_FSK2_FIFOOVERRUN_MASK       0x10
#define IRQ_FSK2_PACKETSENT_MASK        0x08
#define IRQ_FSK2_PAYLOADREADY_MASK      0x04
#define IRQ_FSK2_CRCOK_MASK             0x02
#define IRQ_FSK2_LOWBAT_MASK            0x01

// ----------------------------------------
// DIO function mappings                D0D1D2D3
#define MAP_DIO0_LORA_RXDONE   0x00  // 00------
#define MAP_DIO0_LORA_TXDONE   0x40  // 01------
#define MAP_DIO1_LORA_RXTOUT   0x00  // --00----
#define MAP_DIO1_LORA_NOP      0x30  // --11----
#define MAP_DIO2_LORA_NOP      0x0C  // ----11--

#define MAP_DIO0_FSK_READY     0x00  // 00------ (packet sent / payload ready)
#define MAP_DIO1_FSK_NOP       0x30  // --11----
#define MAP_DIO2_FSK_TXNOP     0x04  // ----01--
#define MAP_DIO2_FSK_TIMEOUT   0x08  // ----10--

#define MAP_DIO5_CLKOUT        0x20  // --10----


// FSK IMAGECAL defines
#define RF_IMAGECAL_AUTOIMAGECAL_MASK               0x7F
#define RF_IMAGECAL_AUTOIMAGECAL_ON                 0x80
#define RF_IMAGECAL_AUTOIMAGECAL_OFF                0x00  // Default

#define RF_IMAGECAL_IMAGECAL_MASK                   0xBF
#define RF_IMAGECAL_IMAGECAL_START                  0x40

#define RF_IMAGECAL_IMAGECAL_RUNNING                0x20
#define RF_IMAGECAL_IMAGECAL_DONE                   0x00  // Default


#define DEFAULT_MAC_PREAMBLE                        0x12  //Archos preamble

#define DEFAULT_INVERT_IQ                           0  //Archos configuration (never inverted)

#define DEFAULT_PAYLOAD_LENGTH                      64

#define ASSERT(a)                                   {if (!a) {while (1);}}

struct radio_t RADIO;


/*
 * AES computation context variable
 */
static aes_context AesContext;

// RADIO STATE
// (initialized by radio_init(), used by radio_rand1())
static uint8_t randbuf[16];


#ifdef CFG_sx1276_radio
#define LNA_RX_GAIN (0x20 | 0x03)
#elif CFG_sx1272_radio
#define LNA_RX_GAIN (0x20 | 0x03)
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif


static void writeReg(uint8_t addr, uint8_t data)
{
	system_set_sx_nss(0);
	spi_rw(addr | 0x80);
	spi_rw(data);
	system_set_sx_nss(1);
}

static uint8_t readReg(uint8_t addr)
{
	system_set_sx_nss(0);
	spi_rw(addr & 0x7F);
	uint8_t val = spi_rw(0x00);
	system_set_sx_nss(1);
	return val;
}

static void writeBuf(uint8_t addr, uint8_t *buf, uint8_t len)
{
	system_set_sx_nss(0);
	spi_rw(addr | 0x80);
	for (uint8_t i = 0; i < len; i++) {
		spi_rw(buf[i]);
	}
	system_set_sx_nss(1);
}

static void readBuf(uint8_t addr, uint8_t *buf, uint8_t len)
{
	system_set_sx_nss(0);
	spi_rw(addr & 0x7F);
	for (uint8_t i = 0; i < len; i++) {
		buf[i] = spi_rw(0x00);
	}
	system_set_sx_nss(1);
}

static void opmode(uint8_t mode)
{
	writeReg(RegOpMode, (readReg(RegOpMode) & ~OPMODE_MASK) | mode);
}

static void opmodeLora()
{
	uint8_t u = OPMODE_LORA;
#ifdef CFG_sx1276_radio
	// u |= 0x8;   // TBD: sx1276 high freq
#endif
	writeReg(RegOpMode, u);
}

static void opmodeFSK()
{
	uint8_t u = 0;
#ifdef CFG_sx1276_radio
	// u |= 0x8;   // TBD: sx1276 high freq
#endif
	writeReg(RegOpMode, u);
}

// configure LoRa modem (cfg1, cfg2)
static void configLoraModem()
{
	sf_t sf = getSf(RADIO.rps);

#ifdef CFG_sx1276_radio
	uint8_t mc1 = 0, mc2 = 0, mc3 = 0;

	switch (getBw(RADIO.rps)) {
		case BW7_8:
			mc1 |= SX1276_MC1_BW_7_8;
			break;
		case BW10_4:
			mc1 |= SX1276_MC1_BW_10_4;
			break;
		case BW15_6:
			mc1 |= SX1276_MC1_BW_15_6;
			break;
		case BW20_8:
			mc1 |= SX1276_MC1_BW_20_8;
			break;
		case BW31_25:
			mc1 |= SX1276_MC1_BW_31_25;
			break;
		case BW41_7:
			mc1 |= SX1276_MC1_BW_41_7;
			break;
		case BW62_5:
			mc1 |= SX1276_MC1_BW_62_5;
			break;
		case BW125:
			mc1 |= SX1276_MC1_BW_125;
			break;
		case BW250:
			mc1 |= SX1276_MC1_BW_250;
			break;
		case BW500:
			mc1 |= SX1276_MC1_BW_500;
			break;
		default:
			ASSERT(0);
	}
	switch (getCr(RADIO.rps)) {
		case CR_4_5:
			mc1 |= SX1276_MC1_CR_4_5;
			break;
		case CR_4_6:
			mc1 |= SX1276_MC1_CR_4_6;
			break;
		case CR_4_7:
			mc1 |= SX1276_MC1_CR_4_7;
			break;
		case CR_4_8:
			mc1 |= SX1276_MC1_CR_4_8;
			break;
		default:
			ASSERT(0);
	}

	if (getIh(RADIO.rps)) {
		mc1 |= SX1276_MC1_IMPLICIT_HEADER_MODE_ON;
		writeReg(LORARegPayloadLength, getIh(RADIO.rps)); // required length
	}
	// set ModemConfig1
	writeReg(LORARegModemConfig1, mc1);

	mc2 = (sf + 6 - SF6) << 4;
	if (getNocrc(RADIO.rps) == 0) {
		mc2 |= SX1276_MC2_RX_PAYLOAD_CRCON;
	}
	mc2 |= (RADIO.rxsyms >> 8) & ~SX1276_MC2_RX_SYMBTIMEOUTMSB_MASK;
	writeReg(LORARegModemConfig2, mc2);

	if (sf == SF6) {
		writeReg(LORARegDetectOptimize, ((readReg(LORARegDetectOptimize) & 0xF8) | 0x05));
		writeReg(LORARegDetectionThreshold, 0x0C);
	} else {
		writeReg(LORARegDetectOptimize, ((readReg(LORARegDetectOptimize) & 0xF8) | 0x03));
		writeReg(LORARegDetectionThreshold, 0x0A);
	}

	mc3 = SX1276_MC3_AGCAUTO;

	// turn on low data rate optimization when symbol_duration > 16ms
	if (((sf == SF12) && (getBw(RADIO.rps) <= BW250)) || ((sf == SF11) && (getBw(RADIO.rps) <= BW125)) ||
	    ((sf == SF10) && (getBw(RADIO.rps) <= BW62_5)) || ((sf == SF9) && (getBw(RADIO.rps) <= BW31_25)) ||
	    ((sf == SF8) && (getBw(RADIO.rps) <= BW15_6))) {
		mc3 |= SX1276_MC3_LOW_DATA_RATE_OPTIMIZE;
	}
	writeReg(LORARegModemConfig3, mc3);
	writeReg(LORARegPpmCorrection, RADIO.ppm_offset);
#elif CFG_sx1272_radio
	uint8_t mc1 = (getBw(RADIO.rps) << 6);

	switch (getCr(RADIO.rps)) {
		case CR_4_5:
			mc1 |= SX1272_MC1_CR_4_5;
			break;
		case CR_4_6:
			mc1 |= SX1272_MC1_CR_4_6;
			break;
		case CR_4_7:
			mc1 |= SX1272_MC1_CR_4_7;
			break;
		case CR_4_8:
			mc1 |= SX1272_MC1_CR_4_8;
			break;
	}

	if ((sf == SF11 || sf == SF12) && getBw(RADIO.rps) == BW125) {
		mc1 |= SX1272_MC1_LOW_DATA_RATE_OPTIMIZE;
	}

	if (getNocrc(RADIO.rps) == 0) {
		mc1 |= SX1272_MC1_RX_PAYLOAD_CRCON;
	}

	if (getIh(RADIO.rps)) {
		mc1 |= SX1272_MC1_IMPLICIT_HEADER_MODE_ON;
		writeReg(LORARegPayloadLength, getIh(RADIO.rps)); // required length
	}
	// set ModemConfig1
	writeReg(LORARegModemConfig1, mc1);

	// set ModemConfig2 (sf, AgcAutoOn=1 SymbTimeoutHi=00)
	writeReg(LORARegModemConfig2, ((sf + 6 - SF6) << 4) | 0x04);

	if (sf == SF6) {
		writeReg(LORARegDetectOptimize, ((readReg(LORARegDetectOptimize) & 0xF8) | 0x05));
		writeReg(LORARegDetectionThreshold, 0x0C);
	} else {
		writeReg(LORARegDetectOptimize, ((readReg(LORARegDetectOptimize) & 0xF8) | 0x03));
		writeReg(LORARegDetectionThreshold, 0x0A);
	}
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif /* CFG_sx1272_radio */
}

static void set_frequency(uint32_t freq)
{
	// set frequency: FQ = (FRF * 32 Mhz) / (2 ^ 19)
	uint64_t frf = ((uint64_t) freq << 19) / 32000000;
	writeReg(RegFrfMsb, (uint8_t) (frf >> 16));
	writeReg(RegFrfMid, (uint8_t) (frf >> 8));
	writeReg(RegFrfLsb, (uint8_t) (frf >> 0));
}

static void configChannel()
{
	set_frequency(RADIO.freq);
}


static void configPower()
{
	int8_t pw = (int8_t) RADIO.txpow;

#ifdef CFG_sx1276_radio

#if defined(TX_RFO_PORT) && defined(TX_RFO_PIN)
#if defined(TX_PA_PORT) && defined(TX_PA_PIN)
	if (pw > 17) {
		/* Enable +20 dBm max output on PA_BOOST */
		writeReg(RegPaDac, (readReg(RegPaDac) & ~0x7) | 0x7);
		pw = 17;
	} else {
		/* +14 dBm max output on RFO, +17 dBm max output on PA_BOOST */
		writeReg(RegPaDac, (readReg(RegPaDac) & ~0x7) | 0x4);
	}

	if (pw > 15) {
		/* Select PA_BOOST output */
		writeReg(RegPaConfig, (uint8_t) (0xF0 | ((pw - 2) & 0xf)));
	} else {
		/* Select RFO output */
		writeReg(RegPaConfig, (uint8_t) (0x70 | (pw & 0xf)));
	}
#else
	if (pw > 15) {
		pw = 15;
	}

	/* +14 dBm max output on RFO */
	writeReg(RegPaDac, (readReg(RegPaDac) & ~0x7) | 0x4);
	/* Select RFO output (only one available on the board) */
	writeReg(RegPaConfig, (uint8_t) (0x70 | (pw & 0xf)));
#endif
#else
	if (pw > 17) {
		/* Enable +20 dBm max output on PA_BOOST */
		writeReg(RegPaDac, (readReg(RegPaDac) & ~0x7) | 0x7);
		pw = 17;
	} else {
		/* Min output power is +2 dBm on PA_BOOST */
		if (pw < 2) {
			pw = 2;
		}

		/* +17 dBm max output on PA_BOOST */
		writeReg(RegPaDac, (readReg(RegPaDac) & ~0x7) | 0x4);
	}

	/* Select PA_BOOST output (default) */
	writeReg(RegPaConfig, (uint8_t) (0x80 | ((pw - 2) & 0xf)));
#endif

	/* Disable the current limiter */
	writeReg(RegOcp, (readReg(RegOcp) & ~0x3F) | 0x1F);


#elif CFG_sx1272_radio
	// set PA config (2-17 dBm using PA_BOOST)
	if (pw > 17) {
		pw = 17;
	} else if (pw < 2) {
		pw = 2;
	}
	writeReg(RegPaConfig, (uint8_t) (0x80 | (pw - 2)));
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif /* CFG_sx1272_radio */
}

static void configRFPathTX()
{
	if ((int8_t) RADIO.txpow > 15) {
		system_try_select_rf_path(PATH_TX_PA);
	} else {
		system_try_select_rf_path(PATH_TX_RFO);
	}
}

static void configRFPathRX()
{
	system_try_select_rf_path(PATH_RX);
}

static void txfsk()
{

	// select FSK modem (from sleep mode)
	writeReg(RegOpMode, 0x00); // FSK, BT=0.5
	ASSERT(readReg(RegOpMode) == 0);
	// enter standby mode (required for FIFO loading))
	opmode(OPMODE_STANDBY);
	// set bitrate
	writeReg(FSKRegBitrateMsb, 9); // 0x02); // 50kbps
	writeReg(FSKRegBitrateLsb, 0); // 0x80);
	// set frequency deviation
	writeReg(FSKRegFdevMsb, 0); // 0x01); // +/- 25kHz
	writeReg(FSKRegFdevLsb, 0); // 0x99);
	// frame and packet handler settings
	writeReg(FSKRegPreambleMsb, 0x00);
	writeReg(FSKRegPreambleLsb, 0x08);
	writeReg(FSKRegPacketConfig1, 0x48); // 0xD0);
// writeReg(FSKRegPacketConfig2, 0);//0x40);
#if 0
    writeReg(FSKRegSyncConfig, 0x12);
    writeReg(FSKRegSyncValue1, 0xC1);
    writeReg(FSKRegSyncValue2, 0x94);
    writeReg(FSKRegSyncValue3, 0xC1);
#endif
	// configure frequency
	configChannel();
	// configure output power
	configPower();

	// set the IRQ mapping DIO0=PacketSent DIO1=NOP DIO2=NOP
	writeReg(RegDioMapping1, MAP_DIO0_FSK_READY /*|MAP_DIO1_FSK_NOP|MAP_DIO2_FSK_TXNOP*/);

	// initialize the payload size and address pointers
	writeReg(FSKRegPayloadLength, 0); // RADIO.dataLen+1); // (insert length byte into payload))

	// download length byte and buffer to the radio FIFO
	//    writeReg(RegFifo, RADIO.dataLen);
	//    writeBuf(RegFifo, RADIO.frame, RADIO.dataLen);

	// enable antenna switch for TX
	configRFPathTX();

	// now we actually start the transmission
	opmode(OPMODE_TX);
}

static void txlora()
{
	// select LoRa modem (from sleep mode)
	// writeReg(RegOpMode, OPMODE_LORA);
	opmodeLora();
	ASSERT((readReg(RegOpMode) & OPMODE_LORA) != 0);

	// enter standby mode (required for FIFO loading))
	opmode(OPMODE_STANDBY);
	// configure LoRa modem (cfg1, cfg2)
	configLoraModem();
	// configure frequency
	configChannel();
	// configure output power
	writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec
	configPower();
	// set sync word
	writeReg(LORARegSyncWord, RADIO.sync_word);

	if (RADIO.invert_iq == 1) {
		writeReg(LORARegInvertIQ, readReg(LORARegInvertIQ) | (1 << 6));
		writeReg(LORARegInvertIQ2, 0x19);
	} else {
		writeReg(LORARegInvertIQ, readReg(LORARegInvertIQ) & (~(1 << 6)));
		writeReg(LORARegInvertIQ2, 0x1D);
	}

	// set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
	writeReg(RegDioMapping1, MAP_DIO0_LORA_TXDONE | MAP_DIO1_LORA_NOP | MAP_DIO2_LORA_NOP);

	// uint8_t tmp = readReg(RegDioMapping2) & 0x30;
	writeReg(RegDioMapping2, MAP_DIO5_CLKOUT | 0);
	//    writeReg(0x24, 0);

	// clear all radio IRQ flags
	writeReg(LORARegIrqFlags, 0xFF);
	// mask all IRQs but TxDone
	writeReg(LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK);

	if (RADIO.preamble_len > 0) {
		writeReg(LORARegPreambleMsb, (RADIO.preamble_len >> 8) & 0xff);
		writeReg(LORARegPreambleLsb, (uint8_t) (RADIO.preamble_len & 0xFF));
	}

	// initialize the payload size and address pointers
	writeReg(LORARegFifoTxBaseAddr, 0x00);
	writeReg(LORARegFifoAddrPtr, 0x00);
	writeReg(LORARegPayloadLength, RADIO.dataLen);

	// download buffer to the radio FIFO
	writeBuf(RegFifo, RADIO.frame, RADIO.dataLen);

	// enable antenna switch for TX
	configRFPathTX();

	// now we actually start the transmission
	opmode(OPMODE_TX);
}

// start transmitter (buf=RADIO.frame, len=RADIO.dataLen)
static void starttx()
{
	while ((readReg(RegOpMode) & OPMODE_MASK) != OPMODE_SLEEP);
	if (getSf(RADIO.rps) == FSK) { // FSK modem
		txfsk();
	} else { // LoRa modem
		txlora();
	}
	// the radio will go back to STANDBY mode as soon as the TX is finished
	// the corresponding IRQ will inform us about completion.
}

enum { RXMODE_SINGLE, RXMODE_SCAN, RXMODE_RSSI };

static const uint8_t rxlorairqmask[] = {
	[RXMODE_SINGLE] = IRQ_LORA_RXDONE_MASK | IRQ_LORA_RXTOUT_MASK | IRQ_LORA_CRCERR_MASK,
	[RXMODE_SCAN] = IRQ_LORA_RXDONE_MASK | IRQ_LORA_CRCERR_MASK, [RXMODE_RSSI] = 0x00,
};

// start LoRa receiver (time=RADIO.rxtime, timeout=RADIO.rxsyms, result=RADIO.frame[RADIO.dataLen])
static void rxlora(uint8_t rxmode)
{
	// select LoRa modem (from sleep mode)
	opmodeLora();
	ASSERT((readReg(RegOpMode) & OPMODE_LORA) != 0);
	// enter standby mode (warm up))
	opmode(OPMODE_STANDBY);
	// don't use MAC settings at startup
	if (rxmode == RXMODE_RSSI) { // use fixed settings for rssi scan
		writeReg(LORARegModemConfig1, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1);
		writeReg(LORARegModemConfig2, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2);
	} else { // single or continuous rx mode
		// configure LoRa modem (cfg1, cfg2)
		configLoraModem();
		// configure frequency
		configChannel();
	}
	// set LNA gain
	writeReg(RegLna, LNA_RX_GAIN);

	if (RADIO.preamble_len > 0) {
		writeReg(LORARegPreambleMsb, (RADIO.preamble_len >> 8) & 0xff);
		writeReg(LORARegPreambleLsb, (uint8_t) (RADIO.preamble_len & 0xFF));
	}

	// set max payload size
	writeReg(LORARegPayloadMaxLength, RADIO.payload_length);
	// use inverted I/Q signal (prevent mote-to-mote communication)
	if (RADIO.invert_iq == 1) {
		writeReg(LORARegInvertIQ, readReg(LORARegInvertIQ) | (1 << 6));
		writeReg(LORARegInvertIQ2, 0x19);
	} else {
		writeReg(LORARegInvertIQ, readReg(LORARegInvertIQ) & (~(1 << 6)));
		writeReg(LORARegInvertIQ2, 0x1D);
	}

	// ERRATA 2.1: Sensitivity Optimization with a 500 kHz Bandwidth
	if(getBw(RADIO.rps) == BW500) {
		if (RADIO.freq > 525000000) {
			writeReg(LORAReserved36, 0x02);
			writeReg(LORAReserved3A, 0x64);
		} else {
			writeReg(LORAReserved36, 0x02);
			writeReg(LORAReserved3A, 0x7F);
		}
	} else {
		writeReg(LORAReserved36, 0x03);
	}

	// ERRATA 2.3: Receiver Spurious Reception of a LoRa Signal
	if (getBw(RADIO.rps) == BW500) {
		writeReg(LORARegDetectOptimize, readReg(LORARegDetectOptimize) | 0x80);
	} else {
		writeReg(LORARegDetectOptimize, readReg(LORARegDetectOptimize) & 0x7F);
		writeReg(LORAReserved30, 0x00);
		switch (getBw(RADIO.rps)) {
			case BW7_8:
				writeReg(LORAReserved2F, 0x48);
				RADIO.freq += 7810;
				configChannel();
				break;
			case BW10_4:
				writeReg(LORAReserved2F, 0x44);
				RADIO.freq += 10420;
				configChannel();
				break;
			case BW15_6:
				writeReg(LORAReserved2F, 0x44);
				RADIO.freq += 15620;
				configChannel();
				break;
			case BW20_8:
				writeReg(LORAReserved2F, 0x44);
				RADIO.freq += 20830;
				configChannel();
				break;
			case BW31_25:
				writeReg(LORAReserved2F, 0x44);
				RADIO.freq += 31250;
				configChannel();
				break;
			case BW41_7:
				writeReg(LORAReserved2F, 0x44);
				RADIO.freq += 41670;
				configChannel();
				break;
			case BW62_5:
				writeReg(LORAReserved2F, 0x40);
				break;
			case BW125:
				writeReg(LORAReserved2F, 0x40);
				break;
			case BW250:
				writeReg(LORAReserved2F, 0x40);
				break;
		}
	}

	// set symbol timeout (for single rx)
	writeReg(LORARegSymbTimeoutLsb, (uint8_t) (RADIO.rxsyms & 0xFF));
	// set sync word
	writeReg(LORARegSyncWord, RADIO.sync_word);

	// configure DIO mapping DIO0=RxDone DIO1=RxTout DIO2=NOP
	writeReg(RegDioMapping1, MAP_DIO0_LORA_RXDONE | MAP_DIO1_LORA_RXTOUT | MAP_DIO2_LORA_NOP);

	// uint8_t tmp = readReg(RegDioMapping2) & 0x30;
	writeReg(RegDioMapping2, MAP_DIO5_CLKOUT | 0);
	//    writeReg(0x24, 0);

	// clear all radio IRQ flags
	writeReg(LORARegIrqFlags, 0xFF);

	// enable required radio IRQs
	writeReg(LORARegIrqFlagsMask, ~rxlorairqmask[rxmode]);

	// enable antenna switch for RX
	configRFPathRX();

	// now instruct the radio to receive
	if (rxmode == RXMODE_SINGLE) {       // single rx
		os_wait_until(RADIO.rxtime); // busy wait until exact rx time
		opmode(OPMODE_RX_SINGLE);
	} else { // continous rx (scan or rssi)
		opmode(OPMODE_RX);
	}
}

static void rxfsk(uint8_t rxmode)
{
	// only single rx (no continuous scanning, no noise sampling)
	ASSERT(rxmode == RXMODE_SINGLE);
	// select FSK modem (from sleep mode)
	// writeReg(RegOpMode, 0x00); // (not LoRa)
	opmodeFSK();
	ASSERT((readReg(RegOpMode) & OPMODE_LORA) == 0);
	// enter standby mode (warm up))
	opmode(OPMODE_STANDBY);
	// configure frequency
	configChannel();
	// set LNA gain
	// writeReg(RegLna, 0x20|0x03); // max gain, boost enable
	writeReg(RegLna, LNA_RX_GAIN);
	// configure receiver
	writeReg(FSKRegRxConfig, 0x1E); // AFC auto, AGC, trigger on preamble?!?
	// set receiver bandwidth
	writeReg(FSKRegRxBw, 0x0B); // 50kHz SSb
	// set AFC bandwidth
	writeReg(FSKRegAfcBw, 0x12); // 83.3kHz SSB
	// set preamble detection
	writeReg(FSKRegPreambleDetect, 0xAA); // enable, 2 bytes, 10 chip errors
	// set sync config
	writeReg(FSKRegSyncConfig, 0x12); // no auto restart, preamble 0xAA, enable, fill FIFO, 3 bytes sync
	// set packet config
	writeReg(FSKRegPacketConfig1, 0xD8); // var-length, whitening, crc, no auto-clear, no adr filter
	writeReg(FSKRegPacketConfig2, 0x40); // packet mode
	// set sync value
	writeReg(FSKRegSyncValue1, 0xC1);
	writeReg(FSKRegSyncValue2, 0x94);
	writeReg(FSKRegSyncValue3, 0xC1);
	// set preamble timeout
	writeReg(FSKRegRxTimeout2, 0xFF); //(RADIO.rxsyms+1)/2);
	// set bitrate
	writeReg(FSKRegBitrateMsb, 0x02); // 50kbps
	writeReg(FSKRegBitrateLsb, 0x80);
	// set frequency deviation
	writeReg(FSKRegFdevMsb, 0x01); // +/- 25kHz
	writeReg(FSKRegFdevLsb, 0x99);

	// configure DIO mapping DIO0=PayloadReady DIO1=NOP DIO2=TimeOut
	writeReg(RegDioMapping1, MAP_DIO0_FSK_READY | MAP_DIO1_FSK_NOP | MAP_DIO2_FSK_TIMEOUT);

	// enable antenna switch for RX
	configRFPathRX();

	// now instruct the radio to receive
	os_wait_until(RADIO.rxtime); // busy wait until exact rx time
	opmode(OPMODE_RX);	   // no single rx mode available in FSK
}

static void startrx(uint8_t rxmode)
{
	ASSERT((readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP);
	if (getSf(RADIO.rps) == FSK) { // FSK modem
		rxfsk(rxmode);
	} else { // LoRa modem
		rxlora(rxmode);
	}
	// the radio will go back to STANDBY mode as soon as the RX is finished
	// or timed out, and the corresponding IRQ will inform us about completion.
}

static void rx_chain_calibration(void)
{
	uint8_t pa_backup;
	uint32_t frf_backup;

	/* Save context */
	pa_backup = readReg(RegPaConfig);
	frf_backup = ((uint32_t) readReg(RegFrfMsb) << 16) |
		      ((uint32_t) readReg(RegFrfMid) << 8) |
		      ((uint32_t) readReg(RegFrfLsb) << 0);

	/* Cut the PA just in case, RFO output, power = -1 dBm */
	writeReg(RegPaConfig, 0);

	/* Sets a Frequency in LF band */
	set_frequency(434000000);

	/* Launch Rx chain calibration for LF band */
	writeReg(FSKRegImageCal, (readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
	while ((readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING);

	/* Sets a Frequency in HF band */
	set_frequency(868000000);

	/* Launch Rx chain calibration for HF band */
	writeReg(FSKRegImageCal, (readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
	while ((readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING);

	/* Restore context */
	writeReg(RegPaConfig, pa_backup);
	writeReg(RegFrfMsb, (uint8_t) (frf_backup >> 16));
	writeReg(RegFrfMid, (uint8_t) (frf_backup >> 8));
	writeReg(RegFrfLsb, (uint8_t) (frf_backup >> 0));
}

// get random seed from wideband noise rssi
void radio_init()
{
	uint8_t keybuf[16];

	system_disable_irqs();

	system_enable_sx_power(1);

	// manually reset radio
	radio_reset(1);
	os_delay(ms2ostime(1)); // wait >100us
	radio_reset(0);
	os_delay(ms2ostime(5)); // wait 5ms

	writeReg(0x24, 0);

	opmode(OPMODE_SLEEP);

	RADIO.sync_word = DEFAULT_MAC_PREAMBLE;
	RADIO.invert_iq = DEFAULT_INVERT_IQ;
	RADIO.payload_length = DEFAULT_PAYLOAD_LENGTH;

#ifdef CFG_sx1276_radio
#ifdef HAS_TCXO
	/* If we use a TCXO, be sure the TCXO bit is enabled */
	writeReg(RegTcxo, (readReg(RegTcxo) | 0x10));
#endif
#endif

	// some sanity checks, e.g., read version number
	uint8_t v = readReg(RegVersion);
#ifdef CFG_sx1276_radio
	if (!(v == 0x11) && !(v == 0x12))
		ASSERT(0);
#elif CFG_sx1272_radio
	ASSERT(v == 0x22);
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif
	/* Start the clock for RSSI reading and RF calibration loops */
	system_enable_sx_clock(1);

	// seed 15-byte randomness via noise rssi
	rxlora(RXMODE_RSSI);
	while ((readReg(RegOpMode) & OPMODE_MASK) != OPMODE_RX); // continuous rx
	for (int i = 1; i < 16; i++) {
		for (int j = 0; j < 8; j++) {
			uint8_t b; // wait for two non-identical subsequent least-significant bits
			while ((b = readReg(LORARegRssiWideband) & 0x01) == (readReg(LORARegRssiWideband) & 0x01));
			randbuf[i] = (randbuf[i] << 1) | b;
		}
	}
	randbuf[0] = 16; // set initial index

	// seed 16-byte randomness via noise rssi for the AES KEY
	rxlora(RXMODE_RSSI);
	while ((readReg(RegOpMode) & OPMODE_MASK) != OPMODE_RX); // continuous rx
	for (int i = 0; i < 16; i++) {
		for (int j = 0; j < 8; j++) {
			uint8_t b; // wait for two non-identical subsequent least-significant bits
			while ((b = readReg(LORARegRssiWideband) & 0x01) == (readReg(LORARegRssiWideband) & 0x01));
			keybuf[i] = (keybuf[i] << 1) | b;
		}
	}
	aes_set_key(keybuf, 16, &AesContext);

	rx_chain_calibration();

	opmode(OPMODE_SLEEP);

	/* Disable all RF paths */
	system_try_select_rf_path(PATH_NONE);

	/* Stop the clock */
	system_enable_sx_clock(0);

	system_enable_irqs();
}

void radio_reset(uint8_t val)
{
	if (val) {
#ifdef CFG_sx1276_radio
		system_set_sx_rst(0); // drive RST pin low
#else
		system_set_sx_rst(1); // drive RST pin high
#endif
	} else {
		system_set_sx_rst(2); // configure RST pin floating!
	}
}

// return next random byte derived from seed buffer
// (buf[0] holds index of next byte to be returned)
uint8_t radio_rand1()
{
	uint8_t i = randbuf[0];
	ASSERT(i != 0);
	if (i == 16) {
		aes_encrypt(randbuf, randbuf, &AesContext);
		i = 0;
	}
	uint8_t v = randbuf[i++];
	randbuf[0] = i;
	return v;
}

uint8_t radio_rssi()
{
	system_disable_irqs();
	uint8_t r = readReg(LORARegRssiValue);
	system_enable_irqs();
	return r;
}

static const uint16_t LORA_RXDONE_FIXUP[] = {
	[FSK] = us2ostime(0),      // (   0 ticks)
	[SF7] = us2ostime(0),      // (   0 ticks)
	[SF8] = us2ostime(1648),   // (  54 ticks)
	[SF9] = us2ostime(3265),   // ( 107 ticks)
	[SF10] = us2ostime(7049),  // ( 231 ticks)
	[SF11] = us2ostime(13641), // ( 447 ticks)
	[SF12] = us2ostime(31189), // (1022 ticks)
};

// called by hal ext IRQ handler
// (radio goes to stanby mode after tx/rx operations)
void radio_irq_handler(uint8_t dio)
{
	os_time_t now = os_get_time();

	if (dio > 2) {
		/* Handle only SX_D0 to SX_D2 for now */
		return;
	}

	if ((readReg(RegOpMode) & OPMODE_LORA) != 0) { // LORA modem
		uint8_t flags = readReg(LORARegIrqFlags);
		if (flags & IRQ_LORA_TXDONE_MASK) {
			// save exact tx time
			RADIO.txend = now - us2ostime(43); // TXDONE FIXUP
		} else if (flags & IRQ_LORA_RXDONE_MASK) {
			// save exact rx time
			if (getBw(RADIO.rps) == BW125) {
				now -= LORA_RXDONE_FIXUP[getSf(RADIO.rps)];
			}
			RADIO.rxtime = now;
			// read the PDU and inform the MAC that we received something
			RADIO.dataLen = (readReg(LORARegModemConfig1) & SX1272_MC1_IMPLICIT_HEADER_MODE_ON) ? readReg(LORARegPayloadLength)
													    : readReg(LORARegRxNbBytes);
			// set FIFO read address pointer
			writeReg(LORARegFifoAddrPtr, readReg(LORARegFifoRxCurrentAddr));
			// now read the FIFO
			readBuf(RegFifo, RADIO.frame, RADIO.dataLen);
			// read rx quality parameters
			RADIO.snr = ((int8_t) readReg(LORARegPktSnrValue)) / 4; // SNR [dB] = value / 4;
			int16_t rssi = readReg(LORARegPktRssiValue);
			if (RADIO.snr < 0) {
				RADIO.rssi = rssi + (rssi / 16) + RADIO.snr - 157;
			} else {
				RADIO.rssi = rssi + (rssi / 16) - 157;
			}

			// remove LNA gain (if any)
			RADIO.rssi -= LNA_GAIN;

			RADIO.freq_delta = (readReg(LORARegFeiMsb) << 16);
			RADIO.freq_delta |= (readReg(LORARegFeiMib) << 8);
			RADIO.freq_delta |= (readReg(LORARegFeiLsb));
			if (((readReg(LORARegModemConfig1) & 0x1) && (readReg(LORARegModemConfig2) & (0x1 << 2))) ||
			    (!(readReg(LORARegModemConfig1) & 0x1) && (readReg(LORARegHopChannel) & (0x1 << 6)))) {
				RADIO.crc_ok = (flags & IRQ_LORA_CRCERR_MASK) == 0;
			} else {
				RADIO.crc_ok = 2;
			}
		} else if (flags & IRQ_LORA_RXTOUT_MASK) {
			// indicate timeout
			RADIO.dataLen = 0;
		}
		// mask all radio IRQs
		if ((readReg(0x1) & OPMODE_MASK) != OPMODE_RX) {
			writeReg(LORARegIrqFlagsMask, 0xFF);
		}
		// clear radio IRQ flags
		writeReg(LORARegIrqFlags, 0xFF);
	} else { // FSK modem
		uint8_t flags1 = readReg(FSKRegIrqFlags1);
		uint8_t flags2 = readReg(FSKRegIrqFlags2);
		if (flags2 & IRQ_FSK2_PACKETSENT_MASK) {
			// save exact tx time
			RADIO.txend = now;
		} else if (flags2 & IRQ_FSK2_PAYLOADREADY_MASK) {
			// save exact rx time
			RADIO.rxtime = now;
			// read the PDU and inform the MAC that we received something
			RADIO.dataLen = readReg(FSKRegPayloadLength);
			// now read the FIFO
			readBuf(RegFifo, RADIO.frame, RADIO.dataLen);
			// read rx quality parameters
			RADIO.snr = 0;  // determine snr
			RADIO.rssi = 0; // determine rssi
		} else if (flags1 & IRQ_FSK1_TIMEOUT_MASK) {
			// indicate timeout
			RADIO.dataLen = 0;
		} else {
			while (1);
		}
	}
	// go from stanby to sleep
	if ((readReg(0x1) & OPMODE_MASK) != OPMODE_RX) {
		opmode(OPMODE_SLEEP);

		/* Stop the clock */
		system_enable_sx_clock(0);

		/* Disable DCDC high power mode */
		system_set_dcdc_high_power(0);
	}
	// run os job (use preset func ptr)
	if (RADIO.osjob.callback != NULL) {
		os_post_job(&RADIO.osjob, RADIO.osjob.callback);
	}
}

uint8_t radio_read(uint8_t reg)
{
	return readReg(reg);
}

void radio_write(uint8_t addr, uint8_t data)
{
	writeReg(addr, data);
}

void os_radio(uint8_t mode)
{
	system_disable_irqs();
	switch (mode) {
		case RADIO_RST:
			/* Be sure PA_BOOST is disabled */
			RADIO.txpow = 0;
			configPower();
			/* Disable all RF paths (up to 40µA per path) */
			system_try_select_rf_path(PATH_NONE);
			// put radio to sleep
			opmode(OPMODE_SLEEP);

			/* Stop the clock (likely done already) */
			system_enable_sx_clock(0);

			/* Disable DCDC high power mode (likely done already) */
			system_set_dcdc_high_power(0);
			break;

		case RADIO_TX:
			/* Enable DCDC high power mode */
			system_set_dcdc_high_power(1);

			/* Start the clock */
			system_enable_sx_clock(1);

			// transmit frame now
			starttx(); // buf=RADIO.frame, len=RADIO.dataLen
			break;

		case RADIO_RX:
			/* Enable DCDC high power mode */
			system_set_dcdc_high_power(1);

			/* Start the clock */
			system_enable_sx_clock(1);

			// receive frame now (exactly at rxtime)
			startrx(RXMODE_SINGLE); // buf=RADIO.frame, time=RADIO.rxtime, timeout=RADIO.rxsyms
			break;

		case RADIO_RXON:
			/* Enable DCDC high power mode */
			system_set_dcdc_high_power(1);

			/* Start the clock */
			system_enable_sx_clock(1);

			// start scanning for beacon now
			startrx(RXMODE_SCAN); // buf=RADIO.frame
			break;
	}
	system_enable_irqs();
}
