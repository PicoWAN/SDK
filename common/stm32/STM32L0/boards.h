/*! \file
 * \brief Definitions of various boards based on the STM32L0
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

#ifndef _BOARDS_H_
#define _BOARDS_H_

#include <gpio.h>

#if defined(CFG_murata_sychip_board)
/* ***********************************************************************************
 *
 * MURATA SyChip Board (early prototype with an STM32L082CZ)
 *
 *************************************************************************************/

#define HAS_TCXO

// RF characteristics
#define MAX_RF_POWER			20 // dBm
#define MIN_RF_POWER			0 // dBm
#define ANTENNA_GAIN			0 // dBi
#define LNA_GAIN			0 // dB

// GPIOs
#define NSS_PORT			GPIO_PORT_A // NSS: PA15, sx1276
#define NSS_PIN				15

#define RX_PORT				GPIO_PORT_A // RX:  PA1
#define RX_PIN				1
#define TX_RFO_PORT			GPIO_PORT_C // TX_RFO:  PC2
#define TX_RFO_PIN			2
#define TX_PA_PORT			GPIO_PORT_C // TX_PA:  PC1
#define TX_PA_PIN			1
#define TCXO_PORT			GPIO_PORT_A // PA12
#define TCXO_PIN			12

#define RST_PORT			GPIO_PORT_C // RST: PC0
#define RST_PIN				0

#define DIO0_PORT			GPIO_PORT_B // DIO0: PB4, sx1276  (line 1 irq handler)
#define DIO0_PIN			4
#define DIO0_TYPE			GPIO_PUPD_NONE
#define DIO1_PORT			GPIO_PORT_B // DIO1: PB1, sx1276  (line 2 irq handler)
#define DIO1_PIN			1
#define DIO1_TYPE			GPIO_PUPD_NONE
#define DIO2_PORT			GPIO_PORT_B // DIO2: PB0, sx1276  (line 3 irq handler)
#define DIO2_PIN			0
#define DIO2_TYPE			GPIO_PUPD_NONE

#define LED0_PORT			GPIO_PORT_B // Red LED: PB12
#define LED0_PIN			12
#define LED0_MODE			LED_ACTIVE_HIGH
#define LED1_PORT			GPIO_PORT_B // Yellow LED: PB13
#define LED1_PIN			13
#define LED1_MODE			LED_ACTIVE_HIGH
#define LED2_PORT			GPIO_PORT_B // Green LED: PB14
#define LED2_PIN			14
#define LED2_MODE			LED_ACTIVE_HIGH

#define BUTTON0_PORT			GPIO_PORT_B // PB15
#define BUTTON0_PIN			15
#define BUTTON0_MODE			BUTTON_ACTIVE_LOW
#define BUTTON0_TYPE			GPIO_PUPD_UP

#define OUTPUT_PINS			{{NSS_PORT, NSS_PIN}, {RX_PORT, RX_PIN}, {TX_RFO_PORT, TX_RFO_PIN}, \
					 {TX_PA_PORT, TX_PA_PIN}, \
					 {LED0_PORT, LED0_PIN}, {LED1_PORT, LED1_PIN}, {LED2_PORT, LED2_PIN}, \
					 {TCXO_PORT, TCXO_PIN}}
#define INPUT_PINS			{{DIO0_PORT, DIO0_PIN, DIO0_TYPE}, {DIO1_PORT, DIO1_PIN, DIO1_TYPE}, {DIO2_PORT, DIO2_PIN, DIO2_TYPE}, \
					 {BUTTON0_PORT, BUTTON0_PIN, BUTTON0_TYPE}}

// SPI
#define SCK_PORT			GPIO_PORT_B // SCK:  PB3
#define SCK_PIN				3
#define MISO_PORT			GPIO_PORT_A // MISO: PA6
#define MISO_PIN			6
#define MOSI_PORT			GPIO_PORT_A // MOSI: PA7
#define MOSI_PIN			7

#define GPIO_AF_SPI1			0x00

// Console USART
#define CONSOLE_USART			LPUART_PORT_1

#define CONSOLE_USART_TX_PORT		GPIO_PORT_A // PA2
#define CONSOLE_USART_TX_PIN		2
#define CONSOLE_USART_RX_PORT		GPIO_PORT_A // PA3
#define CONSOLE_USART_RX_PIN		3

#define CONSOLE_USART_AF		0x06

#define LOWPOWER_PINS			{{NSS_PORT, NSS_PIN}, {TX_RFO_PORT, TX_RFO_PIN}, {RX_PORT, RX_PIN}, \
					 {TX_PA_PORT, TX_PA_PIN}, {RST_PORT, RST_PIN}, \
					 {DIO0_PORT, DIO0_PIN}, {DIO1_PORT, DIO1_PIN}, {DIO2_PORT, DIO2_PIN}, \
					 {LED0_PORT, LED0_PIN}, {LED1_PORT, LED1_PIN}, {LED2_PORT, LED2_PIN}, \
					 {CONSOLE_USART_TX_PORT, CONSOLE_USART_TX_PIN}, {CONSOLE_USART_RX_PORT, CONSOLE_USART_RX_PIN}, \
					 {BUTTON0_PORT, BUTTON0_PIN}, \
					 {TCXO_PORT, TCXO_PIN}}
#elif defined(CFG_murata_discovery_board)
/* ***********************************************************************************
 *
 * MURATA Discovery Kit (B-L072Z-LRWAN1)
 *
 *************************************************************************************/

#define HAS_TCXO

// RF characteristics
#define MAX_RF_POWER			20 // dBm
#define MIN_RF_POWER			0 // dBm
#define ANTENNA_GAIN			0 // dBi
#define LNA_GAIN			0 // dB

// GPIOs
#define NSS_PORT			GPIO_PORT_A // NSS: PA15, sx1276
#define NSS_PIN				15

#define RX_PORT				GPIO_PORT_A // RX:  PA1
#define RX_PIN				1
#define TX_RFO_PORT			GPIO_PORT_C // TX_RFO:  PC2
#define TX_RFO_PIN			2
#define TX_PA_PORT			GPIO_PORT_C // TX_PA:  PC1
#define TX_PA_PIN			1
#define TCXO_PORT			GPIO_PORT_A // PA12
#define TCXO_PIN			12

#define RST_PORT			2 // RST: PC0
#define RST_PIN				0

#define DIO0_PORT			GPIO_PORT_B // DIO0: PB4, sx1276  (line 1 irq handler)
#define DIO0_PIN			4
#define DIO0_TYPE			GPIO_PUPD_NONE
#define DIO1_PORT			GPIO_PORT_B // DIO1: PB1, sx1276  (line 2 irq handler)
#define DIO1_PIN			1
#define DIO1_TYPE			GPIO_PUPD_NONE
#define DIO2_PORT			GPIO_PORT_B // DIO2: PB0, sx1276  (line 3 irq handler)
#define DIO2_PIN			0
#define DIO2_TYPE			GPIO_PUPD_NONE

#define LED0_PORT			GPIO_PORT_B // Red LED: PB7 (LED4)
#define LED0_PIN			7
#define LED0_MODE			LED_ACTIVE_HIGH
#define LED1_PORT			GPIO_PORT_B // Blue LED: PB6 (LED3)
#define LED1_PIN			6
#define LED1_MODE			LED_ACTIVE_HIGH
#define LED2_PORT			GPIO_PORT_A // Green LED: PA5 (LED2)
#define LED2_PIN			5
#define LED2_MODE			LED_ACTIVE_HIGH
#define LED3_PORT			GPIO_PORT_B // Green LED: PB5 (LED1)
#define LED3_PIN			5
#define LED3_MODE			LED_ACTIVE_HIGH

#define BUTTON0_PORT			GPIO_PORT_B // PB2
#define BUTTON0_PIN			2
#define BUTTON0_MODE			BUTTON_ACTIVE_LOW
#define BUTTON0_TYPE			GPIO_PUPD_NONE

#define OUTPUT_PINS			{{NSS_PORT, NSS_PIN}, {RX_PORT, RX_PIN}, {TX_RFO_PORT, TX_RFO_PIN}, \
					 {TX_PA_PORT, TX_PA_PIN}, \
					 {LED0_PORT, LED0_PIN}, {LED1_PORT, LED1_PIN}, {LED2_PORT, LED2_PIN}, {LED3_PORT, LED3_PIN}, \
					 {TCXO_PORT, TCXO_PIN}}
#define INPUT_PINS			{{DIO0_PORT, DIO0_PIN, DIO0_TYPE}, {DIO1_PORT, DIO1_PIN, DIO1_TYPE}, {DIO2_PORT, DIO2_PIN, DIO2_TYPE}, \
					 {BUTTON0_PORT, BUTTON0_PIN, BUTTON0_TYPE}}

// SPI
#define SCK_PORT			GPIO_PORT_B // SCK:  PB3
#define SCK_PIN				3
#define MISO_PORT			GPIO_PORT_A // MISO: PA6
#define MISO_PIN			6
#define MOSI_PORT			GPIO_PORT_A // MOSI: PA7
#define MOSI_PIN			7

#define GPIO_AF_SPI1			0x00

// Console USART
#define CONSOLE_USART			LPUART_PORT_1

#define CONSOLE_USART_TX_PORT		GPIO_PORT_A // PA2
#define CONSOLE_USART_TX_PIN		2
#define CONSOLE_USART_RX_PORT		GPIO_PORT_A // PA3
#define CONSOLE_USART_RX_PIN		3

#define CONSOLE_USART_AF		0x06

#define LOWPOWER_PINS			{{NSS_PORT, NSS_PIN}, {TX_RFO_PORT, TX_RFO_PIN}, {RX_PORT, RX_PIN}, \
					 {TX_PA_PORT, TX_PA_PIN}, {RST_PORT, RST_PIN}, \
					 {DIO0_PORT, DIO0_PIN}, {DIO1_PORT, DIO1_PIN}, {DIO2_PORT, DIO2_PIN}, \
					 {LED0_PORT, LED0_PIN}, {LED1_PORT, LED1_PIN}, {LED2_PORT, LED2_PIN}, {LED3_PORT, LED3_PIN}, \
					 {CONSOLE_USART_TX_PORT, CONSOLE_USART_TX_PIN}, {CONSOLE_USART_RX_PORT, CONSOLE_USART_RX_PIN}, \
					 {BUTTON0_PORT, BUTTON0_PIN}, \
					 {TCXO_PORT, TCXO_PIN}}
#elif defined(CFG_murata_module_board)
/* ***********************************************************************************
 *
 * MURATA CMWX1ZZABZ module alone
 *
 *************************************************************************************/

#define HAS_TCXO

// RF characteristics
#define MAX_RF_POWER			20 // dBm
#define MIN_RF_POWER			0 // dBm
#define ANTENNA_GAIN			0 // dBi
#define LNA_GAIN			0 // dB

// GPIOs
#define NSS_PORT			GPIO_PORT_A // NSS: PA15, sx1276
#define NSS_PIN				15

#define RX_PORT				GPIO_PORT_A // RX:  PA1
#define RX_PIN				1
#define TX_RFO_PORT			GPIO_PORT_C // TX_RFO:  PC2
#define TX_RFO_PIN			2
#define TX_PA_PORT			GPIO_PORT_C // TX_PA:  PC1
#define TX_PA_PIN			1
#define TCXO_PORT			GPIO_PORT_A // PA12
#define TCXO_PIN			12

#define RST_PORT			GPIO_PORT_C // RST: PC0
#define RST_PIN				0

#define DIO0_PORT			GPIO_PORT_B // DIO0: PB4, sx1276  (line 1 irq handler)
#define DIO0_PIN			4
#define DIO0_TYPE			GPIO_PUPD_NONE
#define DIO1_PORT			GPIO_PORT_B // DIO1: PB1, sx1276  (line 2 irq handler)
#define DIO1_PIN			1
#define DIO1_TYPE			GPIO_PUPD_NONE
#define DIO2_PORT			GPIO_PORT_B // DIO2: PB0, sx1276  (line 3 irq handler)
#define DIO2_PIN			0
#define DIO2_TYPE			GPIO_PUPD_NONE

#define OUTPUT_PINS			{{NSS_PORT, NSS_PIN}, {RX_PORT, RX_PIN}, {TX_RFO_PORT, TX_RFO_PIN}, \
					 {TX_PA_PORT, TX_PA_PIN}, \
					 {TCXO_PORT, TCXO_PIN}}
#define INPUT_PINS			{{DIO0_PORT, DIO0_PIN, DIO0_TYPE}, {DIO1_PORT, DIO1_PIN, DIO1_TYPE}, {DIO2_PORT, DIO2_PIN, DIO2_TYPE}}

// SPI
#define SCK_PORT			GPIO_PORT_B // SCK:  PB3
#define SCK_PIN				3
#define MISO_PORT			GPIO_PORT_A // MISO: PA6
#define MISO_PIN			6
#define MOSI_PORT			GPIO_PORT_A // MOSI: PA7
#define MOSI_PIN			7

#define GPIO_AF_SPI1			0x00

// Console USART
#define CONSOLE_USART			LPUART_PORT_1

#define CONSOLE_USART_TX_PORT		GPIO_PORT_A // PA2
#define CONSOLE_USART_TX_PIN		2
#define CONSOLE_USART_RX_PORT		GPIO_PORT_A // PA3
#define CONSOLE_USART_RX_PIN		3

#define CONSOLE_USART_AF		0x06

#define LOWPOWER_PINS			{{NSS_PORT, NSS_PIN}, {TX_RFO_PORT, TX_RFO_PIN}, {RX_PORT, RX_PIN}, \
					 {TX_PA_PORT, TX_PA_PIN}, {RST_PORT, RST_PIN}, \
					 {DIO0_PORT, DIO0_PIN}, {DIO1_PORT, DIO1_PIN}, {DIO2_PORT, DIO2_PIN}, \
					 {CONSOLE_USART_TX_PORT, CONSOLE_USART_TX_PIN}, {CONSOLE_USART_RX_PORT, CONSOLE_USART_RX_PIN}, \
					 {TCXO_PORT, TCXO_PIN}}
#elif defined(CFG_nucleo_board)
/* ***********************************************************************************
 *
 * STM32 Nucleo pack (NUCLEO-L073RZ + SX1276MB1LAS)
 *
 *************************************************************************************/

// RF characteristics
#define MAX_RF_POWER			14 // dBm
#define MIN_RF_POWER			0 // dBm
#define ANTENNA_GAIN			0 // dBi
#define LNA_GAIN			0 // dB

// GPIOs
#define NSS_PORT			GPIO_PORT_B // NSS: PB6, sx1276
#define NSS_PIN				6

#define TX_RFO_PORT			GPIO_PORT_C // TX_RFO:  PC1
#define TX_RFO_PIN			1

#define RST_PORT			GPIO_PORT_A // RST: PA0
#define RST_PIN				0

#define DIO0_PORT			GPIO_PORT_A // DIO0: PA10, sx1276  (line 1 irq handler)
#define DIO0_PIN			10
#define DIO0_TYPE			GPIO_PUPD_NONE
#define DIO1_PORT			GPIO_PORT_B // DIO1: PB3, sx1276  (line 2 irq handler)
#define DIO1_PIN			3
#define DIO1_TYPE			GPIO_PUPD_NONE
#define DIO2_PORT			GPIO_PORT_B // DIO2: PB5, sx1276  (line 3 irq handler)
#define DIO2_PIN			5
#define DIO2_TYPE			GPIO_PUPD_NONE

#define LED0_PORT			GPIO_PORT_A // Red LED: PA1
#define LED0_PIN			1
#define LED0_MODE			LED_ACTIVE_HIGH
#define LED1_PORT			GPIO_PORT_A // Blue LED: PA4
#define LED1_PIN			4
#define LED1_MODE			LED_ACTIVE_HIGH

#define BUTTON0_PORT			GPIO_PORT_C // PC13
#define BUTTON0_PIN			13
#define BUTTON0_MODE			BUTTON_ACTIVE_LOW
#define BUTTON0_TYPE			GPIO_PUPD_NONE

#define OUTPUT_PINS			{{NSS_PORT, NSS_PIN}, {TX_RFO_PORT, TX_RFO_PIN}, \
					 {LED0_PORT, LED0_PIN}, {LED1_PORT, LED1_PIN}}
#define INPUT_PINS			{{DIO0_PORT, DIO0_PIN, DIO0_TYPE}, {DIO1_PORT, DIO1_PIN, DIO1_TYPE}, {DIO2_PORT, DIO2_PIN, DIO2_TYPE}, \
					 {BUTTON0_PORT, BUTTON0_PIN, BUTTON0_TYPE}}

// SPI
#define SCK_PORT			GPIO_PORT_A // SCK:  PA5
#define SCK_PIN				5
#define MISO_PORT			GPIO_PORT_A // MISO: PA6
#define MISO_PIN			6
#define MOSI_PORT			GPIO_PORT_A // MOSI: PA7
#define MOSI_PIN			7

#define GPIO_AF_SPI1			0x00

// Console USART
#define CONSOLE_USART			LPUART_PORT_1

#define CONSOLE_USART_TX_PORT		GPIO_PORT_A // PA2
#define CONSOLE_USART_TX_PIN		2
#define CONSOLE_USART_RX_PORT		GPIO_PORT_A // PA3
#define CONSOLE_USART_RX_PIN		3

#define CONSOLE_USART_AF		0x06

#define LOWPOWER_PINS			{{NSS_PORT, NSS_PIN}, {TX_RFO_PORT, TX_RFO_PIN}, \
					 {RST_PORT, RST_PIN}, \
					 {DIO0_PORT, DIO0_PIN}, {DIO1_PORT, DIO1_PIN}, {DIO2_PORT, DIO2_PIN}, \
					 {LED0_PORT, LED0_PIN}, {LED1_PORT, LED1_PIN}, \
					 {CONSOLE_USART_TX_PORT, CONSOLE_USART_TX_PIN}, {CONSOLE_USART_RX_PORT, CONSOLE_USART_RX_PIN}, \
					 {BUTTON0_PORT, BUTTON0_PIN}}
#elif defined(CFG_picoshield_board)
/* ***********************************************************************************
 *
 * Archos PicoShield for Arduino
 *
 *************************************************************************************/

#define HAS_TCXO

// RF characteristics
#define MAX_RF_POWER			20 // dBm
#define MIN_RF_POWER			0 // dBm
#define ANTENNA_GAIN			0 // dBi
#define LNA_GAIN			0 // dB

// GPIOs
#define NSS_PORT			GPIO_PORT_A // NSS: PA15, sx1276
#define NSS_PIN				15

#define RX_PORT				GPIO_PORT_A // RX:  PA1
#define RX_PIN				1
#define TX_RFO_PORT			GPIO_PORT_C // TX_RFO:  PC2
#define TX_RFO_PIN			2
#define TX_PA_PORT			GPIO_PORT_C // TX_PA:  PC1
#define TX_PA_PIN			1
#define TCXO_PORT			GPIO_PORT_H // PH1
#define TCXO_PIN			1

#define RST_PORT			GPIO_PORT_C // RST: PC0
#define RST_PIN				0

#define DIO0_PORT			GPIO_PORT_B // DIO0: PB4, sx1276  (line 1 irq handler)
#define DIO0_PIN			4
#define DIO0_TYPE			GPIO_PUPD_NONE
#define DIO1_PORT			GPIO_PORT_B // DIO1: PB1, sx1276  (line 2 irq handler)
#define DIO1_PIN			1
#define DIO1_TYPE			GPIO_PUPD_NONE
#define DIO2_PORT			GPIO_PORT_B // DIO2: PB0, sx1276  (line 3 irq handler)
#define DIO2_PIN			0
#define DIO2_TYPE			GPIO_PUPD_NONE

#define LED0_PORT			GPIO_PORT_B // Red LED: PB2
#define LED0_PIN			2
#define LED0_MODE			LED_ACTIVE_HIGH
#define LED1_PORT			GPIO_PORT_A // Green LED: PA8
#define LED1_PIN			8
#define LED1_MODE			LED_ACTIVE_HIGH

#define OUTPUT_PINS			{{NSS_PORT, NSS_PIN}, {RX_PORT, RX_PIN}, {TX_RFO_PORT, TX_RFO_PIN}, \
					 {TX_PA_PORT, TX_PA_PIN}, \
					 {LED0_PORT, LED0_PIN}, {LED1_PORT, LED1_PIN}, \
					 {TCXO_PORT, TCXO_PIN}}
#define INPUT_PINS			{{DIO0_PORT, DIO0_PIN, DIO0_TYPE}, {DIO1_PORT, DIO1_PIN, DIO1_TYPE}, {DIO2_PORT, DIO2_PIN, DIO2_TYPE}}

// SPI
#define SCK_PORT			GPIO_PORT_B // SCK:  PB3
#define SCK_PIN				3
#define MISO_PORT			GPIO_PORT_A // MISO: PA6
#define MISO_PIN			6
#define MOSI_PORT			GPIO_PORT_A // MOSI: PA7
#define MOSI_PIN			7

#define GPIO_AF_SPI1			0x00

// Console USART
#define CONSOLE_USART			LPUART_PORT_1

#define CONSOLE_USART_TX_PORT		GPIO_PORT_A // PA2
#define CONSOLE_USART_TX_PIN		2
#define CONSOLE_USART_RX_PORT		GPIO_PORT_A // PA3
#define CONSOLE_USART_RX_PIN		3

#define CONSOLE_USART_AF		0x06

#define LOWPOWER_PINS			{{NSS_PORT, NSS_PIN}, {TX_RFO_PORT, TX_RFO_PIN}, {RX_PORT, RX_PIN}, \
					 {TX_PA_PORT, TX_PA_PIN}, {RST_PORT, RST_PIN}, \
					 {DIO0_PORT, DIO0_PIN}, {DIO1_PORT, DIO1_PIN}, {DIO2_PORT, DIO2_PIN}, \
					 {LED0_PORT, LED0_PIN}, {LED1_PORT, LED1_PIN}, \
					 {CONSOLE_USART_TX_PORT, CONSOLE_USART_TX_PIN}, {CONSOLE_USART_RX_PORT, CONSOLE_USART_RX_PIN}, \
					 {TCXO_PORT, TCXO_PIN}}
#else

#error Missing board configuration !

#endif


#endif /* _BOARDS_H_ */
