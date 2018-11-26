/* ***********************************************************************************
 *
 * MURATA CMWX1ZZABZ module alone
 *
 *************************************************************************************/

#pragma once

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
