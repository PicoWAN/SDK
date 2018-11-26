/* ***********************************************************************************
 *
 * STM32 Nucleo pack (NUCLEO-L073RZ + SX1276MB1LAS)
 *
 *************************************************************************************/

#pragma once

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
