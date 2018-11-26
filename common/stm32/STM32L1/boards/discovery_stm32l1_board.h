/* ***********************************************************************************
 *
 * STM32L152 Discovery Kit (32L152CDISCOVERY) + SX1276RF1JAS board from Semtech
 *
 *************************************************************************************/

#pragma once

// RF characteristics
#define MAX_RF_POWER			20 // dBm
#define MIN_RF_POWER			2 // dBm
#define ANTENNA_GAIN			-6 // dBi
#define LNA_GAIN			0 // dB

// GPIOs
#define NSS_PORT			GPIO_PORT_A // NSS: PA4, sx1276
#define NSS_PIN				4

#define TX_PA_PORT			GPIO_PORT_C // TX_PA:  PC6
#define TX_PA_PIN			6

#define RST_PORT			GPIO_PORT_A // RST: PA11
#define RST_PIN				11

#define DIO0_PORT			GPIO_PORT_C // DIO0: PC1, sx1276   (line 1 irq handler)
#define DIO0_PIN			1
#define DIO0_TYPE			GPIO_PUPD_NONE
#define DIO1_PORT			GPIO_PORT_C // DIO1: PC2, sx1276  (line 2 irq handler)
#define DIO1_PIN			2
#define DIO1_TYPE			GPIO_PUPD_NONE
#define DIO2_PORT			GPIO_PORT_C // DIO2: PC3, sx1276  (line 3 irq handler)
#define DIO2_PIN			3
#define DIO2_TYPE			GPIO_PUPD_NONE

#define LED0_PORT			GPIO_PORT_B // Blue LED: PB6
#define LED0_PIN			6
#define LED0_MODE			LED_ACTIVE_HIGH
#define LED1_PORT			GPIO_PORT_B // Green LED: PB7
#define LED1_PIN			7
#define LED1_MODE			LED_ACTIVE_HIGH

#define BUTTON0_PORT			GPIO_PORT_A // PA0
#define BUTTON0_PIN			0
#define BUTTON0_TYPE			GPIO_PUPD_NONE
#define BUTTON0_MODE			BUTTON_ACTIVE_HIGH

#define OUTPUT_PINS			{{NSS_PORT, NSS_PIN}, {TX_PA_PORT, TX_PA_PIN}, {LED0_PORT, LED0_PIN}, {LED1_PORT, LED1_PIN}}
#define INPUT_PINS			{{DIO0_PORT, DIO0_PIN, DIO0_TYPE}, {DIO1_PORT, DIO1_PIN, DIO1_TYPE}, {DIO2_PORT, DIO2_PIN, DIO2_TYPE}, {BUTTON0_PORT, BUTTON0_PIN, BUTTON0_TYPE}}

// SPI
#define SCK_PORT			GPIO_PORT_A // SCK:  PA5
#define SCK_PIN				5
#define MISO_PORT			GPIO_PORT_A // MISO: PA6
#define MISO_PIN			6
#define MOSI_PORT			GPIO_PORT_A // MOSI: PA7
#define MOSI_PIN			7

#define GPIO_AF_SPI1			0x05

// Console USART
#define CONSOLE_USART			USART_PORT_1

#define CONSOLE_USART_TX_PORT		GPIO_PORT_A
#define CONSOLE_USART_TX_PIN		9
#define CONSOLE_USART_RX_PORT		GPIO_PORT_A
#define CONSOLE_USART_RX_PIN		10

#define CONSOLE_USART_AF		0x07

#define LOWPOWER_PINS			{{NSS_PORT, NSS_PIN}, {TX_PA_PORT, TX_PA_PIN}, {RST_PORT, RST_PIN}, \
					 {DIO0_PORT, DIO0_PIN}, {DIO1_PORT, DIO1_PIN}, {DIO2_PORT, DIO2_PIN}, \
					 {LED0_PORT, LED0_PIN}, {LED1_PORT, LED1_PIN}, {BUTTON0_PORT, BUTTON0_PIN}, \
					 {SCK_PORT, SCK_PIN}, {MISO_PORT, MISO_PIN}, {MOSI_PORT, MOSI_PIN}}
