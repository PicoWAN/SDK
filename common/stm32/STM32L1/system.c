/*
 * system - System related stuff
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
#include <system.h>
#include <gpio.h>
#include <usart.h>
#include "boards.h"
#include "stm32l1xx.h"

/* --------- PWR registers bit address in the alias region ---------- */
#define PWR_OFFSET		(PWR_BASE - PERIPH_BASE)

/* --- CR Register ---*/

/* Alias word address of DBP bit */
#define CR_OFFSET		(PWR_OFFSET + 0x00)
#define DBP_BitNumber		0x08
#define CR_DBP_BB		(PERIPH_BB_BASE + (CR_OFFSET * 32) + (DBP_BitNumber * 4))

/* Alias word address of PVDE bit */
#define PVDE_BitNumber		0x04
#define CR_PVDE_BB		(PERIPH_BB_BASE + (CR_OFFSET * 32) + (PVDE_BitNumber * 4))

/* Alias word address of ULP bit */
#define ULP_BitNumber		0x09
#define CR_ULP_BB		(PERIPH_BB_BASE + (CR_OFFSET * 32) + (ULP_BitNumber * 4))

/* Alias word address of FWU bit */
#define FWU_BitNumber		0x0A
#define CR_FWU_BB		PERIPH_BB_BASE + (CR_OFFSET * 32) + (FWU_BitNumber * 4))

/* Variables used to save GPIO configuration */
static uint32_t GPIOA_MODER, GPIOB_MODER, GPIOC_MODER, GPIOD_MODER, GPIOE_MODER, GPIOH_MODER;
static uint32_t GPIOA_OSPEEDR, GPIOB_OSPEEDR, GPIOC_OSPEEDR, GPIOD_OSPEEDR, GPIOE_OSPEEDR, GPIOH_OSPEEDR;
static uint32_t GPIOA_OTYPER, GPIOB_OTYPER, GPIOC_OTYPER, GPIOD_OTYPER, GPIOE_OTYPER, GPIOH_OTYPER;
static uint32_t GPIOA_PUPDR, GPIOB_PUPDR, GPIOC_PUPDR, GPIOD_PUPDR, GPIOE_PUPDR, GPIOH_PUPDR;

static const io_pin_t output_pins[] = OUTPUT_PINS;
static const io_pin_t input_pins[] = INPUT_PINS;

static const io_pin_t lowpower_pins[] = LOWPOWER_PINS;

/* Pins to exclude from the low-power confiuration that have been registered by the application (must end with {PIN_END, PIN_END}) */
static io_pin_t *app_lowpower_pins = NULL;

#if defined(BATT_SYNC_PORT) && defined(BATT_SYNC_PIN)
static uint8_t high_power_ref_counter = 0;
#endif
#if defined(TCXO_PORT) && defined(TCXO_PIN)
static uint8_t sx_clock_ref_counter = 0;
#endif

#if defined(TCXO_PORT) && defined(TCXO_PIN)
static os_job_t disable_sx_clock_job;
#endif
#if defined(BATT_SYNC_PORT) && defined(BATT_SYNC_PIN)
static os_job_t disable_dcdc_job;
#endif

static uint8_t irqlevel = 0;


static void system_io_init(void)
{
	uint8_t i;

	/* Enable clocks for GPIO ports A, B, C, D, E and H */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOEEN | RCC_AHBENR_GPIOHEN;

	/* GPIOs: configure as output and set to LOW */
	for (i = 0; i < (sizeof(output_pins) / sizeof(io_pin_t)); i++) {
		gpio_cfg_pin(output_pins[i].port, output_pins[i].pin, GPIO_PMODE_OUTPUT, GPIO_OMODE_PUSH_PULL, GPIO_PUPD_NONE, GPIO_OSPEED_VHIGH, 0);
		gpio_set_pin(output_pins[i].port, output_pins[i].pin, 0);
	}

	/* GPIOs: configure as input */
	for (i = 0; i < (sizeof(input_pins) / sizeof(io_pin_t)); i++) {
		gpio_cfg_pin(input_pins[i].port, input_pins[i].pin, GPIO_PMODE_INPUT, GPIO_OMODE_PUSH_PULL, input_pins[i].pupd, GPIO_OSPEED_VHIGH, 0);
	}

	/* Enable and configure IRQ for the radio GPIOs */
#ifndef CFG_DISABLE_RADIO_IRQ_HANDLER
#if defined(DIO0_PORT) && defined(DIO0_PIN)
	gpio_config_irq(DIO0_PORT, DIO0_PIN, GPIO_IRQ_MODE_RISING);
#endif
#if defined(DIO1_PORT) && defined(DIO1_PIN)
	gpio_config_irq(DIO1_PORT, DIO1_PIN, GPIO_IRQ_MODE_RISING);
#endif
#if defined(DIO2_PORT) && defined(DIO2_PIN)
	gpio_config_irq(DIO2_PORT, DIO2_PIN, GPIO_IRQ_MODE_RISING);
#endif
#if defined(DIO3_PORT) && defined(DIO3_PIN)
	gpio_config_irq(DIO3_PORT, DIO3_PIN, GPIO_IRQ_MODE_RISING);
#endif
#if defined(DIO4_PORT) && defined(DIO4_PIN)
	gpio_config_irq(DIO4_PORT, DIO4_PIN, GPIO_IRQ_MODE_RISING);
#endif
#endif

	/* VBAT Measurement */
#if defined(BATT_ANA_PORT) && defined(BATT_ANA_PIN)
	gpio_cfg_pin(BATT_ANA_PORT, BATT_ANA_PIN, GPIO_PMODE_ANALOG, GPIO_OMODE_PUSH_PULL, GPIO_PUPD_NONE, GPIO_OSPEED_VHIGH, 0);
#endif

#if defined(SCK_PORT) && defined(SCK_PIN) && defined(MISO_PORT) && defined(MISO_PIN) && defined(MOSI_PORT) && defined(MOSI_PIN)
	/* SPI: use alternate function SPI1 (SCK, MISO, MOSI) */
	gpio_cfg_pin(SCK_PORT, SCK_PIN, GPIO_PMODE_ALT_FUNC, GPIO_OMODE_PUSH_PULL, GPIO_PUPD_NONE, GPIO_OSPEED_VHIGH, GPIO_AF_SPI1);
	gpio_cfg_pin(MISO_PORT, MISO_PIN, GPIO_PMODE_ALT_FUNC, GPIO_OMODE_PUSH_PULL, GPIO_PUPD_DOWN, GPIO_OSPEED_VHIGH, GPIO_AF_SPI1);
	gpio_cfg_pin(MOSI_PORT, MOSI_PIN, GPIO_PMODE_ALT_FUNC, GPIO_OMODE_PUSH_PULL, GPIO_PUPD_NONE, GPIO_OSPEED_VHIGH, GPIO_AF_SPI1);
#endif

#if defined(CONSOLE_USART_TX_PORT) && defined(CONSOLE_USART_TX_PIN)
	/* USART pins in USART mode */
	gpio_cfg_pin(CONSOLE_USART_TX_PORT, CONSOLE_USART_TX_PIN,
		     GPIO_PMODE_ALT_FUNC, GPIO_OMODE_PUSH_PULL, GPIO_PUPD_NONE, GPIO_OSPEED_VHIGH, CONSOLE_USART_AF);
#endif
#if defined(CONSOLE_USART_RX_PORT) && defined(CONSOLE_USART_RX_PIN)
	gpio_cfg_pin(CONSOLE_USART_RX_PORT, CONSOLE_USART_RX_PIN,
		     GPIO_PMODE_ALT_FUNC, GPIO_OMODE_PUSH_PULL, GPIO_PUPD_NONE, GPIO_OSPEED_VHIGH, CONSOLE_USART_AF);
#endif

#ifdef CFG_picotag_board
	/* WIFI pins left floating (high impedance) */
	gpio_cfg_pin(WIFI_GPIO0_PORT, WIFI_GPIO0_PIN, GPIO_PMODE_INPUT, GPIO_OMODE_PUSH_PULL, GPIO_PUPD_NONE, GPIO_OSPEED_VHIGH, 0);
	gpio_cfg_pin(WIFI_USART_TX_PORT, WIFI_USART_TX_PIN, GPIO_PMODE_INPUT, GPIO_OMODE_PUSH_PULL, GPIO_PUPD_NONE, GPIO_OSPEED_VHIGH, 0);
	gpio_cfg_pin(WIFI_USART_RX_PORT, WIFI_USART_RX_PIN, GPIO_PMODE_INPUT, GPIO_OMODE_PUSH_PULL, GPIO_PUPD_NONE, GPIO_OSPEED_VHIGH, 0);

	/* Be sure the WIFI chip is in a sane state by enabling it first,
	 * otherwise it can consume power even if the IOs are properly set.
	 */
	gpio_set_pin(WIFI_ENABLE_PORT, WIFI_ENABLE_PIN, 1);
	os_delay(ms2ostime(100));
	gpio_set_pin(WIFI_ENABLE_PORT, WIFI_ENABLE_PIN, 0);
#endif

#if defined(SCL_PORT) && defined(SCL_PIN) && defined(SDA_PORT) && defined(SDA_PIN)
	/* I2C1 */
	gpio_cfg_pin(SCL_PORT, SCL_PIN, GPIO_PMODE_ALT_FUNC, GPIO_OMODE_PUSH_PULL, GPIO_PUPD_NONE, GPIO_OSPEED_VHIGH, GPIO_AF_I2C1);
	gpio_cfg_pin(SDA_PORT, SDA_PIN, GPIO_PMODE_ALT_FUNC, GPIO_OMODE_PUSH_PULL, GPIO_PUPD_NONE, GPIO_OSPEED_VHIGH, GPIO_AF_I2C1);
#endif
}

#if defined(BATT_SYNC_PORT) && defined(BATT_SYNC_PIN)
static void disable_dcdc_task(os_job_t *j)
{
	/* Put the DCDC converter in Hysteresis mode (50 mA max) */
	gpio_set_pin(BATT_SYNC_PORT, BATT_SYNC_PIN, 0);

	/* Wait 500 µs to be sure the DCDC is stable */
	os_delay(us2ostime(500));
}
#endif

void system_set_dcdc_high_power(uint8_t en)
{
#if defined(BATT_SYNC_PORT) && defined(BATT_SYNC_PIN)
	/* Disable the ref counter until we are sure it is properly released */
	if (en) {
		// high_power_ref_counter++;
		high_power_ref_counter = 1;
	} else if (high_power_ref_counter > 0) {
		// high_power_ref_counter--;
		high_power_ref_counter = 0;
	}

	if (high_power_ref_counter > 0) {
		os_cancel_job(&disable_dcdc_job);
		if (gpio_get_pin(BATT_SYNC_PORT, BATT_SYNC_PIN) == 0) {
			/* Put the DCDC converter in PWM mode (500 mA max) */
			gpio_set_pin(BATT_SYNC_PORT, BATT_SYNC_PIN, 1);

			/* Wait 500 µs to be sure the DCDC is stable */
			os_delay(us2ostime(500));
		}
	} else {
		if (gpio_get_pin(BATT_SYNC_PORT, BATT_SYNC_PIN) == 1) {
			/* Put the DCDC converter in Hysteresis mode (50 mA max) in 10 ms */
			os_post_delayed_job(&disable_dcdc_job, os_get_time() + ms2ostime(10), disable_dcdc_task);
		}
	}
#endif
}

#if defined(TCXO_PORT) && defined(TCXO_PIN)
static void disable_sx_clock_task(os_job_t *j)
{
	/* Disable the 32 MHz TCXO */
	gpio_set_pin(TCXO_PORT, TCXO_PIN, 0);
}
#endif

void system_enable_sx_power(uint8_t en)
{
#if defined(POWER_RADIO_PORT) && defined(POWER_RADIO_PIN)
	if (en) {
		/* Enable SX power supply */
		gpio_set_pin(POWER_RADIO_PORT, POWER_RADIO_PIN, 1);
	} else {
		/* Disable SX power supply */
		gpio_set_pin(POWER_RADIO_PORT, POWER_RADIO_PIN, 0);
	}
#endif
}

void system_enable_sx_clock(uint8_t en)
{
#if defined(TCXO_PORT) && defined(TCXO_PIN)
	if (en) {
		sx_clock_ref_counter++;
	} else if (sx_clock_ref_counter > 0) {
		sx_clock_ref_counter--;
	}

	if (sx_clock_ref_counter > 0) {
		os_cancel_job(&disable_sx_clock_job);
		if (gpio_get_pin(TCXO_PORT, TCXO_PIN) == 0) {
			/* Enable the 32 MHz TCXO */
			gpio_set_pin(TCXO_PORT, TCXO_PIN, 1);

			/* Wait 1 ms to be sure the TCXO is stable */
			os_delay(ms2ostime(1));
		}
	} else {
		/* Disable the 32 MHz TCXO in 10 ms */
		os_post_delayed_job(&disable_sx_clock_job, os_get_time() + ms2ostime(10), disable_sx_clock_task);
	}
#endif
}

void system_try_select_rf_path(rf_path_t path)
{
	switch (path) {
		case PATH_RX:
#if defined(RX_PORT) && defined(RX_PIN)
			gpio_set_pin(RX_PORT, RX_PIN, 1);
#endif
#if defined(TX_RFO_PORT) && defined(TX_RFO_PIN)
			gpio_set_pin(TX_RFO_PORT, TX_RFO_PIN, 0);
#endif
#if defined(TX_PA_PORT) && defined(TX_PA_PIN)
			gpio_set_pin(TX_PA_PORT, TX_PA_PIN, 0);
#endif
			break;

		case PATH_TX_RFO:
			/* If both PA and TX pins exists, enable TX and disable PA.
			 * Otherwise, enable the one available.
			 */
#if defined(RX_PORT) && defined(RX_PIN)
			gpio_set_pin(RX_PORT, RX_PIN, 0);
#endif
#if defined(TX_RFO_PORT) && defined(TX_RFO_PIN)
			gpio_set_pin(TX_RFO_PORT, TX_RFO_PIN, 1);
#if defined(TX_PA_PORT) && defined(TX_PA_PIN)
			gpio_set_pin(TX_PA_PORT, TX_PA_PIN, 0);
#endif
#else
#if defined(TX_PA_PORT) && defined(TX_PA_PIN)
			gpio_set_pin(TX_PA_PORT, TX_PA_PIN, 1);
#endif
#endif
			break;

		case PATH_TX_PA:
			/* If both PA and TX pins exists, enable PA and disable TX.
			 * Otherwise, enable the one available.
			 */
#if defined(RX_PORT) && defined(RX_PIN)
			gpio_set_pin(RX_PORT, RX_PIN, 0);
#endif
#if defined(TX_PA_PORT) && defined(TX_PA_PIN)
			gpio_set_pin(TX_PA_PORT, TX_PA_PIN, 1);
#if defined(TX_RFO_PORT) && defined(TX_RFO_PIN)
			gpio_set_pin(TX_RFO_PORT, TX_RFO_PIN, 0);
#endif
#else
#if defined(TX_RFO_PORT) && defined(TX_RFO_PIN)
			gpio_set_pin(TX_RFO_PORT, TX_RFO_PIN, 1);
#endif
#endif
			break;

		case PATH_NONE:
			/* Try to disable every path */
#if defined(RX_PORT) && defined(RX_PIN)
			gpio_set_pin(RX_PORT, RX_PIN, 0);
#endif
#if defined(TX_PA_PORT) && defined(TX_PA_PIN)
			gpio_set_pin(TX_PA_PORT, TX_PA_PIN, 0);
#endif
#if defined(TX_RFO_PORT) && defined(TX_RFO_PIN)
			gpio_set_pin(TX_RFO_PORT, TX_RFO_PIN, 0);
#endif
			break;
	}
}

void system_set_sx_nss(uint8_t state)
{
	gpio_set_pin(NSS_PORT, NSS_PIN, state);
}

/* Set the RST pin to either a value (0 or 1), or keep it floating (2) */
void system_set_sx_rst(uint8_t state)
{
	if (state == 0 || state == 1) {
		gpio_cfg_pin(RST_PORT, RST_PIN, GPIO_PMODE_OUTPUT, GPIO_OMODE_PUSH_PULL, GPIO_PUPD_NONE, GPIO_OSPEED_VHIGH, 0);
		gpio_set_pin(RST_PORT, RST_PIN, state);
	} else {
		/* Keep RST floating */
		gpio_cfg_pin(RST_PORT, RST_PIN, GPIO_PMODE_INPUT, GPIO_OMODE_PUSH_PULL, GPIO_PUPD_NONE, GPIO_OSPEED_VHIGH, 0);
	}
}

extern void radio_irq_handler(uint8_t dio);

/* EXTI (GPIOs) IRQ handler */
void EXTI_IRQHandler(void)
{
#ifndef CFG_DISABLE_RADIO_IRQ_HANDLER
#if defined(DIO0_PIN)
	/* DIO 0 */
	if ((EXTI->PR & (1 << DIO0_PIN)) != 0) {
		EXTI->PR = (1 << DIO0_PIN);
		radio_irq_handler(0);
	}
#endif
#if defined(DIO1_PIN)
	/* DIO 1 */
	if ((EXTI->PR & (1 << DIO1_PIN)) != 0) {
		EXTI->PR = (1 << DIO1_PIN);
		radio_irq_handler(1);
	}
#endif
#if defined(DIO2_PIN)
	/* DIO 2 */
	if ((EXTI->PR & (1 << DIO2_PIN)) != 0) {
		EXTI->PR = (1 << DIO2_PIN);
		radio_irq_handler(2);
	}
#endif
#if defined(DIO3_PIN)
	/* DIO 3 */
	if ((EXTI->PR & (1 << DIO3_PIN)) != 0) {
		EXTI->PR = (1 << DIO3_PIN);
		radio_irq_handler(3);
	}
#endif
#if defined(DIO4_PIN)
	/* DIO 4 */
	if ((EXTI->PR & (1 << DIO4_PIN)) != 0) {
		EXTI->PR = (1 << DIO4_PIN);
		radio_irq_handler(4);
	}
#endif
#endif

	/* Buttons if any */
	{
		extern void button_irq_handler(void);
		button_irq_handler();
	}

	/* Accelerometer INT if any */
	{
		extern void accelerometer_irq_handler(void);
		accelerometer_irq_handler();
	}
}

void EXTI0_IRQHandler(void)
{
	EXTI_IRQHandler();
}

void EXTI1_IRQHandler(void)
{
	EXTI_IRQHandler();
}

void EXTI2_IRQHandler(void)
{
	EXTI_IRQHandler();
}

void EXTI3_IRQHandler(void)
{
	EXTI_IRQHandler();
}

void EXTI4_IRQHandler(void)
{
	EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler(void)
{
	EXTI_IRQHandler();
}

void EXTI15_10_IRQHandler(void)
{
	EXTI_IRQHandler();
}

void system_disable_irqs(void)
{
	__disable_irq();
	irqlevel++;
}

void system_enable_irqs(void)
{
	if (--irqlevel == 0) {
		__enable_irq();
	}
}

void system_register_lowpower_pins(io_pin_t *pins)
{
	app_lowpower_pins = pins;
}

static void save_gpio_config_for_lowpower(void)
{
	uint16_t simple_masks[6] = {0};
	uint32_t double_masks[6] = {0};
	uint8_t i;

	/* Compute the masks */
	for (i = 0; i < (sizeof(lowpower_pins) / sizeof(io_pin_t)); i++) {
		simple_masks[lowpower_pins[i].port] |= 0x1 << lowpower_pins[i].pin;
		double_masks[lowpower_pins[i].port] |= 0x3 << (lowpower_pins[i].pin * 2);
	}

	if (app_lowpower_pins != NULL) {
		for (i = 0; app_lowpower_pins[i].port != PIN_END && app_lowpower_pins[i].pin != PIN_END; i++) {
			simple_masks[app_lowpower_pins[i].port] |= 0x1 << app_lowpower_pins[i].pin;
			double_masks[app_lowpower_pins[i].port] |= 0x3 << (app_lowpower_pins[i].pin * 2);
		}
	}

	/* Save the GPIO configuration */
	GPIOA_MODER = GPIOA->MODER;
	GPIOB_MODER = GPIOB->MODER;
	GPIOC_MODER = GPIOC->MODER;
	GPIOD_MODER = GPIOD->MODER;
	GPIOE_MODER = GPIOE->MODER;
	GPIOH_MODER = GPIOH->MODER;

	GPIOA_OSPEEDR = GPIOA->OSPEEDR;
	GPIOB_OSPEEDR = GPIOB->OSPEEDR;
	GPIOC_OSPEEDR = GPIOC->OSPEEDR;
	GPIOD_OSPEEDR = GPIOD->OSPEEDR;
	GPIOE_OSPEEDR = GPIOE->OSPEEDR;
	GPIOH_OSPEEDR = GPIOH->OSPEEDR;

	GPIOA_OTYPER = GPIOA->OTYPER;
	GPIOB_OTYPER = GPIOB->OTYPER;
	GPIOC_OTYPER = GPIOC->OTYPER;
	GPIOD_OTYPER = GPIOD->OTYPER;
	GPIOE_OTYPER = GPIOE->OTYPER;
	GPIOH_OTYPER = GPIOH->OTYPER;

	GPIOA_PUPDR = GPIOA->PUPDR;
	GPIOB_PUPDR = GPIOB->PUPDR;
	GPIOC_PUPDR = GPIOC->PUPDR;
	GPIOD_PUPDR = GPIOD->PUPDR;
	GPIOE_PUPDR = GPIOE->PUPDR;
	GPIOH_PUPDR = GPIOH->PUPDR;

	/* Configure all GPIO port pins in Analog input mode (trigger OFF), except the registered low-power pins */
	GPIOD->MODER = 0xFFFFFFFF;
	GPIOE->MODER = 0xFFFFFFFF;
	// GPIOH->MODER = 0xFFFFFFFF;

	GPIOA->MODER = (GPIOA->MODER & double_masks[0]) | ~double_masks[0]; // Analog
	GPIOA->OSPEEDR = (GPIOA->OSPEEDR & double_masks[0]);		    // 400KHz
	GPIOA->OTYPER = (GPIOA->OTYPER & simple_masks[0]);		    // PP
	GPIOA->PUPDR = (GPIOA->PUPDR & double_masks[0]);		    // NOPULL

	GPIOB->MODER = (GPIOB->MODER & double_masks[1]) | ~double_masks[1]; // Analog
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR & double_masks[1]);		    // 400KHz
	GPIOB->OTYPER = (GPIOB->OTYPER & simple_masks[1]);		    // PP
	GPIOB->PUPDR = (GPIOB->PUPDR & double_masks[1]);		    // NOPULL

	GPIOC->MODER = (GPIOC->MODER & double_masks[2]) | ~double_masks[2]; // Analog
	GPIOC->OSPEEDR = (GPIOC->OSPEEDR & double_masks[2]);		    // 400KHz
	GPIOC->OTYPER = (GPIOC->OTYPER & simple_masks[2]);		    // PP
	GPIOC->PUPDR = (GPIOC->PUPDR & double_masks[2]);		    // NOPULL

	GPIOH->MODER = (GPIOH->MODER & double_masks[5]) | ~double_masks[5]; // Analog
	GPIOH->OSPEEDR = (GPIOH->OSPEEDR & double_masks[5]);		    // 400KHz
	GPIOH->OTYPER = (GPIOH->OTYPER & simple_masks[5]);		    // PP
	GPIOH->PUPDR = (GPIOH->PUPDR & double_masks[5]);		    // NOPULL
}

static void restore_gpio_config(void)
{
	GPIOA->MODER = GPIOA_MODER;
	GPIOB->MODER = GPIOB_MODER;
	GPIOC->MODER = GPIOC_MODER;
	GPIOD->MODER = GPIOD_MODER;
	GPIOE->MODER = GPIOE_MODER;
	GPIOH->MODER = GPIOH_MODER;

	GPIOA->OSPEEDR = GPIOA_OSPEEDR;
	GPIOB->OSPEEDR = GPIOB_OSPEEDR;
	GPIOC->OSPEEDR = GPIOC_OSPEEDR;
	GPIOD->OSPEEDR = GPIOD_OSPEEDR;
	GPIOE->OSPEEDR = GPIOE_OSPEEDR;
	GPIOH->OSPEEDR = GPIOH_OSPEEDR;

	GPIOA->OTYPER = GPIOA_OTYPER;
	GPIOB->OTYPER = GPIOB_OTYPER;
	GPIOC->OTYPER = GPIOC_OTYPER;
	GPIOD->OTYPER = GPIOD_OTYPER;
	GPIOE->OTYPER = GPIOE_OTYPER;
	GPIOH->OTYPER = GPIOH_OTYPER;

	GPIOA->PUPDR = GPIOA_PUPDR;
	GPIOB->PUPDR = GPIOB_PUPDR;
	GPIOC->PUPDR = GPIOC_PUPDR;
	GPIOD->PUPDR = GPIOD_PUPDR;
	GPIOE->PUPDR = GPIOE_PUPDR;
	GPIOH->PUPDR = GPIOH_PUPDR;
}

/* Sleep mode */
void system_sleep(void)
{
	/* Suspend execution until IRQ */
	__WFI();
}

/* Low Power Sleep mode */
void system_sleep_low_power(void)
{
	/* Wait for any pending UART transmission to complete */
	usart_sync(USART_PORT_1);
	usart_sync(USART_PORT_2);
	usart_sync(USART_PORT_3);

	/* Set IO in lowpower configuration*/
	save_gpio_config_for_lowpower();

	/* Disable Fast WakeUp (up to 3ms wake up time) */
	//*(__IO uint32_t *) CR_FWU_BB = (uint32_t)DISABLE;

	/* Disable PVD */
	*(__IO uint32_t *) CR_PVDE_BB = (uint32_t) DISABLE;

	/* Enable Ultra low power mode (up to 3ms wake up time) */
	//*(__IO uint32_t *) CR_ULP_BB = (uint32_t)ENABLE;

	/* Reset RCC */
	/* Set MSION bit */
	RCC->CR |= (uint32_t) 0x00000100;
	/* Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], MCOSEL[2:0] and MCOPRE[2:0] bits */
	RCC->CFGR &= (uint32_t) 0x88FFC00C;
	/* Reset HSION, HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t) 0xEEFEFFFE;
	/* Reset HSEBYP bit */
	RCC->CR &= (uint32_t) 0xFFFBFFFF;
	/* Reset PLLSRC, PLLMUL[3:0] and PLLDIV[1:0] bits */
	RCC->CFGR &= (uint32_t) 0xFF02FFFF;
	/* Disable all interrupts */
	RCC->CIR = 0x00000000;

	/* Switch Flash mode to no latency (WS 0) */
	FLASH->ACR &= (uint32_t) (~((uint32_t) FLASH_ACR_LATENCY));

	/* Disable Prefetch Buffer */
	FLASH->ACR &= (uint32_t) (~((uint32_t) FLASH_ACR_PRFTEN));

	/* Disable 64bit access */
	FLASH->ACR &= (uint32_t) (~((uint32_t) FLASH_ACR_ACC64));

	/* Disable FLASH during Sleep */
	FLASH->ACR |= FLASH_ACR_SLEEP_PD;

	/* Select MSI as system clock source */
	RCC->CFGR = (RCC->CFGR & (uint32_t) ((uint32_t) ~(RCC_CFGR_SW))) | (uint32_t) RCC_CFGR_SW_MSI;

	/* Wait until MSI is used as system clock source */
	while ((RCC->CFGR & (uint32_t) RCC_CFGR_SWS) != (uint32_t) RCC_CFGR_SWS_MSI);

	/* Select the Voltage Range 3 (1.2V) */
	PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_1 | PWR_CR_VOS_0;

	/* Wait Until the Voltage Regulator is ready */
	while ((PWR->CSR & PWR_CSR_VOSF) != RESET);

	/* HCLK = SYSCLK/2 (32KHz) */
	// RCC->CFGR = (RCC->CFGR & (uint32_t) ((uint32_t)~RCC_CFGR_HPRE)) | (uint32_t)RCC_CFGR_HPRE_DIV2;

	/* Disable PLL */
	RCC->CR &= (uint32_t) ((uint32_t) ~RCC_CR_PLLON);

	/* Disable HSI clock */
	RCC->CR &= (uint32_t) ((uint32_t) ~RCC_CR_HSION);

	/* Disable HSE clock */
	RCC->CR &= (uint32_t) ((uint32_t) ~RCC_CR_HSEON);

	/* Disable LSE clock */
	// RCC->CSR &= (uint32_t) ((uint32_t)~RCC_CSR_LSEON);

	/* Disable LSI clock */
	RCC->CSR &= (uint32_t) ((uint32_t) ~RCC_CSR_LSION);

	/* Ask for the Low Power sleep mode */
	PWR->CR &= ~PWR_CR_PDDS;
	PWR->CR |= PWR_CR_LPSDSR;

	/* Clear SLEEPDEEP bit of Cortex System Control Register */
	SCB->SCR &= (uint32_t) ~((uint32_t) SCB_SCR_SLEEPDEEP);

	/* Suspend execution until IRQ */
	__WFI();

	/* Enable HSI Clock */
	RCC->CR |= ((uint32_t) RCC_CR_HSION);

	/* Wait until HSI is ready */
	while ((RCC->CR & RCC_CR_HSIRDY) == RESET);

	/* Enable 64-bit access */
	FLASH->ACR |= FLASH_ACR_ACC64;

	/* Enable Prefetch Buffer */
	FLASH->ACR |= FLASH_ACR_PRFTEN;

	/* Switch Flash mode to 1 cycle latency (WS 1) */
	FLASH->ACR |= FLASH_ACR_LATENCY;

	/* Select the Voltage Range 1 (1.8V) */
	PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_0;

	/* Wait Until the Voltage Regulator is ready */
	while ((PWR->CSR & PWR_CSR_VOSF) != RESET);

	/* PLL configuration */
	RCC->CFGR &= (uint32_t) ((uint32_t) ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV));
	RCC->CFGR |= (uint32_t) (RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLDIV3);

	/* Enable PLL Clock */
	RCC->CR |= ((uint32_t) RCC_CR_PLLON);

	/* Wait until PLL is ready */
	while ((RCC->CR & RCC_CR_PLLRDY) == RESET);

	/* Select PLL as system clock source */
	RCC->CFGR = (RCC->CFGR & (uint32_t) ((uint32_t) ~(RCC_CFGR_SW))) | (uint32_t) RCC_CFGR_SW_PLL;

	/* Wait until PLL is used as system clock source */
	while ((RCC->CFGR & (uint32_t) RCC_CFGR_SWS) != (uint32_t) RCC_CFGR_SWS_PLL);

	/* HCLK = SYSCLK/1 */
	RCC->CFGR = (RCC->CFGR & (uint32_t) ((uint32_t) ~RCC_CFGR_HPRE)) | (uint32_t) RCC_CFGR_HPRE_DIV1;

	/* PCLK2 = HCLK/1 */
	RCC->CFGR = (RCC->CFGR & (uint32_t) ((uint32_t) ~RCC_CFGR_PPRE2)) | (uint32_t) RCC_CFGR_PPRE2_DIV1;

	/* PCLK1 = HCLK/1 */
	RCC->CFGR = (RCC->CFGR & (uint32_t) ((uint32_t) ~RCC_CFGR_PPRE1)) | (uint32_t) RCC_CFGR_PPRE1_DIV1;

	/* Be sure to disable any Low Power mode */
	PWR->CR &= (uint32_t) ~((uint32_t) PWR_CR_LPRUN);
	PWR->CR &= (uint32_t) ~((uint32_t) PWR_CR_LPSDSR);

	/* Disable Ultra low power mode */
	//*(__IO uint32_t *) CR_ULP_BB = (uint32_t)DISABLE;

	/* Disable FLASH during SLeep LP */
	FLASH->ACR &= (uint32_t) (~((uint32_t) FLASH_ACR_SLEEP_PD));

	restore_gpio_config();

	/* Clear Wake Up flag */
	PWR->CR |= PWR_CSR_WUF << 2;

	/* Enable PVD */
	*(__IO uint32_t *) CR_PVDE_BB = (uint32_t) ENABLE;
}

void system_reboot(void)
{
	NVIC_SystemReset();
}

void system_init(void)
{
	system_disable_irqs();

	/* Configure IOs and TIMER9 (tick counter) */
	system_io_init();

	RCC->APB2LPENR = 0; // disable APB2 peripheral clocks in Sleep mode
	RCC->APB1LPENR = 0; // disable APB1 peripheral clocks in Sleep mode
	RCC->AHBLPENR = 0; // disable AHB peripheral and GPIO clocks in Sleep mode

	RCC->APB1LPENR |= RCC_APB2LPENR_TIM9LPEN; // enable TIM9 in Sleep mode

	/* Configure MSI at 65KHz */
	RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_MSIRANGE) | RCC_ICSCR_MSIRANGE_0;

	system_enable_irqs();
}
