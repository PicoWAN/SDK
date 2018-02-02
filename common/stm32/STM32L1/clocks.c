/*
 * clocks - Clocks driver
 * Based on stm32l1xx_rcc.c from ST Library
 *
 * Copyright (c) 2015 STMicroelectronics International N.V.. All rights reserved.
 * Copyright (c) 2018, Archos S.A.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted, provided that the following conditions are met:
 *
 * 1.   Redistribution of source code must retain the above copyright notice, this list
 *      of conditions and the following disclaimer.
 * 2.   Redistributions in binary form must reproduce the above copyright notice, this
 *      list of conditions and the following disclaimer in the documentation and/or
 *      other materials provided with the distribution.
 * 3.   Neither the name of STMicroelectronics nor the names of other contributors to
 *      this software may be used to endorse or promote products derived from this
 *      software without specific written permission.
 * 4.   This software, including modifications and/or derivative works of this
 *      software, must execute solely and exclusively on microcontroller or
 *      microprocessor devices manufactured by or for STMicroelectronics.
 * 5.   Redistribution and use of this software other than as permitted under this
 *      license is void and will automatically terminate your rights under this
 *      license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" AND
 * EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * OF THIRD PARTY INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT
 * PERMITTED BY LAW. IN NO EVENT SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <os.h>
#include "clocks.h"
#include "stm32l1xx.h"

static __I uint8_t PLLMulTable[9] = {3, 4, 6, 8, 12, 16, 24, 32, 48};
static __I uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};


/**
  * @brief  Returns the frequencies of the System, AHB and APB busses clocks.
  * @note     The frequency returned by this function is not the real frequency
  *           in the chip. It is calculated based on the predefined constant and
  *           the SYSCLK source:
  *
  * @note     If SYSCLK source is MSI, function returns values based on MSI
  *             value as defined by the MSI range
  *
  * @note     If SYSCLK source is HSI, function returns values based on HSI_VALUE(*)
  *
  * @note     If SYSCLK source is HSE, function returns values based on HSE_VALUE(**)
  *
  * @note     If SYSCLK source is PLL, function returns values based on HSE_VALUE(**)
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *
  *         (*) HSI_VALUE is a constant defined in stm32l1xx.h file (default value
  *             16 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.
  *
  *         (**) HSE_VALUE is a constant defined in stm32l1xx.h file (default value
  *              8 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              return wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *
  * @param  clocks: pointer to a clocks_t structure which will hold
  *         the clocks frequencies.
  *
  * @note     This function can be used by the user application to compute the
  *           baudrate for the communication peripherals or configure other parameters.
  * @note     Each time SYSCLK, HCLK, PCLK1 and/or PCLK2 clock changes, this function
  *           must be called to update the structure's field. Otherwise, any
  *           configuration based on this function will be incorrect.
  *
  * @retval None
  */
void clocks_get_freq(clocks_t *clocks)
{
	uint32_t tmp = 0, pllmul = 0, plldiv = 0, pllsource = 0, presc = 0, msirange = 0;

	/* Get SYSCLK source -------------------------------------------------------*/
	tmp = RCC->CFGR & RCC_CFGR_SWS;

	switch (tmp) {
		case 0x00: /* MSI used as system clock */
			msirange = (RCC->ICSCR & RCC_ICSCR_MSIRANGE) >> 13;
			clocks->SYSCLK_Frequency = (32768 * (1 << (msirange + 1)));
			break;
		case 0x04: /* HSI used as system clock */
			clocks->SYSCLK_Frequency = HSI_VALUE;
			break;
		case 0x08: /* HSE used as system clock */
			clocks->SYSCLK_Frequency = HSE_VALUE;
			break;
		case 0x0C: /* PLL used as system clock */
			/* Get PLL clock source and multiplication factor ----------------------*/
			pllmul = RCC->CFGR & RCC_CFGR_PLLMUL;
			plldiv = RCC->CFGR & RCC_CFGR_PLLDIV;
			pllmul = PLLMulTable[(pllmul >> 18)];
			plldiv = (plldiv >> 22) + 1;

			pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;

			if (pllsource == 0x00) {
				/* HSI oscillator clock selected as PLL clock source */
				clocks->SYSCLK_Frequency = (((HSI_VALUE) *pllmul) / plldiv);
			} else {
				/* HSE selected as PLL clock source */
				clocks->SYSCLK_Frequency = (((HSE_VALUE) *pllmul) / plldiv);
			}
			break;
		default: /* MSI used as system clock */
			msirange = (RCC->ICSCR & RCC_ICSCR_MSIRANGE) >> 13;
			clocks->SYSCLK_Frequency = (32768 * (1 << (msirange + 1)));
			break;
	}
	/* Compute HCLK, PCLK1, PCLK2 and ADCCLK clocks frequencies ----------------*/
	/* Get HCLK prescaler */
	tmp = RCC->CFGR & RCC_CFGR_HPRE;
	tmp = tmp >> 4;
	presc = APBAHBPrescTable[tmp];
	/* HCLK clock frequency */
	clocks->HCLK_Frequency = clocks->SYSCLK_Frequency >> presc;

	/* Get PCLK1 prescaler */
	tmp = RCC->CFGR & RCC_CFGR_PPRE1;
	tmp = tmp >> 8;
	presc = APBAHBPrescTable[tmp];
	/* PCLK1 clock frequency */
	clocks->PCLK1_Frequency = clocks->HCLK_Frequency >> presc;

	/* Get PCLK2 prescaler */
	tmp = RCC->CFGR & RCC_CFGR_PPRE2;
	tmp = tmp >> 11;
	presc = APBAHBPrescTable[tmp];
	/* PCLK2 clock frequency */
	clocks->PCLK2_Frequency = clocks->HCLK_Frequency >> presc;
}
