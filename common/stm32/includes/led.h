/*! \file
 * \brief LED driver
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

#ifndef _LED_H_
#define _LED_H_

#include <stdint.h>

#define MAX_LEDS	4

#define MAX_SLOTS	16

#define SLOT_0		(1 << 0)
#define SLOT_1		(1 << 1)
#define SLOT_2		(1 << 2)
#define SLOT_3		(1 << 3)
#define SLOT_4		(1 << 4)
#define SLOT_5		(1 << 5)
#define SLOT_6		(1 << 6)
#define SLOT_7		(1 << 7)
#define SLOT_8		(1 << 8)
#define SLOT_9		(1 << 9)
#define SLOT_10		(1 << 10)
#define SLOT_11		(1 << 11)
#define SLOT_12		(1 << 12)
#define SLOT_13		(1 << 13)
#define SLOT_14		(1 << 14)
#define SLOT_15		(1 << 15)

#define ON_OFF_SLOTS_PERIOD_2_DUTY_50	(SLOT_0 | SLOT_2 | SLOT_4 | SLOT_6 | SLOT_8 | SLOT_10 | SLOT_12 | SLOT_14)
#define ON_OFF_SLOTS_PERIOD_4_DUTY_50	(SLOT_0 | SLOT_1 | SLOT_4 | SLOT_5 | SLOT_8 | SLOT_9 | SLOT_12 | SLOT_13)
#define ON_OFF_SLOTS_PERIOD_8_DUTY_50	(SLOT_0 | SLOT_1 | SLOT_2 | SLOT_3 | SLOT_8 | SLOT_9 | SLOT_10 | SLOT_11)
#define ON_OFF_SLOTS_PERIOD_16_DUTY_50	(SLOT_0 | SLOT_1 | SLOT_2 | SLOT_3 | SLOT_4 | SLOT_5 | SLOT_6 | SLOT_7)

enum led_state {
	LED_OFF = 0,
	LED_ON = 1,
	LED_BLINK = 2
};

enum led_mode {
	LED_ACTIVE_LOW = 0,
	LED_ACTIVE_HIGH = 1
};


/*!
 * \brief   Initializes the LED driver.
 */
void led_init(void);

/*!
 * \brief   Enables or disable an LED.
 *
 * \param   num The number of the LED.
 * \param   state The new state of the LED. This can be LED_OFF, LED_ON, or LED_BLINK.
 */
void led_set(uint8_t num, enum led_state state);

/*!
 * \brief   Makes an LED blink.
 *
 * \details This function makes an LED blink. A timer will wake up 16 times in 1.2s, giving 16 slots where the LED can be
 *          on or off. Slots are numbered from 0 to 15 and last for 75 ms each.
 *
 * \param   num The number of the LED.
 * \param   led_on_mask A mask of all 16 slots. Set only the slots where the LED will be on.
 * \param   repeat Number of repeats. 0 forever.
 */
void led_blink(uint8_t num, uint16_t led_on_mask, uint8_t repeat);

/*!
 * \brief   Gets the current state of an LED.
 *
 * \param   num The number of the LED.
 *
 * \retval  led_state The current state of the LED.
 */
enum led_state led_get(uint8_t num);

/*!
 * \brief   Toggles an LED.
 *
 * \details This function toggles an LED. A blinking LED will be toggled OFF.
 *
 * \param   num The number of the LED.
 */
void led_toggle(uint8_t num);

#endif /* _LED_H_ */
