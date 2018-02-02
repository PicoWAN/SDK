/*! \file
 * \brief Button driver with debounce
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

#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <stdint.h>

#define MAX_BUTTONS		4

enum button_mode {
	BUTTON_ACTIVE_LOW = 0,
	BUTTON_ACTIVE_HIGH = 1
};

enum button_press_duration {
	BUTTON_SHORT_PRESS = 0,
	BUTTON_LONG_PRESS = 1
};


/*!
 * \brief   Initializes the button driver.
 *
 * \details This function initializes the button driver, and allows to register a button callback that will be
 *          executed everytime a button is pressed. The drivers supports short and long presses, and already
 *          performes a debounce.
 *
 * \param   cb The callback to register.
 */
void button_init(void (*cb)(uint8_t num, enum button_press_duration duration));

/*!
 * \brief   Checks if a button is currently pressed.
 *
 * \param   num The number of the button to check.
 *
 * \retval  uint8_t 1 if the button is pressed, 0 otherwise.
 */
uint8_t button_is_pressed(uint8_t num);

#endif /* _BUTTON_H_ */
