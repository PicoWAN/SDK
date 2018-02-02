/*! \file
 * \brief Simple AT console
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

#ifndef _CONSOLE_H_
#define _CONSOLE_H_

#include <os.h>

typedef enum {
	CMD_OK		= 0, // Command has been successful
	CMD_ERROR	= 1, // Command has failed
	CMD_DELAYED	= 2, // Result is delayed (a call to console_return_ok() or console_return_error() is mandatory)
} cmd_ret_val_t;

struct command {
	char *name;
	cmd_ret_val_t (*cb)(char *args); // Returns 0 on success, 1 otherwise
};


/*!
 * \brief   Initializes the console.
 *
 * \param   usart The USART number to which the console will be attached. See usart.h for the available USARTs.
 */
void console_init(uint8_t usart);

/*!
 * \brief   Enables the AT command interface.
 *
 * \details The function enables the AT command interface, which means that the USART output will be parsed as AT commands.
 *          If this is not enabled, the RAW data are directly sent to the main application using the RAW callback.
 *
 * \param   en 1 to enable the AT command interface, 0 to disable it.
 */
void console_enable_commands(uint8_t en);

/*!
 * \brief   Built-in command that lists the available commands.
 *
 * \details This is a special built-in command that lists the registered commands. It is intended to be added in the command
 *          list as an "HELP" command.
 *
 * \param   args The arguments (not used, and must be set to NULL if the function is directly called)
 *
 * \retval  cmd_ret_val_t CMD_OK if the command succeeded, CMD_ERROR otherwise.
 */
cmd_ret_val_t console_commands_help(char *args);

/*!
 * \brief   Explicitly returns that the previously called AT command has succeeded.
 *
 * \details The function returns that the previously called AT command has succeeded. It should only be used if the last AT
 *          command called behaves asynchronously, and thus returned CMD_DELAYED.
 */
void console_return_ok(void);

/*!
 * \brief   Explicitly returns that the previously called AT command has failed.
 *
 * \details The function returns that the previously called AT command has failed. It should only be used if the last AT
 *          command called behaves asynchronously, and thus returned CMD_DELAYED.
 */
void console_return_error(void);

/*!
 * \brief   Registers a list of AT commands.
 *
 * \details The function allows the main application to register a list of AT commands. It will only be used if the command interface is enabled.
 *
 * \param   list The list of AT commands.
 */
void console_register_cmds(struct command *list);

/*!
 * \brief   Registers a RAW callback
 *
 * \details The function allows the main application to register a RAW callback. It will only be used if the command interface is disabled.
 *
 * \param   cb The RAW callback to register.
 */
void console_register_raw_callback(void (*cb)(uint8_t *args, uint16_t length));

#endif
