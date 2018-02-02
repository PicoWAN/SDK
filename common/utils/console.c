/*
 * console - Simple AT console
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

#include <boards.h>
#include <os.h>
#include <string.h>
#include <usart.h>
#include <xprintf.h>
#include "console.h"


/* Enable this to send back to the host what has been received */
#define CONSOLE_ECHO

#define RX_LINE_BUFFER_SIZE		1024

#ifdef CFG_CONSOLE_RX_ENABLED
static void console_irq_handler();

/* Buffer that will hold the received data until the line is complete */
static uint8_t rx_line_buffer[RX_LINE_BUFFER_SIZE];
static uint16_t rx_line_pos = 0;

static uint8_t console_AT_job_done = 1;
static uint8_t console_raw_job_done = 1;
static os_job_t console_job;

#ifdef CONSOLE_ECHO
static uint16_t echo_pos = 0;
static os_job_t echo_job;
#endif

#endif

static uint8_t is_commands_enabled = 0;

static struct command *cmd_list = NULL;

static uint8_t console_usart = 0;

/* This callback will only be called when the commands are disabled */
static void (*raw_cb)(uint8_t *args, uint16_t length) = NULL;


void console_init(uint8_t usart)
{
	console_usart = usart;

#ifdef CFG_CONSOLE_RX_ENABLED
	usart_enable_rx(console_usart, &console_irq_handler);
#endif
}

void console_enable_commands(uint8_t en)
{
	is_commands_enabled = !!en;
}

cmd_ret_val_t console_commands_help(char *args)
{
	uint32_t i;

	xprintf("Available commands");
#ifdef CFG_CONSOLE_AT_PREFIX
	xprintf(" (AT+<cmd>[=...])");
#endif
	xprintf(":\n");
	for (i = 0; cmd_list[i].name != NULL; i++) {
		xprintf("  ");
		xprintf(cmd_list[i].name);
		xprintf("\n");
	}

	return CMD_OK;
}

void console_return_ok(void)
{
#ifdef CFG_CONSOLE_RX_ENABLED
#ifdef CFG_CONSOLE_AT_PREFIX
	xprintf("\nOK\n");
#endif
	console_AT_job_done = 1;
#endif
}

void console_return_error(void)
{
#ifdef CFG_CONSOLE_RX_ENABLED
	xprintf("\nERROR\n");
	console_AT_job_done = 1;
#endif
}

#ifdef CFG_CONSOLE_RX_ENABLED
static void console_parse_line(void)
{
	uint32_t i;
	uint16_t cmd_len;
	char *cmd_args;
	uint16_t input_cmd_len = rx_line_pos;
	uint8_t *input_cmd = rx_line_buffer;
	cmd_ret_val_t ret;

	/* Trim ending \n/\r and add the final '\0' */
	rx_line_buffer[rx_line_pos - 1] = '\0';
	rx_line_pos--;
	if (rx_line_buffer[rx_line_pos - 1] == '\r' || rx_line_buffer[rx_line_pos - 1] == '\n') {
		rx_line_buffer[rx_line_pos - 1] = '\0';
		rx_line_pos--;
	}

	// xprintf("Received line : %s\n", rx_line_buffer);

#ifdef CFG_CONSOLE_AT_PREFIX
	/* Lines must start with "AT" */
	if (rx_line_pos < 2 || rx_line_buffer[0] != 'A' || rx_line_buffer[1] != 'T') {
		/* Wrong syntax */
		console_return_error();
		return;
	}

	if (rx_line_pos == 2) {
		/* AT only is OK */
		console_return_ok();
		return;
	}

	if (rx_line_buffer[2] != '+') {
		/* Only AT+<something> is supported */
		console_return_error();
		return;
	}

	if (cmd_list == NULL) {
		/* There is no command registered */
		console_return_error();
		return;
	}

	input_cmd += 3;
	input_cmd_len -= 3;
#endif

	/* Now we parse the actual command (<cmd>[=<params>], only one command per line supoprted) */
	for (i = 0; cmd_list[i].name != NULL; i++) {
		cmd_len = strlen(cmd_list[i].name);
		if (input_cmd_len >= cmd_len && !strncmp((char *) (&input_cmd[0]), cmd_list[i].name, cmd_len)) {
			if (input_cmd[cmd_len] == '=') {
				/* Some arguments are present */
				cmd_args = (char *) &input_cmd[cmd_len + 1];
			} else if (input_cmd[cmd_len] == '?') {
				/* Sends the '?' as argument */
				cmd_args = (char *) &input_cmd[cmd_len];
			} else {
				/* No argument */
				cmd_args = NULL;
			}

			break;
		}
	}

	if (cmd_list[i].name == NULL || cmd_list[i].cb == NULL) {
		/* Unknown command */
		console_return_error();
		return;
	}

	ret = cmd_list[i].cb(cmd_args);

	switch (ret) {
		case CMD_OK:
			console_return_ok();
			break;
		case CMD_ERROR:
			console_return_error();
			break;
		case CMD_DELAYED:
			/* The result will be delayed */
			break;
	}
}
#endif

void console_register_cmds(struct command *list)
{
	cmd_list = list;
}

void console_register_raw_callback(void (*cb)(uint8_t *args, uint16_t length))
{
	raw_cb = cb;
}

#ifdef CFG_CONSOLE_RX_ENABLED
static void parse_cmd_task(os_job_t *j)
{
	console_parse_line();
	rx_line_pos = 0;
	echo_pos = 0;
}

static void raw_callback_task(os_job_t *j)
{
	raw_cb(rx_line_buffer, rx_line_pos);
	console_raw_job_done = 1;
	rx_line_pos = 0;
	echo_pos = 0;
}

static void console_schedule_job(void (*job)(os_job_t *))
{
	os_post_job(&console_job, job);
	if (is_commands_enabled) {
		console_AT_job_done = 0;
	} else {
		console_raw_job_done = 0;
	}
}

#ifdef CONSOLE_ECHO
static void echo_task(os_job_t *j)
{
	while (echo_pos < rx_line_pos) {
		xprintf("%c", (char) rx_line_buffer[echo_pos++]);
	}
}
#endif

static void console_read(void)
{
	uint8_t should_parse_line = is_commands_enabled;

	/* Read as much as possible */
	while (usart_is_rx_not_empty(console_usart)) {
		if ((!console_AT_job_done && is_commands_enabled) || (!console_raw_job_done && !is_commands_enabled)) {
			/* A job is running, discard everything */
			usart_getc(console_usart);
			continue;
		}

		rx_line_buffer[rx_line_pos++] = usart_getc(console_usart);

		if (!should_parse_line) {
			if (rx_line_pos == RX_LINE_BUFFER_SIZE) {
				/* Buffer is full, call the raw callback now */
				if (raw_cb != NULL) {
					console_schedule_job(raw_callback_task);
				} else {
					rx_line_pos = 0;
				}
			}

			/* Commands disabled, no need to parse the data */
			continue;
		}

#ifdef CONSOLE_ECHO
		os_post_job(&echo_job, echo_task);
#endif

		if (rx_line_buffer[rx_line_pos - 1] == '\n' || rx_line_buffer[rx_line_pos - 1] == '\r' || rx_line_pos >= RX_LINE_BUFFER_SIZE) {
#ifdef CONSOLE_ECHO
			rx_line_buffer[rx_line_pos++] = '\n';
#endif
			/* The line is complete or the buffer
			 * is full, let's parse it
			 */
			console_schedule_job(parse_cmd_task);
		} else if (rx_line_buffer[rx_line_pos - 1] == 0x08) {
			/* Backspace, remove it along with one char */
			if (rx_line_pos >= 2) {
				rx_line_pos -= 2;
			}
			if (echo_pos > rx_line_pos) {
				echo_pos = rx_line_pos;
			}
#ifdef CONSOLE_ECHO
		// xprintf(" \b");
#endif
		} else if (rx_line_buffer[rx_line_pos - 1] == 0x18) {
			/* Cancel, delete the current line */
			rx_line_pos = 0;
			echo_pos = 0;
		}
	}

	if (!should_parse_line && rx_line_pos > 0 && console_raw_job_done) {
		/* Some data in the buffer for the raw callback */
		if (raw_cb != NULL) {
			console_schedule_job(raw_callback_task);
		} else {
			rx_line_pos = 0;
		}
	}
}

static void console_irq_handler()
{
	if (usart_is_rx_not_empty(console_usart)) { // pending
		console_read();
	}
}
#endif /* CFG_CONSOLE_RX_ENABLED */
