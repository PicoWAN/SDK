/*!-----------------------------------------------------------------------
 * \file
 * \brief Universal string handler for user console interface
 *
 *-------------------------------------------------------------------------
 *
 *  Copyright (C) 2011, ChaN, all right reserved.
 *  Copyright (C) 2018, Archos S.A., all right reserved.
 *
 * * This software is a free software and there is NO WARRANTY.
 * * No restriction on use. You can use, modify and redistribute it for
 *   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
 * * Redistributions of source code must retain the above copyright notice.
 *
 *------------------------------------------------------------------------*/
#ifndef _STRFUNC
#define _STRFUNC

#include <stdarg.h>

#define _USE_XFUNC_OUT	1	/* 1: Use output functions */
#define	_CR_CRLF		1	/* 1: Convert \n ==> \r\n in the output char */

#define _USE_XFUNC_IN	0	/* 1: Use input function */
#define	_LINE_ECHO		0	/* 1: Echo back input chars in xgets function */


#if _USE_XFUNC_OUT

/*!
 * \brief   Put a character.
 *
 * \param   c The character to put.
 */
void xputc   (char c);

/*!
 * \brief   Put a null-terminated string.
 *
 * \param   str The string to put.
 */
void xputs   (const char* str);

/*!
 * \brief   Formatted string output (va_list version).
 *
 * \param   fmt The formatted output.
 * \param   arp The list of arguments.
 */
void xvprintf(const char* fmt, va_list arp);

/*!
 * \brief   Formatted string output to USART.
 *
 * \details This function prints a formatted output (no floating point support).
 *          Call				| Output
 *          ------------------------------------|-------------
 *          xprintf("%d", 1234);                | "1234"
 *          xprintf("%6d,%3d%%", -200, 5);      | "  -200,  5%"
 *          xprintf("%-6u", 100);               | "100   "
 *          xprintf("%ld", 12345678L);          | "12345678"
 *          xprintf("%04x", 0xA3);              | "00a3"
 *          xprintf("%08LX", 0x123ABC);         | "00123ABC"
 *          xprintf("%016b", 0x550F);           | "0101010100001111"
 *          xprintf("%s", "String");            | "String"
 *          xprintf("%-4s", "abc");             | "abc "
 *          xprintf("%4s", "abc");              | " abc"
 *          xprintf("%c", 'a');                 | "a"
 *
 * \param   fmt The formatted output.
 * \param   ... Variable list of arguments.
 */
void xprintf (const char* fmt, ...);

/*!
 * \brief   Formatted string output into a buffer.
 *
 * \details This function is the same as xprintf(), but will fill a buffer.
 *
 * \param   fmt The formatted output.
 * \param   ... Variable list of arguments.
 */
void xsprintf(char* buff, const char* fmt, ...);
#define DW_CHAR		sizeof(char)
#define DW_SHORT	sizeof(short)
#define DW_LONG		sizeof(long)
#endif

#if _USE_XFUNC_IN
int xgetc(void);
int xgets(char *buff, int len);
int xatoi(char **str, long *res);
#endif

#endif
