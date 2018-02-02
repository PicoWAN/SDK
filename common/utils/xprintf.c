/*------------------------------------------------------------------------/
/  Universal string handler for user console interface
/-------------------------------------------------------------------------/
/
/  Copyright (C) 2011, ChaN, all right reserved.
/  Copyright (C) 2018, Archos S.A., all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/-------------------------------------------------------------------------*/
#include "xprintf.h"
#include "boards.h"
#include "usart.h"

#if _USE_XFUNC_OUT
#include <stdarg.h>
static char *outptr;

/* Define this to enable uint64_t support in xprintf() using the "%lld\u\x" syntax.
 * Please note that on a 32 bit target, a call to va_arg(, unsigned long long) for
 * something else than a uint64_t will behave incorrectly and will corrupt the va list.
 * To work properly, the stack pointer must be 8-byte aligned.
 */
//#define XPRINTF_UINT64_SUPPORT

/*----------------------------------------------*/
/* Put a character                              */
/*----------------------------------------------*/

void xputc(char c)
{
	if (_CR_CRLF && c == '\n')
		xputc('\r'); /* CR -> CRLF */

	if (outptr) {
		*outptr++ = (unsigned char) c;
		return;
	}
	usart_putc(CONSOLE_USART, (unsigned char) c);
}


/*----------------------------------------------*/
/* Put a null-terminated string                 */
/*----------------------------------------------*/

void xputs(		   /* Put a string to the default device */
	   const char *str /* Pointer to the string */
	   )
{
	while (*str)
		xputc(*str++);
}

/*----------------------------------------------*/
/* Formatted string output                      */
/*----------------------------------------------*/
/*  xprintf("%d", 1234);			"1234"
    xprintf("%6d,%3d%%", -200, 5);	"  -200,  5%"
    xprintf("%-6u", 100);			"100   "
    xprintf("%ld", 12345678L);		"12345678"
    xprintf("%04x", 0xA3);			"00a3"
    xprintf("%08LX", 0x123ABC);		"00123ABC"
    xprintf("%016b", 0x550F);		"0101010100001111"
    xprintf("%s", "String");		"String"
    xprintf("%-4s", "abc");			"abc "
    xprintf("%4s", "abc");			" abc"
    xprintf("%c", 'a');				"a"
    xprintf("%f", 10.0);            <xprintf lacks floating point support>
*/

void xvprintf(const char *fmt, /* Pointer to the format string */
	      va_list arp      /* Pointer to arguments */
	      )
{
	unsigned int r, i, j, w, f;
	unsigned long long v;
	char s[32], c, d, *p;


	for (;;) {
		c = *fmt++;					/* Get a char */
		if (!c)
			break;					/* End of format? */
		if (c != '%') {					/* Pass through it if not a % sequense */
			xputc(c);
			continue;
		}
		f = 0;
		c = *fmt++;					/* Get first char of the sequense */
		if (c == '0') {					/* Flag: '0' padded */
			f = 1;
			c = *fmt++;
		} else {
			if (c == '-') {				/* Flag: left justified */
				f = 2;
				c = *fmt++;
			}
		}
		for (w = 0; c >= '0' && c <= '9'; c = *fmt++)	/* Minimum width */
			w = w * 10 + c - '0';
		if (c == 'l' || c == 'L') {			/* Prefix: Size is long int */
			f |= 4;
			c = *fmt++;
#ifdef XPRINTF_UINT64_SUPPORT
			if (c == 'l' || c == 'L') {		/* Prefix: Size is long long int */
				f |= 16;
				c = *fmt++;
			}
#endif
		}
		if (!c)
			break;					/* End of format? */
		d = c;
		if (d >= 'a')
			d -= 0x20;
		switch (d) {      				/* Type is... */
			case 'S':				/* String */
				p = va_arg(arp, char *);
				for (j = 0; p[j]; j++);
				while (!(f & 2) && j++ < w)
					xputc(' ');
				xputs(p);
				while (j++ < w)
					xputc(' ');
				continue;
			case 'C':				/* Character */
				xputc((char) va_arg(arp, int));
				continue;
			case 'B':				/* Binary */
				r = 2;
				break;
			case 'O':				/* Octal */
				r = 8;
				break;
			case 'D':				/* Signed decimal */
			case 'I':				/* Signed decimal */
			case 'U':				/* Unsigned decimal */
				r = 10;
				break;
			case 'X':				/* Hexdecimal */
				r = 16;
				break;
			default:				/* Unknown type (passthrough) */
				xputc(c);
				continue;
		}
		if (f & 16) {
			v = (d == 'D' || d == 'I') ? (unsigned long long) va_arg(arp, long long)
						   : (unsigned long long) va_arg(arp, unsigned long long);
		} else if (f & 4) {
			v = (d == 'D' || d == 'I') ? (unsigned long long) va_arg(arp, long) : (unsigned long long) va_arg(arp, unsigned long);
		} else {
			v = (d == 'D' || d == 'I') ? (unsigned long long) va_arg(arp, int) : (unsigned long long) va_arg(arp, unsigned int);
		}
		if ((d == 'D' || d == 'I') && (v & 0x8000000000000000ULL)) {
			v = 0 - v;
			f |= 8;
		}
		i = 0;
		do {
			d = (char) (v % r);
			v /= r;
			if (d > 9)
				d += (c == 'x') ? 0x27 : 0x07;
			s[i++] = d + '0';
		} while (v && i < sizeof(s));
		if (f & 8)
			s[i++] = '-';
		j = i;
		d = (f & 1) ? '0' : ' ';
		while (!(f & 2) && j++ < w)
			xputc(d);
		do
			xputc(s[--i]);
		while (i);
		while (j++ < w)
			xputc(' ');
	}
}


void xprintf(		      /* Put a formatted string to the default device */
	     const char *fmt, /* Pointer to the format string */
	     ...	      /* Optional arguments */
	     )
{
	va_list arp;


	va_start(arp, fmt);
	xvprintf(fmt, arp);
	va_end(arp);
}


void xsprintf(		       /* Put a formatted string to the memory */
	      char *buff,      /* Pointer to the output buffer */
	      const char *fmt, /* Pointer to the format string */
	      ...	       /* Optional arguments */
	      )
{
	va_list arp;


	outptr = buff; /* Switch destination for memory */

	va_start(arp, fmt);
	xvprintf(fmt, arp);
	va_end(arp);

	*outptr = 0; /* Terminate output string with a \0 */
	outptr = 0;  /* Switch destination for device */
}
#endif /* _USE_XFUNC_OUT */


#if _USE_XFUNC_IN
int xgetc(void)
{
	while (!usart_is_rx_not_empty(CONSOLE_USART)) {
	}
	return usart_getc(CONSOLE_USART);
}

/*----------------------------------------------*/
/* Get a line from the input                    */
/*----------------------------------------------*/

int xgets(	      /* 0:End of stream, 1:A line arrived */
	  char *buff, /* Pointer to the buffer */
	  int len     /* Buffer length */
	  )
{
	int c, i;

	i = 0;
	for (;;) {
		c = xgetc();			/* Get a char from the incoming stream */
		if (!c)
			return 0;		/* End of stream? */
		if (c == '\r')
			break;			/* End of line? */
		if (c == '\b' && i) {		/* Back space? */
			i--;
			if (_LINE_ECHO)
				xputc(c);
			continue;
		}
		if (c >= ' ' && i < len - 1) {	/* Visible chars */
			buff[i++] = c;
			if (_LINE_ECHO)
				xputc(c);
		}
	}
	buff[i] = 0;				/* Terminate with a \0 */
	if (_LINE_ECHO)
		xputc('\n');
	return 1;
}

#if 0
int xfgets (	/* 0:End of stream, 1:A line arrived */
	unsigned char (*func)(void),	/* Pointer to the input stream function */
	char* buff,	/* Pointer to the buffer */
	int len		/* Buffer length */
)
{
	int (*pf)(void);
	int n;


	pf = xfunc_in;			/* Save current input device */
	xfunc_in = func;		/* Switch input to specified device */
	n = xgets(buff, len);	/* Get a line */
	xfunc_in = pf;			/* Restore input device */

	return n;
}


/*----------------------------------------------*/
/* Get a value of the string                    */
/*----------------------------------------------*/
/*	"123 -5   0x3ff 0b1111 0377  w "
	    ^                           1st call returns 123 and next ptr
	       ^                        2nd call returns -5 and next ptr
                   ^                3rd call returns 1023 and next ptr
                          ^         4th call returns 15 and next ptr
                               ^    5th call returns 255 and next ptr
                                  ^ 6th call fails and returns 0
*/

int xatoi (			/* 0:Failed, 1:Successful */
	char **str,		/* Pointer to pointer to the string */
	long *res		/* Pointer to the valiable to store the value */
)
{
	unsigned long val;
	unsigned char c, r, s = 0;


	*res = 0;

	while ((c = **str) == ' ') (*str)++;	/* Skip leading spaces */

	if (c == '-') {		/* negative? */
		s = 1;
		c = *(++(*str));
	}

	if (c == '0') {
		c = *(++(*str));
		switch (c) {
		case 'x':		/* hexdecimal */
			r = 16; c = *(++(*str));
			break;
		case 'b':		/* binary */
			r = 2; c = *(++(*str));
			break;
		default:
			if (c <= ' ') return 1;	/* single zero */
			if (c < '0' || c > '9') return 0;	/* invalid char */
			r = 8;		/* octal */
		}
	} else {
		if (c < '0' || c > '9') return 0;	/* EOL or invalid char */
		r = 10;			/* decimal */
	}

	val = 0;
	while (c > ' ') {
		if (c >= 'a') c -= 0x20;
		c -= '0';
		if (c >= 17) {
			c -= 7;
			if (c <= 9) return 0;	/* invalid char */
		}
		if (c >= r) return 0;		/* invalid char for current radix */
		val = val * r + c;
		c = *(++(*str));
	}
	if (s) val = 0 - val;			/* apply sign if needed */

	*res = val;
	return 1;
}
#endif

#endif /* _USE_XFUNC_IN */
