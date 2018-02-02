/*
 * stm32-sections - Linker script for common sections
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
 * * Neither the name of the University of California, Berkeley nor the
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

SECTIONS
{
	/* The beginning of the startup code (stored in the .isr_vector section) goes to FLASH */
	.isr_vector : {
		. = ALIGN(4);
		KEEP(*(.isr_vector))		/* Startup code */
		. = ALIGN(4);
	} > FLASH

	/* The program code is stored in the .text section, which goes to Flash */
	.text : {
		. = ALIGN(4);
		/* .FWVersion section (Firmware version) on 4 bytes, just after the ISR vector */
		.text_start = .;
		KEEP (*(.FWVersion))
		. = .text_start + 0x4;

		*(.text)			/* normal code */
		*(.text.*)			/* -ffunction-sections code */
		*(.glue_7)			/* arm to thumb glue code */
		*(.glue_7t)			/* thumb to arm glue code */

		/* Necessary KEEP sections (see http://sourceware.org/ml/newlib/2005/msg00255.html) */
		KEEP (*(.init))
		KEEP (*(.fini))

		. = ALIGN(4);
		_etext = .;
	} > FLASH

	.rodata : {
		. = ALIGN(4);
		*(.rodata)         /* .read-only data (constants, strings, etc.) */
		*(.rodata*)        /* -fdata-sections read only data */
		. = ALIGN(4);
	} > FLASH

	.ARM.extab : {
		*(.ARM.extab*)
		*(.gnu.linkonce.armextab.*)
	} > FLASH

	.ARM : {
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;
	} > FLASH

	.preinit_array : {
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array*))
		PROVIDE_HIDDEN (__preinit_array_end = .);
	} > FLASH

	.init_array : {
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array*))
		PROVIDE_HIDDEN (__init_array_end = .);
	} > FLASH

	.fini_array : {
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(SORT(.fini_array.*)))
		KEEP (*(.fini_array*))
		PROVIDE_HIDDEN (__fini_array_end = .);
	} > FLASH

	/* Used by the startup in order to initialize the .data section */
	_sidata = LOADADDR(.data);

	/* Initialized data section (the startup will copy the initial values from FLASH to RAM) */
	.data :
	{
		. = ALIGN(4);
		/* Used by the startup in order to initialize the .data secion */
		_sdata = . ;
		_data = . ;

		*(.data)
		*(.data.*)
		*(.RamFunc)

		. = ALIGN(4);
		/* Used by the startup in order to initialize the .data secion */
		_edata = . ;
	} > RAM AT > FLASH

	/* Uninitialized data section */
	.bss :
	{
		. = ALIGN(4);
		/* Used by the startup in order to initialize the .bss secion */
		_sbss = .;
		_bss = .;

		*(.bss)
		*(.bss.*)
		*(COMMON)

		. = ALIGN(4);
		/* Used by the startup in order to initialize the .bss secion */
		_ebss = . ;
	} > RAM

	/* Check that there is enough RAM left for Heap and Stack */
	._heap_stack :
	{
		. = ALIGN(4);
		PROVIDE ( end = . );
		PROVIDE ( _end = . );
		. = . + _Min_Heap_Size;
		. = . + _Min_Stack_Size;
		. = ALIGN(4);
	} > RAM

	/* Remove the debugs from the standard libraries */
	/DISCARD/ :
	{
		libc.a ( * )
		libm.a ( * )
		libgcc.a ( * )
	}

	.ARM.attributes 0 : {
		*(.ARM.attributes)
	}
}
