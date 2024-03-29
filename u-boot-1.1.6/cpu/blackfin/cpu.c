/*
 * U-boot - cpu.c CPU specific functions
 *
 * Copyright (c) 2005-2007 Analog Devices Inc.
 *
 * (C) Copyright 2000-2004
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
 * MA 02110-1301 USA
 */

#include <common.h>
#include <command.h>
#include <asm/blackfin.h>
#include <asm/cplb.h>
#include <asm/mach-common/bits/core.h>
#include <asm/mach-common/bits/ebiu.h>
#include <asm/mach-common/bits/trace.h>

#include "cpu.h"
#include "serial.h"

__attribute__ ((__noreturn__))
void cpu_init_f(ulong bootflag, ulong loaded_from_ldr)
{
	/* Build a NOP slide over the LDR jump block.  Whee! */
	serial_early_puts("NOP Slide\n");
	char nops[0xC];
	memset(nops, 0x00, sizeof(nops));
	extern char _stext_l1;
	memcpy(&_stext_l1 - sizeof(nops), nops, sizeof(nops));

	if (!loaded_from_ldr) {
		/* Relocate sections into L1 if the LDR didn't do it -- don't
		 * check length because the linker script does the size
		 * checking at build time.
		 */
		serial_early_puts("L1 Relocate\n");
		extern char _stext_l1, _etext_l1, _stext_l1_lma;
		memcpy(&_stext_l1, &_stext_l1_lma, (&_etext_l1 - &_stext_l1));
		extern char _sdata_l1, _edata_l1, _sdata_l1_lma;
		memcpy(&_sdata_l1, &_sdata_l1_lma, (&_edata_l1 - &_sdata_l1));
	}
#if defined(__ADSPBF537__) || defined(__ADSPBF536__) || defined(__ADSPBF534__)
	/* The BF537 bootrom will reset the EBIU_AMGCTL register on us
	 * after it has finished loading the LDR.  So configure it again.
	 */
	else
		bfin_write_EBIU_AMGCTL(CONFIG_EBIU_AMGCTL_VAL);
#endif

#ifdef CONFIG_DEBUG_DUMP
	/* Turn on hardware trace buffer */
	bfin_write_TBUFCTL(TBUFPWR | TBUFEN);
#endif

#ifndef CONFIG_PANIC_HANG
	/* Reset upon a double exception rather than just hanging.
	 * Do not do bfin_read on SWRST as that will reset status bits.
	 */
	bfin_write_SWRST(DOUBLE_FAULT);
#endif

	serial_early_puts("Board init flash\n");
	board_init_f(bootflag);
}

int exception_init(void)
{
	bfin_write_EVT3(trap);
	return 0;
}

int irq_init(void)
{
#ifdef SIC_IMASK0
	bfin_write_SIC_IMASK0(0);
	bfin_write_SIC_IMASK1(0);
# ifdef SIC_IMASK2
	bfin_write_SIC_IMASK2(0);
# endif
#elif defined(SICA_IMASK0)
	bfin_write_SICA_IMASK0(0);
	bfin_write_SICA_IMASK1(0);
#else
	bfin_write_SIC_IMASK(0);
#endif
	bfin_write_EVT2(evt_default);	/* NMI */
	bfin_write_EVT5(evt_default);	/* hardware error */
	bfin_write_EVT6(evt_default);	/* core timer */
	bfin_write_EVT7(evt_default);
	bfin_write_EVT8(evt_default);
	bfin_write_EVT9(evt_default);
	bfin_write_EVT10(evt_default);
	bfin_write_EVT11(evt_default);
	bfin_write_EVT12(evt_default);
	bfin_write_EVT13(evt_default);
	bfin_write_EVT14(evt_default);
	bfin_write_EVT15(evt_default);
	bfin_write_ILAT(0);
	CSYNC();
	return 0;
}
