/*
 * U-boot - serial.c Blackfin Serial Driver
 *
 * Copyright (c) 2005-2007 Analog Devices Inc.
 *
 * Copyright (c) 2003	Bas Vermeulen <bas@buyways.nl>,
 * 			BuyWays B.V. (www.buyways.nl)
 *
 * Based heavily on:
 * blkfinserial.c: Serial driver for BlackFin DSP internal USRTs.
 * Copyright(c) 2003	Metrowerks	<mwaddel@metrowerks.com>
 * Copyright(c)	2001	Tony Z. Kou	<tonyko@arcturusnetworks.com>
 * Copyright(c)	2001-2002 Arcturus Networks Inc. <www.arcturusnetworks.com>
 *
 * Based on code from 68328 version serial driver imlpementation which was:
 * Copyright (C) 1995       David S. Miller    <davem@caip.rutgers.edu>
 * Copyright (C) 1998       Kenneth Albanowski <kjahds@kjahds.com>
 * Copyright (C) 1998, 1999 D. Jeff Dionne     <jeff@uclinux.org>
 * Copyright (C) 1999       Vladimir Gurevich  <vgurevic@cisco.com>
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
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/segment.h>
#include <asm/bitops.h>
#include <asm/delay.h>
#include <asm/io.h>
#include "serial.h"

unsigned long pll_div_fact;

#define IRQ_UART1_RX_BIT		0x0200 //SIC_ISR1
#define IRQ_UART1_TX_BIT		0x0400 //SIC_ISR1
#define IRQ_UART1_ERROR_BIT	0x40 //SIC_ISR1

void calc_baud(void)
{
	unsigned char i;
	int temp;
	u_long sclk = get_sclk();

	for (i = 0; i < sizeof(baud_table) / sizeof(int); i++) {
		temp = sclk / (baud_table[i] * 8);
		if ((temp & 0x1) == 1) {
			temp++;
		}
		temp = temp / 2;
		hw_baud_table[i].dl_high = (temp >> 8) & 0xFF;
		hw_baud_table[i].dl_low = (temp) & 0xFF;
	}
}

void serial_setbrg(void)
{
	int i;
	DECLARE_GLOBAL_DATA_PTR;

	calc_baud();

	for (i = 0; i < sizeof(baud_table) / sizeof(int); i++) {
		if (gd->baudrate == baud_table[i])
			break;
	}

	/* Enable UART */
	*pUART1_GCTL |= UCEN;
	__builtin_bfin_ssync();

	/* Set DLAB in LCR to Access DLL and DLH */
	ACCESS_LATCH;
	__builtin_bfin_ssync();

	*pUART1_DLL = hw_baud_table[i].dl_low;
	__builtin_bfin_ssync();
	*pUART1_DLH = hw_baud_table[i].dl_high;
	__builtin_bfin_ssync();

	/* Clear DLAB in LCR to Access THR RBR IER */
	ACCESS_PORT_IER;
	__builtin_bfin_ssync();

	/* Enable ERBFI and ELSI interrupts
	 * to poll SIC_ISR register*/
	*pUART1_IER_SET = ELSI_S | ERBFI_S | ETBEI_S;
	__builtin_bfin_ssync();

	/* Set LCR to Word Lengh 8-bit word select */
	*pUART1_LCR = WLS_8;
	__builtin_bfin_ssync();

	return;
}

int serial_init(void)
{
	serial_setbrg();
	return (0);
}

void serial_putc(const char c)
{
	if ((*pUART1_LSR) & TEMT) {
		if (c == '\n')
			serial_putc('\r');

		local_put_char(c);
	}

	while (!((*pUART1_LSR) & TEMT))
		SYNC_ALL;

	return;
}

int serial_tstc(void)
{
	if (*pUART1_LSR & DR)
		return 1;
	else
		return 0;
}

int serial_getc(void)
{
	unsigned short uart_lsr_val, uart_rbr_val;
	unsigned long isr_val;
	int ret;

	/* Poll for RX Interrupt */
	while (!((isr_val =
		 *(volatile unsigned long *)SIC_ISR1) & IRQ_UART1_RX_BIT));
	asm("csync;");

	uart_lsr_val = *pUART1_LSR;	/* Clear status bit */
	uart_rbr_val = *pUART1_RBR;	/* getc() */

	if (isr_val & IRQ_UART1_ERROR_BIT) {
		ret = -1;
	} else {
		ret = uart_rbr_val & 0xff;
	}

	return ret;
}

void serial_puts(const char *s)
{
	while (*s) {
		serial_putc(*s++);
	}
}

static void local_put_char(char ch)
{
	int flags = 0;
	unsigned long isr_val;

	save_and_cli(flags);

	/* Poll for TX Interruput */
	while (!((isr_val = *pSIC_ISR1) & IRQ_UART1_TX_BIT)) ;
	asm("csync;");

	*pUART1_THR = ch;	/* putc() */

	if (isr_val & IRQ_UART1_ERROR_BIT) {
		printf("?");
	}

	restore_flags(flags);

	return;
}
