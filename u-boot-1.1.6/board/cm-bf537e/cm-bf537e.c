/*
 * U-boot - cm-bf537e.c
 *
 * Copyright (c) 2005 blackfin.uclinux.org
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <config.h>
#include <command.h>
#include <asm/blackfin.h>
#include <asm/io.h>
#include <linux/etherdevice.h>

int checkboard(void)
{
	printf("Board: Bluetechnix CM-BF537E board\n");
	printf("       Support: http://blackfin.uclinux.org/\n");
	return 0;
}

long int initdram(int board_type)
{
	DECLARE_GLOBAL_DATA_PTR;
#ifdef DEBUG
	int brate;
	char *tmp = getenv("baudrate");
	brate = simple_strtoul(tmp, NULL, 16);
	printf("Serial Port initialized with Baud rate = %x\n", brate);
	printf("SDRAM attributes:\n");
	printf("tRCD %d SCLK Cycles,tRP %d SCLK Cycles,tRAS %d SCLK Cycles"
	       "tWR %d SCLK Cycles,CAS Latency %d SCLK cycles \n",
	       3, 3, 6, 2, 3);
	printf("SDRAM Begin: 0x%x\n", CFG_SDRAM_BASE);
	printf("Bank size = %d MB\n", CFG_MAX_RAM_SIZE >> 20);
#endif
	gd->bd->bi_memstart = CFG_SDRAM_BASE;
	gd->bd->bi_memsize = CFG_MAX_RAM_SIZE;
	return CFG_MAX_RAM_SIZE;
}

int board_get_enetaddr(unsigned char *mac_addr)
{
	/* make something up */
	const char s[] = __DATE__;
	size_t i;
	u32 cycles;
	for (i = 0; i < 6; ++i) {
	      asm("%0 = CYCLES;":"=r"(cycles));
		mac_addr[i] = cycles ^ s[i];
	}
	mac_addr[0] = (mac_addr[0] | 0x02) & ~0x01;	/* make it local unicast */
	return 0;
}

#if defined(CONFIG_MISC_INIT_R)
/* miscellaneous platform dependent initialisations */
int misc_init_r(void)
{
	return 0;
}
#endif
