/*
 * U-boot - bf533_string.c Contains library routines.
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
#include <asm/setup.h>
#include <asm/page.h>
#include <asm/cpu/defBF533.h>

char *strcpy(char *dest, const char *src)
{
	char *xdest = dest;
	char temp = 0;

	__asm__ __volatile__
		("1:\t%2 = B [%1++] (Z);\n\t"
		"B [%0++] = %2;\n\t"
		"CC = %2;\n\t"
		"if cc jump 1b (bp);\n":"=a"(dest), "=a"(src), "=d"(temp)
		:"0"(dest), "1"(src), "2"(temp):"memory");

	return xdest;
}

char *strncpy(char *dest, const char *src, size_t n)
{
	char *xdest = dest;
	char temp = 0;

	if (n == 0)
		return xdest;

	__asm__ __volatile__
		("1:\t%3 = B [%1++] (Z);\n\t"
		"B [%0++] = %3;\n\t"
		"CC = %3;\n\t"
		"if ! cc jump 2f;\n\t"
		"%2 += -1;\n\t"
		"CC = %2 == 0;\n\t"
		"if ! cc jump 1b (bp);\n"
		"2:\n":"=a"(dest), "=a"(src), "=da"(n), "=d"(temp)
		:"0"(dest), "1"(src), "2"(n), "3"(temp)
		:"memory");

	return xdest;
}

int strcmp(const char *cs, const char *ct)
{
	char __res1, __res2;

	__asm__
		("1:\t%2 = B[%0++] (Z);\n\t"	/* get *cs */
		"%3 = B[%1++] (Z);\n\t"		/* get *ct */
		"CC = %2 == %3;\n\t"		/* compare a byte */
		"if ! cc jump 2f;\n\t"		/* not equal, break out */
		"CC = %2;\n\t"			/* at end of cs? */
		"if cc jump 1b (bp);\n\t"	/* no, keep going */
		"jump.s 3f;\n"			/* strings are equal */
		"2:\t%2 = %2 - %3;\n"		/* *cs - *ct */
		"3:\n":	"=a"(cs), "=a"(ct), "=d"(__res1),
		"=d"(__res2)
		: "0"(cs), "1"(ct));

	return __res1;
}

int strncmp(const char *cs, const char *ct, size_t count)
{
	char __res1, __res2;

	if (!count)
		return 0;

	__asm__
		("1:\t%3 = B[%0++] (Z);\n\t"	/* get *cs */
		"%4 = B[%1++] (Z);\n\t"		/* get *ct */
		"CC = %3 == %4;\n\t"		/* compare a byte */
		"if ! cc jump 3f;\n\t"		/* not equal, break out */
		"CC = %3;\n\t"			/* at end of cs? */
		"if ! cc jump 4f;\n\t"		/* yes, all done */
		"%2 += -1;\n\t"			/* no, adjust count */
		"CC = %2 == 0;\n\t" "if ! cc jump 1b;\n"	/* more to do, keep going */
		"2:\t%3 = 0;\n\t"		/* strings are equal */
		"jump.s    4f;\n" "3:\t%3 = %3 - %4;\n"	/* *cs - *ct */
 		"4:":	"=a"(cs), "=a"(ct), "=da"(count), "=d"(__res1),
		"=d"(__res2)
		: "0"(cs), "1"(ct), "2"(count));

	return __res1;
}

/*
 * memcpy - Copy one area of memory to another
 * @dest: Where to copy to
 * @src: Where to copy from
 * @count: The size of the area.
 *
 * You should not use this function to access IO space, use memcpy_toio()
 * or memcpy_fromio() instead.
 */
void * memcpy(void * dest,const void *src,size_t count)
{
	char *tmp = (char *) dest, *s = (char *) src;

	if ((tmp >= (char*)L1_ISRAM) && (tmp < (char*)L1_ISRAM_END)) {
		/* Copy sram functions from sdram to L1 sram */
		/* Setup destination start address */
		*(volatile unsigned long *)MDMA_D0_START_ADDR = (unsigned long)dest;
		/* Setup destination xcount */
		*(volatile unsigned short *)MDMA_D0_X_COUNT = count / 2 ;
		/* Setup destination xmodify */
		*(volatile unsigned short *)MDMA_D0_X_MODIFY = 2;

		/* Setup Source start address */
		*(volatile unsigned long *)MDMA_S0_START_ADDR = (unsigned long)src;
		/* Setup Source xcount */
		*(volatile unsigned short *)MDMA_S0_X_COUNT = count /2 ;
		/* Setup Source xmodify */
		*(volatile unsigned short *)MDMA_S0_X_MODIFY = 2;

		/* Set word size to 16, set to read, enable interrupt for wakeup */
		/* Enable source DMA */
		*(volatile unsigned short *)MDMA_S0_CONFIG = (WDSIZE16 | DMAEN);
		asm("ssync;");
		/* Enable Destination DMA */
		*(volatile unsigned short *)MDMA_D0_CONFIG = (DI_EN | WDSIZE16 | WNR | DMAEN);

		/* Go into idle and wait for wakeup that indicates DMA is done */
		asm("idle;");
		/* Clear interrupt */
		*(volatile unsigned short *)MDMA_D0_IRQ_STATUS = 0x1;
	}
	else
	{
		while (count--)
			*tmp++ = *s++;
	}

	return dest;
}
