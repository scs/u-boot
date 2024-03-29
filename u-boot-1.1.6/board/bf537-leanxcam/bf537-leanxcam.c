/*
 * U-boot - bf537-leanxcam.c
 *  
 * (C) Copyright 2008
 * Supercomputing Systems AG <www.scs.ch>
 * 
 * Configuation settings for the OSCAR boards: LEANXCAM, INDXCAM
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

#define POST_WORD_ADDR 0xFF903FFC

int checkboard(void)
{
	printf("Board: LEANXCAM, SCS AG open source camera\n");
	printf("       Support: http://leanxcam.ch/\n");
	return 0;
}

#if defined(CONFIG_BFIN_IDE)

void cf_outb(unsigned char val, volatile unsigned char *addr)
{
	*(addr) = val;
	SSYNC();
}

unsigned char cf_inb(volatile unsigned char *addr)
{
	volatile unsigned char c;

	c = *(addr);
	SSYNC();

	return c;
}

void cf_insw(unsigned short *sect_buf, unsigned short *addr, int words)
{
	int i;

	for (i = 0; i < words; i++)
		*(sect_buf + i) = *(addr);
	SSYNC();
}

void cf_outsw(unsigned short *addr, unsigned short *sect_buf, int words)
{
	int i;

	for (i = 0; i < words; i++)
		*(addr) = *(sect_buf + i);
	SSYNC();
}
#endif				/* CONFIG_BFIN_IDE */

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

int board_get_enetaddr(uchar *mac_addr)
{
#ifdef CFG_NO_FLASH
# define USE_MAC_IN_FLASH 0
#else
# define USE_MAC_IN_FLASH 1
#endif

	puts("Warning: Generating 'random' MAC address\n");

	/* make something up */
	const char s[] = __DATE__;
	size_t i;
	u32 cycles;
	for (i = 0; i < 6; ++i) {
		asm("%0 = CYCLES;" : "=r" (cycles));
		mac_addr[i] = cycles ^ s[i];
	}
	mac_addr[0] = (mac_addr[0] | 0x02) & ~0x01; /* make it local unicast */
	return 0;
}

void board_reset(void)
{
	/* workaround for weak pull ups on ssel */
	if (CONFIG_BFIN_BOOT_MODE == BFIN_BOOT_SPI_MASTER) {
		bfin_write_PORTF_FER(bfin_read_PORTF_FER() & ~PF10);
		bfin_write_PORTFIO_SET(PF10);
		udelay(1);
	}
}

#if defined(CONFIG_MISC_INIT_R)
/* miscellaneous platform dependent initialisations */
int misc_init_r(void)
{
#ifndef CFG_NO_FLASH
	/* we use the last sector for the MAC address / POST DXE */
	extern flash_info_t flash_info[];
	flash_protect(FLAG_PROTECT_SET, 0x203F0000, 0x203FFFFF, &flash_info[0]);
#endif

#if defined(CONFIG_BFIN_IDE)
#if defined(CONFIG_BFIN_TRUE_IDE)
	/* Enable ATASEL when in True IDE mode */
	printf("Using CF True IDE Mode\n");
	cf_outb(0, (unsigned char *)CONFIG_CF_ATASEL_ENA);
	udelay(1000);
#elif defined(CONFIG_BFIN_CF_IDE)
	/* Disable ATASEL when we're in Common Memory Mode */
	printf("Using CF Common Memory Mode\n");
	cf_outb(0, (unsigned char *)CONFIG_CF_ATASEL_DIS);
	udelay(1000);
#elif defined(CONFIG_BFIN_HDD_IDE)
	printf("Using HDD IDE Mode\n");
#endif
	ide_init();
#endif				/* CONFIG_BFIN_IDE */
	return 0;
}
#endif				/* CONFIG_MISC_INIT_R */

#ifdef CONFIG_POST
#if (CONFIG_BFIN_BOOT_MODE != BFIN_BOOT_BYPASS)
/* Using sw10-PF5 as the hotkey */
int post_hotkeys_pressed(void)
{
	return 0;
}
#else
/* Using sw10-PF5 as the hotkey */
int post_hotkeys_pressed(void)
{
	int delay = 3;
	int i;
	unsigned short value;

	*pPORTF_FER &= ~PF5;
	*pPORTFIO_DIR &= ~PF5;
	*pPORTFIO_INEN |= PF5;

	printf("########Press SW10 to enter Memory POST########: %2d ", delay);
	while (delay--) {
		for (i = 0; i < 100; i++) {
			value = *pPORTFIO & PF5;
			if (value != 0) {
				break;
			}
			udelay(10000);
		}
		printf("\b\b\b%2d ", delay);
	}
	printf("\b\b\b 0");
	printf("\n");
	if (value == 0)
		return 0;
	else {
		printf("Hotkey has been pressed, Enter POST . . . . . .\n");
		return 1;
	}
}
#endif
#endif

#if defined(CONFIG_POST) || defined(CONFIG_LOGBUFFER)
void post_word_store(ulong a)
{
	volatile ulong *save_addr = (volatile ulong *)POST_WORD_ADDR;
	*save_addr = a;
}

ulong post_word_load(void)
{
	volatile ulong *save_addr = (volatile ulong *)POST_WORD_ADDR;
	return *save_addr;
}
#endif

#ifdef CONFIG_POST
int uart_post_test(int flags)
{
	return 0;
}

#define BLOCK_SIZE 0x10000
#define VERIFY_ADDR 0x2000000
extern int erase_block_flash(int);
extern int write_data(long lStart, long lCount, uchar * pnData);
int flash_post_test(int flags)
{
	unsigned short *pbuf, *temp;
	int offset, n, i;
	int value = 0;
	int result = 0;
	printf("\n");
	pbuf = (unsigned short *)VERIFY_ADDR;
	temp = pbuf;
	for (n = FLASH_START_POST_BLOCK; n < FLASH_END_POST_BLOCK; n++) {
		offset = (n - 7) * BLOCK_SIZE;
		printf("--------Erase   block:%2d..", n);
		erase_block_flash(n);
		printf("OK\r");
		printf("--------Program block:%2d...", n);
		write_data(CFG_FLASH_BASE + offset, BLOCK_SIZE, pbuf);
		printf("OK\r");
		printf("--------Verify  block:%2d...", n);
		for (i = 0; i < BLOCK_SIZE; i += 2) {
			if (*(unsigned short *)(CFG_FLASH_BASE + offset + i) !=
			    *temp++) {
				value = 1;
				result = 1;
			}
		}
		if (value)
			printf("failed\n");
		else
			printf("OK		%3d%%\r",
			       (int)(
				     (n + 1 -
				      FLASH_START_POST_BLOCK) *
				     100 / (FLASH_END_POST_BLOCK -
					    FLASH_START_POST_BLOCK)));

		temp = pbuf;
		value = 0;
	}
	printf("\n");
	if (result)
		return -1;
	else
		return 0;
}

/****************************************************
 * LED1 ---- PF6	LED2 ---- PF7		    *
 * LED3 ---- PF8	LED4 ---- PF9		    *
 * LED5 ---- PF10	LED6 ---- PF11		    *
 ****************************************************/
int led_post_test(int flags)
{
	*pPORTF_FER &= ~(PF6 | PF7 | PF8 | PF9 | PF10 | PF11);
	*pPORTFIO_DIR |= PF6 | PF7 | PF8 | PF9 | PF10 | PF11;
	*pPORTFIO_INEN &= ~(PF6 | PF7 | PF8 | PF9 | PF10 | PF11);
	*pPORTFIO &= ~(PF6 | PF7 | PF8 | PF9 | PF10 | PF11);
	udelay(1000000);
	printf("LED1 on");
	*pPORTFIO |= PF6;
	udelay(1000000);
	printf("\b\b\b\b\b\b\b");
	printf("LED2 on");
	*pPORTFIO |= PF7;
	udelay(1000000);
	printf("\b\b\b\b\b\b\b");
	printf("LED3 on");
	*pPORTFIO |= PF8;
	udelay(1000000);
	printf("\b\b\b\b\b\b\b");
	printf("LED4 on");
	*pPORTFIO |= PF9;
	udelay(1000000);
	printf("\b\b\b\b\b\b\b");
	printf("LED5 on");
	*pPORTFIO |= PF10;
	udelay(1000000);
	printf("\b\b\b\b\b\b\b");
	printf("lED6 on");
	*pPORTFIO |= PF11;
	printf("\b\b\b\b\b\b\b ");
	return 0;
}

/************************************************
 *  SW10 ---- PF5	SW11 ---- PF4		*
 *  SW12 ---- PF3	SW13 ---- PF2		*
 ************************************************/
int button_post_test(int flags)
{
	int i, delay = 5;
	unsigned short value = 0;
	int result = 0;

	*pPORTF_FER &= ~(PF5 | PF4 | PF3 | PF2);
	*pPORTFIO_DIR &= ~(PF5 | PF4 | PF3 | PF2);
	*pPORTFIO_INEN |= (PF5 | PF4 | PF3 | PF2);

	printf("\n--------Press SW10: %2d ", delay);
	while (delay--) {
		for (i = 0; i < 100; i++) {
			value = *pPORTFIO & PF5;
			if (value != 0) {
				break;
			}
			udelay(10000);
		}
		printf("\b\b\b%2d ", delay);
	}
	if (value != 0)
		printf("\b\bOK");
	else {
		result = -1;
		printf("\b\bfailed");
	}

	delay = 5;
	printf("\n--------Press SW11: %2d ", delay);
	while (delay--) {
		for (i = 0; i < 100; i++) {
			value = *pPORTFIO & PF4;
			if (value != 0) {
				break;
			}
			udelay(10000);
		}
		printf("\b\b\b%2d ", delay);
	}
	if (value != 0)
		printf("\b\bOK");
	else {
		result = -1;
		printf("\b\bfailed");
	}

	delay = 5;
	printf("\n--------Press SW12: %2d ", delay);
	while (delay--) {
		for (i = 0; i < 100; i++) {
			value = *pPORTFIO & PF3;
			if (value != 0) {
				break;
			}
			udelay(10000);
		}
		printf("\b\b\b%2d ", delay);
	}
	if (value != 0)
		printf("\b\bOK");
	else {
		result = -1;
		printf("\b\bfailed");
	}

	delay = 5;
	printf("\n--------Press SW13: %2d ", delay);
	while (delay--) {
		for (i = 0; i < 100; i++) {
			value = *pPORTFIO & PF2;
			if (value != 0) {
				break;
			}
			udelay(10000);
		}
		printf("\b\b\b%2d ", delay);
	}
	if (value != 0)
		printf("\b\bOK");
	else {
		result = -1;
		printf("\b\bfailed");
	}
	printf("\n");
	return result;
}
#endif
