/*
 * (C) Copyright 2004
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2004
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
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
#include <command.h>
#include <malloc.h>
#include <devices.h>
#include <version.h>
#include <net.h>
#include <environment.h>
#include "blackfin_board.h"

static void mem_malloc_init(void)
{
	mem_malloc_start = CFG_MALLOC_BASE;
	mem_malloc_end = (CFG_MALLOC_BASE + CFG_MALLOC_LEN);
	mem_malloc_brk = mem_malloc_start;
	memset((void *) mem_malloc_start, 0,
	       mem_malloc_end - mem_malloc_start);
}

void *sbrk(ptrdiff_t increment)
{
	ulong old = mem_malloc_brk;
	ulong new = old + increment;

	if ((new < mem_malloc_start) || (new > mem_malloc_end)) {
		return (NULL);
	}
	mem_malloc_brk = new;

	return ((void *) old);
}

static int display_banner(void)
{
	printf("%s\n", version_string);
	printf("%s\n", moreinfo_string);

	return (0);
}

static void display_flash_config(ulong size)
{
	puts("Flash Size ");
	print_size(size, "\n");
	return;
}

static int init_baudrate(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	uchar tmp[64];
	int i = getenv_r("baudrate", tmp, sizeof(tmp));
	gd->bd->bi_baudrate = gd->baudrate = (i > 0)
	    ? (int) simple_strtoul(tmp, NULL, 10)
	    : CONFIG_BAUDRATE;
	return (0);
}

#ifdef DEBUG
static void display_global_data(void)
{
	DECLARE_GLOBAL_DATA_PTR;
	bd_t *bd;
	bd = gd->bd;
	printf("--flags:%x\n", gd->flags);
	printf("--board_type:%x\n", gd->board_type);
	printf("--baudrate:%x\n", gd->baudrate);
	printf("--have_console:%x\n", gd->have_console);
	printf("--ram_size:%x\n", gd->ram_size);
	printf("--reloc_off:%x\n", gd->reloc_off);
	printf("--env_addr:%x\n", gd->env_addr);
	printf("--env_valid:%x\n", gd->env_valid);
	printf("--bd:%x %x\n", gd->bd, bd);
	printf("---bi_baudrate:%x\n", bd->bi_baudrate);
	printf("---bi_ip_addr:%x\n", bd->bi_ip_addr);
	printf("---bi_enetaddr:%x %x %x %x %x %x\n", 
				bd->bi_enetaddr[0],
				bd->bi_enetaddr[1],
				bd->bi_enetaddr[2],
				bd->bi_enetaddr[3],
				bd->bi_enetaddr[4],
				bd->bi_enetaddr[5]);
	printf("---bi_arch_number:%x\n", bd->bi_arch_number);
	printf("---bi_boot_params:%x\n", bd->bi_boot_params);
	printf("---bi_memstart:%x\n", bd->bi_memstart);
	printf("---bi_memsize:%x\n", bd->bi_memsize);
	printf("---bi_flashstart:%x\n", bd->bi_flashstart);
	printf("---bi_flashsize:%x\n", bd->bi_flashsize);
	printf("---bi_flashoffset:%x\n", bd->bi_flashoffset);
	printf("--jt:%x *:%x\n", gd->jt, *(gd->jt));
}
#endif

/*
 * All attempts to come up with a "common" initialization sequence
 * that works for all boards and architectures failed: some of the
 * requirements are just _too_ different. To get rid of the resulting
 * mess of board dependend #ifdef'ed code we now make the whole
 * initialization sequence configurable to the user.
 *
 * The requirements for any new initalization function is simple: it
 * receives a pointer to the "global data" structure as it's only
 * argument, and returns an integer return code, where 0 means
 * "continue" and != 0 means "fatal error, hang the system".
 */

void board_init_f(ulong bootflag)
{
	DECLARE_GLOBAL_DATA_PTR;
	ulong addr;
	bd_t *bd;
	unsigned long cck;

	gd = (gd_t *) (CFG_GBL_DATA_ADDR);
	memset((void *) gd, 0, sizeof(gd_t));

        /* Board data initialization */
        addr = (CFG_GBL_DATA_ADDR + sizeof(gd_t));

        /* Align to 4 byte boundary */
        addr &= ~(4 - 1);
        bd = (bd_t*)addr;
        gd->bd = bd;
        memset((void *) bd, 0, sizeof(bd_t));

	/* Initialize */
	init_IRQ();
	env_init();		/* initialize environment */
	cck = get_clock();
	get_sclk();
	init_baudrate();	/* initialze baudrate settings */
	serial_init();		/* serial communications setup */
	console_init_f();
	display_banner();	/* say that we are here */
	checkboard();
	rtc_init();
	initdram(0);
	timer_init();
	printf("Core Clock %d MHz\n",cck/1000000);
	printf("System Clock %d MHz\n",sclk/1000000);
	board_init_r((gd_t *) gd, 0x20000010);
}

void board_init_r(gd_t * id, ulong dest_addr)
{
	DECLARE_GLOBAL_DATA_PTR;
	cmd_tbl_t *cmdtp;
	ulong size;
	extern void malloc_bin_reloc(void);
	char *s, *e;
	bd_t *bd;
	int i;
	gd = id;
	gd->flags |= GD_FLG_RELOC;	/* tell others: relocation done */
	bd = gd->bd;

	/* There are some other pointer constants we must deal with */
	/* configure available FLASH banks */
	size = flash_init();
	display_flash_config(size);
	bd->bi_flashstart = CFG_FLASH_BASE;
	bd->bi_flashsize = size;
	bd->bi_flashoffset = 0;

	/* initialize malloc() area */
	mem_malloc_init();
	malloc_bin_reloc();

	/* relocate environment function pointers etc. */
	env_relocate();

	/* board MAC address */
	s = getenv("ethaddr");
	for (i = 0; i < 6; ++i) {
		bd->bi_enetaddr[i] = s ? simple_strtoul(s, &e, 16) : 0;
		if (s)
			s = (*e) ? e + 1 : e;
	}

	/* IP Address */
	bd->bi_ip_addr = getenv_IPaddr("ipaddr");

	/* Initialize devices */
	devices_init();
	jumptable_init();

	/* Initialize the console (after the relocation and devices init) */
	console_init_r();

	/* Initialize from environment */
	if ((s = getenv("loadaddr")) != NULL) {
		load_addr = simple_strtoul(s, NULL, 16);
	}
#if (CONFIG_COMMANDS & CFG_CMD_NET)
	if ((s = getenv("bootfile")) != NULL) {
		copy_filename(BootFile, s, sizeof(BootFile));
	}
#endif
#if defined(CONFIG_MISC_INIT_R)
	/* miscellaneous platform dependent initialisations */
	misc_init_r();
#endif

#ifdef CONFIG_DRIVER_SMC91111
	printf("Compiled with SMC91111 support\n");
#endif

#ifdef DEBUG
	display_global_data(void);
#endif

	/* main_loop() can return to retry autoboot, if so just run it again. */
	for (;;) {
		main_loop();
	}
}

void hang(void)
{
	puts("### ERROR ### Please RESET the board ###\n");
	for (;;);
}
