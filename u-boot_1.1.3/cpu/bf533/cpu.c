/*
 * U-boot - cpu.c CPU specific functions
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
#include <asm/blackfin.h>
#include <command.h>
#include <asm/entry.h>

int do_reset(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	__asm__ __volatile__
	("cli r3;"
	"P0 = %0;"
	"JUMP (P0);"
	:
	: "r" (L1_ISRAM)
	);

	return 0;
}

/* These functions are just used to satisfy the linker */
int cpu_init(void)
{
	return 0;
}

int cleanup_before_linux(void)
{
	return 0;
}

void icache_enable(void)
{
}

void icache_disable(void)
{
}

int icache_status(void)
{
	return 0;
}

void dcache_enable(void)
{
}

void dcache_disable(void)
{
}

int dcache_status(void)
{
	return 0;
}