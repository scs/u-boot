/*
 * (C) Copyright 2003
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 *	PROJECT				:	BFIN
 *	VERSION				:	2.0
 *	FILE				:	bf533_linux.c
 *	MODIFIED DATE			:	29 jun 2004
 *	AUTHOR				:	BFin Project-ADI
 *	LOCATION			:	LG Soft India,Bangalore
 */

/* Dummy functions, currently not in Use */

#include <common.h>
#include <command.h>
#include <image.h>
#include <zlib.h>
#include <asm/byteorder.h>

#define	LINUX_MAX_ENVS		256
#define	LINUX_MAX_ARGS		256

#ifdef CONFIG_SHOW_BOOT_PROGRESS
#include <status_led.h>
#define SHOW_BOOT_PROGRESS(arg)	show_boot_progress(arg)
#else
#define SHOW_BOOT_PROGRESS(arg)
#endif

extern image_header_t header;
extern int do_reset(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[]);
void do_bootm_linux(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[],
		    ulong addr, ulong * len_ptr, int verify)
{

}