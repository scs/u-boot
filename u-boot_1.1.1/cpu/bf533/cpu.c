/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Alex Zuepke <azu@sysgo.de>
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

**********************************************************************************************************

                        PROJECT                 :       BFIN
                        VERISON                 :       2.0
                        FILE                    :       cpu.c
                        MODIFIED DATE           :       29 jun 2004
                        AUTHOR                  :       BFin Project-ADI
                        LOCATION                :       LG Soft India,Bangalore

***********************************************************************************************************/


/* CPU specific code */

#include <common.h>
#include <asm/blackfin.h>
#include <command.h>

int cpu_init (void)
{
	return 0;
}

int cleanup_before_linux (void)
{
	return 0;
}


int do_reset (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{

	printf("Reset function Not enabled \n");
/*
	*(unsigned volatile short *)(SYSCR) = 0x10;
	*(unsigned volatile short *)(SWRST) = 0x7;
*/
	return (0);
}

void icache_enable (void)
{

}

void icache_disable (void)
{

}

int icache_status (void)
{
	return 0;
}

void dcache_enable (void)
{

}

void dcache_disable (void)
{

}

int dcache_status (void)
{
	return 0;
}
