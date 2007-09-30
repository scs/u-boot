/*
 *  U-boot - cpu.h
 *
 *  Copyright (c) 2005-2007 Analog Devices Inc.
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

#ifndef _CPU_H_
#define _CPU_H_

#include <command.h>

void bfin_reset(void) __attribute__((__noreturn__));
int do_reset(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
void blackfin_irq_panic(int reason, struct pt_regs *reg);
asmlinkage void trap(void);
void dump(struct pt_regs *regs);

#endif
