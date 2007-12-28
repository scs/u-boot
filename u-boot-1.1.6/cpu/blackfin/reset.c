/*
 * reset.c - logic for resetting the cpu
 *
 * Copyright (c) 2005-2007 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#include <common.h>
#include <command.h>
#include <asm/blackfin.h>
#include "cpu.h"

/* A system soft reset makes external memory unusable so force
 * this function into L1.  We use the compiler ssync here rather
 * than SSYNC() because it's safe (no interrupts and such) and
 * we save some L1.  We do not need to force sanity in the SYSCR
 * register as the BMODE selection bit is cleared by the soft
 * reset while the Core B bit (on dual core parts) is cleared by
 * the core reset.
 */
__attribute__ ((__l1_text__, __noreturn__))
void bfin_reset(void)
{
	while (1) {
		/* Not entirely sure why this ssync is needed ... */
		__builtin_bfin_ssync();
		/* Initiate system software reset of peripherals */
		bfin_write_SWRST(0x7);
		__builtin_bfin_ssync();
		/* Clear system software reset */
		bfin_write_SWRST(0);
		__builtin_bfin_ssync();
		/* Issue core reset */
		asm("raise 1");
	}
}

/* We need to trampoline ourselves up into L1 since our linker
 * does not have relaxtion support and will only generate a
 * PC relative call with a 25 bit immediate.  This is not enough
 * to get us from the top of SDRAM into L1.
 */
__attribute__ ((__noreturn__))
static inline void bfin_reset_trampoline(void)
{
	if (board_reset)
		board_reset();
	while (1)
		asm("jump (%0);" : : "a" (bfin_reset));
}

__attribute__ ((__noreturn__))
void bfin_reset_or_hang(void)
{
#ifdef CONFIG_PANIC_HANG
	hang();
#else
	bfin_reset_trampoline();
#endif
}

int do_reset(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	bfin_reset_trampoline();
	return 0;
}
