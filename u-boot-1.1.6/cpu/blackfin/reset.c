/*
 * reset.c - logic for resetting the cpu
 *
 * Copyright (c) 2005-2007 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#include <common.h>
#include <command.h>
#include <asm/blackfin.h>
#include <asm/mach-common/bits/bootrom.h>
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
static void bfin_reset(void)
{
	/* Wait for completion of "system" events such as cache line
	 * line fills so that we avoid infinite stalls later on as
	 * much as possible.  This code is in L1, so it won't trigger
	 * any such event after this point in time.
	 */
	__builtin_bfin_ssync();

	while (1) {
		/* Initiate System software reset. */
		bfin_write_SWRST(0x7);

		/* Due to the way reset is handled in the hardware, we need
		 * to delay for 7 SCLKS.  The only reliable way to do this is
		 * to calculate the CCLK/SCLK ratio and multiply 7.  For now,
		 * we'll assume worse case which is a 1:15 ratio.
		 */
		asm(
			"LSETUP (1f, 1f) LC0 = %0\n"
			"1: nop;"
			:
			: "a" (15 * 7)
			: "LC0", "LB0", "LT0"
		);

		/* Clear System software reset */
		bfin_write_SWRST(0);

		/* Wait for the SWRST write to complete.  Cannot rely on SSYNC
		 * though as the System state is all reset now.
		 */
		asm(
			"LSETUP (1f, 1f) LC1 = %0\n"
			"1: nop;"
			:
			: "a" (15 * 1)
			: "LC1", "LB1", "LT1"
		);

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
		if (0 /*BOOTROM_CAPS_SYSCONTROL -- current silicon is broken*/)
			syscontrol(SYSCTRL_SOFTRESET, NULL, NULL);
		else
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
