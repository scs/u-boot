/*
 * blackfin-config-post.h - setup common defines for Blackfin boards based on config.h
 *
 * Copyright (c) 2007-2008 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __ASM_BLACKFIN_CONFIG_POST_H__
#define __ASM_BLACKFIN_CONFIG_POST_H__

/* Sanity check CONFIG_BFIN_CPU */
#ifndef CONFIG_BFIN_CPU
# error CONFIG_BFIN_CPU: your board config needs to define this
#endif

/* Make sure the structure is properly aligned */
#if ((CFG_GBL_DATA_ADDR & -4) != CFG_GBL_DATA_ADDR)
# error CFG_GBL_DATA_ADDR: must be 4 byte aligned
#endif

/* Set default CONFIG_VCO_HZ if need be */
#if !defined(CONFIG_VCO_HZ)
# if (CONFIG_CLKIN_HALF == 0)
#  define CONFIG_VCO_HZ (CONFIG_CLKIN_HZ * CONFIG_VCO_MULT)
# else
#  define CONFIG_VCO_HZ ((CONFIG_CLKIN_HZ * CONFIG_VCO_MULT) / 2)
# endif
#endif

/* Set default CONFIG_CCLK_HZ if need be */
#if !defined(CONFIG_CCLK_HZ)
# if (CONFIG_PLL_BYPASS == 0)
#  define CONFIG_CCLK_HZ (CONFIG_VCO_HZ / CONFIG_CCLK_DIV)
# else
#  define CONFIG_CCLK_HZ CONFIG_CLKIN_HZ
# endif
#endif

/* Set default CONFIG_SCLK_HZ if need be */
#if !defined(CONFIG_SCLK_HZ)
# if (CONFIG_PLL_BYPASS == 0)
#  define CONFIG_SCLK_HZ (CONFIG_VCO_HZ / CONFIG_SCLK_DIV)
# else
#  define CONFIG_SCLK_HZ CONFIG_CLKIN_HZ
# endif
#endif

/* Since we use these to program PLL registers directly,
 * make sure the values are sane and won't screw us up.
 */
#if (CONFIG_VCO_MULT & 0x3F) != CONFIG_VCO_MULT
# error CONFIG_VCO_MULT: Invalid value: must fit in 6 bits (0 - 63)
#endif
#if (CONFIG_CLKIN_HALF & 0x1) != CONFIG_CLKIN_HALF
# error CONFIG_CLKIN_HALF: Invalid value: must be 0 or 1
#endif
#if (CONFIG_PLL_BYPASS & 0x1) != CONFIG_PLL_BYPASS
# error CONFIG_PLL_BYPASS: Invalid value: must be 0 or 1
#endif

/* Using L1 scratch pad makes sense for everyone by default. */
#ifndef CMD_LINE_ADDR
# define CMD_LINE_ADDR L1_SRAM_SCRATCH
#endif

/* Default/common Blackfin memory layout */
#ifndef CFG_SDRAM_BASE
# define CFG_SDRAM_BASE 0
#endif
#ifndef CFG_MAX_RAM_SIZE
# define CFG_MAX_RAM_SIZE (CONFIG_MEM_SIZE * 1024 * 1024)
#endif
#ifndef CFG_MONITOR_BASE
# define CFG_MONITOR_BASE (CFG_MAX_RAM_SIZE - CFG_MONITOR_LEN)
#endif
#ifndef CFG_MALLOC_BASE
# define CFG_MALLOC_BASE (CFG_MONITOR_BASE - CFG_MALLOC_LEN)
#endif
#ifndef CFG_GBL_DATA_ADDR
# define CFG_GBL_DATA_ADDR (CFG_MALLOC_BASE - CFG_GBL_DATA_SIZE)
#endif
#ifndef CFG_STACKBASE
# define CFG_STACKBASE (CFG_GBL_DATA_ADDR - 4)
#endif
#ifndef CFG_MEMTEST_START
# define CFG_MEMTEST_START 0
#endif
#ifndef CFG_MEMTEST_END
# define CFG_MEMTEST_END (CFG_STACKBASE - 8096)
#endif

/* Check to make sure everything fits in external RAM */
#if ((CFG_MONITOR_BASE + CFG_MONITOR_LEN) > CFG_MAX_RAM_SIZE)
# error Memory Map does not fit into configuration
#endif

/* Default/common Blackfin environment settings */
#ifndef CONFIG_LOADADDR
# define CONFIG_LOADADDR 0x1000000
#endif
#ifndef CFG_LOAD_ADDR
# define CFG_LOAD_ADDR CONFIG_LOADADDR
#endif
#ifndef CFG_BOOTM_LEN
# define CFG_BOOTM_LEN 0x4000000
#endif
#ifndef CFG_PROMPT
# define CFG_PROMPT "bfin> "
#endif
#ifndef CFG_CBSIZE
# if (CONFIG_COMMANDS & CFG_CMD_KGDB)
#  define CFG_CBSIZE 1024
# else
#  define CFG_CBSIZE 256
# endif
#endif
#ifndef CFG_BARGSIZE
# define CFG_BARGSIZE CFG_CBSIZE
#endif
#ifndef CFG_PBSIZE
# define CFG_PBSIZE (CFG_CBSIZE + sizeof(CFG_PROMPT) + 16)
#endif
#ifndef CFG_MAXARGS
# define CFG_MAXARGS 16
#endif
#ifndef CFG_HZ
# define CFG_HZ 1000
#endif
#ifndef CFG_BAUDRATE_TABLE
# define CFG_BAUDRATE_TABLE { 9600, 19200, 38400, 57600, 115200 }
#endif

#endif
