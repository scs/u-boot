/*
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
 

#ifndef __CONFIG_BF537_LEANXCAM_H__
#define __CONFIG_BF537_LEANXCAM_H__

#ifndef NO_HEADERS
# include <asm/blackfin-config-pre.h>
#endif

/*
 * Processor Settings
 */
#define CONFIG_BFIN_CPU			bf537-0.3

#define CONFIG_BF537_LEANXCAM

/*
 * Clock Settings
 *	CCLK = (CLKIN * VCO_MULT) / CCLK_DIV
 *	SCLK = (CLKIN * VCO_MULT) / SCLK_DIV
 */
/* CONFIG_CLKIN_HZ is any value in Hz					*/
#define CONFIG_CLKIN_HZ			25000000
/* CLKIN_HALF controls the DF bit in PLL_CTL      0 = CLKIN		*/
/*                                                1 = CLKIN / 2		*/
#define CONFIG_CLKIN_HALF		0
/* PLL_BYPASS controls the BYPASS bit in PLL_CTL  0 = do not bypass	*/
/*                                                1 = bypass PLL	*/
#define CONFIG_PLL_BYPASS		0
/* VCO_MULT controls the MSEL (multiplier) bits in PLL_CTL		*/
/* Values can range from 0-63 (where 0 means 64)			*/
#define CONFIG_VCO_MULT			20
#define CONFIG_VCO_HZ           ( CONFIG_CLKIN_HZ * CONFIG_VCO_MULT )
/* CCLK_DIV controls the core clock divider				*/
/* Values can be 1, 2, 4, or 8 ONLY					*/
#define CONFIG_CCLK_DIV			1
/* SCLK_DIV controls the system clock divider				*/
/* Values can range from 1-15						*/
#define CONFIG_SCLK_DIV			4

#define CONFIG_CCLK_HZ          ( CONFIG_VCO_HZ / CONFIG_CCLK_DIV )
#define CONFIG_SCLK_HZ          ( CONFIG_VCO_HZ / CONFIG_SCLK_DIV )

#define	CFG_HZ					1000	/* decrementer freq: 10 ms ticks */

/*
 * Cache Settings
 */
#define CONFIG_ICACHE_ON
#define CONFIG_DCACHE_ON

/*
 * Memory Settings
 */
#define CONFIG_MEM_ADD_WDTH	10
#define CONFIG_MEM_SIZE		64	/* [MB] */
#define CONFIG_MEM_MT48LC32M16A2_75    	1

#define CONFIG_EBIU_SDRRC_VAL	0x306
#define CONFIG_EBIU_SDGCTL_VAL	0x91114d
#define CONFIG_EBIU_SDBCTL_VAL	(EBSZ_64 | EBCAW_10 | EBE)

#define CONFIG_EBIU_AMGCTL_VAL	0xFF
#define CONFIG_EBIU_AMBCTL0_VAL	0x7BB07BB0
#define CONFIG_EBIU_AMBCTL1_VAL	0xFFC27BB0

#define CFG_MAX_RAM_SIZE    (CONFIG_MEM_SIZE * 1024*1024)
#define CFG_MONITOR_LEN		(256 * 1024)	/* Reserve 256 kB for monitor */
#define	CFG_MALLOC_LEN		(128 * 1024)	/* Reserve 128 kB for malloc()	*/
#define	CFG_CBSIZE			256	/* Console I/O Buffer Size */
#define CFG_GBL_DATA_SIZE	0x4000
#define	CFG_PROMPT			"u-boot> "	/* Monitor Command Prompt */
#define	CFG_PBSIZE			(CFG_CBSIZE+sizeof(CFG_PROMPT)+16)	/* Print Buffer Size */
#define CFG_BARGSIZE		CFG_CBSIZE	/* Boot Argument Buffer Size */

#define	CFG_SDRAM_BASE		0x00000000
#define CFG_MONITOR_BASE	(CFG_MAX_RAM_SIZE - CFG_MONITOR_LEN)
#define CFG_MALLOC_BASE		(CFG_MONITOR_BASE - CFG_MALLOC_LEN)
#define CFG_GBL_DATA_ADDR	(CFG_MALLOC_BASE - CFG_GBL_DATA_SIZE)
#define CONFIG_STACKBASE	(CFG_GBL_DATA_ADDR  - 4)

#define CFG_FLASH_BASE		0x10000000

#define CONFIG_LOADADDR			0x1000000
#define CFG_LOADADDR_LINUX		0x2000000		
#define	CFG_LOAD_ADDR			CONFIG_LOADADDR	/* default load address */


/*
 *	Serial console Settings 
 */
#define CONFIG_UART_CONSOLE	0
/*#define DEBUG 1*/
/*#define CONFIG_DEBUG_EARLY_SERIAL	1
  #define CONFIG_DEBUG_SERIAL	1*/
#define	CFG_MAXARGS			16	/* max number of command args */
#define CFG_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }
#define CONFIG_BAUDRATE		115200

/*
 * Network Settings
 */
#define CONFIG_IPADDR		192.168.1.10
#define CONFIG_GATEWAYIP	192.168.1.1
#define CONFIG_SERVERIP		192.168.1.2
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_HOSTNAME		bf537-leanXcam 

#define ADI_CMDS_NETWORK	1 	
#define CONFIG_BFIN_MAC		
#define CONFIG_NETCONSOLE	1
#define CONFIG_NET_MULTI	1


/*
 * Dataflash Settings
 * 
 * 0x0	aera0: monitor (size=CFG_MONITOR_LEN)
 * 		aera1: env_uboot (size=CFG_ENV_SIZE)
 * 		aera2: env_uboot redundant (size=CFG_ENV_SIZE_REDUND)
 * 		aera3: linux...
 */

#define CFG_MONITOR_SIZE                0x20000 		/* 128k partition on flash reserved for U-boot image. */
#define CFG_ENV_SIZE					0x4000			/* 8k environment; twice due to redundency */
#define CFG_ENV_SIZE_REDUND				CFG_ENV_SIZE	/* same for redundency */ 
#define CFG_DATAFLASH_SIZE              0x400000 		/* 4 MB */
#define CFG_DATAFLASH_LOGIC_ADDR_CS0    0x10000000      /*Logical adress for Flash 1 (CS0, cs=0) */
#define CFG_DATAFLASH_LOGIC_ADDR_CS3    0x20000000      /*Logical adress for Flash 2 (CS3, cs=3)*/


#define CFG_FLASH_BASE_MONITOR		CFG_DATAFLASH_LOGIC_ADDR_CS0
#define CFG_FLASH_BASE_ENV_UBOOT	(CFG_FLASH_BASE_MONITOR + CFG_MONITOR_SIZE)
#define CFG_FLASH_BASE_ENV_REDUND	(CFG_FLASH_BASE_ENV_UBOOT + CFG_ENV_SIZE)
/*#define CFG_FLASH_BASE_LINUX		(CFG_FLASH_BASE_ENV_REDUND + CFG_ENV_SIZE)*/
#define CFG_FLASH_BASE_LINUX		0x10028000
 
#define CONFIG_HAS_DATAFLASH            1
#define CFG_SPI_WRITE_TOUT              (5*CFG_HZ)
#define CFG_MAX_DATAFLASH_BANKS         2        /* 2 flash chips (program / data) */
#define CFG_MAX_DATAFLASH_PAGES         8192

#define CONFIG_DATAFLASH_CUSTOM_AREAS   /* Customize the areas in the dataflash and their protection */
/* Area 0: U-Boot monitor */
#define CFG_DATAFLASH_AREA0_START       0x0
#define CFG_DATAFLASH_AREA0_END         CFG_DATAFLASH_AREA0_START + CFG_MONITOR_SIZE
#define CFG_DATAFLASH_AREA0_PROTECT     0
/* Area 1: U-Boot environment */
#define CFG_DATAFLASH_AREA1_START       CFG_DATAFLASH_AREA0_END
#define CFG_DATAFLASH_AREA1_END         CFG_DATAFLASH_AREA1_START + CFG_ENV_SIZE
#define CFG_DATAFLASH_AREA1_PROTECT     0
/* Area 2: U-Boot environment redundant */
#define CFG_DATAFLASH_AREA2_START       CFG_DATAFLASH_AREA1_END
#define CFG_DATAFLASH_AREA2_END         CFG_DATAFLASH_AREA2_START + CFG_ENV_SIZE
#define CFG_DATAFLASH_AREA2_PROTECT     0
/* Area 1: Linux */
#define CFG_DATAFLASH_AREA3_START       CFG_DATAFLASH_AREA2_END
#define CFG_DATAFLASH_AREA3_END         CFG_DATAFLASH_SIZE
#define CFG_DATAFLASH_AREA3_PROTECT     0


#define CFG_NO_FLASH 
#define CFG_MAX_FLASH_BANKS	0	/* max number of memory banks */
#define CFG_MAX_FLASH_SECT	0	/* max number of sectors on one chip */


/*
 * Environment variables
 */
#define	CFG_ENV_IS_IN_DATAFLASH	1

#ifdef CFG_ENV_IS_IN_DATAFLASH
	#define CFG_ENV_ADDR			CFG_FLASH_BASE_ENV_UBOOT
	#define CFG_ENV_ADDR_REDUND		CFG_FLASH_BASE_ENV_REDUND
	#define	CFG_ENV_SECT_SIZE		CFG_ENV_SIZE	/* Total Size of Environment Sector */
#endif /* CFG_ENV_IS_IN_DATAFLASH */


#define CONFIG_EXTRA_ENV_SETTINGS                               \
	"flash_base_linux=" MK_STR(CFG_FLASH_BASE_LINUX) "\0" \
	"flash_base_monitor=" MK_STR(CFG_FLASH_BASE_MONITOR) "\0" \
	"loadaddr-linux="  MK_STR(CFG_LOADADDR_LINUX) "\0" \
	"boot=bootm $(flash_base_linux)\0" \
	"nokgdbargs=setenv bootargs root=/dev/mtdblock0 rw console=ttyBF0,115200\0"	\
    "nfsargs=setenv bootargs root=/dev/nfs rw \0"             \
    "nfsroot=$(serverip):$(rootpath) console=ttyBF0,57600\0"                     \
    "upduboot=tftp $(loadaddr) u-boot.ldr; cp.b $(loadaddr) $(flash_base_monitor) $(filesize) \0"                 \
	"updlinux=tftp $(loadaddr-linux) uImage; cp.b $(loadaddr-linux) $(flash_base_linux) $(filesize)\0"    \
	"updbootflash=tftp $(loadaddr) boot-flash; cp.b $(loadaddr) $(flash_base_monitor) $(filesize)\0"    \
	"tstuboot=tftp $(loadaddr) u-boot.bin; go $(loadaddr)\0"  \
	"tstlinux=tftp $(loadaddr-linux) uImage; bootm $(loadaddr-linux)\0" \
	""



/* CONFIG_SPI_BAUD controls the SPI peripheral clock divider		*/
/* Values can range from 2-65535					*/
/* SCK Frequency = SCLK / (2 * CONFIG_SPI_BAUD)				*/
#define CONFIG_SPI
#define CONFIG_SPI_BAUD			4
#define CONFIG_SPI_BAUD_INITBLOCK	4


/*
 * Boot behaviour
 */
#define CONFIG_BFIN_BOOT_MODE	BFIN_BOOT_UART
#define CONFIG_BOOTDELAY	2			/* Delay before linux launch. */
#define CONFIG_PANIC_HANG	1			/* No automatic reboot when panic. */
#define CONFIG_LOADS_ECHO	1  			/* Echo on for serial download  */
#define CFG_AUTOLOAD   		"no"		/*rarpb, bootp or dhcp commands will perform only a */
										/* configuration lookup from the BOOTP/DHCP server, */
#define CONFIG_BOOTCOMMAND 	"run boot"	/* flash boot */


/*
 * Linux parameters
 */
 #define CONFIG_ROOTPATH	/romfs
 #define CONFIG_BOOTARGS 	"root=/dev/mtdblock0 rw console=ttyBF0,115200"
 #define CFG_BOOTMAPSZ		(8 * 1024*1024)	/* Initial Memory map for Linux */


/*
 * Misc Settings
 */
#define CFG_LONGHELP 1				/* Long help comments */
#define CONFIG_CMDLINE_EDITING 1	/* Extra console functionality: editing, history */

/*
 * PowerOn Self Test; POST
 */
#define CFG_MEMTEST_START	0x0	/* memtest works on */
#define CFG_MEMTEST_END		( (CONFIG_MEM_SIZE - 1) * 1024*1024)	/* whole DRAM */
 
#undef CONFIG_POST	/* Define if want to do post memory test */
/*#define CONFIG_POST 		( CFG_POST_MEMORY | \
				  CFG_POST_UART	  | \
				  CFG_POST_FLASH  | \
				  CFG_POST_ETHER) */ 
#ifdef CONFIG_POST
#define CFG_CMD_POST_DIAG	CFG_CMD_DIAG
#define FLASH_START_POST_BLOCK	11	/* Should > = 11 */
#define FLASH_END_POST_BLOCK	71	/* Should < = 71 */
#endif

/*
 * U-boot commands
 */
#define CONFIG_BFIN_CMD		(CONFIG_CMD_DFL | CFG_CMD_PING)
#define CONFIG_COMMANDS			(((CONFIG_BFIN_CMD| \
					 CFG_CMD_ELF	| \
					 CFG_CMD_CACHE  | \
					 CFG_CMD_DHCP   | \
					 ADD_IDE_CMD	| \
					 CFG_CMD_FLASH  | \
					 CFG_CMD_ENV	| \
					 CFG_CMD_ASKENV	| \
					 CFG_CMD_MII    | \
					 CFG_CMD_NET    | \
					 CFG_CMD_POST_DIAG) & ~(CFG_CMD_IMLS))  & ~(CFG_CMD_FLASH)) 

#ifndef NO_HEADERS
/* this must be included AFTER the definition of CONFIG_COMMANDS (if any) */
#include <cmd_confdefs.h>					 
		 

#include <asm/blackfin-config-post.h>
#endif

/*-----------------------------------------------------------------------
 * return codes from flash_write():
 */
#define ERR_OK						0
#define ERR_TIMOUT					1
#define ERR_NOT_ERASED				2
#define ERR_PROTECTED				4
#define ERR_INVAL					8
#define ERR_ALIGN					16
#define ERR_UNKNOWN_FLASH_VENDOR	32
#define ERR_UNKNOWN_FLASH_TYPE		64
#define ERR_PROG_ERROR				128

/*-----------------------------------------------------------------------
 * Protection Flags for flash_protect():
 */
#define FLAG_PROTECT_SET	0x01
#define FLAG_PROTECT_CLEAR	0x02

#endif
