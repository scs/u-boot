/*
 * U-boot - Configuration file for BF538F EZ-Kit Lite board
 */

#ifndef __CONFIG_BF538F_EZKIT_H__
#define __CONFIG_BF538F_EZKIT_H__

#include <asm/blackfin-config-pre.h>


/*
 * Processor Settings
 */
#define CONFIG_BFIN_CPU             bf538-0.3
#define CONFIG_BFIN_BOOT_MODE       BFIN_BOOT_BYPASS


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
#define CONFIG_VCO_MULT			21
/* CCLK_DIV controls the core clock divider				*/
/* Values can be 1, 2, 4, or 8 ONLY					*/
#define CONFIG_CCLK_DIV			1
/* SCLK_DIV controls the system clock divider				*/
/* Values can range from 1-15						*/
#define CONFIG_SCLK_DIV			4


/*
 * Memory Settings
 */
#define CONFIG_MEM_ADD_WDTH	10
#define CONFIG_MEM_SIZE		64

#define CONFIG_EBIU_SDRRC_VAL	(0x03F6)
#define CONFIG_EBIU_SDGCTL_VAL	(SCTLE | PSS | TWR_2 | TRCD_3 | TRP_3 | TRAS_6 | PASR_ALL | CL_3)
#define CONFIG_EBIU_SDBCTL_VAL	(EBSZ_64 | EBCAW_10 | EBE)

#define CONFIG_EBIU_AMGCTL_VAL	(CDPRIO | AMBEN_ALL | AMCKEN)
#define CONFIG_EBIU_AMBCTL0_VAL	(B1WAT_7 | B1RAT_11 | B1HT_2 | B1ST_3 | B0WAT_7 | B0RAT_11 | B0HT_2 | B0ST_3)
#define CONFIG_EBIU_AMBCTL1_VAL	(B3WAT_7 | B3RAT_11 | B3HT_2 | B3ST_3 | B2WAT_7 | B2RAT_11 | B2HT_2 | B2ST_3)

#define CFG_MONITOR_LEN		(256 * 1024)	/* Reserve 256 kB for monitor */
#define CFG_MALLOC_LEN		(384 * 1024)	/* Reserve 384 kB for malloc() (video/spi are big) */
#define CFG_GBL_DATA_SIZE	0x4000


/*
 * Network Settings
 */
#define ADI_CMDS_NETWORK	1
#define CONFIG_DRIVER_SMC91111	1
#define CONFIG_SMC91111_BASE	0x20310300
#define CONFIG_HOSTNAME		bf538f-ezkit
/* Uncomment next line to use fixed MAC address */
/* #define CONFIG_ETHADDR	02:80:ad:20:31:e8 */


/*
 * Flash Settings
 */
#define CFG_FLASH_BASE		0x20000000
#define CFG_FLASH_CFI		/* The flash is CFI compatible */
#define CFG_FLASH_CFI_DRIVER	/* Use common CFI driver */
#define CFG_FLASH_PROTECTION
#define CFG_MAX_FLASH_BANKS	1
#define CFG_MAX_FLASH_SECT	71	/* some have 67 sectors (M29W320DB), but newer have 71 (M29W320EB) */

#if (CONFIG_BFIN_BOOT_MODE == BFIN_BOOT_SPI_MASTER)
#define CFG_ENV_IS_IN_EEPROM	1
#define CFG_ENV_OFFSET		0x4000
#define CFG_ENV_HEADER		(CFG_ENV_OFFSET + 0x16e) /* 0x12A is the length of LDR file header */
#else
#define CFG_ENV_IS_IN_FLASH	1
#define CFG_ENV_ADDR		0x20004000
#define CFG_ENV_OFFSET		(CFG_ENV_ADDR - CFG_FLASH_BASE)
#endif
#define CFG_ENV_SIZE		0x2000
#define CFG_ENV_SECT_SIZE	0x2000	/* Total Size of Environment Sector */
#if (CONFIG_BFIN_BOOT_MODE == BFIN_BOOT_BYPASS)
#define ENV_IS_EMBEDDED
#else
#define ENV_IS_EMBEDDED_CUSTOM
#endif

/* CONFIG_SPI_BAUD controls the SPI peripheral clock divider		*/
/* Values can range from 2-65535					*/
/* SCK Frequency = SCLK / (2 * CONFIG_SPI_BAUD)				*/
#define CONFIG_SPI_BAUD			2
#define CONFIG_SPI_BAUD_INITBLOCK	4


/*
 * I2C Settings
 */
#define CONFIG_HARD_I2C		1	/* I2C TWI */
#define CFG_I2C_SPEED		50000
#define CFG_I2C_SLAVE		0


/*
 * Misc Settings
 */
#define CONFIG_RTC_BFIN

#define CONFIG_BFIN_COMMANDS ( CFG_BFIN_CMD_CPLBINFO )


/*
 * Pull in common ADI header for remaining command/environment setup
 */
#include <configs/bfin_adi_common.h>

#include <asm/blackfin-config-post.h>

#endif
