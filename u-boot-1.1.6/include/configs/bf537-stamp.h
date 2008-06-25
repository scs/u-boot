/* 
 * U-boot - Configuration file for BF537 LCV board
 */

#ifndef __CONFIG_BF537_H__
#define __CONFIG_BF537_H__

//#define CONFIG_BF537LCV
#define CFG_LONGHELP		1
#define CONFIG_CMDLINE_EDITING	1
#define CONFIG_BAUDRATE		115200
/* Set default serial console for bf537 */
#define CONFIG_UART_CONSOLE	0
#define CONFIG_BF537		1
#define CONFIG_BOOTDELAY	2
/* define CONFIG_BF537_STAMP_LEDCMD to enable LED command*/
/*#define CONFIG_BF537_STAMP_LEDCMD	1*/

/*
 * Boot Mode Set  
 * Blackfin can support several boot modes
 */
#define BF537_BYPASS_BOOT	0x0011  /* Bootmode 0: Execute from 16-bit externeal memory ( bypass BOOT ROM) 	*/
#define BF537_PARA_BOOT		0x0012  /* Bootmode 1: Boot from 8-bit or 16-bit flash 				*/
#define BF537_SPI_MASTER_BOOT	0x0014	/* Bootmode 3: SPI master mode boot from SPI flash			*/
#define BF537_SPI_SLAVE_BOOT	0x0015	/* Bootmode 4: SPI slave mode boot from SPI flash			*/
#define BF537_TWI_MASTER_BOOT	0x0016	/* Bootmode 5: TWI master mode boot from EEPROM				*/
#define BF537_TWI_SLAVE_BOOT	0x0017	/* Bootmode 6: TWI slave mode boot from EEPROM				*/
#define BF537_UART_BOOT		0x0018	/* Bootmode 7: UART slave mdoe boot via UART host			*/
/* Define the boot mode */
//#define BFIN_BOOT_MODE		BF537_BYPASS_BOOT
//#define BFIN_BOOT_MODE		BF537_SPI_MASTER_BOOT
#define BFIN_BOOT_MODE		BF537_UART_BOOT

#define CONFIG_PANIC_HANG 1

#define ADSP_BF534		0x34
#define ADSP_BF536		0x36
#define ADSP_BF537		0x37
#define BFIN_CPU		ADSP_BF537

/* This sets the default state of the cache on U-Boot's boot */
#define CONFIG_ICACHE_ON
#define CONFIG_DCACHE_ON

/* Define if want to do post memory test */
#undef CONFIG_POST_TEST

/* Define where the uboot will be loaded by on-chip boot rom */
#define APP_ENTRY 0x00001000

//#define CONFIG_RTC_BF533	1
#define CONFIG_BOOT_RETRY_TIME	-1	/* Enable this if bootretry required, currently its disabled */

/* CONFIG_CLKIN_HZ is any value in Hz                            */
#define CONFIG_CLKIN_HZ          25000000
/* CONFIG_CLKIN_HALF controls what is passed to PLL 0=CLKIN      */
/*                                                  1=CLKIN/2    */
#define CONFIG_CLKIN_HALF               0
/* CONFIG_PLL_BYPASS controls if the PLL is used 0=don't bypass  */
/*                                               1=bypass PLL    */
#define CONFIG_PLL_BYPASS               0
/* CONFIG_VCO_MULT controls what the multiplier of the PLL is.   */
/* Values can range from 1-64                                    */
#define CONFIG_VCO_MULT			20
/* CONFIG_CCLK_DIV controls what the core clock divider is       */
/* Values can be 1, 2, 4, or 8 ONLY                              */
#define CONFIG_CCLK_DIV			1
/* CONFIG_SCLK_DIV controls what the peripheral clock divider is */
/* Values can range from 1-15                                    */
#define CONFIG_SCLK_DIV			4
/* CONFIG_SPI_BAUD controls the SPI peripheral clock divider     */
/* Values can range from 2-65535                                 */
/* SCK Frequency = SCLK / (2 * CONFIG_SPI_BAUD)                  */
#define CONFIG_SPI_BAUD                 4
#if (BFIN_BOOT_MODE == BF537_SPI_MASTER_BOOT)
#define CONFIG_SPI_BAUD_INITBLOCK						   4
#endif

#if ( CONFIG_CLKIN_HALF == 0 )
#define CONFIG_VCO_HZ           ( CONFIG_CLKIN_HZ * CONFIG_VCO_MULT )
#else
#define CONFIG_VCO_HZ           (( CONFIG_CLKIN_HZ * CONFIG_VCO_MULT ) / 2 )
#endif

#if (CONFIG_PLL_BYPASS == 0)
#define CONFIG_CCLK_HZ          ( CONFIG_VCO_HZ / CONFIG_CCLK_DIV )
#define CONFIG_SCLK_HZ          ( CONFIG_VCO_HZ / CONFIG_SCLK_DIV )
#else
#define CONFIG_CCLK_HZ          CONFIG_CLKIN_HZ
#define CONFIG_SCLK_HZ          CONFIG_CLKIN_HZ
#endif

#if (BFIN_BOOT_MODE == BF537_SPI_MASTER_BOOT)
#if (CONFIG_SCLK_HZ / (2*CONFIG_SPI_BAUD) > 20000000) 
#define CONFIG_SPI_FLASH_FAST_READ 1 /* Needed if SPI_CLK > 20 MHz */
#else
#undef CONFIG_SPI_FLASH_FAST_READ
#endif
#endif

#define CONFIG_MEM_SIZE                 64             /* 128, 64, 32, 16 */
#define CONFIG_MEM_ADD_WDTH            	10             /* 8, 9, 10, 11    */
#define CONFIG_MEM_MT48LC32M16A2_75    	1

#define CONFIG_LOADS_ECHO	1

#define CFG_AUTOLOAD                    "no"    /*rarpb, bootp or dhcp commands will perform only a */
                                                /* configuration lookup from the BOOTP/DHCP server, */

#define CFG_BOOTM_LEN	0x2000000               /* specify max uncompressed image file size that bootm 
						   might have to be able to uncompress */

#define CFG_NO_FLASH

/*
 * Dataflash Settings
 */
#define CONFIG_HAS_DATAFLASH            1
#define CFG_SPI_WRITE_TOUT              (5*CFG_HZ)
#define CFG_MAX_DATAFLASH_BANKS         2        /*2 dataflash chips on lcv*/
#define CFG_MAX_DATAFLASH_PAGES         8192
#define CFG_DATAFLASH_LOGIC_ADDR_CS0    0x10000000      /*Logical adress for Flash 1 (CS0, cs=0) */
#define CFG_DATAFLASH_LOGIC_ADDR_CS3    0x20000000      /*Logical adress for Flash 2 (CS3, cs=3)*/

//#define CFG_ENV_IS_IN_DATAFLASH

/*
 * Network Settings
 */
/* network support */
#if (BFIN_CPU != ADSP_BF534)
#define CONFIG_IPADDR           192.168.11.10
#define CONFIG_NETMASK          255.255.255.0
#define CONFIG_GATEWAYIP        192.168.11.1
#define CONFIG_SERVERIP         172.18.1.76
#define CONFIG_HOSTNAME         BF537-LCV
#define CONFIG_DP83848                         
#define CONFIG_MII              
#endif

#define CONFIG_ROOTPATH		/romfs
/* Uncomment next line to use fixed MAC address. This is the default
MAC address in the address range of the industrial LCV board. */
#define CONFIG_ETHADDR          00:20:e3:22:00:00 
/* This is the routine that copies the MAC in Flash to the 'ethaddr' setting */

#define CFG_LONGHELP			1
//#define CONFIG_BOOTDELAY	        -1 //disable autoboot
#define CONFIG_BOOT_RETRY_TIME		-1	/* Enable this if bootretry required, currently its disabled */
//#define CONFIG_BOOTCOMMAND 		"run tstlinux"  /* tftp boot  */
#define CONFIG_BOOTCOMMAND 		"run boot"      /* flash boot */

#if (BFIN_BOOT_MODE == BF537_BYPASS_BOOT) && defined(CONFIG_POST_TEST)
/* POST support */
#define CONFIG_POST 		( CFG_POST_MEMORY | \
				  CFG_POST_UART	  | \
				  CFG_POST_FLASH  | \
				  CFG_POST_ETHER)
#else 
#undef CONFIG_POST
#endif

#ifdef CONFIG_POST
#define CFG_CMD_POST_DIAG	CFG_CMD_DIAG
#define FLASH_START_POST_BLOCK 11       /* Should > = 11 */
#define FLASH_END_POST_BLOCK   71       /* Should < = 71 */
#else 
#define CFG_CMD_POST_DIAG	0
#endif

/* CF-CARD IDE-HDD Support */

//#define CONFIG_BFIN_TRUE_IDE    /* Add CF flash card support */
//#define CONFIG_BFIN_CF_IDE    /* Add CF flash card support */
//#define CONFIG_BFIN_HDD_IDE   /* Add IDE Disk Drive (HDD) support */


#if defined(CONFIG_BFIN_CF_IDE) || defined(CONFIG_BFIN_HDD_IDE) || defined(CONFIG_BFIN_TRUE_IDE) 
#  define CONFIG_BFIN_IDE	1
#  define ADD_IDE_CMD           CFG_CMD_IDE
#else
#  define ADD_IDE_CMD           0
#endif

/*#define CONFIG_BF537_NAND */		/* Add nand flash support */

#ifdef CONFIG_BF537_NAND
#  define ADD_NAND_CMD		CFG_CMD_NAND
#else
#  define ADD_NAND_CMD		0
#endif

#define CONFIG_NETCONSOLE	1
#define CONFIG_NET_MULTI	1

#if (BFIN_CPU == ADSP_BF534)
#define CONFIG_BFIN_CMD		(CONFIG_CMD_DFL & ~CFG_CMD_NET) 
#else
#define CONFIG_BFIN_CMD		(CONFIG_CMD_DFL | CFG_CMD_PING)
#endif

#if ((BFIN_BOOT_MODE == BF537_BYPASS_BOOT) || (BFIN_BOOT_MODE == BF537_UART_BOOT))
#define CONFIG_COMMANDS			(((CONFIG_BFIN_CMD| \
					 CFG_CMD_ELF	| \
					 CFG_CMD_CACHE  | \
					 CFG_CMD_DHCP   | \
					 ADD_IDE_CMD	| \
					 CFG_CMD_FLASH  | \
					 CFG_CMD_MII    | \
					 CFG_CMD_NET    | \
					 CFG_CMD_POST_DIAG) & ~(CFG_CMD_FLASH)) & ~(CFG_CMD_IMLS))
//CFG_CMD_JFFS2; CFG_CMD_EEPROM;CFG_CMD_NAND;CFG_CMD_I2C;CFG_CMD_DATE
#elif (BFIN_BOOT_MODE == BF537_SPI_MASTER_BOOT)
#define CONFIG_COMMANDS			(CONFIG_BFIN_CMD| \
					 CFG_CMD_ELF	| \
					 CFG_CMD_I2C	| \
					 CFG_CMD_CACHE  | \
					 ADD_IDE_CMD	| \
					 CFG_CMD_DATE)
//CFG_CMD_JFFS2; CFG_CMD_EEPROM
#endif

#define CONFIG_BOOTARGS "root=/dev/mtdblock0 rw console=ttyBF0,115200 kgdboe=@192.168.1.1/,@192.168.1.3/"	
#define CONFIG_LOADADDR	0x1000000

#if ((BFIN_BOOT_MODE == BF537_BYPASS_BOOT) || (BFIN_BOOT_MODE == BF537_UART_BOOT))
#if (BFIN_CPU != ADSP_BF534)
#define CONFIG_EXTRA_ENV_SETTINGS                               \
	"boot=bootm 0x10020000\0" \
	"nokgdbargs=setenv bootargs root=/dev/mtdblock0 rw console=ttyBF0,115200\0"	\
        "nfsargs=setenv bootargs root=/dev/nfs rw \0"             \
        "nfsroot=$(serverip):$(rootpath) console=ttyBF0,57600\0"                     \
        "upduboot=tftp 0x1000000 u-boot.ldr; cp.b 0x1000000 0x10000000 $(filesize) \0"                 \
	"updlinux=tftp 0x2000000 uImage; cp.b 0x2000000 0x10020000 $(filesize)\0"    \
	"tstuboot=tftp 0x1000000 u-boot.bin; go 0x1000000\0"  \
	"tstlinux=tftp 0x2000000 uImage; bootm 0x2000000\0" \
	"updatedhcp=dhcp;set serverip 172.18.128.16; tftp 1000000 u-boot.bin; go 1000000\0"\
	""
#else
#define CONFIG_EXTRA_ENV_SETTINGS                               \
        "ramargs=setenv bootargs root=/dev/mtdblock0 rw console=ttyBF0,57600\0" \
        "flashboot=bootm 0x20100000\0" \
        ""
#endif
#elif (BFIN_BOOT_MODE == BF537_SPI_MASTER_BOOT)
#if (BFIN_CPU != ADSP_BF534)
#define CONFIG_EXTRA_ENV_SETTINGS					\
        "ramargs=setenv bootargs root=/dev/mtdblock0 rw console=ttyBF0,57600\0"		\
        "nfsargs=setenv bootargs root=/dev/nfs rw "			\
        "nfsroot=$(serverip):$(rootpath) console=ttyBF0,57600\0"				\
        "addip=setenv bootargs $(bootargs) "				\
        "ip=$(ipaddr):$(serverip):$(gatewayip):$(netmask)"		\
        ":$(hostname):eth0:off\0"					\
    	"ramboot=tftpboot $(loadaddr) linux;"				\
        "run ramargs;run addip;bootelf\0"				\
        "nfsboot=tftpboot $(loadaddr) linux;"				\
        "run nfsargs;run addip;bootelf\0"				\
        "flashboot=bootm 0x20100000\0"					\
        "update=tftpboot $(loadaddr) u-boot.ldr;"				\
        "eeprom write $(loadaddr) 0x0 $(filesize);\0"			\
        ""
#else
#define CONFIG_EXTRA_ENV_SETTINGS                               \
        "ramargs=setenv bootargs root=/dev/mtdblock0 rw console=ttyBF0,57600\0" \
        "flashboot=bootm 0x20100000\0" \
        ""
#endif
#endif

/* this must be included AFTER the definition of CONFIG_COMMANDS (if any) */
#include <cmd_confdefs.h>

#if (BFIN_BOOT_MODE == BF537_SPI_MASTER_BOOT)
#if (BFIN_CPU == ADSP_BF534)
#define	CFG_PROMPT		"serial_bf534> "	/* Monitor Command Prompt */
#elif (BFIN_CPU == ADSP_BF536)
#define	CFG_PROMPT		"serial_bf536> "	/* Monitor Command Prompt */
#else
#define	CFG_PROMPT		"serial_bf537> "	/* Monitor Command Prompt */
#endif
#else
#if (BFIN_CPU == ADSP_BF534)
#define	CFG_PROMPT		"bf534> "	/* Monitor Command Prompt */
#elif (BFIN_CPU == ADSP_BF536)
#define	CFG_PROMPT		"bf536> "	/* Monitor Command Prompt */
#else
#define	CFG_PROMPT		"u-boot> "	/* Monitor Command Prompt */
#endif
#endif

#if (CONFIG_COMMANDS & CFG_CMD_KGDB)
#define	CFG_CBSIZE		1024	/* Console I/O Buffer Size */
#else
#define	CFG_CBSIZE		256	/* Console I/O Buffer Size */
#endif
#define CFG_MAX_RAM_SIZE       	(CONFIG_MEM_SIZE * 1024*1024)
#define	CFG_PBSIZE		(CFG_CBSIZE+sizeof(CFG_PROMPT)+16)	/* Print Buffer Size */
#define	CFG_MAXARGS		16	/* max number of command args */
#define CFG_BARGSIZE		CFG_CBSIZE	/* Boot Argument Buffer Size */
#define CFG_MEMTEST_START	0x0	/* memtest works on */
#define CFG_MEMTEST_END		( (CONFIG_MEM_SIZE - 1) * 1024*1024)	/* 1 ... 63 MB in DRAM */
#define	CFG_LOAD_ADDR		CONFIG_LOADADDR	/* default load address */
#define	CFG_HZ			1000	/* decrementer freq: 10 ms ticks */
#define CFG_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }
#define	CFG_SDRAM_BASE		0x00000000

#define CFG_FLASH_BASE		0x20000000

#define	CFG_MONITOR_LEN		(256 << 10)	/* Reserve 256 kB for Monitor	*/
#define CFG_MONITOR_BASE	(CFG_MAX_RAM_SIZE - CFG_MONITOR_LEN)
#define	CFG_MALLOC_LEN		(128 << 10)	/* Reserve 128 kB for malloc()	*/
#define CFG_MALLOC_BASE		(CFG_MONITOR_BASE - CFG_MALLOC_LEN)
#define CFG_GBL_DATA_SIZE	0x4000
#define CFG_GBL_DATA_ADDR	(CFG_MALLOC_BASE - CFG_GBL_DATA_SIZE)
#define CONFIG_STACKBASE	(CFG_GBL_DATA_ADDR  - 4)

#define	CFG_BOOTMAPSZ		(8 << 20)	/* Initial Memory map for Linux */
#define CFG_MAX_FLASH_BANKS	0	/* max number of memory banks */
#define CFG_MAX_FLASH_SECT	0	/* max number of sectors on one chip */

#if (BFIN_BOOT_MODE == BF537_BYPASS_BOOT) || (BFIN_BOOT_MODE == BF537_UART_BOOT)    /* for bf537-stamp, uart boot mode still store env in flash */
#define	CFG_ENV_IS_IN_FLASH	1
#define CFG_ENV_SIZE		0x2000
#define CFG_ENV_ADDR		0x20004000
#define CFG_ENV_OFFSET		(CFG_ENV_ADDR - CFG_FLASH_BASE)
#elif (BFIN_BOOT_MODE == BF537_SPI_MASTER_BOOT)
//#define CFG_ENV_IS_IN_EEPROM           1
#define CFG_ENV_OFFSET                 0x4000
#define CFG_ENV_HEADER                 (CFG_ENV_OFFSET + 0x16e)        /* 0x12A is the length of LDR file header */
#endif

#define	CFG_ENV_SECT_SIZE	0x2000	/* Total Size of Environment Sector */
//#if (BFIN_BOOT_MODE == BF537_BYPASS_BOOT)
#define ENV_IS_EMBEDDED
//#endif

/* JFFS Partition offset set  */
//#define CFG_JFFS2_FIRST_BANK 0
//#define CFG_JFFS2_NUM_BANKS  1
/* 512k reserved for u-boot */
//#define CFG_JFFS2_FIRST_SECTOR                 15

//#define CONFIG_SPI

/*
 * Stack sizes
 */
//#define CONFIG_STACKSIZE        (128*1024)      /* regular stack */

//#define POLL_MODE		1
//#define FLASH_TOT_SECT		71
//#define FLASH_SIZE		0x400000
//#define CFG_FLASH_SIZE		0x400000


/*
 * Initialize PSD4256 registers for using I2C
 */
//#define CONFIG_MISC_INIT_R

//#define CFG_BOOTM_LEN			0x4000000       /* Large Image Length, set to 64 Meg */


/*
 * I2C settings
 * By default PF1 is used as SDA and PF0 as SCL on the Stamp board
 */

//#define CONFIG_SOFT_I2C			1*/	/* I2C bit-banged		*/
//#define CONFIG_HARD_I2C			1	/* I2C TWI */
#if defined CONFIG_HARD_I2C
#define CONFIG_TWICLK_KHZ		50
#endif

#if defined CONFIG_SOFT_I2C


/*
 * Software (bit-bang) I2C driver configuration
 */

#define PF_SCL				PF0
#define PF_SDA				PF1

#define I2C_INIT			(*pFIO_DIR |=  PF_SCL); asm("ssync;")
#define I2C_ACTIVE			(*pFIO_DIR |=  PF_SDA); *pFIO_INEN &= ~PF_SDA; asm("ssync;")
#define I2C_TRISTATE			(*pFIO_DIR &= ~PF_SDA); *pFIO_INEN |= PF_SDA; asm("ssync;")
#define I2C_READ			((volatile)(*pFIO_FLAG_D & PF_SDA) != 0); asm("ssync;")
#define I2C_SDA(bit)			if(bit) { \
							*pFIO_FLAG_S = PF_SDA; \
							asm("ssync;"); \
						} \
					else    { \
							*pFIO_FLAG_C = PF_SDA; \
							asm("ssync;"); \
						}
#define I2C_SCL(bit)			if(bit) { \
							*pFIO_FLAG_S = PF_SCL; \
							asm("ssync;"); \
						} \
					else    { \
							*pFIO_FLAG_C = PF_SCL; \
							asm("ssync;"); \
						}
#define I2C_DELAY			udelay(5)	/* 1/4 I2C clock duration */
#endif

//#define CFG_I2C_SPEED			50000
//#define CFG_I2C_SLAVE			0xFE


/* 0xFF, 0x7BB07BB0, 0x22547BB0 */
/* #define AMGCTLVAL            (AMBEN_P0 | AMBEN_P1 | AMBEN_P2 | AMCKEN)
#define AMBCTL0VAL              (B1WAT_7 | B1RAT_11 | B1HT_2 | B1ST_3 | B1TT_4 | ~B1RDYPOL |    \
                                ~B1RDYEN | B0WAT_7 | B0RAT_11 | B0HT_2 | B0ST_3 | B0TT_4 | ~B0RDYPOL | ~B0RDYEN)
#define AMBCTL1VAL              (B3WAT_2 | B3RAT_2 | B3HT_1 | B3ST_1 | B3TT_4 | B3RDYPOL | ~B3RDYEN |   \
                                B2WAT_7 | B2RAT_11 | B2HT_2 | B2ST_3 | B2TT_4 | ~B2RDYPOL | ~B2RDYEN)
*/

#define AMGCTLVAL               0xFF
#define AMBCTL0VAL              0x7BB07BB0
#define AMBCTL1VAL              0xFFC27BB0

//#define CONFIG_VDSP		1

#ifdef CONFIG_VDSP
#define ET_EXEC_VDSP		0x8
#define SHT_STRTAB_VDSP		0x1
#define ELFSHDRSIZE_VDSP	0x2C
#define VDSP_ENTRY_ADDR		0xFFA00000
#endif

#if defined(CONFIG_BFIN_IDE)

#define CONFIG_DOS_PARTITION            1

#endif
#endif
