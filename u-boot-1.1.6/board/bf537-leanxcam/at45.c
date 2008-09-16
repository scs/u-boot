/* Driver for ATMEL DataFlash support
 * Author : Hamid Ikdoumi (Atmel)
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
 *
 */

/* ATMEL DataFlash Driver for ADSP-BF537 Blackfin DSP,
 * modified by Fabian Brugger */
/* Also moved all chip-select and chip-deselect into
 * AT45F_SpiWrite function, and made sure information on
 * which chip to select gets through to AT45F_SpiWrite.*/

#include <config.h>
#include <common.h>
//#include <asm/hardware.h>

//static inline int __attribute__ ((format (printf, 1, 2))) pr_debug(const char * fmt, ...)
//{
//        return 0;
//}

#ifndef ssync 
#define ssync __builtin_bfin_ssync
#endif

#ifdef CONFIG_HAS_DATAFLASH
#include <dataflash.h>

#define AT45C_SPI_CLK	10000000	/* Max Value = 10MHz to be compliant to
the Continuous Array Read function */

/* AC Characteristics */
/* DLYBS = tCSS = 250ns min and DLYBCT = tCSH = 250ns */
#define DATAFLASH_TCSS	(0xC << 16)
#define DATAFLASH_TCHS	(0x1 << 24)

#define AT45C_TIMEOUT_WRDY			200000
#define AT91C_SPI_PCS0_SERIAL_DATAFLASH		0xE     /* Chip Select 0 : NPCS0 %1110 */
#define AT91C_SPI_PCS3_DATAFLASH_CARD		0x7     /* Chip Select 3 : NPCS3 %0111 */

/*--------------------------------------------------------------------------------------*/
/* FUNCTIONS SPECIFIC TO BF537*/
/*--------------------------------------------------------------------------------------*/

void AT45F_SpiInit(void) {
  
  /*-------------------------------------------------------------------*/
  /*	SPI DataFlash Init								*/
  /*-------------------------------------------------------------------*/
  
  *pPORTF_FER|=(PF5|PF10|PF11|PF12|PF13);   //activate SPI pins
  *pPORT_MUX|=PFS5E;                        //mux chips select 5 for flash2
  
  //SCK = SCLK / (2 x SPI_BAUD)
  *pSPI_BAUD=CONFIG_SPI_BAUD;
  
  *pSPI_FLG=0xff00|(FLS1|FLS5);             //add FLS5 for flash2

  *pSPI_CTL=0|(SPE|CPHA|SZ|MSTR|CPOL);      //active low clk, no WOM, 8-bit words, MSBF
}



void AT45F_SpiEnable(int cs) {

  if (cs==0)
    {
      *pSPI_FLG&=0xfffffdff; //CS0 for flash1
    }
  else
    {
      *pSPI_FLG&=0xffffdfff; //CS3 for flash2
    }
}

void AT45F_SpiDisable(void)
{
  *pSPI_FLG|=0xff00; //unselect all flashes
  //spi_wait(40);      //wait 40 cycles (40x2ns), minimum CS\ high time = 50ns
}


void AT45F_WaitRXS(void)
{	
  while (*pSPI_STAT&RXS);	
  while (!(*pSPI_STAT&RXS));
}

unsigned char * AT45F_TransferByte(unsigned char* b)
{
  // sends b, returns what was in RDBR
  // and receives new byte to RDBR
  *pSPI_TDBR=*b;
  ssync;
  *b=(unsigned char)*pSPI_RDBR;
  AT45F_WaitRXS();
  *b=*pSPI_SHADOW;
  return b;
}

unsigned int AT45F_SpiWrite (int cs, AT45PS_DataflashDesc pDesc )
{
	unsigned int timeout;
	int i;

	pDesc->state = BUSY;
	
	/* Chip Select*/
	AT45F_SpiEnable(cs);	

	/* Transmit OpCode*/
	for (i=0; i<pDesc->tx_cmd_size;i++)
	  {
	      *((pDesc->rx_cmd_pt)+i)=*AT45F_TransferByte(pDesc->tx_cmd_pt+i);
	  }

	/* Transmit Data*/
	for (i=0; i<pDesc->tx_data_size; i++)
	  {
	    *((pDesc->rx_data_pt)+i)=*AT45F_TransferByte(pDesc->tx_data_pt+i);
	  }
	
	/* Chip Deselect*/
	AT45F_SpiDisable();

	pDesc->state = IDLE;
	
	timeout=0;
	if (timeout >= CFG_SPI_WRITE_TOUT){
		printf("Error Timeout\n\r");
		return DATAFLASH_ERROR;
	}

	return DATAFLASH_OK;
}



/*END FUNCTIONS SPECIFIC TO BF537*/


/*----------------------------------------------------------------------*/
/* \fn    AT45F_DataFlashSendCommand					*/
/* \brief Generic function to send a command to the dataflash		*/
/*----------------------------------------------------------------------*/
AT45S_DataFlashStatus AT45F_DataFlashSendCommand(
	AT45PS_DataFlash pDataFlash,
	unsigned char OpCode,
	unsigned int CmdSize,
	unsigned int DataflashAddress)
{
 	unsigned int adr;	

	if ( (pDataFlash->pDataFlashDesc->state) != IDLE)
		return DATAFLASH_BUSY;
	/* process the address to obtain page address and byte address */
	adr = ((DataflashAddress / (pDataFlash->pDevice->pages_size)) << pDataFlash->pDevice->page_offset) + (DataflashAddress % (pDataFlash->pDevice->pages_size));
	/* fill the  command  buffer */
	pDataFlash->pDataFlashDesc->command[0] = OpCode;
	if (pDataFlash->pDevice->pages_number >= 16384) {
		pDataFlash->pDataFlashDesc->command[1] = (unsigned char)((adr & 0x0F000000) >> 24);
		pDataFlash->pDataFlashDesc->command[2] = (unsigned char)((adr & 0x00FF0000) >> 16);
		pDataFlash->pDataFlashDesc->command[3] = (unsigned char)((adr & 0x0000FF00) >> 8);
		pDataFlash->pDataFlashDesc->command[4] = (unsigned char)(adr & 0x000000FF);
	} else {
		pDataFlash->pDataFlashDesc->command[1] = (unsigned char)((adr & 0x00FF0000) >> 16);
		pDataFlash->pDataFlashDesc->command[2] = (unsigned char)((adr & 0x0000FF00) >> 8);
		pDataFlash->pDataFlashDesc->command[3] = (unsigned char)(adr & 0x000000FF) ;
		pDataFlash->pDataFlashDesc->command[4] = 0;
	}
	pDataFlash->pDataFlashDesc->command[5] = 0;
	pDataFlash->pDataFlashDesc->command[6] = 0;
	pDataFlash->pDataFlashDesc->command[7] = 0;

	/* Initialize the SpiData structure for the spi write fuction */
	pDataFlash->pDataFlashDesc->tx_cmd_pt   =  pDataFlash->pDataFlashDesc->command ;
	pDataFlash->pDataFlashDesc->tx_cmd_size =  CmdSize ;
	pDataFlash->pDataFlashDesc->rx_cmd_pt   =  pDataFlash->pDataFlashDesc->command ;
	pDataFlash->pDataFlashDesc->rx_cmd_size =  CmdSize ;

	/* send the command and read the data */
	return AT45F_SpiWrite (pDataFlash->pDevice->cs, pDataFlash->pDataFlashDesc);
}


/*----------------------------------------------------------------------*/
/* \fn    AT45F_DataFlashGetStatus					*/
/* \brief Read the status register of the dataflash			*/
/*----------------------------------------------------------------------*/
AT45S_DataFlashStatus AT45F_DataFlashGetStatus(int cs, AT45PS_DataflashDesc pDesc)
{
	AT45S_DataFlashStatus status;

	/* if a transfert is in progress ==> return 0 */
	if( (pDesc->state) != IDLE)
		return DATAFLASH_BUSY;

	/* first send the read status command (D7H) */
	pDesc->command[0] = DB_STATUS;
	pDesc->command[1] = 0;

	pDesc->DataFlash_state  = GET_STATUS;
	pDesc->tx_data_size 	= 0 ;	/* Transmit the command and receive response */
	pDesc->tx_cmd_pt 		= pDesc->command ;
	pDesc->rx_cmd_pt 		= pDesc->command ;
	pDesc->rx_cmd_size 		= 2 ;
	pDesc->tx_cmd_size 		= 2 ;
	status = AT45F_SpiWrite (cs, pDesc);

	pDesc->DataFlash_state = *( (unsigned char *) (pDesc->rx_cmd_pt) +1);

	return status;
}


/*----------------------------------------------------------------------*/
/* \fn    AT45F_Data					*/
/* \brief wait for dataflash ready (bit7 of the status register == 1)	*/
/*----------------------------------------------------------------------*/
AT45S_DataFlashStatus AT45F_DataFlashWaitReady(int cs, AT45PS_DataflashDesc pDataFlashDesc, unsigned int timeout)
{
	pDataFlashDesc->DataFlash_state = IDLE;

	do {
		AT45F_DataFlashGetStatus(cs, pDataFlashDesc);
		timeout--;
	} while( ((pDataFlashDesc->DataFlash_state & 0x80) != 0x80) && (timeout > 0) );

	if((pDataFlashDesc->DataFlash_state & 0x80) != 0x80)
		return DATAFLASH_ERROR;

	return DATAFLASH_OK;
}


/*------------------------------------------------------------------------------*/
/* Function Name       : AT45F_DataFlashFousRead 				*/
/* Object              : Continuous stream Read 				*/
/* Input Parameters    : DataFlash Service					*/
/*						: <src> = dataflash address	*/
/*                     : <*dataBuffer> = data buffer pointer			*/
/*                     : <sizeToRead> = data buffer size			*/
/* Return value		: State of the dataflash				*/
/*------------------------------------------------------------------------------*/
AT45S_DataFlashStatus AT45F_DataFlashContinuousRead (
	AT45PS_DataFlash pDataFlash,
	int src,
	unsigned char *dataBuffer,
	int sizeToRead )
{
	AT45S_DataFlashStatus status;
	/* Test the size to read in the device */
	if ( (src + sizeToRead) > (pDataFlash->pDevice->pages_size * (pDataFlash->pDevice->pages_number)))
		return DATAFLASH_MEMORY_OVERFLOW;

	pDataFlash->pDataFlashDesc->rx_data_pt = dataBuffer;
	pDataFlash->pDataFlashDesc->rx_data_size = sizeToRead;
	pDataFlash->pDataFlashDesc->tx_data_pt = dataBuffer;
	pDataFlash->pDataFlashDesc->tx_data_size = sizeToRead;

	status = AT45F_DataFlashSendCommand (pDataFlash, DB_CONTINUOUS_ARRAY_READ, 8, src);
	/* Send the command to the dataflash */
	return(status);
}


/*------------------------------------------------------------------------------*/
/* Function Name       : AT45F_DataFlashPagePgmBuf				*/
/* Object              : Main memory page program through buffer 1 or buffer 2	*/
/* Input Parameters    : DataFlash Service					*/
/*						: <*src> = Source buffer	*/
/*                     : <dest> = dataflash destination address			*/
/*                     : <SizeToWrite> = data buffer size			*/
/* Return value		: State of the dataflash				*/
/*------------------------------------------------------------------------------*/
AT45S_DataFlashStatus AT45F_DataFlashPagePgmBuf(
	AT45PS_DataFlash pDataFlash,
	unsigned char *src,
	unsigned int dest,
	unsigned int SizeToWrite)
{
	int cmdsize;
	pDataFlash->pDataFlashDesc->tx_data_pt = src ;
	pDataFlash->pDataFlashDesc->tx_data_size = SizeToWrite ;
	pDataFlash->pDataFlashDesc->rx_data_pt = src;
	pDataFlash->pDataFlashDesc->rx_data_size = SizeToWrite;

	cmdsize = 4;
	/* Send the command to the dataflash */
	if (pDataFlash->pDevice->pages_number >= 16384)
		cmdsize = 5;
	return(AT45F_DataFlashSendCommand (pDataFlash, DB_PAGE_PGM_BUF1, cmdsize, dest));
}


/*------------------------------------------------------------------------------*/
/* Function Name       : AT45F_MainMemoryToBufferTransfert			*/
/* Object              : Read a page in the SRAM Buffer 1 or 2			*/
/* Input Parameters    : DataFlash Service					*/
/*                     : Page concerned						*/
/*                     : 							*/
/* Return value		: State of the dataflash				*/
/*------------------------------------------------------------------------------*/
AT45S_DataFlashStatus AT45F_MainMemoryToBufferTransfert(
	AT45PS_DataFlash pDataFlash,
	unsigned char BufferCommand,
	unsigned int page)
{
	int cmdsize;
	/* Test if the buffer command is legal */
	if ((BufferCommand != DB_PAGE_2_BUF1_TRF) && (BufferCommand != DB_PAGE_2_BUF2_TRF))
		return DATAFLASH_BAD_COMMAND;

	/* no data to transmit or receive */
	pDataFlash->pDataFlashDesc->tx_data_size = 0;
	cmdsize = 4;
	if (pDataFlash->pDevice->pages_number >= 16384)
		cmdsize = 5;
	return(AT45F_DataFlashSendCommand (pDataFlash, BufferCommand, cmdsize, page*pDataFlash->pDevice->pages_size));
}


/*----------------------------------------------------------------------------- */
/* Function Name       : AT45F_DataFlashWriteBuffer				*/
/* Object              : Write data to the internal sram buffer 1 or 2		*/
/* Input Parameters    : DataFlash Service					*/
/*			: <BufferCommand> = command to write buffer1 or buffer2	*/
/*                     : <*dataBuffer> = data buffer to write			*/
/*                     : <bufferAddress> = address in the internal buffer	*/
/*                     : <SizeToWrite> = data buffer size			*/
/* Return value		: State of the dataflash				*/
/*------------------------------------------------------------------------------*/
AT45S_DataFlashStatus AT45F_DataFlashWriteBuffer (
	AT45PS_DataFlash pDataFlash,
	unsigned char BufferCommand,
	unsigned char *dataBuffer,
	unsigned int bufferAddress,
	int SizeToWrite )
{
	int cmdsize;
	/* Test if the buffer command is legal */
	if ((BufferCommand != DB_BUF1_WRITE) && (BufferCommand != DB_BUF2_WRITE))
		return DATAFLASH_BAD_COMMAND;

	/* buffer address must be lower than page size */
	if (bufferAddress > pDataFlash->pDevice->pages_size)
		return DATAFLASH_BAD_ADDRESS;
	
	if ( (pDataFlash->pDataFlashDesc->state)  != IDLE)
		return DATAFLASH_BUSY;

	/* Send first Write Command */
	pDataFlash->pDataFlashDesc->command[0] = BufferCommand;
	pDataFlash->pDataFlashDesc->command[1] = 0;
	if (pDataFlash->pDevice->pages_number >= 16384) {
	    	pDataFlash->pDataFlashDesc->command[2] = 0;
	    	pDataFlash->pDataFlashDesc->command[3] = (unsigned char)(((unsigned int)(bufferAddress &  pDataFlash->pDevice->byte_mask)) >> 8) ;
	    	pDataFlash->pDataFlashDesc->command[4] = (unsigned char)((unsigned int)bufferAddress  & 0x00FF) ;
		cmdsize = 5;
	} else {
	    	pDataFlash->pDataFlashDesc->command[2] = (unsigned char)(((unsigned int)(bufferAddress &  pDataFlash->pDevice->byte_mask)) >> 8) ;
	    	pDataFlash->pDataFlashDesc->command[3] = (unsigned char)((unsigned int)bufferAddress  & 0x00FF) ;
	    	pDataFlash->pDataFlashDesc->command[4] = 0;
		cmdsize = 4;
	}

	pDataFlash->pDataFlashDesc->tx_cmd_pt 	 = pDataFlash->pDataFlashDesc->command ;
	pDataFlash->pDataFlashDesc->tx_cmd_size = cmdsize ;
	pDataFlash->pDataFlashDesc->rx_cmd_pt 	 = pDataFlash->pDataFlashDesc->command ;
	pDataFlash->pDataFlashDesc->rx_cmd_size = cmdsize ;

	pDataFlash->pDataFlashDesc->rx_data_pt 	= dataBuffer ;
	pDataFlash->pDataFlashDesc->tx_data_pt 	= dataBuffer ;
	pDataFlash->pDataFlashDesc->rx_data_size 	= SizeToWrite ;
	pDataFlash->pDataFlashDesc->tx_data_size 	= SizeToWrite ;

	return AT45F_SpiWrite(pDataFlash->pDevice->cs, pDataFlash->pDataFlashDesc);
}

/*------------------------------------------------------------------------------*/
/* Function Name       : AT45F_PageErase                                        */
/* Object              : Erase a page 						*/
/* Input Parameters    : DataFlash Service					*/
/*                     : Page concerned						*/
/*                     : 							*/
/* Return value		: State of the dataflash				*/
/*------------------------------------------------------------------------------*/
AT45S_DataFlashStatus AT45F_PageErase(
	AT45PS_DataFlash pDataFlash,
	unsigned int page)
{
	int cmdsize;
	/* Test if the buffer command is legal */
	/* no data to transmit or receive */
    	pDataFlash->pDataFlashDesc->tx_data_size = 0;

	cmdsize = 4;
	if (pDataFlash->pDevice->pages_number >= 16384)
		cmdsize = 5;
	return(AT45F_DataFlashSendCommand (pDataFlash, DB_PAGE_ERASE, cmdsize, page*pDataFlash->pDevice->pages_size));
}


/*------------------------------------------------------------------------------*/
/* Function Name       : AT45F_BlockErase                                       */
/* Object              : Erase a Block 						*/
/* Input Parameters    : DataFlash Service					*/
/*                     : Page concerned						*/
/*                     : 							*/
/* Return value		: State of the dataflash				*/
/*------------------------------------------------------------------------------*/
AT45S_DataFlashStatus AT45F_BlockErase(
	AT45PS_DataFlash pDataFlash,
	unsigned int block)
{
	int cmdsize;
	/* Test if the buffer command is legal */
	/* no data to transmit or receive */
    	pDataFlash->pDataFlashDesc->tx_data_size = 0;
	cmdsize = 4;
	if (pDataFlash->pDevice->pages_number >= 16384)
		cmdsize = 5;
	return(AT45F_DataFlashSendCommand (pDataFlash, DB_BLOCK_ERASE,cmdsize, block*8*pDataFlash->pDevice->pages_size));
}

/*------------------------------------------------------------------------------*/
/* Function Name       : AT45F_WriteBufferToMain				*/
/* Object              : Write buffer to the main memory			*/
/* Input Parameters    : DataFlash Service					*/
/*		: <BufferCommand> = command to send to buffer1 or buffer2	*/
/*                     : <dest> = main memory address				*/
/* Return value		: State of the dataflash				*/
/*------------------------------------------------------------------------------*/
AT45S_DataFlashStatus AT45F_WriteBufferToMain (
	AT45PS_DataFlash pDataFlash,
	unsigned char BufferCommand,
	unsigned int dest )
{
	int cmdsize;
	/* Test if the buffer command is correct */
	if ((BufferCommand != DB_BUF1_PAGE_PGM) &&
	    (BufferCommand != DB_BUF1_PAGE_ERASE_PGM) &&
	    (BufferCommand != DB_BUF2_PAGE_PGM) &&
	    (BufferCommand != DB_BUF2_PAGE_ERASE_PGM) )
		return DATAFLASH_BAD_COMMAND;

	/* no data to transmit or receive */
	pDataFlash->pDataFlashDesc->tx_data_size = 0;

	cmdsize = 4;
	if (pDataFlash->pDevice->pages_number >= 16384)
		cmdsize = 5;
	/* Send the command to the dataflash */
	return(AT45F_DataFlashSendCommand (pDataFlash, BufferCommand, cmdsize, dest));
}


/*------------------------------------------------------------------------------*/
/* Function Name       : AT45F_PartialPageWrite					*/
/* Object              : Erase partielly a page					*/
/* Input Parameters    : <page> = page number					*/
/*			: <AdrInpage> = adr to begin the fading			*/
/*                     : <length> = Number of bytes to erase			*/
/*------------------------------------------------------------------------------*/
AT45S_DataFlashStatus AT45F_PartialPageWrite (
	AT45PS_DataFlash pDataFlash,
	unsigned char *src,
	unsigned int dest,
	unsigned int size)
{
	unsigned int page;
	unsigned int AdrInPage;

	page = dest / (pDataFlash->pDevice->pages_size);
	AdrInPage = dest % (pDataFlash->pDevice->pages_size);

	/* Read the contents of the page in the Sram Buffer */
	AT45F_MainMemoryToBufferTransfert(pDataFlash, DB_PAGE_2_BUF1_TRF, page);
	AT45F_DataFlashWaitReady(pDataFlash->pDevice->cs, pDataFlash->pDataFlashDesc, AT45C_TIMEOUT_WRDY);
	/*Update the SRAM buffer */
	AT45F_DataFlashWriteBuffer(pDataFlash, DB_BUF1_WRITE, src, AdrInPage, size);

	AT45F_DataFlashWaitReady(pDataFlash->pDevice->cs, pDataFlash->pDataFlashDesc, AT45C_TIMEOUT_WRDY);

	/* Erase page if a 128 Mbits device */
	if (pDataFlash->pDevice->pages_number >= 16384) {
		AT45F_PageErase(pDataFlash, page);
		/* Rewrite the modified Sram Buffer in the main memory */
		AT45F_DataFlashWaitReady(pDataFlash->pDevice->cs, pDataFlash->pDataFlashDesc, AT45C_TIMEOUT_WRDY);
	}

	/* Rewrite the modified Sram Buffer in the main memory */
	return(AT45F_WriteBufferToMain(pDataFlash, DB_BUF1_PAGE_ERASE_PGM, (page*pDataFlash->pDevice->pages_size)));
}


/*------------------------------------------------------------------------------*/
/* Function Name       : AT45F_DataFlashWrite					*/
/* Object              :							*/
/* Input Parameters    : <*src> = Source buffer					*/
/*                     : <dest> = dataflash adress				*/
/*                     : <size> = data buffer size				*/
/*------------------------------------------------------------------------------*/
AT45S_DataFlashStatus AT45F_DataFlashWrite(
	AT45PS_DataFlash pDataFlash,
	unsigned char *src,
	int dest,
	int size )
{
	unsigned int length;
	unsigned int page;
	unsigned int status;

	if ( (dest + size) > (pDataFlash->pDevice->pages_size * (pDataFlash->pDevice->pages_number)))
		return DATAFLASH_MEMORY_OVERFLOW;

	/* If destination does not fit a page start address */
	if ((dest % ((unsigned int)(pDataFlash->pDevice->pages_size)))  != 0 ) {
		length = pDataFlash->pDevice->pages_size - (dest % ((unsigned int)(pDataFlash->pDevice->pages_size)));

		if (size < length)
			length = size;

		if(!AT45F_PartialPageWrite(pDataFlash,src, dest, length))
			return DATAFLASH_ERROR;

		AT45F_DataFlashWaitReady(pDataFlash->pDevice->cs, pDataFlash->pDataFlashDesc, AT45C_TIMEOUT_WRDY);

		/* Update size, source and destination pointers */
		size -= length;
		dest += length;
		src += length;
	}

	while (( size - pDataFlash->pDevice->pages_size ) >= 0 ) {
		/* program dataflash page */
		page = (unsigned int)dest / (pDataFlash->pDevice->pages_size);

		status = AT45F_DataFlashWriteBuffer(pDataFlash, DB_BUF1_WRITE, src, 0, pDataFlash->pDevice->pages_size);
		AT45F_DataFlashWaitReady(pDataFlash->pDevice->cs, pDataFlash->pDataFlashDesc, AT45C_TIMEOUT_WRDY);

		status = AT45F_PageErase(pDataFlash, page);
		AT45F_DataFlashWaitReady(pDataFlash->pDevice->cs, pDataFlash->pDataFlashDesc, AT45C_TIMEOUT_WRDY);
		if (!status)
			return DATAFLASH_ERROR;

		status = AT45F_WriteBufferToMain (pDataFlash, DB_BUF1_PAGE_PGM, dest);
		if(!status)
			return DATAFLASH_ERROR;

		AT45F_DataFlashWaitReady(pDataFlash->pDevice->cs, pDataFlash->pDataFlashDesc, AT45C_TIMEOUT_WRDY);

		/* Update size, source and destination pointers */
		size -= pDataFlash->pDevice->pages_size ;
		dest += pDataFlash->pDevice->pages_size ;
		src  += pDataFlash->pDevice->pages_size ;
	}
	
	/* If still some bytes to read */
	if ( size > 0 ) {
		/* program dataflash page */
		if(!AT45F_PartialPageWrite(pDataFlash, src, dest, size) )
			return DATAFLASH_ERROR;

		AT45F_DataFlashWaitReady(pDataFlash->pDevice->cs,pDataFlash->pDataFlashDesc, AT45C_TIMEOUT_WRDY);
	}
	return DATAFLASH_OK;
}


/*------------------------------------------------------------------------------*/
/* Function Name       : AT45F_DataFlashRead 					*/
/* Object              : Read a block in dataflash				*/
/* Input Parameters    : 							*/
/* Return value		: 							*/
/*------------------------------------------------------------------------------*/
int AT45F_DataFlashRead(
	AT45PS_DataFlash pDataFlash,
	unsigned long addr,
	unsigned long size,
	char *buffer)
{
	unsigned long SizeToRead;

	if(AT45F_DataFlashWaitReady(pDataFlash->pDevice->cs, pDataFlash->pDataFlashDesc, AT45C_TIMEOUT_WRDY) != DATAFLASH_OK)
		return -1;

	while (size) {
		SizeToRead = (size < 0x8000)? size:0x8000;

		if (AT45F_DataFlashWaitReady(pDataFlash->pDevice->cs, pDataFlash->pDataFlashDesc, AT45C_TIMEOUT_WRDY) != DATAFLASH_OK)
			return -1;

		if (AT45F_DataFlashContinuousRead (pDataFlash, addr, (uchar *)buffer, SizeToRead) != DATAFLASH_OK)
			return -1;

		size -= SizeToRead;
		addr += SizeToRead;
		buffer += SizeToRead;
	}

	return DATAFLASH_OK;
}


/*------------------------------------------------------------------------------*/
/* Function Name       : AT45F_DataflashProbe 					*/
/* Object              : 							*/
/* Input Parameters    : 							*/
/* Return value	       : Dataflash status register				*/
/*------------------------------------------------------------------------------*/
int AT45F_DataflashProbe(int cs, AT45PS_DataflashDesc pDesc)
{
	AT45F_DataFlashGetStatus(cs, pDesc);	
	return((pDesc->command[1] == 0xFF)? 0: pDesc->command[1] & 0x3C);
}


/*------------------------------------*/
/* Set Dataflash to "power of 2" mode */
/*------------------------------------*/
//c.f. blackfin anomaly list: atmel flash must be programmed
//to have "power of 2" page size

void AT45F_SpiEnsurePowerOf2(AT45PS_DATAFLASH_INFO pDataFlashInfo)
{
  unsigned char OpCode = 0x3D;
  unsigned int CmdSize = 4;
  unsigned int RestOpCode = 0x2A80A6;
  
  AT45PS_DataFlash pDataFlash;
  pDataFlash->pDataFlashDesc = &pDataFlashInfo->Desc;
  pDataFlash->pDevice = &pDataFlashInfo->Device;

  // When sending the 4 opcodes (0x3D,0x2A,0x80,0xA6) think of them as
  // 0x3D=opcode and 0x2A,0x80,0xA6 as the three address byte
  AT45F_DataFlashSendCommand(pDataFlash, OpCode, CmdSize, RestOpCode);
  
  AT45F_DataFlashWaitReady(pDataFlash->pDevice->cs, pDataFlash->pDataFlashDesc, AT45C_TIMEOUT_WRDY);
}


#endif
