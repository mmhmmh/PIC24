/*****************************************************************************
*
* File: spi1.c
* 
* Copyright S. Brennen Ball, 2006-2007
* 
* The author provides no guarantees, warantees, or promises, implied or
*	otherwise.  By using this software you agree to indemnify the author
* 	of any damages incurred by using it.
* 
*****************************************************************************/
#include "system.h"

unsigned char spi1_send_read_byte(unsigned char byte)
{
	BYTE outData = 0x00;
	SPI1BUF = byte;

	while(SPI1STATbits.SPITBF);
	while(SPI1STATbits.SPIRBF == 0);

	outData = SPI1BUF;						//read the buffer
	return outData;
}	
