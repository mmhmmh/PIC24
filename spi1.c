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
	//	CSN1_LAT = 0;                           //Select Device
	if (SPI1CON1bits.MODE16) {
        SPI1BUF = byte;
	} else {
        SPI1BUF = byte & 0xff;    /*  byte write  */
	}
	while(SPI1STATbits.SPITBF);
//	CSN1_LAT = 1;                           //Deselect Device
	outData = SPI1BUF;						//read the buffer
	return outData;
}	
