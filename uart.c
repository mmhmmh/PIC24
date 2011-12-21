/****************************************************************************
* PIC24 Generic UART driver
*****************************************************************************
* FileName:     uart.c
* Dependencies: uart.h
* Processor:    PIC24
* Hardware:     none
* Complier:     Microchip C30 v3.10 or higher
* Company:      Microchip Technology, Inc.
*
* Copyright and Disclaimer Notice
*
* Copyright ©2007-2008 Microchip Technology Inc.  All rights reserved.
*
* Microchip licenses to you the right to use, modify, copy and distribute
* Software only when embedded on a Microchip microcontroller or digital
* signal controller and used with a Microchip radio frequency transceiver,
* which are integrated into your product or third party product (pursuant
* to the terms in the accompanying license agreement).
*
* You should refer to the license agreement accompanying this Software for
* additional information regarding your rights and obligations.
*
* SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
* WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A
* PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE
* LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY,
* CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY
* DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO
* ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES,
* LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS,
* TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT
* NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*
* Author           Date        Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
KO                 12-Feb-2008  Modified to use HardwareProfile.h
KO                 11-Oct-2006  v1.0
Anton Alkhimenok   18-Oct-2005
Anton Alkhimenok   17-Feb-2009  Added UARTChar2Hex(), UARTHex2Char(),
                                      UARTClearError(), UARTDataReceived()
Brant Ivey         09-Sep-2009  Changed name to uart.c & modified to use
                                generic UART.
                                Added option to invert UART.
*****************************************************************************/


/****************************************************************************
  Section: Includes
  ***************************************************************************/
#include "uart.h"


/*******************************************************************************
Function: UARTGetChar()

Precondition:
    UARTInit must be called prior to calling this routine.

Overview:
    This routine waits for a byte to be received.  It then returns that byte.

Input: None.

Output: Byte received.

*******************************************************************************/
char UARTGetChar()
{
    char Temp;

    while(_UxRXIF == 0);

    Temp = UxRXREG;
    _UxRXIF = 0;
    return Temp;
}

/*******************************************************************************
Function: UARTInit()

Precondition: None.

Overview:
    This routine Sets up the UART module.

Input: None.

Output: None.

Notes:
    Allow the peripheral to Set the I/O pin directions.  If we Set the TRIS
    bits manually, then when we disable the UART, the shape of the stop bit
    changes, and some terminal programs have problems.
*******************************************************************************/
void UARTInit()
{
    UxMODEbits.UARTEN = 1;
    UxBRG = BAUDRATEREG;
    UxMODEbits.BRGH = BRGH_VAL;

    #ifdef INVERT_UART
    UxMODEbits.RXINV = 1;  //Invert RX and TX Idle state
    UxSTAbits.UTXINV = 1;
    #endif

    UxSTAbits.UTXEN = 1;


    _UxRXIF = 0;
}

/*******************************************************************************
Function: UARTIsPressed()

Precondition:
    UARTInit must be called prior to calling this routine.

Overview:
    This routine checks to see if there is a new byte in UART reception buffer.

Input: None.

Output:
    0 : No new data received.
    1 : Data is in the receive buffer

*******************************************************************************/
char UARTIsPressed()
{
    if(_UxRXIF == 1)
        return 1;
    return 0;
}

/*******************************************************************************
Function: UARTPrintString( char *str )

Precondition:
    UARTInit must be called prior to calling this routine.

Overview:
    This function prints a string of characters to the UART.

Input: Pointer to a null terminated character string.

Output: None.

*******************************************************************************/
void UARTPrintString( char *str )
{
    unsigned char c;

    while((c = *str++))
    {
        UARTPutChar(c);
    }
}

/*******************************************************************************
Function: UARTPutChar( char ch )

Precondition:
    UARTInit must be called prior to calling this routine.

Overview:
    This routine writes a character to the Transmit FIFO, and then waits for the
    Transmit FIFO to be empty.

Input: Byte to be sent.

Output: None.

*******************************************************************************/
void UARTPutChar( char ch )
{
    UxTXREG = ch;
    #if !defined(__PIC32MX__)
        Nop();
    #endif
    while(UxSTAbits.TRMT == 0);
}

/*******************************************************************************
Function: UARTPutDec(unsigned char dec)

Precondition:
    UARTInit must be called prior to calling this routine.

Overview:
    This function converts decimal data into a string and outputs it to UART.

Input: Binary data.

Output: None.

*******************************************************************************/
void  UARTPutDec(unsigned char dec)
{
    unsigned char res;

    res = dec;

    if (res/100)
    {
        UARTPutChar( res/100 + '0' );
    }
    res = res - (res/100)*100;

    if (res/10)
    {
        UARTPutChar( res/10 + '0' );
    }
    res = res - (res/10)*10;

    UARTPutChar( res + '0' );
}

/*******************************************************************************
Function: UARTPutHex

Precondition:
    UARTInit must be called prior to calling this routine.

Overview:
    This function converts hex data into a string and outputs it to UART.

Input: Binary data.

Output: None.

*******************************************************************************/

const unsigned char CharacterArray[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void UARTPutHex( int toPrint )
{
    int printVar;

    printVar = toPrint;
    toPrint = (toPrint>>4) & 0x0F;
    UARTPutChar( CharacterArray[toPrint] );

    toPrint = printVar & 0x0F;
    UARTPutChar( CharacterArray[toPrint] );

    return;
}

/*******************************************************************************
Function: UARTPutHexWord(unsigned int toPrint)

Precondition:
    UARTInit must be called prior to calling this routine.

Overview:
    This function converts hex data into a string and outputs it to UART.

Input: Binary data.

Output: None.

*******************************************************************************/
#if defined( __C30__ ) || defined( __PIC32MX__ )
void UARTPutHexWord( unsigned int toPrint )
{
    unsigned int printVar;

    printVar = (toPrint>>12) & 0x0F;
    UARTPutChar( CharacterArray[printVar] );

    printVar = (toPrint>>8) & 0x0F;
    UARTPutChar( CharacterArray[printVar] );

    printVar = (toPrint>>4) & 0x0F;
    UARTPutChar( CharacterArray[printVar] );

    printVar = toPrint & 0x0F;
    UARTPutChar( CharacterArray[printVar] );

    return;
}

void UARTPutHexDWord( unsigned long toPrint )
{
    unsigned long printVar;

    printVar = (toPrint>>28) & 0x0F;
    UARTPutChar( CharacterArray[printVar] );

    printVar = (toPrint>>24) & 0x0F;
    UARTPutChar( CharacterArray[printVar] );

    printVar = (toPrint>>20) & 0x0F;
    UARTPutChar( CharacterArray[printVar] );

    printVar = (toPrint>>16) & 0x0F;
    UARTPutChar( CharacterArray[printVar] );

    printVar = (toPrint>>12) & 0x0F;
    UARTPutChar( CharacterArray[printVar] );

    printVar = (toPrint>>8) & 0x0F;
    UARTPutChar( CharacterArray[printVar] );

    printVar = (toPrint>>4) & 0x0F;
    UARTPutChar( CharacterArray[printVar] );

    printVar = toPrint & 0x0F;
    UARTPutChar( CharacterArray[printVar] );

    return;
}

#endif

/*********************************************************************
Function: char UARTChar2Hex(char ch)

PreCondition: none

Input: ASCII to be converted

Output: number

Side Effects: none

Overview: converts ASCII coded digit into number

Note: none

********************************************************************/
char UARTChar2Hex(char ch){
  //Wrong char
  if(ch > 102)
    return 0;

  //From a to f
  if(ch > 96)
    return (ch-87);

  //Wrong char
  if(ch > 70)
    return 0;

  //From A to F
  if(ch > 64)
    return (ch-55);

  //Wrong char
  if(ch > 57)
    return 0;

  //From 0 - 9
  if(ch > 47)
    return(ch-48);
  else
  //Wrong char
    return 0;
}

/*********************************************************************
Function: char UARTHex2Char(char hex)

PreCondition: none

Input: number

Output: ASCII code

Side Effects: none

Overview: converts low nibble into ASCII coded digit

Note: none

********************************************************************/
char UARTHex2Char(char hex){
char h;
  h = hex&0x0f;
  //From 0xa to 0xf
  if(h>9)
    return (h+55);
  else
    return (h+48);
}

/*********************************************************************
Function: void UARTClrError(void)

PreCondition: none

Input: none

Output: character received

Side Effects: none

Overview: wait for character

Note: none

********************************************************************/
void UARTClrError(void){
    //Clear error flag
    if(UxSTAbits.OERR)
        UxSTAbits.OERR = 0;
}



