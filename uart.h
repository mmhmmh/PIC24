/****************************************************************************
* PIC24 Generic UART driver header
*****************************************************************************
* FileName:     uart.h
* Dependencies: system.h
* Processor:    PIC24
* Hardware:     none
* Complier:     Microchip C30 v3.10 or higher
* Company:      Microchip Technology, Inc.
*
* Copyright and Disclaimer Notice
*
* Copyright ?007-2008 Microchip Technology Inc.  All rights reserved.
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

#ifndef _UART_H
#define _UART_H


/****************************************************************************
  Section: Includes
  ***************************************************************************/
#include "system.h"


/****************************************************************************
  Section: UART baud rate calculation
  ***************************************************************************/
#ifdef BRGH_VAL
    #define BRG_DIV 4
#else
    #define BRG_DIV 16
#endif

#define BAUDRATEREG    ((FCY + (BRG_DIV/2*BAUDRATE))/BRG_DIV/BAUDRATE-1)
#define BAUD_ACTUAL    (FCY/BRG_DIV/(BAUDRATEREG+1))

#define BAUD_ERROR          ((BAUD_ACTUAL > BAUDRATE) ? BAUD_ACTUAL-BAUDRATE : BAUDRATE-BAUD_ACTUAL)
#define BAUD_ERROR_PRECENT  ((BAUD_ERROR*100+BAUDRATE/2)/BAUDRATE)

#if (BAUD_ERROR_PRECENT > 3)
    #error "UART frequency error is worse than 3%"
#elif (BAUD_ERROR_PRECENT > 2)
    #warning "UART frequency error is worse than 2%"
#endif


/****************************************************************************
  Section: UART generic register defInitions
  ***************************************************************************/
//This section defines a generic Set of UART registers (e.g. UxMODE) which
//allow any UART module on the device to be used.
//The define UARTNUM selects which UART module is used.

#define UARTREG2(a,b)     U##a##b
#define UARTREG(a,b)    UARTREG2(a,b)
#define UARTINT2(a,b)     _U##a##b
#define UARTINT(a,b)    UARTINT2(a,b)

#define UxMODE      UARTREG(UARTNUM,MODE)
#define UxBRG       UARTREG(UARTNUM,BRG)
#define UxSTA       UARTREG(UARTNUM,STA)
#define UxRXREG     UARTREG(UARTNUM,RXREG)
#define UxTXREG     UARTREG(UARTNUM,TXREG)
#define UxMODEbits  UARTREG(UARTNUM,MODEbits)
#define UxSTAbits   UARTREG(UARTNUM,STAbits)
#define _UxMD       UARTINT(UARTNUM,MD)
#define _UxRXIF     UARTINT(UARTNUM,RXIF)
#define _UxTXIF     UARTINT(UARTNUM,TXIF)

#define mFlushUART() while(!UxSTAbits.TRMT)




/****************************************************************************
  Section: UART function prototypes
  ***************************************************************************/

/*********************************************************************
Function: char UARTGetChar()

PreCondition: none

Input: none

Output: last character received

Side Effects: none

Overview: returns last character received

Note: none

********************************************************************/
char UARTGetChar();

/*********************************************************************
Function: void UARTPutChar(char ch)

PreCondition: none

Input: none

Output: none

Side Effects: none

Overview: puts character

Note: none
********************************************************************/
void UARTPutChar( char ch );

/*********************************************************************
Function: void UARTInit(void)

PreCondition: none

Input: none

Output: none

Side Effects: none

Overview: Initializes UART

Note: none
********************************************************************/
void UARTInit();

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
char UARTIsPressed();

/*******************************************************************************
Function: UARTPrintString( char *str )

Precondition:
    UARTInit must be called prior to calling this routine.

Overview:
    This function prints a string of characters to the UART.

Input: Pointer to a null terminated character string.

Output: None.

*******************************************************************************/
void UARTPrintString( char *str );

/*******************************************************************************
Function: UARTPutDec(unsigned char dec)

Precondition:
    UARTInit must be called prior to calling this routine.

Input: Binary data

Output: none

Side Effects: none

Overview: This function converts decimal data into a string
          and outputs it to UART.

Note: none
*******************************************************************************/
void UARTPutDec( unsigned char dec );

/*******************************************************************************
Function: UARTPutHex

Precondition:
    UARTInit must be called prior to calling this routine.

Input: Binary data

Output: none

Side Effects: none

Overview: This function converts hex data into a string
          and outputs it to UART.

Note: none
*******************************************************************************/
void UARTPutHex( int toPrint );

/*******************************************************************************
Function: UARTPutHexWord(unsigned int toPrint)

Precondition:
    UARTInit must be called prior to calling this routine.

Input: Binary data

Output: none

Side Effects: none

Overview: This function converts hex data into a string
          and outputs it to UART.

Note: none
*******************************************************************************/
#if defined( __C30__ ) || defined( __PIC32MX__ )
void UARTPutHexWord( unsigned int toPrint );
void UARTPutHexDWord( unsigned long int toPrint );
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
char UARTChar2Hex(char ch);

/*********************************************************************
Function: char UARTHex2Char(char hex)

PreCondition: none

Input: number

Output: ASCII code

Side Effects: none

Overview: converts low nibble into ASCII coded digit

Note: none

********************************************************************/
char UARTHex2Char(char hex);

/*********************************************************************
Function: void UARTClrError(void)

PreCondition: none

Input: none

Output: character received

Side Effects: none

Overview: wait for character

Note: none

********************************************************************/
void UARTClrError(void);

/*********************************************************************
Macros: UARTDataReceived()

PreCondition: none

Input: none

Output: zero if character is not received

Side Effects: none

Overview: checks if data is available

Note: none

********************************************************************/
#define UARTDataReceived() (UxSTAbits.URXDA)

#endif


