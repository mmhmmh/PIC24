/****************************************************************************
* XLP 16-bit Dev board external I2C EEPROM communications
*****************************************************************************
* FileName:     eeprom.c
* Dependencies: system.h
* Processor:    PIC24F16KA102
* Hardware:     XLP 16-bit Development Board
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
* Author           Date         Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Brant Ivey &     9/18/09      Initial Release
* Eric Yang
* Brant Ivey       4/1/10       Modified to add Harvester support.
*                               General bug fixes.
*****************************************************************************/

/****************************************************************************
  Section: Includes
  ***************************************************************************/
#include "system.h"


/****************************************************************************
  Section: I2C Operation Functions
  ***************************************************************************/


/****************************************************************************
  Function:
    void InitI2C(void)

  Summary:
    Initialize I2C to communicate with EEPROM.

  Description:
    Initializes I2C for communication with external EEPROM.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void InitI2C(void)
{
    _I2C1MD = 0;            //clear PMD bits for I2C and T2
    _T2MD = 0;

    I2C1CON = 0x8000;       //Enable I2C
    I2C1BRG = I2CBRG_VAL;   //Set baud rate
    _MI2C1P = 1;            //Set interrupt priority to prevent ISR execution
}


void InitSPI(void)
{
	_SPI1MD = 0;

	IEC0bits.SPI1IE = 0;

	SPI1CON1 = 0;
	SPI1CON1bits.CKE = 1;
	SPI1CON1bits.CKP = 0;
	SPI1CON1bits.MSTEN = 1;
	SPI1CON1bits.PPRE0 = 0;
	SPI1CON1bits.PPRE1 = 0;
	SPI1CON1bits.SPRE0 = 1;
	SPI1CON1bits.SPRE1 = 1;
	SPI1CON1bits.SPRE2 = 1;

	SPI1CON2 = 0;
	//SPI1CON2bits.SPIBEN = 1;

	SPI1STATbits.SPIROV = 0;
	SPI1STATbits.SPIEN = 1;

}

/****************************************************************************
  Function:
    void CloseI2C(void)

  Summary:
    Disable I2C module.

  Description:
    DisableS I2C module and Timer2.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void CloseI2C(void)
{
    I2C1CON = 0;    //turn off I2C

    _I2C1MD = 1;    //disable I2C and Timer2 to save power
    _T2MD = 1;
}


/****************************************************************************
  Function:
    void I2CWait(void)

  Summary:
    Wait for I2C interrupt.

  Description:
    Waits for I2C interrupt to occur to control I2C protocol.  Enables Timer2
    to provide timeout source to indicate communication failure.

  Precondition:
    InitI2C()

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    Puts device in Idle mode while waiting to save power.
  ***************************************************************************/
int I2CWait(void)
{
    _MI2C1IF = 0;   //Enable I2C interrupt to provide Idle wakeup source
    _MI2C1IE = 1;

    T2IntOn();      //Enable Timer2 for comms failure detection
    while((!_T2IF) && (!_MI2C1IF))
    {
        Idle();
    }
    T2IntOff();

    _MI2C1IE = 0;   //disable interrupt
    _MI2C1IF = 0;

    if(_T2IF)
    {
        _T2IF = 0;
        return -1;  //T2 interrupt - comms failure
    }

    return 0;       //I2C interupt - comms ok
}


/****************************************************************************
  Function:
    void T2IntOn(void)

  Summary:
    Turn on Timer2 and Enable interrupt.

  Description:
    Turns on Timer2 and Enables interrupt to provide wakeup source.
    Interrupt priority at level 1 to prevent ISR execution.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void T2IntOn(void)
{
    _T2MD = 0;
    TMR2 = 0x0000;
    PR2 = 0xFFFF;
    _T2IF = 0;
    _T2IP = 1;
    _T2IE = 1;
    T2CON = 0x8000;
}


/****************************************************************************
  Function:
    void T2IntOff(void)

  Summary:
    Turn off Timer2 and disable interrupt.

  Description:
    Turns of Timer2 and DisableS interrupt.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void T2IntOff(void)
{
    T2CON = 0;
    _T2IE = 0;
    _T2IF = 0;
}


/****************************************************************************
  Section: EEPROM communication functions
  ***************************************************************************/

/****************************************************************************
  Function:
    BYTE writePacket(WORD addr, BYTE* packet, BYTE numBytes)

  Summary:
    Transmit a packet of data to EEPROM.

  Description:
    Transmits a packet of data to EEPROM.

  Precondition:
    InitI2C()

  Parameters:
    WORD addr       - address in EEPROM to write data to
    BYTE* packet    - pointer to array containing data packet
    BYTE numbytes   - number of bytes in packet

  Returns:
    BYTE - success or failure
  ***************************************************************************/
