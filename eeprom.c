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
BYTE writePacket(WORD addr, BYTE* packet, BYTE numBytes)
{
    int i;
    char test;

    //Send a start bit
    I2C1CONbits.SEN = 1;
    test = I2CWait();     //wait for interrupt

    //Transmit the I2C address byte
    I2C1TRN = EEPROM_ADDRESS | I2C_WRITE;
    test = I2CWait();     //wait for interrupt

    //Transmit data address
    I2C1TRN = addr>>8;
    test = I2CWait();     //wait for interrupt
    I2C1TRN = addr;
    test = I2CWait();     //wait for interrupt

    //Transmit data packet
    for(i = 0; i<numBytes;i++)
    {
        if(i>0 && I2C1STATbits.ACKSTAT)
        {
            //NACK,reTransmit
            i--;
        }

        I2C1TRN = packet[i];
        test = I2CWait();     //wait for interrupt
    }

    //Send the stop bit, ending this session
    I2C1CONbits.PEN = 1;
    test = I2CWait();     //wait for interrupt

    return 0;
}


/****************************************************************************
  Function:
    BYTE readPacket(WORD addr, BYTE* packet, BYTE numBytes)

  Summary:
    Read a packet of data from EEPROM.

  Description:
    Reads a packet of data from EEPROM.

  Precondition:
    InitI2C()

  Parameters:
    WORD addr       - address in EEPROM to read data from
    BYTE* packet    - pointer to array to write data to
    BYTE numbytes   - number of bytes in packet

  Returns:
    BYTE - success or failure
  ***************************************************************************/
BYTE readPacket(WORD addr,BYTE *packet, BYTE numBytes)
{
    int i;

    //Send a start bit
    I2C1CONbits.SEN = 1;
    I2CWait();     //wait for interrupt

    //Transmit the I2C address byte
    I2C1TRN = EEPROM_ADDRESS | I2C_WRITE;
    I2CWait();     //wait for interrupt

    //Transmit data address
    I2C1TRN = addr>>8;
    I2CWait();     //wait for interrupt
    I2C1TRN = addr;
    I2CWait();     //wait for interrupt


    //Send a restart bit
    I2C1CONbits.RSEN = 1;
    I2CWait();     //wait for interrupt

    //Transmit the I2C address byte
    I2C1TRN = EEPROM_ADDRESS | I2C_READ;
    I2CWait();     //wait for interrupt

    //Read data packet
    for(i = 0; i<numBytes;i++)
    {

        //Receive a data byte
        I2C1CONbits.RCEN = 1;
        I2CWait();     //wait for interrupt

        //Read data
        packet[i] = I2C1RCV;

        //Transmit NACK on last byte, ACK on others
        if(i==numBytes-1)
        {
            I2C1CONbits.ACKDT = 1;
        }
        else
        {
            I2C1CONbits.ACKDT = 0;
        }

        I2C1CONbits.ACKEN = 1;
        I2CWait();     //wait for interrupt
    }

    //Send the stop bit, ending this session
    I2C1CONbits.PEN = 1;
    I2CWait();     //wait for interrupt
    return 0;
}


/****************************************************************************
  Function:
    BYTE writeWait()

  Summary:
    Wait for EEPROM to finish write cycle.

  Description:
    Transmits dummy packet to EEPROM to determine if write is still in progress.
    ACK means the write is finished, NACK means EEPROM is still writing.

  Precondition:
    InitI2C()

  Parameters:
    None

  Returns:
    BYTE - ACK (0) or NACK (1)
  ***************************************************************************/
BYTE writeWait()
{
    BYTE retVal;

    //Send a start bit
    I2C1CONbits.SEN = 1;
    I2CWait();          //wait for interrupt

    //Transmit the I2C address byte
    I2C1TRN = EEPROM_ADDRESS | I2C_WRITE;
    I2CWait();          //wait for interrupt

    //0 = ACK from EEPROM, ready to write new data
    //1 = NACK from EEPROM, write still ongoing
    retVal = I2C1STATbits.ACKSTAT;  

    //Send the stop bit, ending this session
    I2C1CONbits.PEN = 1;
    I2CWait();          //wait for interrupt

    return retVal;
}


/****************************************************************************
  Function:
    void StoreAddress(void)

  Summary:
    Save current EEPROM address pointer to EEPROM in case of reset.

  Description:
    Saves the current pointer to the last used EEPROM address so that data
    is not lost in case of reset event.

  Precondition:
    InitI2C()

  Parameters:
    Global variable - eeAddress.

  Returns:
    None.
  ***************************************************************************/
void StoreAddress (void)
{
    BYTE datVal[2];

    datVal[0] = eeAddress;
    datVal[1] = eeAddress >>8;

    writePacket(0x0, datVal, 2);

    //wait in Idle for EEPROM write to complete
    while(writeWait())
    {
        IdleUs(EEPROM_ACK_WAIT);
    }
}



/****************************************************************************
  Function:
    void GetAddress(void)

  Summary:
    Read last EEPROM address pointer from EEPROM.

  Description:
    Reads the last saved EEPROM address value from EEPROM.

  Precondition:
    InitI2C()

  Parameters:
    None.

  Returns:
    Global variable - eeAddress.
  ***************************************************************************/
void GetAddress (void)
{
    BYTE dataVal[2];
    BYTE *pByte;

    //Init pointer and read EERPOM
    pByte = dataVal;
    readPacket(0,pByte,0x02);

    //Load address from EEPROM
    eeAddress = 0xFFF0 & (dataVal[0] + (((WORD)dataVal[1])<<8));

    //Address is corrupt, reset to 0x10
    if(eeAddress<0x10 || eeAddress > 0x7FFF)
    {
        eeAddress = 0x10;
    }

}


/****************************************************************************
  Function:
    void GetTimestamp(WORD addr)

  Summary:
    Read last EEPROM timestamp from EEPROM and writes to RTCC

  Description:
    Reads the last saved EEPROM timestamp from EEPROM and writes to RTCC

  Precondition:
    InitI2C()

  Parameters:
    WORD addr - EEPROM address to read from.

  Returns:
    Global variable - rtcc is modified.
  ***************************************************************************/
void GetTimestamp(WORD addr)
{
    BYTE dataVal[7];
    BYTE *pData;

    //Read timestamp
    pData = dataVal;
    readPacket(addr,pData,0x7);

    //Read timestamp data into rtcc structure
    rtcc.byte.year = *pData++;
    rtcc.byte.mon = *pData++;
    rtcc.byte.mday = *pData++;
    rtcc.byte.wday = *pData++;
    rtcc.byte.hour = *pData++;
    rtcc.byte.min = *pData++;
    rtcc.byte.sec = *pData++;

    UpdateRTCC();   //update RTCC module with new time

}


/****************************************************************************
  Function:
    void StoreData(WORD addr, WORD data, BYTE type)

  Summary:
    Store input data to EERPOM.

  Description:
    Stores the input data to EEPROM and timestamps.

  Precondition:
    None.

  Parameters:
    WORD addr - address in EEPROM to write to
    WORD data - data to Store to EEPROM
    BYTE type - SENSOR_TYPES indicator of data type

  Returns:
    None.
  ***************************************************************************/
void StoreData(WORD addr, WORD data, BYTE type)
{
    BYTE packet[16];
    BYTE *pData;

    pData = packet; //Init packet buffer

    //Read RTCC and load packet buffer with timestamp
    ReadRTCC();
    *pData++ = rtcc.byte.year;
    *pData++ = rtcc.byte.mon;
    *pData++ = rtcc.byte.mday;
    *pData++ = rtcc.byte.wday;
    *pData++ = rtcc.byte.hour;
    *pData++ = rtcc.byte.min;
    *pData++ = rtcc.byte.sec;


    //write datatype and data to buffer
    *pData++ = type;
    *pData++ = data>>8;
    *pData++ = data;

    writePacket(addr,packet,10);    //send data to EEPROM
    
    //wait in Idle for EEPROM write to complete
    while(writeWait())
    {
        IdleUs(EEPROM_ACK_WAIT);
    }

    eeAddress+=0x10;    //Increment address after write & save

    if(eeAddress > 0x7FFF)
    {
        eeAddress = 0x10;   //EERPOM max is 0x7FFF, addresses 0x00-0x0F is state retention data
    }

    StoreAddress();

}


/****************************************************************************
  Function:
    void StoreSensorData(void)

  Summary:
    Store active sensor's current data to EEPROM

  Description:
    Stores the most recent sensor data to the EEPROM.

  Precondition:
    SampleSensors()

  Parameters:
    Global sensor value variables - tempVal, potVal, vddVal, pressedCT1-3

  Returns:
    None.
  ***************************************************************************/
void StoreSensorData(void)
{
    
}//end StoreSensorData()


/****************************************************************************
  Function:
    void TransmitEEPROM(WORD addrMin, WORD addrMax)

  Summary:
    Transmit data from EEPROM over UART

  Description:
    Transmits data in input range from EEPROM out over UART

  Precondition:
    None.

  Parameters:
    WORD addrMin - starting address in EEPROM to read
    WORD addrMax - ending address

  Returns:
    None.
  ***************************************************************************/
void TransmitEEPROM(WORD addrMin,WORD addrMax)
{


}//end TransmitEEPROM(..)


