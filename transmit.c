/****************************************************************************
* XLP 16-bit Development Board Data Transmission
*****************************************************************************
* FileName:     transmit.c
* Dependencies: system.h
* Processor:    PIC24F16KA102
* Hardware:     XLP 16-bit Development Board
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
* Brant Ivey       4/1/10      Initial Release
*****************************************************************************/

/****************************************************************************
  Section: Includes
  ***************************************************************************/
#include "system.h"



/****************************************************************************
  Section: Data Transmission functions
  ***************************************************************************/


/****************************************************************************
  Function:
    void TransmitData(void)

  Summary:
    Transmits current state of Demo and most recent data Sample.

  Description:
    Transmits MCU low power states, active sensor and sensor data out UART.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void TransmitData(void)
{
    //turn on external circuits if disabled
    #ifdef USE_PWRCTL
    if(PWRCTL_PORT)
    {
        mPWR_ON();
        IdleMs(EEPROM_WAIT);
    }
    #endif

    InitI2C();      //Initialize I2C to read from EEPROM
    _UxMD = 0;      //Enable UART
    UARTInit();

    switch(lpMode)  //send low power mode information
    {
        case LP_SLEEP:
            UARTPrintString("Current LPMODE: Sleep\r\n");
            break;

        case LP_DPSLP:
            UARTPrintString("Current LPMODE: DeepSleep\r\n");
            break;

        case LP_IDLE:
            UARTPrintString("Current LPMODE: Idle\r\n");
            break;

        default:
            break;
    }

    switch(tasks.bits.mode)      //send current sensor statues
    {
        case MODE_TEMP:
            UARTPrintString("Current Sensor: Temperature\r\n");
            TransmitDataType(eeAddress - 0x10,TEMP_DATA);
            break;

        case MODE_POT:
            UARTPrintString("Current Sensor: POT Voltage\r\n");
            TransmitDataType(eeAddress - 0x20,VDD_DATA);
            TransmitDataType(eeAddress - 0x10,POT_DATA);
            break;

        #ifdef USE_CAPTOUCH
        case MODE_CAP:
            UARTPrintString("Current Sensor: CAP touch button\r\n");
            #ifndef CT2_IGNORE
            TransmitDataType(eeAddress - 0x30,CT1_DATA);
            TransmitDataType(eeAddress - 0x20,CT2_DATA);
            TransmitDataType(eeAddress - 0x10,CT3_DATA); //Display CT1, CT2, and CT3 states
            #else
            TransmitDataType(eeAddress - 0x20,CT1_DATA);
            TransmitDataType(eeAddress - 0x10,CT3_DATA); //Display CT1 and CT3 states
            #endif
            break;
        #endif

        case MODE_ALL:
            #ifdef USE_CAPTOUCH
            UARTPrintString("Current Sensor: Temperature, VDD, POT Voltage and CAP touch buttons\r\n");
    
                #ifndef CT2_IGNORE
                TransmitDataType(eeAddress - 0x60,TEMP_DATA);
                TransmitDataType(eeAddress - 0x50,VDD_DATA);
                TransmitDataType(eeAddress - 0x40,POT_DATA);
                TransmitDataType(eeAddress - 0x30,CT1_DATA);
                TransmitDataType(eeAddress - 0x20,CT2_DATA);
                TransmitDataType(eeAddress - 0x10,CT3_DATA); //Display CT1, CT2, and CT3 states
                #else
                TransmitDataType(eeAddress - 0x50,TEMP_DATA);
                TransmitDataType(eeAddress - 0x40,VDD_DATA);
                TransmitDataType(eeAddress - 0x30,POT_DATA);
                TransmitDataType(eeAddress - 0x20,CT1_DATA);
                TransmitDataType(eeAddress - 0x10,CT3_DATA); //Display CT1 and CT3 states
                #endif

            #else
            UARTPrintString("Current Sensor: Temperature, VDD, and POT Voltage\r\n");
            TransmitDataType(eeAddress - 0x30,TEMP_DATA);
            TransmitDataType(eeAddress - 0x20,VDD_DATA);
            TransmitDataType(eeAddress - 0x10,POT_DATA);
            #endif
            break;

        default:
            break;
    }

    UARTPrintString("\r\n");

    CloseI2C();         //disable peripherals and external circuits to save power.
    #ifdef USE_PWRCTL
    mPWR_OFF();
    #endif
    _UxMD = 1;

}//end TransmitData()


/****************************************************************************
  Function:
    void TransmitSampleModeChange(void)

  Summary:
    Transmit sensor change information.

  Description:
    Transmits new sensor information after a active sensor change.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void TransmitSampleModeChange(void)
{
    #ifdef USE_PWRCTL
    //turn on external circuits
    mPWR_ON();
    IdleMs(UART_WAIT);
    #endif

    //Send current sensor info
    switch(tasks.bits.mode)
    {
        case MODE_TEMP:
            UARTPrintString("Changing sensor to Temperature   \r\n");
            break;

        case MODE_POT:
            UARTPrintString("Changing sensor to POT Voltage   \r\n");
            break;

        #ifdef USE_CAPTOUCH
        case MODE_CAP:
            UARTPrintString("Changing sensor to CAP touch button   \r\n");
            break;
        #endif

        case MODE_ALL:
            #ifdef USE_CAPTOUCH
            UARTPrintString("Changing sensor to Temperature, VDD, POT Voltage and CAP touch button   \r\n");
            #else
            UARTPrintString("Changing sensor to Temperature, VDD, and POT Voltage\r\n");
            #endif
            break;

        default:
            break;
    }

    #ifdef USE_PWRCTL
    mPWR_OFF();  //disable external circuits to save power
    #endif
}

/****************************************************************************
  Function:
    void TransmitLPModeChange(void)

  Summary:
    Transmit low power mode change information.

  Description:
    Transmits new low power modeinformation after a mode change.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void TransmitLPModeChange(void)
{
    #ifdef USE_PWRCTL
    //turn on external circuits
    mPWR_ON();
    IdleMs(UART_WAIT);
    #endif

    switch(lpMode)
    {
        case LP_SLEEP:
            UARTPrintString("Changing low power mode to Sleep.\r\n");
            break;

        case LP_DPSLP:
            UARTPrintString("Changing low power mode to Deep Sleep.\r\n");
            break;

        case LP_IDLE:
            UARTPrintString("Changing low power mode to Idle.\r\n");
            break;

        default:
            break;
    }

    #ifdef USE_PWRCTL
    mPWR_OFF();  //disable external circuits to save power
    #endif
}//end TransmitLPModeChange



/****************************************************************************
  Function:
    void TransmitDataType(WORD addr, SENSOR_MODES dataType)

  Summary:
    Transmits most recently sampled data of a partiuclar type.

  Description:
    Transmits the most recently aquired data of a particular type.  Valid data
    types are members of the SENSOR_TYPES enum.  The data's EEPROM address
    and current time are also output with the data.

  Precondition:
    None.

  Parameters:
    addr     - address of the data in EEPROM
    dataType - selects which data variable is output

  Returns:
    None.
  ***************************************************************************/
void TransmitDataType(WORD addr, SENSOR_MODES dataType)
{

    WORD temp;

    //Read data packet from EEPROM
    UARTPrintString("Current Data...");

    //Display address
    UARTPrintString("Addr: 0x");
    UARTPutHexWord(addr);

    //Display Timestamp
    UARTPutChar(' ');
    UARTPutHex(rtcc.byte.mon);        //month
    UARTPutChar('/');
    UARTPutHex(rtcc.byte.mday);        //day
    UARTPrintString("/20");
    UARTPutHex(rtcc.byte.year);        //year
    UARTPutChar(' ');
    UARTPutHex(rtcc.byte.wday);        //week day
    UARTPutChar(' ');
    UARTPutHex(rtcc.byte.hour);        //hour
    UARTPutChar(':');
    UARTPutHex(rtcc.byte.min);        //minute
    UARTPutChar(':');
    UARTPutHex(rtcc.byte.sec);        //second
    UARTPutChar(' ');

    //Display Datatype
    switch(dataType)
    {
        case POT_DATA:
            UARTPrintString("POT Voltage: ");

            temp = potVal/1000;
            UARTPutChar(temp%10 + '0');

            UARTPutChar('.');

            temp = potVal/100;
            UARTPutChar(temp%10 + '0');

            temp = potVal/10;
            UARTPutChar(temp%10 + '0');

            temp = potVal;
            UARTPutChar(temp%10 + '0');
            UARTPutChar('V');
            break;
        case VDD_DATA:
            UARTPrintString("VDD Voltage: ");

            temp = vddVal/1000;
            UARTPutChar(temp%10 + '0');

            UARTPutChar('.');

            temp = vddVal/100;
            UARTPutChar(temp%10 + '0');

            temp = vddVal/10;
            UARTPutChar(temp%10 + '0');

            temp = vddVal;
            UARTPutChar(temp%10 + '0');
            UARTPutChar('V');
            break;
        case TEMP_DATA:
            UARTPrintString("Temp: ");

            temp = tempVal/100;
            UARTPutChar(temp%10 + '0');

            temp = tempVal/10;
            UARTPutChar(temp%10 + '0');

            UARTPutChar('.');

            temp = tempVal;
            UARTPutChar(temp%10 + '0');

            UARTPutChar('C');

            break;
        case CT1_DATA:
            UARTPrintString("Touch: ");

            if(pressedCT1)
                UARTPrintString("Cap touch button 1 pressed ");
            else
                UARTPrintString("Cap touch button 1 unpressed ");
            break;

        case CT3_DATA:
            UARTPrintString("Touch: ");

            if(pressedCT3)
                UARTPrintString("Cap touch button 3 pressed ");
            else
                UARTPrintString("Cap touch button 3 unpressed ");
            break;

        case CT2_DATA:
            UARTPrintString("Touch: ");

            if(pressedCT2)
                UARTPrintString("Cap touch button 2 pressed ");
            else
                UARTPrintString("Cap touch button 2 unpressed ");
            break;

        default:
            UARTPrintString("Error: Unknown Data.");
            break;
    }

    //Display data
    UARTPutChar('\r');
    UARTPutChar('\n');

}//end TransmitEEPROM(..)
