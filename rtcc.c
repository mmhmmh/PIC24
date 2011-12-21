/****************************************************************************
* XLP 16-bit Dev board RTCC driver
*****************************************************************************
* FileName:     rtcc.c
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


/******************************************************************************
  Section: RTCC Functions
 ******************************************************************************/


/****************************************************************************
  Function:
    void InitRTCC(void)

  Summary:
    Initialize RTCC for timekeeping.

  Description:
    Initializes RTCC for timekeeping and Enables RTCC alarm for wakeup.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void InitRTCC(void)
{
    _RTCCMD = 0;
    EnableSOSC();                   //Enable second oscillator
    SetRTCWREN ();                  //Enable RTCC value registers write

    //ALARM CONFIGURATION
    ALCFGRPT = 0;                   //Disable alarm before modifying
    
    switch(ALARM_PERIOD)
    {
        case ONE_SECOND:
            ALCFGRPTbits.AMASK = 1; //Alarm Every 1 second
            break;
        case TEN_SECOND:
            ALCFGRPTbits.AMASK = 2; //Alarm Every 10 seconds
            break;
        default:
            break;
    }

    ALCFGRPTbits.ARPT = 0;          //Alarm will not repeat
    ALCFGRPTbits.CHIME = 0;         //Alarm CHIME Enable
    ALCFGRPTbits.ALRMEN = 0 ;       //Alarm Enabled

    //RTCC CONFIGURATION
    RCFGCALbits.RTCOE = 0;          //RTCC output disabled
    RCFGCALbits.CAL=0;              //RTC Drift Calibration
    RCFGCALbits.RTCEN = 0;          //RTCC Enabled
    RCFGCALbits.RTCWREN = 0;        //Disabled RTCC value write

    PADCFG1bits.RTSECSEL = 0b11;    //RTCC Seconds Clock is not selected for the RTCC pin

    //Interrupt configuration
    IFS3bits.RTCIF = 0;             //Clear RTCC Alarm interrupt flag
    IEC3bits.RTCIE = 0;             //RTCC Alarm interrupt Enabled
    IPC15bits.RTCIP = 4;            //Set the interrupt priority 1
}


/****************************************************************************
  Function:
    void SetRTCCAlarm(BYTE mask)

  Summary:
    Set RTCC alarm mask

  Description:
    Sets the RTCC alarm mask to input value

  Precondition:
    None.

  Parameters:
    BYTE mask - mask to apply to RTCC alarm

  Returns:
    None.
  ***************************************************************************/
void SetRTCCAlarm(BYTE mask)
{
    ALCFGRPTbits.ALRMEN = 0;    //Disable alarm before modifying
    ALCFGRPTbits.AMASK = mask;  //Alarm Every 10 seconds
    ALCFGRPTbits.ALRMEN = 1 ;   //Alarm Enabled
}


/****************************************************************************
  Function:
    void UpdateRTCC(void)

  Summary:
    Update RTCC with time loaded into rtcc global variable

  Description:
    Update RTCC with time loaded into rtcc global variable

  Precondition:
    None.

  Parameters:
    Global variable - rtcc

  Returns:
    None.
  ***************************************************************************/
void UpdateRTCC(void)
{

    SetRTCWREN();           //Enable RTCC register write

    RCFGCALbits.RTCPTR = 0x3;
    RTCVAL=rtcc.w[0];     //update year
    RTCVAL=rtcc.w[1];       //update month,day
    RTCVAL=rtcc.w[2];       //update hour,day of week
    RTCVAL=rtcc.w[3];       //update the min,sec

    RCFGCALbits.RTCWREN = 0;    //Disabled RTCC value write
}


/****************************************************************************
  Function:
    void ReadRTCC(void)

  Summary:
    Load global variable rtcc with current RTCC module time

  Description:
    Load global variable rtcc with current RTCC module time

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Modifies global variable rtcc with current RTCC time.
  ***************************************************************************/
void ReadRTCC(void)
{

    RCFGCALbits.RTCPTR = 0b11;    //Setup pointer to RTCVAL
    rtcc.w[0]=RTCVAL;     //read year
    rtcc.w[1]=RTCVAL;     //read month,day
    rtcc.w[2]=RTCVAL;     //read hour,day of week
    rtcc.w[3]=RTCVAL;     //read min,sec
}


/****************************************************************************
  Function:
    void InitCalendar(void)

  Summary:
    Initialize the RTCC date and time.

  Description:
    Initializes RTCC date and time with values determined at compile time.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Modifies global variable rtcc with Initially saved date and time.

  Remarks:
    __TIME__ and __DATE__ are defined by the compiler at compile time.  Refer
    to compiler documentation for format.
  ***************************************************************************/
void InitCalendar(void)
{
    int dayOfWeek;

    //__TIME__ and __DATE__ based on compile time
    rtcc.byte.sec =  (((__TIME__[6])&0x0F)<<4) | ((__TIME__[7])&0x0F);
    rtcc.byte.min =  (((__TIME__[3])&0x0F)<<4) | ((__TIME__[4])&0x0F);
    rtcc.byte.hour = (((__TIME__[0])&0x0F)<<4) | ((__TIME__[1])&0x0F);
    rtcc.byte.mday = (((__DATE__[4])&0x0F)<<4) | ((__DATE__[5])&0x0F);
    rtcc.byte.year = (((__DATE__[9])&0x0F)<<4) | ((__DATE__[10])&0x0F);

    //Set the month
    switch(__DATE__[0])
    {
        case 'J':
            //January, June, or July
            switch(__DATE__[1])
            {
                case 'a':
                    //January
                    rtcc.byte.mon = 0x01;
                    break;
                case 'u':
                    switch(__DATE__[2])
                    {
                        case 'n':
                            //June
                            rtcc.byte.mon = 0x06;
                            break;
                        case 'l':
                            //July
                            rtcc.byte.mon = 0x07;
                            break;
                    }
                    break;
            }
            break;
        case 'F':
            rtcc.byte.mon = 0x02;
            break;
        case 'M':
            //March,May
            switch(__DATE__[2])
            {
                case 'r':
                    //March
                    rtcc.byte.mon = 0x03;
                    break;
                case 'y':
                    //May
                    rtcc.byte.mon = 0x05;
                    break;
            }
            break;
        case 'A':
            //April, August
            switch(__DATE__[1])
            {
                case 'p':
                    //April
                    rtcc.byte.mon = 0x04;
                    break;
                case 'u':
                    //August
                    rtcc.byte.mon = 0x08;
                    break;
            }
            break;
        case 'S':
            rtcc.byte.mon = 0x09;
            break;
        case 'O':
            rtcc.byte.mon = 0x10;
            break;
        case 'N':
            rtcc.byte.mon = 0x11;
            break;
        case 'D':
            rtcc.byte.mon = 0x12;
            break;
    }

    //Calculate day of week based on date
    dayOfWeek = 6;
    dayOfWeek += ((rtcc.byte.year>>4)*10)+(rtcc.byte.year&0x0F);
    dayOfWeek += ((rtcc.byte.year>>4)*10)+(rtcc.byte.year&0x0F)/4;
    switch(rtcc.byte.mon)
    {
        case 0x01:  //January
            //fall through
        case 0x10:  //October
            //add nothing
            break;

        case 0x02:  //February
        case 0x03:  //March
        case 0x11:  //November
            dayOfWeek += 3;
            break;

        case 0x04:  //April
        case 0x07:  //July
            dayOfWeek += 6;
            break;

        case 0x05:  //May
            dayOfWeek += 1;
            break;

        case 0x06:  //June
            dayOfWeek += 4;
            break;

        case 0x08:  //August
            dayOfWeek += 2;
            break;

        case 0x09:  //September
            //fall through
        case 0x12:  //December
            dayOfWeek += 5;
            break;
    }
    dayOfWeek += ((rtcc.byte.mday>>4)*10)+(rtcc.byte.mday&0x0F);
    dayOfWeek = dayOfWeek%7;
    rtcc.byte.wday = dayOfWeek;

    UpdateRTCC();
}


/****************************************************************************
  Function:
    void GetRTCC(RTCCFORM *rtccVal)

  Summary:
    Load input rtcc value with current RTCC module time

  Description:
    Load input rtcc value with current RTCC module time

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Modifies input variable with current RTCC time.
  ***************************************************************************/
void GetRTCC(RTCCFORM* rtccVal)
{

    RCFGCALbits.RTCPTR = 0b11;    //Setup pointer to RTCVAL
    rtccVal->w[0]=RTCVAL;     //read year
    rtccVal->w[1]=RTCVAL;     //read month,day
    rtccVal->w[2]=RTCVAL;     //read hour,day of week
    rtccVal->w[3]=RTCVAL;     //read min,sec
}


