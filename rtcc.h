/****************************************************************************
* XLP 16-bit Dev board RTCC driver header file
*****************************************************************************
* FileName:     rtcc.h
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
* Brant Ivey       4/1/10      New header created to improve code organization
*****************************************************************************/

#ifndef _RTCC_H
#define _RTCC_H


/****************************************************************************
  Section: Includes
  ***************************************************************************/
#include "system.h"


/****************************************************************************
  Section: Defines and Typedefs
  ***************************************************************************/
#define EnableSOSC()    __builtin_write_OSCCONL(0x02) //turn on 32kHz crystal
#define SetRTCWREN()    __builtin_write_RTCWEN()      //enable RTCC modification

#define ONE_SECOND      1
#define TEN_SECOND      10
#define ALARM_PERIOD    TEN_SECOND  //RTCC alarm period in seconds

//union/structure for read/write of time and date from/to the RTCC device
typedef union
{
    struct
    {
        unsigned char year;       //BCD codification for year, 00-99
        unsigned char rsvd;       //reserved for future use
        unsigned char mday;       //BCD codification for day of the month, 01-31
        unsigned char mon;        //BCD codification for month, 01-12
        unsigned char hour;       //BCD codification for hours, 00-24
        unsigned char wday;       //BCD codification for day of the week, 00-06
        unsigned char sec;        //BCD codification for seconds, 00-59
        unsigned char min;        //BCD codification for minutes, 00-59
    }byte;                        //field access
    unsigned char b[8];           //BYTE access
    unsigned int  w[4];           //16 bits access
    unsigned long int l[2];       //32 bits access
}RTCCFORM;


/****************************************************************************
  Section: Global Variables
  ***************************************************************************/
extern RTCCFORM rtcc;


/****************************************************************************
  Section: Function Prototypes
  ***************************************************************************/
void InitRTCC(void);
void SetRTCCAlarm(BYTE mask);
void UpdateRTCC(void);
void ReadRTCC(void);
void InitCalendar(void);
void GetRTCC(RTCCFORM* rtccVal);

#endif
