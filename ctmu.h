/****************************************************************************
* XLP 16-bit Dev board CTMU cap touch driver header file
*****************************************************************************
* FileName:     ctmu.h
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

#ifndef _CTMU_H
#define _CTMU_H


/****************************************************************************
  Section: Includes
  ***************************************************************************/
#include "system.h"


/****************************************************************************
  Section: CTMU Defines
  ***************************************************************************/
//Diode temperature sensing
#define DIODE_NUM_AVG       0x100   //number of reads in CTMU channel read routine

//Cap touch buttons
#define CAP_NUM_AVG         20      //Defines frequency of average update#define TRIPVAL             100     //Determines amount of change to determine key pressed
#define HYSTVAL             3       //Hysteresis of cap touch pad
#define NUM_CTMU_CH         3       //Number of cap sense channels
#define CT2_IGNORE          //CT2 conflicts with TEMP sensor, to Enable CT2 disconnect JP5 and comment this define out
#define TIMER1_PERIOD       32768/512   //Timer 1 period definition.  Timer 1 uses a 32.768 kHz 
                                        //clock with 1:64 postscaler, so this is a 125 mS period


/****************************************************************************
  Section: CTMU Function Prototypes
  ***************************************************************************/
void InitCapSense(void);
void CapSenseTickInit(void);
void ReadCTMU(int index);
void UpdateCTMU(int index);
void GetDiodeTemp(int Index);

#endif
