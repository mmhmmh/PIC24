/****************************************************************************
* XLP 16-bit Development Board Demo system header
*****************************************************************************
* FileName:     system.h
* Dependencies: XLP16Demo.h, uart.h, GenericTypeDefs.h, HardwareProfile.h
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

#ifndef _SYSTEM_H
#define _SYSTEM_H

//#ifdef __PIC24F16KA102__
//#include "p24F16KA102.h"
//#elif defined __PIC24F16KA301__
#include "p24F16KA301.h"
//#endif

#include "GenericTypeDefs.h"

/*
  Select between different operating modes based on the kit being used.
  The preprocessor macro which selects the desired option is in the
  Preprocessor Macros section of the Project build options.
  (Project->Build options...->Project->MPLAB C30->General)
*/
#if defined(XLP16DEMO)
    #include "XLP16Demo.h"
#elif defined(XLPEHDEMO)
    #include "XLPEHDemo.h"
#else
    #error "Please define a valid operating mode."
#endif

#include "HardwareProfile.h"

//#ifdef USE_HARVESTER_UNDEF
//#include "harvest.h"
//#endif

#include "uart.h"
#include "rtcc.h"
//#include "buttons.h"
//#include "ctmu.h"
#include "spi1.h"
#include "nrf24l01.h"
#include "eeprom.h"
//#include "sensors.h"
//#include "transmit.h"

#endif

