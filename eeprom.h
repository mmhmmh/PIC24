/****************************************************************************
* XLP 16-bit Dev board external I2C EEPROM communications header file
*****************************************************************************
* FileName:     eeprom.h
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
* Author           Date        Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Brant Ivey       4/1/10      New header created to improve code organization
*****************************************************************************/

#ifndef _EEPROM_H
#define _EEPROM_H

/****************************************************************************
  Section: Includes
  ***************************************************************************/
#include "system.h"


/****************************************************************************
  Section: Defines
  ***************************************************************************/
//I2C configuration
#define I2C_CLOCK_FREQ      400000UL
#define I2CBRG_VAL          ((FCY/I2C_CLOCK_FREQ) - (FCY/ 10000000)) - 1
#define I2C_WRITE           0x00
#define I2C_READ            0x01

//EEPROM settings
#define EEPROM_SIZE         32768   //number of eeprom bytes
#define EEPROM_ADDRESS      0xA0    //address of external EEPROM
#define EEPROM_WRITE_WAIT   5       //EEPROM requires 5 ms to complete write
#define EEPROM_ACK_WAIT     1000    //Time (in uS) to wait in while polling
                                    //the EEPROM for write cycle completion


/****************************************************************************
  Section: Function Prototypes
  ***************************************************************************/
//I2C Protocol Functions
void InitI2C(void);
void CloseI2C(void);
int I2CWait(void);
void T2IntOn(void);
void T2IntOff(void);




#endif
