/****************************************************************************
* XLP 16-bit Development Board + Energy Harvesting kit header file
*****************************************************************************
* FileName:     harvest.h
* Dependencies: none
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

#ifndef _HARVEST_H
#define _HARVEST_H


#include "system.h"


/****************************************************************************
  Section: Constants and Defines
  ***************************************************************************/
//battery level management constants
#define MIN_VDD             3000    //3V
#define MAX_BATT_LOOPS      2048    //2048 is average loop count based on testing
#define FULL_CHARGE_TIME    3600    //battery charge time in seconds


#if defined(IPS_XLP_BOARD) && defined(CYMBET_EVAL08)
    #error CONFLICTING HARVESTER DEFS
#endif


/****************************************************************************
  Section: Datatype definitions
  ***************************************************************************/
//battery flags: bit 0 - low batt, bit 1 - charging, bit 2 - full charge
typedef enum
{
    BATT_POWER=0,     //Discharging, No Solar
    LOW_BATT=1,       //Mostly Discharged, No Solar
    MED_CHARGING=2,   //Partial Discharge, Full Solar
    LOW_CHARGING=3,   //Mostly Discharged, Full Solar
    BATT_CHARGED=6,   //Full Charge, Full Solar
} BATTERY_STATES;

//HARVEST_STATUS is limited to a 16-bit field in order to ensure that we can safely
//store the battery status information in the Deep Sleep GPRs along with the app
//operating mode information
typedef union _HARVEST_STATUS
{
    WORD Val;
    struct
    {
        int battRunCount:13;      //Number of loops on battery power, max of 8191
        unsigned char lowBatt:1;    //battery is low
        unsigned char charging:1;   //battery is charging
        unsigned char fullCharge:1; //full battery charge
    } bits;
} HARVEST_STATUS;


/****************************************************************************
  Section: Function Prototypes
  ***************************************************************************/
void InitHarvester(void);
void GetHarvesterState(void);
void HandleHarvesterState(void);
void TransmitHarvesterState(void);
void StoreHarvesterState(void);
void RestoreHarvesterState(void);


/****************************************************************************
  Section: Global Variables
  ***************************************************************************/
extern HARVEST_STATUS harvesterStatus;

#endif


