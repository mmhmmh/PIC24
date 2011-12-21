/****************************************************************************
* XLP 16-bit Dev board Sensor handling & processing header file
*****************************************************************************
* FileName:     sensors.h
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

#ifndef _SENSORS_H
#define _SENSORS_H

/****************************************************************************
  Section: Includes
  ***************************************************************************/
#include "system.h"


/****************************************************************************
  Section: Defines and Typedefs
  ***************************************************************************/
//ADC and sensor configuration
#define TEMP_TAD        31          //fast TAD, so use max Sample time for accuracy
#define POT_TAD         31
#define VBG_TAD         31
#define VBG_VAL         1210UL*1024UL //VBG = 1.21V, VBG_VAL = 1210 mV * 1024 counts (10 bit ADC)
#define VBG_CHANNEL     15
#define VBG_AN          AD1PCFGbits.PCFG15

//temp sensor switch
#define MCP9700         0
#define DIODE           1
#define TEMP_SENSOR     MCP9700   //Temp sensor mode: DIODE or MCP9700

//temp sensor Initial voltages and temp constants
//Temp equation: T = (V - V0)/TC
#define MCP_TC          10      //MCP9700 temp constant TC = 10mV/*C
#define MCP_V0          5000    //MCP V @ 0*C = 500 mV (scaled up by 10 for 0.1*C res)
#define DIODE_TC        28      //1N4148 temp constant TC=-2.8mV/*C
#define DIODE_V0        39000   //1N4148 Diode Vf @ 0*C = 390mV
                                //Diode vals scaled up to Get 0.1*C resolution

//Sensor operating modes enumeration
typedef enum
{
    MODE_TEMP,
    MODE_POT,
#ifdef USE_CAPTOUCH
    MODE_CAP,
#endif
    MODE_ALL
} SENSOR_MODES;

//sensor data type
typedef enum
{
    CT1_DATA=1,
    CT2_DATA=2,
    CT3_DATA=3,
    N0_CT_PRESSED=4,
    POT_DATA=5,
    VDD_DATA=6,
    TEMP_DATA=7
} SENSOR_TYPES;

/****************************************************************************
  Section: Function Prototypes
  ***************************************************************************/

//ADC operation functions
void InitADC(void);
WORD GetADCChannel(BYTE channel,BYTE timeTAD);

//Sensor sampling functions
void SampleSensors(void);
void GetVddVal(void);
void GetTempVal(void);
void GetPOTVal(void);
void GetCapVal(void);


#endif
