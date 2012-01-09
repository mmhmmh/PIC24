/****************************************************************************
* XLP 16-bit Development Board Demo header
*****************************************************************************
* FileName:     XLP16Demo.h
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
* Author           Date         Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Brant Ivey &     9/18/09      Initial Release
* Eric Yang
* Brant Ivey       4/1/10       Modified to add Harvester support.
*                               General bug fixes.
*****************************************************************************/

#ifndef __XLPDEMO_H
#define __XLPDEMO_H


/****************************************************************************
  Section: Includes
  ***************************************************************************/
#include "system.h"


/****************************************************************************
  Section: Constants and Defines
  ***************************************************************************/

/****************************************************************************
 The USE_PWRCTL defines enable external device power control statements:
 mPWR_ON() and mPWR_OFF().
 Two modes are provided: normal and one-shot mode.  In normal power control 
 mode, external devices are powered on as-needed and powered off afterward.  
 This can cause too much peak current consumption for some batteries, 
 so oneshot mode is also provided, which enables external devices at the 
 start of a loop and disables at the end of a loop to reduce current spikes.
  ***************************************************************************/
//#define USE_PWRCTL
//#define USE_PWRCTL_ONESHOT

/****************************************************************************
 For some applications, detection of held buttons requires too much power.
 For these applications, held button detection can be disabled by commenting
 out this define.
 Note:
 - Disabling held button detection removes the ability to switch the low 
   power mode and sensor mode at runtime.
 - Disabling removes the button debouncing logic, which can cause a single 
   button press to be detected multiple times.
  ***************************************************************************/
#define USE_BUTTON_HOLD

//This define enables extra functionality for handling capacitive touch sensing
//When enabled, mode MODE_CAP is enabled and MODE_ALL will include cap touch sensing
//#define USE_CAPTOUCH

//This define enables extra functionality for tracking battery charge level
//when using power supplied by an energy harvesting system.
//#define USE_HARVESTER

//These defines select what harvester module is in use.
//Only one should be selected at a time.
//#define CYMBET_EVAL08
//#define IPS_XLP_BOARD


//Configure default mode and sensor settings
//If USE_BUTTON_HOLD is disabled these settings can't be changed at runtime.
#define DEFAULT_MODE    LP_SLEEP
#define DEFAULT_SENSOR  MODE_ALL

//Configure default UART transmission setting
//This setting can be changed at runtime by pressing S3
#define DEFAULT_TX      TRUE


//External device power-on startup times. This is the amount of time to wait (in mS)
//for a device to be ready after powering it with mPWR_ON()
#define TEMP_WAIT       1   //1 mS
#define EEPROM_WAIT     3   //3 mS
#define UART_WAIT       1   //1 mS
#define POT_WAIT        1   //1 mS
#define PICTAIL_WAIT        //define poweron time for PICtail here
          
//Clock speed constants
#define GetSystemClock()        8000000UL
#define GetPeripheralClock()    (GetSystemClock()/2)
#define GetInstructionClock()   (GetSystemClock()/2)
#define FCY                     GetInstructionClock()

//UART configuration
#define BAUDRATE            1000000UL
#define BRGH_VAL            1
#define INVERT_UART         //Invert UART TX & RX to prevent powering 
                            //disabled circuits by idling high.


/****************************************************************************
  Section: Datatype definitions
  ***************************************************************************/
//Low power operating modes enumeration
typedef enum
{
    LP_IDLE,
    LP_SLEEP,
    LP_DPSLP,
    LP_MAX
} LP_MODES;

//Demo status and operating mode
typedef union _TASK_VAL
{
    BYTE Val;
    struct
    {
        unsigned char mode:2;
        unsigned char sample:1;
        unsigned char transmit:1;
        unsigned char store:1;
        unsigned char button:1;
        unsigned char cap:1;
        unsigned char alarm:1;
    } bits;
} DEMO_TASKS;



/****************************************************************************
  Section: Function Prototypes
  ***************************************************************************/
void InitIO(void);
void InitSystem(void);
void InitSPI(void);

void HandleReset(void);

void IdleUs(int timeUs);
void IdleMs(int timeMs);


/****************************************************************************
  Section: Global Variables
  ***************************************************************************/


/****************************************************************************
  Section: Setup errors
  ***************************************************************************/
#if defined(USE_PWRCTL) && defined(USE_PWRCTL_ONESHOT)
    #error Check demo settings. Both USE_PWRCTL and USE_PWRCTL_ONESHOT are defined, only one of these modes can be used.
#endif

#if defined(USE_HARVESTER) && defined(USE_BUTTON_HOLD)
    #warning USE_BUTTON_HOLD is not recommended for use with harvesters, check demo settings.
#endif

#endif
