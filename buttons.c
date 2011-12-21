/****************************************************************************
* XLP 16-bit Dev board button handling & processing
*****************************************************************************
* FileName:     buttons.c
* Dependencies: system.h
* Processor:    PIC24F16KA102
* Hardware:     XLP 16-bit Development Board
* Complier:     Microchip C30 v3.10 or higher
* Company:        Microchip Technology, Inc.
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

/****************************************************************************
  Section: Includes
  ***************************************************************************/
#include "system.h"


/****************************************************************************
  Section: Button Processing Static Variables
  ***************************************************************************/
WORD sw2HoldTime;
WORD sw3HoldTime;
BYTE sw2Pressed;
BYTE sw3Pressed;


/****************************************************************************
  Section: Button Processing functions
  ***************************************************************************/

/****************************************************************************
  Function:
    void EnableSW2(void)

  Summary:
    Enables SW2 pullup and interrupt.

  Description:
    Configures SW2 I/O as input and Enables pull-up.  INT0 interrupt is turned
    on to provide wakeup from sleep and deep sleep.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void EnableSW2(void)
{
    SW2_TRIS = 1;
    SW2_PULLUP = 1;     //Enable sw2 pull up

    _INT0EP = 1;
    _INT0IF = 0;
    _INT0IP = 4;
    _INT0IE = 1;
}


/****************************************************************************
  Function:
    void DisableSW2(void)

  Summary:
    DisableS SW2 pullup and interrupt.

  Description:
    Configures SW3 I/O as output and DisableS pull-up. DisableS INT0 interrupt.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void DisableSW2(void)
{
    _INT0IE = 0;
    _INT0IF = 0;

    SW2_TRIS = 0;
    SW2_PULLUP = 0;
}


/****************************************************************************
  Function:
    void EnableSW3(void)

  Summary:
    Enables SW3 pullup and interrupt.

  Description:
    Configures SW3 I/O as input and Enables pull-up.  CN interrupt is turned
    on to provide wakeup from sleep.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void EnableSW3(void)
{
    SW3_TRIS = 1;       //sw3 input
    SW3_PULLUP = 1;     //Enable sw3 pull up

    _CNIF = 0;          //configure interrupt for sleep wakeup
    _CNIP = 4;
    _CN12IE = 1;
    _CNIE = 1;
}


/****************************************************************************
  Function:
    void DisableSW3(void)

  Summary:
    DisableS SW3 pullup and interrupt.

  Description:
    Configures SW3 I/O as output and DisableS pull-up. DisableS CN interrupt.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void DisableSW3(void)
{
    _CN12IE = 0;
    _CNIE = 0;
    _CNIF = 0;

    SW3_TRIS = 0;
    SW3_PULLUP = 0;             //disable sw2 pull up
}


/****************************************************************************
  Function:
    void ProcessButtons(void)

  Summary:
    Detects and Processes button press events.

  Description:
    Detects button presses and button hold times.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    Pullups are disabled after detecting press to save current.
    Buttons are measured after every BUTTON_IDLEMS delay to determine if still
    pressed in order to dected button hold events.
  ***************************************************************************/
void ProcessButtons(void)
{

    _INT0IF = 0;    //disable interrupts while Processing
    _INT0IE = 0;
    _CNIF = 0;
    _CNIE = 0;

    SW2_TRIS = 1;
    SW2_PULLUP = 1; //Enable INT0 pullup
    SW3_TRIS = 1;
    SW3_PULLUP = 1; //turn on RB14 pullup

    IdleUs(5);  //allow I/O to transition high after enabling pullup

    sw2Pressed = SW2_PORT ^ 1;  //SW2 is pressed, Set flag
    sw3Pressed = SW3_PORT ^ 1;  //SW3 is pressed, Set flag

	#ifdef USE_BUTTON_HOLD
    sw2HoldTime = 0; //reset hold times
    sw3HoldTime = 0;

    while(SW3_PORT == 0 || SW2_PORT == 0)   //wait while buttons are held
    {
        SW2_PULLUP = 0; //disable pullup to save current while waiting
        SW3_PULLUP = 0; //disable pullup to save current while waiting

        IdleMs(BUTTON_IDLEMS);

        SW2_PULLUP = 1; //re-Enable to test for held button
        SW3_PULLUP = 1; //re-Enable to test for held button
        IdleUs(5);

        if(SW2_PORT == 0)   //increment hold counter while button is pressed
        {
            sw2HoldTime++;
        }

        if(SW3_PORT == 0)
        {
            sw3HoldTime++;
        }
    }
	#endif

    SW2_PULLUP = 0;
    SW3_PULLUP = 0; //disable pullups to save current
}//end ProcessButtons


/****************************************************************************
  Function:
    void HandleButtons(void)

  Summary:
    Handles button press events.

  Description:
    Handles button press events.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    Press SW2 only:     Forces a Sample using active sensor
    Press SW3 only:     Toggles UART TX on/off (cannot wake from Deep Sleep)
    SW2 hold (2 sec):   Switch sensor type
    SW3 hold (2 sec):   Switch low power modes
    SW2 & SW3 hold (2 sec): Transmit EEPROM contents over UART

    Note: SW3 cannot wake MCU from deep sleep.  To use SW3 in deep sleep hold
    SW3 then press & release SW2 to wake device from deep sleep.
    Note: Deep Sleep mode cannot be used with Cap touch sensor.
  ***************************************************************************/
void HandleButtons(void)
{

    #ifdef USE_PWRCTL
    mPWR_ON();   //Enable external circuit power
    #endif

    //Initialize UART to display mode changes
    if(tasks.bits.transmit)
    {
        _UxMD = 0;
        UARTInit();
    }

    //Detect button press events
    if(sw2Pressed && !sw3Pressed)   //SW2 is pressed, SW3 is not pressed
    {

        mSetLED2Tris(0); //Blink LED2 to indicate SW2 press
        mSetLED2(0);

        if(tasks.bits.transmit)
        {
            UARTPrintString("Wake from SW2 press\n\r") ;
        }

        tasks.bits.sample = 1;   //force Sample sensor

        #ifdef USE_BUTTON_HOLD
        //SW2 hold Processing
        if(sw2HoldTime >= (BUTTON_HOLDTIME/BUTTON_IDLEMS))
        {
            tasks.bits.mode += 1;           //switch sensor type

            #ifdef USE_CAPTOUCH
            if(tasks.bits.mode > MODE_POT)  //Sensor type includes cap button detecting
            {
                if(lpMode > LP_SLEEP)       //DPSLP is not supported
                {
                    lpMode = LP_SLEEP;      //force low power mode to sleep

                    if(tasks.bits.transmit) //Transmit switch to PC if TX en
                    {
                        UARTPrintString("Cap Touch buttons not supported in Deep Sleep.\r\n");
                        UARTPrintString("Changing low power mode to Sleep.\r\n");
                    }
                }

                CapSenseTickInit(); //Enable Timer1 for cap button detecting
            }
            else
            {
                _T1MD = 1;          //disable T1 to turn off CTMU interrupt
                _T1IE = 0;
            }
            #endif

            if(tasks.bits.transmit)
            {
                TransmitSampleModeChange(); //Transmit new mode info
            }
        }
        #endif

        mSetLED2(1);              //turn off led2
        mSetLED2Tris(1);


    }
    else if(sw3Pressed) //SW3 is pressed OR SW2 & SW3 pressed
    {

        mSetLED3Tris(0);
        mSetLED3(0);

        if(tasks.bits.transmit)
        {
            UARTPrintString("Wake from SW3 press\n\r") ;
        }

        #ifdef USE_BUTTON_HOLD
        if(sw2HoldTime >= (BUTTON_HOLDTIME/BUTTON_IDLEMS) &&   //Both SW2 and SW3 held
           sw3HoldTime >= (BUTTON_HOLDTIME/BUTTON_IDLEMS))
        {
            InitI2C();  //Enable I2C and UART for data dump
            _UxMD = 0;
            UARTInit();

            //Transmit entire EEPROM contents
            UARTPrintString("Transmitting datalog:\n\r");
            TransmitEEPROM(0,eeAddress);
            UARTPrintString("Datalog Transmit done!\n\r");

        }
        else if(sw3HoldTime >= (BUTTON_HOLDTIME/BUTTON_IDLEMS)) //SW3 only held
        {
            lpMode++;  //switch low power mode
            if(lpMode >= LP_MAX)
            {
                lpMode = LP_IDLE;
            }

            #ifdef USE_CAPTOUCH
            //If Cap sense and Deep Sleep, force change mode to Sleep
            if((tasks.bits.mode > MODE_POT) && (lpMode > LP_SLEEP))
            {
                lpMode = LP_SLEEP;

                if(tasks.bits.transmit)
                {
                    //TX low power mode change information
                    UARTPrintString("Could not switch low power mode to Deep Sleep.\n\r");
                    UARTPrintString("Please switch sensor type first.\n\r") ;
                }
            }
            #endif

            if(tasks.bits.transmit)
            {
                TransmitLPModeChange(); //TX low power mode change information
            }
        }
        else    //SW3 press
        {
            tasks.bits.transmit ^= 1;    //toggle UART Transmit on/off

            if(tasks.bits.transmit) //Transmit on, turn on LED and Enable UART
            {
                _UxMD = 0;
                UARTInit();
                UARTPrintString("UART DISPLAY Enabled.\n\r") ;
            }
        }
        #else
            tasks.bits.transmit ^= 1;    //toggle UART Transmit on/off

            if(tasks.bits.transmit) //Transmit on, turn on LED and Enable UART
            {
                _UxMD = 0;
                UARTInit();
                UARTPrintString("UART DISPLAY Enabled.\n\r") ;
            }
        #endif

        mSetLED3(1);
        mSetLED3Tris(1);

    }

    _UxMD = 1;              //disable UART & external circuits for power savings
    #ifdef USE_PWRCTL
    mPWR_OFF();
    #endif

    tasks.bits.button = 0;  //clear button task flag

}//end HandleButtons


