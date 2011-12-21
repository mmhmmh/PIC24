/****************************************************************************
* XLP 16-bit Dev board + Energy Harvesting kit demo code
*****************************************************************************
* FileName:     harvest.c
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
* Author           Date        Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Brant Ivey       4/1/10      Initial Release
*****************************************************************************/


/****************************************************************************
  Section: Includes
  ***************************************************************************/
#include "system.h"

#ifdef USE_HARVESTER
/****************************************************************************
  Section: Global Variables
  ***************************************************************************/
HARVEST_STATUS harvesterStatus;


/****************************************************************************
  Section: Energy Harvester Functions
  ***************************************************************************/


/****************************************************************************
  Function:
    void InitHarvester()

  Summary:
    Initialize harvester charge state tracking information.

  Description:
    Initializes the harvester charge state tracking variable.  The code reads
    VDD from the harvester.  If VDD is close to max value, assume 50% charge.
    Otherwise, assume battery is low for saftey in order to prevent over 
    discharging.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void InitHarvester()
{

    GetVddVal();    //Sample VDD

    //If VDD is close to max, set counter to 50% charge.
    //Otherwise, assume low battery.
    if(vddVal >= 3400)
    {
        harvesterStatus.bits.battRunCount = MAX_BATT_LOOPS/2;
    }
    else
    {
        harvesterStatus.bits.battRunCount = MAX_BATT_LOOPS;
    }

} //end InitHarvester()


/****************************************************************************
  Function:
    void GetHarvesterState()

  Summary:
    Update harvester state variables based on battery charge and harvester inputs.

  Description:
    Reads the /CHARGE pin and battery charge counter to determine the current
    state of the harvester.  There are 5 harvester states as defined by
    the BATTERY_STATES enumeration in harvest.h.
    
    Battery is fully charged when the /CHARGE pin has been asserted
    for long enough to decrement the battery state variable to 0.

    Battery is low when either a low VDD is detected or the battery state
    variable reaches the maximum loop count.

  Precondition:
    InitHarvester()

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void GetHarvesterState()
{

    #ifdef CYMBET_EVAL08
    //read the /Charge pin
    CHARGE_TRIS = 1;
    IdleMs(1);
    harvesterStatus.bits.charging = !CHARGE_PORT;
    CHARGE_TRIS = 0;

    //1 hr full charge = 3600 sec
    if(harvesterStatus.bits.battRunCount == 0 && harvesterStatus.bits.charging == 1)
    {
        harvesterStatus.bits.fullCharge = 1;   //full charge
    } 
    else
    {        
        harvesterStatus.bits.fullCharge = 0;   //charging           
    }
    #endif

    //When VDD < low batt threshold, or we've been running from 
    //battery power for too long, the battery is low
    if(vddVal < MIN_VDD || harvesterStatus.bits.battRunCount == MAX_BATT_LOOPS)
    {
            harvesterStatus.bits.lowBatt = 1;
            harvesterStatus.bits.fullCharge = 0; 
    }
    else
    {
            harvesterStatus.bits.lowBatt = 0;
    }

} //end GetHarvesterState

/****************************************************************************
  Function:
    void HandleHarvesterState()

  Summary:
    Modifies application operation based on harvester state and tracks battery
    charge status.

  Description:
    Changes application operation based on the harvester and battery status.
    The battery charge state is tracked using a loop counter which is
    incremented and decremented as the app runs from battery power (increment)
    or charges the battery while running from the harvester (decrement).

    Battery discharging is assumed to be linear, with each loop consuming 
    the same amount of current.  

    Battery charging has two states: low to 80% charge, which charges quickly
    (first half of charge time) and 80% to fill, which charges slowly 
    (remaining half of charge time).

    When the battery is fully charged and power from the harvester is present,
    the battery is disconnected from the circuit and the application runs
    directly from harvester power.

    When the battery is in a low charge state, the application is put into
    a low power mode and does not perform any actions until more charge is
    available in the battery.

  Precondition:
    InitHarvester()

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void HandleHarvesterState()
{
    WORD_VAL battState;
    int runCount;

    battState.Val = harvesterStatus.Val>>13;
    runCount = harvesterStatus.bits.battRunCount;   //copy to local variable to remove structure access times
    switch(battState.Val)
    {
        case BATT_CHARGED:          //CHARGED: batoff = 1, runcount = 0
            #ifdef CYMBET_EVAL08
            BATOFF_LAT = 1;         //When fully charged, disconnect batteries and run from harvester
            #endif
            runCount = 0;      
            break;

        case MED_CHARGING:          //MED_CHARGING: batoff = 0
            #ifdef CYMBET_EVAL08
            BATOFF_LAT = 0;         //Connect battery to harvester so it can charge
            #endif

            if(tasks.bits.alarm == 1)  //only do battery charging on RTCC wakeup
            {
    
                //Check the number of loops run from the battery to get an estimate of battery charge level
                if(runCount > (MAX_BATT_LOOPS*2/10))
                {
                    //Solar cell charges the first 80% of the batteries in the first 1/2 of the charge time.
                    //So, if the battery charge level is less than 80%, decrement run counter faster for this quick charge rate.
                    if(runCount > (MAX_BATT_LOOPS*8/10)*ALARM_PERIOD/(FULL_CHARGE_TIME/2))
                    {
                        runCount -= (MAX_BATT_LOOPS*8/10)*ALARM_PERIOD/(FULL_CHARGE_TIME/2);
                    }
                    else
                    {
                        runCount = 0;  //reset to zero if less than the decrement amount to avoid roll-under
                    }
                }
                else
                {
                    //The last 20% of the batteries charge in the remaining 1/2 of the charge time.
                    //So, if the battery charge level is more than 80%, decrement run counter slower for this slow charge rate.
                    if(runCount > (MAX_BATT_LOOPS*2/10)*ALARM_PERIOD/(FULL_CHARGE_TIME/2))
                    {
                        runCount -= (MAX_BATT_LOOPS*2/10)*ALARM_PERIOD/(FULL_CHARGE_TIME/2);
                    }
                    else
                    {
                        runCount = 0;  //reset to zero if less than the decrement amount to avoid roll-under
                    }
                }
            }
    
            break;

        case LOW_CHARGING:          //LOW_CHARGING: batoff = 0, limited actions
            #ifdef CYMBET_EVAL08
            BATOFF_LAT = 0;         //Connect battery to harvester so it can charge
            #endif
            
            if(tasks.bits.alarm  == 1)  //only do battery charging on RTCC wakeup
            {       
                //Check the number of loops run from the battery to get an estimate of battery charge level
                if(runCount > (MAX_BATT_LOOPS*2/10))
                {
                    //Solar cell charges the first 80% of the batteries in the first 1/2 of the charge time.
                    //So, if the battery charge level is less than 80%, decrement run counter faster for this quick charge rate.
                    if(runCount > (MAX_BATT_LOOPS*8/10)*ALARM_PERIOD/(FULL_CHARGE_TIME/2))
                    {
                        runCount -= (MAX_BATT_LOOPS*8/10)*ALARM_PERIOD/(FULL_CHARGE_TIME/2);
                    }
                    else
                    {
                        runCount = 0;  //reset to zero if less than the decrement amount to avoid roll-under
                    }
                }
                else
                {
                    //The last 20% of the batteries charge in the remaining 1/2 of the charge time.
                    //So, if the battery charge level is more than 80%, decrement run counter slower for this slow charge rate.
                    if(runCount > (MAX_BATT_LOOPS*2/10)*ALARM_PERIOD/(FULL_CHARGE_TIME/2))
                    {
                        runCount -= (MAX_BATT_LOOPS*2/10)*ALARM_PERIOD/(FULL_CHARGE_TIME/2);
                    }
                    else
                    {
                        runCount = 0;  //reset to zero if less than the decrement amount to avoid roll-under
                    }
                }
            }
    
            //low battery, limit actions until more charge available
            tasks.bits.button = 0;
            tasks.bits.sample = 0;
            tasks.bits.store = 0;
            tasks.bits.cap = 0;
            break;

        case LOW_BATT:              //LOW_BATT: batoff = 0, timer = 0, limited actions
            #ifdef CYMBET_EVAL08
            BATOFF_LAT = 0;         //Connect battery to circuit so it can power PIC
            #endif

            //low battery, limit actions until more charge available
            tasks.bits.button = 0;
            tasks.bits.sample = 0;
            tasks.bits.store = 0;
            tasks.bits.cap = 0;   
            break;

        case BATT_POWER:            //BATT_POWER: batoff =  0, runcount++, timer = 0
            #ifdef CYMBET_EVAL08
            BATOFF_LAT = 0;         //Connect battery to circuit so it can power PIC
            #endif
         
            //Running from battery, increment counter to keep track of battery level
            runCount++;

            if(tasks.bits.button == 1)
            {
                //pressing buttons consumes more power, so add an extra loop to account for it
                runCount++; 
            }

            //If baud rate of UART is 115200 or less then the baud rate 
            //is slow enough that UART transmissions add a large amount of time
            //to the loop.  So, if transmitting, add some extra counts to account
            //for this longer loop time.
            #if (BAUDRATE <= 115200UL)
            if(tasks.bits.transmit == 1)
            {
                runCount+= 115200UL/BAUDRATE; //115k= 1 extra loop, for slower baud rates, add aditional counts
            }
            #endif

            if(runCount > MAX_BATT_LOOPS)
            {
                runCount = MAX_BATT_LOOPS;
            }

            break;
    }

    harvesterStatus.bits.battRunCount = runCount;   //copy local variable back to run counter

}//end HandleHarvesterState()


/****************************************************************************
  Function:
    void TransmitHarvesterState()

  Summary:
    Transmits the current state of the harvester and battery charge.

  Description:
    Transmits the present state of the harvester (charging or discharging) and
    the battery state (battery % left, charged, or low).

  Precondition:
    InitHarvester()

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void TransmitHarvesterState()
{
    WORD_VAL battState;
    BYTE temp;

    //turn on external circuits if disabled
    #ifdef USE_PWRCTL
    if(PWRCTL_PORT)
    {
        mPWR_ON();
        IdleMs(UART_WAIT);
    }
    #endif

    _UxMD = 0;      //Enable UART
    UARTInit();

    UARTPrintString("\r\nBATT LEVEL: ");

    battState.Val = harvesterStatus.Val>>13;
    switch(battState.Val)
    {
        case BATT_CHARGED:          //CHARGED: FULL
            UARTPrintString("100%, Solar Powered");
            break;

        case LOW_CHARGING:          //LOW_CHARGING: LOW BAT, Charging
            UARTPrintString("LOW BATT ");
            tasks.bits.transmit = 0;   //turn off transmitting to save power

            //NOTE: NO BREAK, USES FALL THROUGH TO SAVE CODE SPACE

        case MED_CHARGING:          //MED_CHARGING: %%, Charging
            UARTPrintString("CHARGING ");

            //NOTE: NO BREAK, USES FALL THROUGH TO SAVE CODE SPACE

        case BATT_POWER:            //BATT_POWER: %%
            temp = (BYTE)((DWORD)(MAX_BATT_LOOPS-harvesterStatus.bits.battRunCount)*100/MAX_BATT_LOOPS);
            if(temp>99)
            {
                temp = 99;
            }
            UARTPutChar((temp/10)%10 + '0');
            UARTPutChar(temp%10 + '0');
            UARTPutChar('%');
            break;

        case LOW_BATT:              //LOW_BATT: LOW BAT
            UARTPrintString("LOW");
            tasks.bits.transmit = 0;    //turn off transmitting to save power
            break;
    }

    UARTPrintString("\r\n");

    #ifdef USE_PWRCTL
    mPWR_OFF();
    #endif
    _UxMD = 1;

}//end TransmitHarvesterState


/****************************************************************************
  Function:
    void StoreHarvesterState(void)

  Summary:
    Save current harvester status to EEPROM.

  Description:
    Saves the current harvester state information to external EEPROM in 
    case of reset event.

  Precondition:
    InitI2C()

  Parameters:
    Global variable - eeAddress.

  Returns:
    None.
  ***************************************************************************/
void StoreHarvesterState (void)
{
    BYTE datVal[2];

    datVal[0] = harvesterStatus.Val;
    datVal[1] = harvesterStatus.Val>>8;

    writePacket(0x2, datVal, 2);

    //wait in Idle for EEPROM write to complete
    while(writeWait())
    {
        IdleUs(EEPROM_ACK_WAIT);
    }
}



/****************************************************************************
  Function:
    void RestoreHarvesterState(void)

  Summary:
    Read havester status from EEPROM.

  Description:
    Read the harvester state bits from the external EEPROM and store to 
    harvesterStatus.

  Precondition:
    InitI2C()

  Parameters:
    None.

  Returns:
    Global variable - harvesterStatus
  ***************************************************************************/
void RestoreHarvesterState (void)
{
    BYTE dataVal[2];
    BYTE *pByte;

    //Init pointer and read EERPOM
    pByte = dataVal;
    readPacket(2,pByte,0x02);

    //Load address from EEPROM
    harvesterStatus.Val = (dataVal[0] + (((WORD)dataVal[1])<<8));

}


#endif
