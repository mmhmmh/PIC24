/****************************************************************************
* XLP 16-bit Dev board CTMU cap touch driver
*****************************************************************************
* FileName:     ctmu.c
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
* Bruce Bohn       8/7/2009     Initial revision 1.0
* Eric Yang        9/11/2009    V1.1
* Brant Ivey       4/1/10       Modified to add Harvester support.
*                               General bug fixes.
*****************************************************************************/

/****************************************************************************
  Section: Includes
  ***************************************************************************/
#include "system.h"


/****************************************************************************
  Section: Cap sense static variables
  ***************************************************************************/
WORD curRawData[NUM_CTMU_CH];   //Storage for CTMU channel values
WORD tripValue[NUM_CTMU_CH];    //Storage for the trip point for each channel
WORD hystValue[NUM_CTMU_CH];    //Storage for the hysterisis value for each channel
WORD avg_delay[NUM_CTMU_CH];    //Storage for count for average update for each channel
WORD immediateValue;            //current button value
WORD averageData[NUM_CTMU_CH];  //slow moving average of CTMU channels
WORD smallAvg[NUM_CTMU_CH];     //storage for fraction of average
int startupCount;               //variable to 'discard' first N Samples


/****************************************************************************
  Section: Cap sense functions
  ***************************************************************************/

/****************************************************************************
  Function:
    void InitCapSense(void)

  Summary:
    Initialize capacitance sensor.

  Description:
    Initialize capacitance sensor buffers and startup count.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void InitCapSense(void)
{
    BYTE index; //Index of what button is being checked.
    BYTE count;

    //Load defaults values for CT button raw and average data
    for (index=0; index < NUM_CTMU_CH; index++)
    {
        curRawData[index] = 0;
        averageData[index] = 0;
    }

    //Load default trip values
    tripValue[0] = TRIPVAL;
    tripValue[1] = TRIPVAL;
    tripValue[2] = TRIPVAL;

    //Load default hystersis values
    hystValue[0] = 0x3;
    hystValue[1] = 0x3;
    hystValue[2] = 0x3;

    //First pass loops to reach steady state average before enabling detection
    startupCount = 32;

    for(count = 0;count < CAP_NUM_AVG;count ++)
    {
        GetCapVal();
    }

}


/****************************************************************************
  Function:
    void CapSenseTickInit(void)

  Summary:
    Initialize timekeeping for capacitance sensing.

  Description:
    Capacitance sensing requires more frequent wakeups.  This Initializes
    Timer1 to wake up MCU more frequently for cap sensing applications.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void CapSenseTickInit( void )
{
    EnableSOSC();           //Enable 32kHz oscillator
    _T1MD = 0;              //clear PMD bit

    TMR1 = 0;               //clear timer
    PR1 = TIMER1_PERIOD;    //Set timer1 period
    T1CON = 0x8022;         //T1 on, 1:64 prescale, external clock, asycnh source

    _T1IF = 0;              //clear flag
    _T1IE = 1;              //Enable interrupt
}


/****************************************************************************
  Function:
    void ReadCTMU(int index)

  Summary:
    Samples input CTMU channel.

  Description:
    Samples input CTMU channel and logs data to CTMU data buffer curRawData.

  Precondition:
    InitCapSense()

  Parameters:
    int index - index of CTMU channel to Sample.

  Returns:
    None.
  ***************************************************************************/
void ReadCTMU(int index)
{

    _CTMUMD = 0;        //Enable CTMU
    if(_ADC1MD)
    {
        _ADC1MD = 0;    //Enable ADC

        POT_AN = 0;     //reset AD1PCFG after clearing PMD
        TEMP_AN = 0;
        TEMP_TRIS = 1;  //TEMP IN
        POT_TRIS = 1;   //POT IN
    }

    CTMUCON = 0;
    _EDG2SEL = 0x3;     //edge2 src = CTED1
    _EDG1POL = 1;       //edge1 postive
    _EDG1SEL = 0x3;     //edge1 src = CTED1
    CTMUICON = 0x300;   //0.55uA

    AD1CON1 = 0x8000;   //adc on
    _CTMUEN = 1;        //CTMU on

    //Ground selected CTMU channel
    switch(index)
    {
        case 0:
            CT1_LAT = 0;                //ground CT1
            CT1_TRIS = 0;
            Nop();Nop();Nop();Nop();Nop();
            CT1_TRIS = 1;               //ct1 as analog input
            CT1_AN = 0;
            AD1CHS = CT1_CHANNEL;
            break;

        case 1:
            CT2_LAT = 0;                //ground CT2
            CT2_TRIS = 0;
            Nop();Nop();Nop();Nop();Nop();
            CT2_TRIS = 1;               //ct2 as analog input
            CT2_AN = 0;
            AD1CHS = CT2_CHANNEL;
            break;

        case 2:
            CT3_LAT = 0;                //ground CT3
            CT3_TRIS = 0;
            Nop();Nop();Nop();Nop();Nop();
            CT3_TRIS = 1;               //CT3 as analog input
            CT3_AN = 0;
            AD1CHS = CT3_CHANNEL;
            break;

        default:
            break;
    }

    //Drain any internal ADC charge
    _IDISSEN = 1;   //Drain any charge on the circuit
    Nop(); Nop(); Nop(); Nop(); Nop();
    _IDISSEN = 0;

    //Charge the capacitive sensor
    _AD1IF = 0;
    _SAMP = 1;      //Manually start the conversion

    _EDG2STAT = 0;  //Make sure edge2 is 0
    _EDG1STAT = 1;  //Set edge1 - Start Charge
    Nop();Nop();Nop();Nop();Nop();Nop();    //Delay for CTMU charge time
    _EDG1STAT = 0;  //Clear edge1 - Stop Charge

    _SAMP = 0;      //Manually Initiate an ADC conversion
    while(!_AD1IF); //Wait for the A/D conversion to finish

    immediateValue = ADC1BUF0;  //Read the value from the A/D conversion

    _SAMP = 0;      //Clear ADC status
    _AD1IF = 0;
    _DONE = 0;

    curRawData[index] = immediateValue; //curRawData array holds the most recent BIGVAL values

    _CTMUMD = 1;    //CTMU & ADC off to save power
    _ADC1MD = 1;

} //end ReadCTMU()


/****************************************************************************
  Function:
    void UpdateCTMU(int index)

  Summary:
    Detect and Handle cap touch button presses.

  Description:
    Compares Sampled CTMU value to running average to detect button presses.
    On button press, indicates input button state and Handles state changes.

  Precondition:
    InitCapSense()

  Parameters:
    int index - index of CTMU channel to Handle.

  Returns:
    None.
  ***************************************************************************/
void UpdateCTMU(int index)
{

    //1. On power-up, reach steady-state readings
    //During power-up, system must be Initialized, Set average ea. pass = raw data
    if (startupCount > 0)
    {
        startupCount--;         //Decr. N # times to establish startup
        averageData[index] = curRawData[index]; //During start up time, Set Average each pass.
    }
    else
    {
        //Startup complete
        if(startupCount == 0)
        {
            startupCount = -1;
        }

        //2. Detect cap sense button press
        if (curRawData[index] < (averageData[index] - tripValue[index]))
        {
            switch(index)
            {
                case 0:
                    if(!pressedCT1)     //log state changes only
                    {
                        tasks.bits.store = 1;
                    }

                    pressedCT1 = 1;     //button is pressed
                    break;

                case 1:
                    if(!pressedCT2)     //log state changes only
                    {
                        tasks.bits.store = 1;
                    }

                    pressedCT2 = 1;      //button is pressed
                    break;

                case 2:
                    if(!pressedCT3)     //log state changes only
                    {
                        tasks.bits.store = 1;
                    }

                    pressedCT3 = 1;     //button is pressed
                    break;

                default:
                    break;
            }//end switch(index)
        }
        else    //3. If pressed criteria not reached, Set to not pressed
        {
            switch(index)
            {
                case 0:
                    if(pressedCT1)      //log state changes only
                    {
                        tasks.bits.store = 1;
                    }

                    pressedCT1 = 0;     //button is pressed
                    break;

                case 1:
                    if(pressedCT2)      //log state changes only
                    {
                        tasks.bits.store = 1;
                    }

                    pressedCT2 = 0;     //button is pressed
                    break;

                case 2:
                    if(pressedCT3)      //log state changes only
                    {
                        tasks.bits.store = 1;
                    }

                    pressedCT3 = 0;     //button is pressed
                    break;

                default:
                    break;
            }//end switch(index)

        }

         //4. Quick-release for a released button.
        //When button is released, reset average to new max value
        if (curRawData[index]  > averageData[index])
        {
            averageData[index] = curRawData[index];
        }

        //5. Average in the new value
        //Average (all buttons) each NUM_AVG times through the algorithm
        if(avg_delay[index] < CAP_NUM_AVG)
        {
            avg_delay[index]++;
        }
        else
        {
            avg_delay[index] = 0;
        }

        if(avg_delay[index] == CAP_NUM_AVG)     //Average in raw value.
        {
            smallAvg[index] = averageData[index] / CAP_NUM_AVG;
            averageData[index] = averageData[index] + ((curRawData[index] / CAP_NUM_AVG) - smallAvg[index]);
            //Note: This is not a true average.  It is:  New Average = Last Average + (New Value/8 - Last Average/8)
        }

    }//end if(startupCount > 0)

}//end UpdateCTMU(..)


/****************************************************************************
  Function:
    void GetDiodeTemp(int Index)

  Summary:
    Measure temperature using CTMU and junction diode.

  Description:
    Measures current temperature by measuring forward voltage of diode with
    constant current applied.

  Precondition:
    None.

  Parameters:
    int index - ADC channel to Sample for diode temp measurement

  Returns:
    None.
  ***************************************************************************/
void GetDiodeTemp(int index)
{
    DWORD total = 0;
    int chrd;
    int zvar;
    int diodeValue;

    _CTMUMD = 0;        //clear PMD bits
    if(_ADC1MD)
    {
        _ADC1MD = 0;

        POT_AN = 0;     //reset AD1PCFG after clearing ADC PMD
        TEMP_AN = 0;
        TEMP_TRIS = 1;
        POT_TRIS = 1;
    }

    CTMUCON = 0;
    _EDG2SEL = 0x3;     //edge2 src = CTED1
    _EDG1POL = 1;       //edge1 postive
    _EDG1SEL = 0x3;     //edge1 src = CTED1

    CTMUICON = 0;
    _IRNG = 0x03;       //0.55uA

    AD1CON1 = 0x8000;   //adc on
    AD1CHS = index;     //select A/D channel

    _CTMUEN = 1;        //Enable CTMU

    //Take DIODE_NUM_AVG readings of diode AN channel
    for(chrd=0; chrd<DIODE_NUM_AVG; chrd++)
    {
        TEMP_AN = 0;
        TEMP_LAT = 0;   //GROUND AN channel
        TEMP_TRIS = 0;
        Nop(); Nop(); Nop(); Nop(); Nop();

        TEMP_AN = 1;    //switch back to AN input
        TEMP_TRIS = 1;

        _IDISSEN = 1;  //Drain any charge on the ADC circuit
        Nop(); Nop(); Nop(); Nop(); Nop();
        _IDISSEN = 0;

        _AD1IF = 0;
        _SAMP = 1;      //Manually start the conversion
        _EDG2STAT = 0;  //Make sure edge2 is 0
        _EDG1STAT = 1;  //Set edge1 - Start Charge

        for (zvar = 0; zvar < 5; zvar++);    //Delay for CTMU charge time

        _EDG1STAT = 0;  //Clear edge1 - Stop Charge
         _SAMP = 0;     //manually Initiate an ADC conversion
        while(!_AD1IF); //Wait for the A/D conversion to finish

        _SAMP = 0;      //clear ADC status
        _AD1IF = 0;
        _AD1IE = 0;

        diodeValue  = ADC1BUF0; //Read the value from the A/D conversion

        total = total + diodeValue; //sum sequential reads
    }

    tempVal = total/DIODE_NUM_AVG;    //Store resulting average to tempVal

    _CTMUMD = 1;        //disable ADC and CTMU for power savings
    _ADC1MD = 1;

}//end GetDiodeTemp(..)



