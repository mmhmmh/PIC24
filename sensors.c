/****************************************************************************
* XLP 16-bit Dev board Sensor handling & processing
*****************************************************************************
* FileName:     sensors.c
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

/****************************************************************************
  Section: Includes
  ***************************************************************************/
#include "system.h"


/******************************************************************************
  Section: ADC Functions
 ******************************************************************************/


/****************************************************************************
  Function:
    void InitADC(void)

  Summary:
    Initialize ADC for sampling.

  Description:
    Initialize ADC for sampling.  DisableS PMD and configures AN pins.
    ADC Settings: autoconvert, manual Sample, internal refs, TAD = TCY

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void InitADC(void)
{
    if(_ADC1MD)
    {
        _ADC1MD = 0;    //clear PMD bit to allow ADC to be used
    
        //AD1PCFG must be re-configured after clearing ADC PMD bit
        AD1PCFG = 0xFFFF;   //all I/O as digital
        POT_AN = 0;     //POT input in Analog mode
        TEMP_AN = 0;    //Temp input in analog mode
        TEMP_TRIS = 1;  //make temp & pot inputs
        POT_TRIS = 1;
    }

    AD1CON1 = 0x80E0;   //Autoconvert, manual Sample
    AD1CON2 = 0x0000;   //int ref, 16-word buffer, mux A
    AD1CON3 = 0x0700;   //TAD = TCY, Tsamp = 7 TAD
}


/****************************************************************************
  Function:
    WORD GetADCChannel(BYTE channel, BYTE timeTAD)

  Summary:
    Samples input ADC channel and returns result

  Description:
    Samples an input ADC channel for specified Sample time and outputs result.

  Precondition:
    None.

  Parameters:
    BYTE channel - ADC channel to perform conversion on.
    BYTE timeTAD - the number of TAD periods to spend sampling the channel.

  Returns:
    WORD - result of ADC conversion on input channel.

  Remarks:
    Idle mode is used while waiting for the ADC conversion to complete to
    reduce device power consumption.
  ***************************************************************************/
WORD GetADCChannel(BYTE channel,BYTE timeTAD)
{
    WORD result;

    InitADC();  //Initialize ADC

    //Setup TAD and channel
    AD1CON3 = ((WORD)timeTAD<<8) | 0x0000;
    AD1CHS = channel;

    //Initiate Sample
    AD1CON1bits.SAMP = 1;
    _AD1IE = 1;             //Enable interrupt to wake from Idle
    _AD1IP = 1;             //Set priority so ADC ISR does not occur after wakeup
    _AD1IF = 0;

    while(!_AD1IF)          //Idle until ADC Sample is finished
    {
        Idle();
    }

    AD1CON1bits.SAMP = 0;   //clear ADC flags and disable interrupt
    _AD1IF = 0;
    _AD1IE = 0;

    result = ADC1BUF0;

    _ADC1MD = 1;            //disable ADC to save power

    return result;
}//end GetADCChannel(...)



/****************************************************************************
  Section: Sensor Functions
  ***************************************************************************/


/****************************************************************************
  Function:
    void SampleSensors(void)

  Summary:
    Selects and Samples active sensor.

  Description:
    Selects and Samples active sensor.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void SampleSensors (void)
{

    //If UART is Enabled, Transmit status
    if(tasks.bits.transmit)
    {
        #ifdef USE_PWRCTL
        mPWR_ON();
        IdleMs(UART_WAIT);
        #endif
        _UxMD = 0;
        UARTInit();
        UARTPrintString("Taking Sample...\r\n") ;
        _UxMD = 1;
    }

    //Select active sensor
    switch(tasks.bits.mode)
    {
        case MODE_TEMP:

            #ifdef USE_PWRCTL
            mPWR_ON();
            IdleMs(TEMP_WAIT);      //turn on external circuits & wait for startup
            #endif

            GetTempVal();           //Sample Temp sensor
            break;

        case MODE_POT:

            #ifdef USE_PWRCTL
            mPWR_ON();
            IdleMs(POT_WAIT);       //turn on external circuits & wait for startup
            #endif

            GetPOTVal();            //Sample POT
            break;

        #ifdef USE_CAPTOUCH
        case MODE_CAP:

            GetCapVal();            //Sample CTMU cap touch channels
            break;
        #endif

        case MODE_ALL:

            #ifdef USE_PWRCTL
            mPWR_ON();
            IdleMs(TEMP_WAIT);      //turn on external circuits & wait for startup
            #endif

            GetTempVal();           //Sample Temp sensor
            GetPOTVal();            //Sample POT

            #ifdef USE_CAPTOUCH
            GetCapVal();            //Sample CTMU cap touch channels
            #endif
            break;

        default:

            break;
    }

    #ifdef USE_PWRCTL
    mPWR_OFF();                      //disable external circuits
    #endif

    tasks.bits.store = 1;        //Store newly Sampled data
    tasks.bits.sample = 0;       //clear sampling task flag

}//end SampleSensors()


/****************************************************************************
  Function:
    void GetVddVal(void)

  Summary:
    Sample the bandgap reference to determine VDD

  Description:
    Samples the internal bandgap reference to determine current VDD level.
    Result is saved as global variable.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Loads global variable vddVal with Sampled VDD level.

  Remarks:
    Bandgap reference typical voltage is 1.2V.  VBG_VAL constant may need to be
    modified to calibrate for part to part variation.
  ***************************************************************************/
void GetVddVal (void)
{
    //Enable bandgap reference
    AD1PCFG &= 0x7FFF; //workaround, no bit def in C30 3.12
    IdleUs(100);    //bandgap stabilization time

    vddVal = GetADCChannel(VBG_CHANNEL,VBG_TAD); //Sample VBG channel

    //Disable bandgap reference
    AD1PCFG |= 0x8000; //workaround, no bit def in C30 3.12

    vddVal = VBG_VAL/vddVal; //Convert ADC result VBG measurement into VDD
}//end GetVddVal


/****************************************************************************
  Function:
    void GetTempVal(void)

  Summary:
    Sample the temperature sensor.

  Description:
    Samples the temperature sensor and saves result as global variable.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Loads global variable tempVal with Sampled temperature.

  Remarks:
    VDD level is required to correctly calculate current temperature.
  ***************************************************************************/
void GetTempVal (void)
{

#if(TEMP_SENSOR  == MCP9700)

    tempVal = GetADCChannel(TEMP_CHANNEL,TEMP_TAD);     //Take TEMP data

    tempVal = (WORD)((vddVal*(long)tempVal)/1024*10);   //convert ADC result to mV
                                                        //*10 to Get temp to 0.1*C
    tempVal = (tempVal - MCP_V0)/MCP_TC;                //Calculate temperature

#elif(TEMP_SENSOR == DIODE)

    GetDiodeTemp(TEMP_CHANNEL);     //Sample temp data from diode

    tempVal = (WORD)((vddVal*(long)tempVal)/1024*100);  //convert ADC result to mV
                                                        //*100 to Get temp to 0.1*C
    tempVal = (DIODE_V0 - tempVal)/DIODE_TC;            //diode equation for temp


#elif (TEMP_SENSOR > 1)
    #error Please define Tempature sensor
#endif

}//end GetTempVal


/****************************************************************************
  Function:
    void GetPOTVal(void)

  Summary:
    Sample the potentiometer.

  Description:
    Samples the potentiometer and saves result as global variable.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Loads global variable potVal with Sampled temperature.

  Remarks:
    VDD level is required to correctly calculate current temperature.
  ***************************************************************************/
void GetPOTVal (void)
{
    //Take POT data
    potVal = GetADCChannel(POT_CHANNEL,POT_TAD);
    potVal = (WORD)((vddVal*(long)potVal)/1024);  //potval in mV
}


/****************************************************************************
  Function:
    void GetCapVal(void)

  Summary:
    Process CTMU Samples to update mTouch button states.

  Description:
    Reads and Processes CTMU results to update mTouch button states.
    Turns on LED if CT button is pressed.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void GetCapVal(void)
{
    BYTE channelSelect;

    //Sample and Process all available cap touch channels
    for(channelSelect = 0;channelSelect < NUM_CTMU_CH;channelSelect ++)
    {
        #ifdef CT2_IGNORE
        if(channelSelect == 1)
        {
            continue;
        }
        #endif

        ReadCTMU(channelSelect);
        UpdateCTMU(channelSelect);
    }

    //Light LED if CT button is pressed.
    if(pressedCT1 || pressedCT2 || pressedCT3)
    {
        #ifdef USE_PWRCTL
        mPWR_ON();
        #endif
        mSetLED3Tris(0);
        mSetLED3(0);     //Turn on LED when CT button pressed
    }
    else
    {
        mSetLED3(1);
        mSetLED3Tris(1);

    }

    tasks.bits.cap = 0;
}//end GetCapVal





