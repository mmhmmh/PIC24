/****************************************************************************
* XLP 16-bit Development Board Demo
*
*   Demo has 3 low power modes and 4 sensor modes.
*   Low power modes:    Sleep (default), Deep Sleep, and Idle
*   Sensor modes:       Temprature, Voltage, Capacitance, and All Sensors
*
*   In Temprature and Voltage modes the MCU wakes up on RTCC interrupt or
*   S2 button press to take a sample and store to external EEPROM.
*
*   In Capacitance and All sensor modes MCU wakes up on T1 interrupt to
*   sample Cap sense buttons.  Sample & storage occur on Cap sense button
*   press only.
*
*   Switch Low power and sensor modes by holding S2 or S3.
*
*   Toggle UART transmission of data with S3 press.  EEPROM datalog can be
*   transmitted out on UART by resetting while holding S3.
*****************************************************************************
* FileName:     main.c
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
* Author           Date         Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Brant Ivey &     9/18/09      Initial Release
* Eric Yang
* Brant Ivey        4/1/10      Modified to add Harvester support.
*                               General bug fixes.
*****************************************************************************/

/****************************************************************************
  Section: Includes and Config bits
  ***************************************************************************/
#include "system.h"
#ifdef __PIC24F32KA301__
    // Setup configuration bits
    _FBS(BSS_OFF & BWRP_OFF)
    _FGS(GSS0_OFF & GWRP_OFF)
    _FOSCSEL(FNOSC_FRCDIV & IESO_OFF & LPRCSEL_HP & SOSCSRC_DIG)
    _FOSC(FCKSM_CSECMD & POSCFREQ_MS & OSCIOFNC_OFF & POSCMOD_NONE & SOSCSEL_SOSCLP)
    _FWDT(FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768)
    _FPOR(MCLRE_ON & BORV_LPBOR & BOREN_BOR3 & I2C1SEL_PRI & PWRTEN_OFF)
    _FICD(ICS_PGx2)
    _FDS(DSWDTEN_OFF & DSBOREN_ON & DSWDTOSC_SOSC & DSWDTPS_DSWDTPSF) //DSWDT SOSC = LPRC
#elif defined __PIC24F16KA102__
    // Setup configuration bits
    _FBS(BSS_OFF & BWRP_OFF)
    _FGS(GCP_OFF & GWRP_OFF)
    _FOSCSEL(FNOSC_FRCDIV & IESO_OFF)
    _FOSC(FCKSM_CSECMD & POSCFREQ_MS & OSCIOFNC_ON & POSCMOD_NONE & SOSCSEL_SOSCLP)
    _FWDT(FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768)
    _FPOR(MCLRE_ON & BORV_LPBOR & BOREN_BOR3 & I2C1SEL_PRI & PWRTEN_OFF)
    _FICD(BKBUG_OFF & ICS_PGx3)
    _FDS(DSWDTEN_OFF & DSBOREN_ON & RTCOSC_SOSC & DSWDTOSC_SOSC & DSWDTPS_DSWDTPSF) //DSWDT SOSC = LPRC
#elif defined(__PIC24FJ64GA102__)
    // Setup configuration bits
    _CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & ICS_PGx3 & FWDTEN_OFF & WINDIS_OFF)
    _CONFIG2(IESO_ON & FCKSM_CSECMD & OSCIOFNC_ON & POSCMOD_NONE & FNOSC_FRC)
    _CONFIG3(SOSCSEL_LPSOSC & WUTSEL_FST & WPDIS_WPDIS & WPCFG_WPCFGDIS)
    _CONFIG4(DSWDTEN_OFF & RTCOSC_SOSC & DSWDTPS_DSWDTPS6 & DSWDTOSC_LPRC & DSBOREN_OFF)
#endif

#define DATA_LEN 19

/****************************************************************************
  Section: Global Variables
  ***************************************************************************/
RTCCFORM rtcc;          //RTCC read/write structure

/****************************************************************************
  Section: Main Function
  ***************************************************************************/

/****************************************************************************
  Function:
    int main(void)

  Summary:
    Main function.

  Description:
    Main function.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
int main(void)
{

    //WORD_VAL modes;
	int j;
	BYTE data[32];

    InitIO();      //configure I/O
    InitSystem();  //Initial system Setup
    //HandleReset(); //Process reset events (POR/DPSLP)

    IdleMs(500);

    //InitI2C();
	InitSPI();
	IdleMs(500);

	nrf24l01_initialize(nrf24l01_CONFIG_DEFAULT_VAL | nrf24l01_CONFIG_PWR_UP | nrf24l01_CONFIG_PRIM_RX, //1 byte CRC, powered up, RX
							   true,								//enable CE
							   nrf24l01_EN_AA_ENAA_ALL, 			//disable auto-ack on all pipes
							   nrf24l01_EN_RXADDR_ERX_ALL, 			//enable receive on all pipes
							   nrf24l01_SETUP_AW_5BYTES, 		//5-byte addressing
							   nrf24l01_SETUP_RETR_DEFAULT_VAL, 	//not using auto-ack, so use default
							   nrf24l01_RF_CH_DEFAULT_VAL, 			//RF channel 3
							   nrf24l01_RF_SETUP_DEFAULT_VAL,  		//2 Mbps, 0 dBm
							   NULL, 								//default receive addresses on all 6 pipes
							   NULL, 								//""
							   nrf24l01_RX_ADDR_P2_DEFAULT_VAL, 	//""
							   nrf24l01_RX_ADDR_P3_DEFAULT_VAL, 	//""
							   nrf24l01_RX_ADDR_P4_DEFAULT_VAL, 	//""
							   nrf24l01_RX_ADDR_P5_DEFAULT_VAL, 	//""
							   NULL, 								//default TX address
							   DATA_LEN, 									//1 byte paylaod width on all 6 pipes
							   DATA_LEN, 									//""
							   DATA_LEN, 									//""
							   DATA_LEN,  									//""
							   DATA_LEN,  									//""
							   DATA_LEN);

	//nrf24l01_initialize_debug(true, DATA_LEN, true);

	IdleMs(500);



	//SetupGyro();
	//SetupMag();
    //SetupAcc();

	BYTE dataVal[6];
    BYTE *pData;
    pData = dataVal;

	BYTE gyroDataVal[6];
   	BYTE *pGyroData;
   	pGyroData = gyroDataVal;

   	BYTE accDataVal[6];
   	BYTE *pAccData;
   	pAccData = accDataVal;

   	_UxMD = 0;      //Enable UART
   	UARTInit();
   	BYTE dataIn = 0x00;

   	while(0) {
   		BYTE dataOut = 0x00;
   		dataIn = 0xFF;
   		//CSN1_LAT = 0;
   		//CSN1_LAT = 1;
   		dataOut = spi1_send_read_byte(dataIn);
   		UARTPutHex(dataOut);
   		IdleMs(1000);
		IdleMs(1000);
		IdleMs(1000);
		IdleMs(1000);
		IdleMs(1000);
   	}

   	while(0) {
   		dataIn += 1;
   		BYTE dataOut = 0x00;
   		//CSN1_LAT = 0;
   		dataOut = spi1_send_read_byte(dataIn);
   		//CSN1_LAT = 1;

   		UARTPrintString("Input:");
   		UARTPutHex(dataIn);
   		UARTPrintString("Output:");
   		UARTPutHex(dataOut);
   		UARTPrintString("\r\n");

   		IdleMs(1000);
		IdleMs(1000);
		IdleMs(1000);
   	}

   	//nrf24l01_power_down();

   	while(0) {
   		BYTE data;

   		CSN1_LAT = 0;
		data = spi1_send_read_byte(0xff);
		UARTPutHex(data);
		CSN1_LAT = 1;

   		CSN1_LAT = 0;
		data = spi1_send_read_byte(0x07);
		UARTPutHex(data);
		data = spi1_send_read_byte(0x00);
		UARTPutHex(data);
		CSN1_LAT = 1;

//   	   	nrf24l01_write_register(nrf24l01_EN_AA, data, 1);
//   	   	nrf24l01_read_register(nrf24l01_EN_AA, data, 1);
   	   	IdleMs(1000);
		IdleMs(1000);
		IdleMs(1000);
		IdleMs(1000);
		IdleMs(1000);
		IdleMs(1000);
		IdleMs(1000);
   	}

   	while(1) {
        while(!(nrf24l01_irq_pin_active() && nrf24l01_irq_rx_dr_active()));

		nrf24l01_read_rx_payload(data, DATA_LEN); //get the payload into data
		nrf24l01_irq_clear_all();

		for (j = 0; j<DATA_LEN; j++) {
			UARTPutHex(data[j]);
		}
		UARTPrintString("\r\n");
   	}

   	while(0) {
   		BYTE registerData[40];
   		BYTE *pRegisterData = registerData;
   		nrf24l01_get_all_registers(pRegisterData);
   		registerData[39] = '\0';
   		int i;
   		for (i=0; i<35;i++) {
   			UARTPrintString("Register");
   			UARTPutHex(i);
   			UARTPrintString(": ");
   			UARTPutHex(registerData[i]);
   			UARTPrintString("\r\n");
   		}
   		IdleMs(1000);
   		IdleMs(1000);
   		IdleMs(1000);
   		IdleMs(1000);
   	    IdleMs(1000);
   	    IdleMs(1000);
   	    IdleMs(1000);
   	    IdleMs(1000);
   	    IdleMs(1000);
   	}

	while(0)
    {
		ReadMag(pData);
		//********************************************************************************************
		//Gyroscope code
		ReadGyro(pGyroData);

		//*****************************************************
		//Acc Code
		ReadAcc(pAccData);

		//***************************************************
		//UART Code
		_UxMD = 0;      //Enable UART
	    UARTInit();

	    #ifdef ACC_OUT_EN
			//UARTPrintString("AccX: ");
			UARTPutHex(pAccData[0]);
			UARTPutHex(pAccData[1]);
			UARTPrintString(",");

			//UARTPrintString("AccY: ");
			UARTPutHex(pAccData[2]);
			UARTPutHex(pAccData[3]);
			UARTPrintString(",");

			//UARTPrintString("AccZ: ");
			UARTPutHex(pAccData[4]);
			UARTPutHex(pAccData[5]);
			UARTPrintString(",");
			//UARTPrintString("\r\n");
		#endif

		#ifdef MAG_OUT_EN
			//UARTPrintString("MagX: ");
			UARTPutHex(pData[0]);
			UARTPutHex(pData[1]);
			UARTPrintString(",");

			//UARTPrintString("MagY: ");
			UARTPutHex(pData[2]);
			UARTPutHex(pData[3]);
			UARTPrintString(",");

			//UARTPrintString("MagZ: ");
			UARTPutHex(pData[4]);
			UARTPutHex(pData[5]);
			UARTPrintString(",");
			//UARTPrintString("\r\n");
		#endif

		#ifdef GYRO_OUT_EN
			//UARTPrintString("GyroX: ");
			UARTPutHex(pGyroData[1]);
			UARTPutHex(pGyroData[0]);
			UARTPrintString(",");

			//UARTPrintString("GyroY: ");
			UARTPutHex(pGyroData[3]);
			UARTPutHex(pGyroData[2]);
			UARTPrintString(",");

			//UARTPrintString("GyroZ: ");
			UARTPutHex(pGyroData[5]);
			UARTPutHex(pGyroData[4]);
			UARTPrintString("\r\n");

		#endif

	    _UxMD = 1;

		IdleMs(100);


    }//end while(1)
	return 0;
}//end main


/****************************************************************************
  Section: MCU Intialization functions
  ***************************************************************************/


/****************************************************************************
  Function:
    void InitIO(void)

  Summary:
    Initializes I/O pins.

  Description:
    Initializes I/O pins.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void InitIO(void)
{
	 LATA = 0;           //all io as output,o
	    LATB = 0;
	    TRISA = 0;
	    TRISB = 0;

	    ANSA = 0; //All digital
	    ANSB = 0; //All digital

	    // 1 is input - 0 is output

	    //I2C Port Configuration
	    SCL_TRIS = 1;       //SCL IH
	    SDA_TRIS = 1;       //SDA IH

	    //UART port configuration
	    UTX_LAT = 0;
	    UTX_TRIS = 0;       //U2TX OL
	    URX_TRIS = 1;       //U2RX IH


	    //Wireless Transmitter
		CSN1_LAT = 1;
		CSN1_TRIS = 0;

		CE1_LAT = 0;
		CE1_TRIS = 0;

		N24_INT_TRIS = 1;

	    SCK_TRIS = 0;
	    SCK_LAT = 0;
	    SDI_TRIS = 1;
	    SDO_TRIS = 0;
	    SDO_LAT = 0;

		//Sensors INT
		INT_M_TRIS = 1;

		INT_G_TRIS = 1;

		INT_A_TRIS = 1;


}//end InitIO


/****************************************************************************
  Function:
    void InitSystem(void)

  Summary:
    Initializes system after a reset.

  Description:
    Initializes default system configuration.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void InitSystem (void)
{
    //Disable unused peripherals
    PMD1 = 0xFFFF;
    PMD2 = 0xFFFF;
    PMD3 = 0xFDFF;  //Enable RTCC
    PMD4 = 0xFFFF;

    AD1CON1 = 0;
    AD1CON2 = 0;
    AD1CON3 = 0;
    AD1CON5 = 0;

    HLVDCON = 0; // Disable HLV

    CLKDIVbits.RCDIV = 0;                   //Set FRCDIV to 8 MHz operation
    __builtin_write_OSCCONL(OSCCON | 0x02); //Enable 32kHz OSC
    SRbits.IPL = 2;                         //CPU > 1, no lvl 1 ISRs occur

}


/****************************************************************************
  Section: Time delay functions
  ***************************************************************************/

/****************************************************************************
  Function:
    void IdleUs(int timeUs)

  Summary:
    Waits in Idle mode for specified time in uS.

  Description:
    Waits in Idle mode using Timer2 for specified time in uS.
    Maximum time of 32 mS.

  Precondition:
    None.

  Parameters:
    int timeUs - time to wait in microseconds (uS)

  Returns:
    None.
  ***************************************************************************/
void IdleUs(int timeUs)
{
    _T2MD = 0;

    //Set T2 to timeout in indicated # mS
    TMR2 = 0;
    PR2 = (GetInstructionClock()/1000000)*timeUs;       //1 uS = 2 * 500 nS period (4 MHz)

    //Enable interrupt for wakeup
    _T2IE = 1;
    _T2IF = 0;
    _T2IP = 1;
    SRbits.IPL = 2; //CPU > T2, no ISR occurs
    T2CON = 0x8000;  //T2 w/ 4 mHz clock

    //Idle until T2 interrupt occurs
    while(!_T2IF)
    {
        Idle();                 //Idle Demo 2
    }

    //Disable interrupt and timer2
    _T2IF = 0;
    _T2IE = 0;
    T2CON = 0;
    _T2MD = 1;
}//end IdleUs


/****************************************************************************
  Function:
    void IdleMs(int timeMs)

  Summary:
    Waits in Idle mode for specified time in mS.

  Description:
    Waits in Idle mode using Timer2 for specified time in mS.
    Maximum time of 2 S.

  Precondition:
    None.

  Parameters:
    int timeMs - time to wait in milliseconds (mS)

  Returns:
    None.
  ***************************************************************************/
void IdleMs(int timeMs)
{
    _T2MD = 0;
    T2CON = 0x0000;

    //Set T2 to timeout in indicated # mS
    TMR2 = 0;
    PR2 = (GetInstructionClock()/64000)*timeMs; //1 mS

    //Enable interrupt for wakeup
    _T2IF = 0;
    _T2IP = 1;
    _T2IE = 1;
    T2CON = 0x8020;  //T2 1:64 mode; 62500 period

    //Idle until T2 interrupt occurs
    while(!_T2IF)
    {
        Idle();
    }

    //Disable interrupt and timer2
    _T2IF = 0;
    _T2IE = 0;
    T2CON = 0;
    _T2MD = 1;

}//end IdleMs


/****************************************************************************
  Section: Interrupt Service Routines
  ***************************************************************************/

/****************************************************************************
  Function:
    void __attribute__ ((interrupt, shadow,auto_psv)) _RTCCInterrupt(void)

  Summary:
    RTCC interrupt handling routine.

  Description:
    Interrupt on RTCC alarm, indicate RTCC wakeup and Initiate new Sample.

  Precondition:
    InitRTCC();
    InitCalendar();

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void __attribute__ ((interrupt, shadow,auto_psv)) _RTCCInterrupt(void)
{
    IFS3bits.RTCIF = 0;     //Clear RTCC Alarm interrupt flag
    _RTCIE = 0;             //Disable RTCC interrupt until sleep

}


/****************************************************************************
  Function:
    void __attribute__ ((interrupt, shadow,auto_psv)) _T2Interrupt(void)

  Summary:


  Description:


  Precondition:


  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void __attribute__ ((interrupt,auto_psv)) _T2Interrupt()
{
    _T2IF = 0;  //Clear T2 interrupt flag
}


/****************************************************************************
  Function:
    void __attribute__ ((interrupt, shadow,auto_psv)) _INT0Interrupt(void)

  Summary:
    ISR for Pushbutton S2

  Description:
    ISR for S2 button press, Initiates button Processing and sensor Samples.

  Precondition:
    EnableS2()

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void __attribute__ ((interrupt, shadow,auto_psv)) _INT0Interrupt(void)
{
    _INT0IF = 0;    //Clear INT0 interrupt flag
    _RTCIE = 0;     //disable RTCC interrupt until sleep

}


/****************************************************************************
  Function:
    void __attribute__ ((interrupt, shadow,auto_psv)) _CNInterrupt(void)

  Summary:
    ISR for Pushbutton S3

  Description:
    ISR for S3 button press, Initiates button Processing.

  Precondition:
    EnableS3()

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void __attribute__ ((interrupt, shadow,auto_psv)) _CNInterrupt(void)
{
    _CNIF = 0;      //Clear CN interrupt flag
    _RTCIE = 0;     //disable RTCC interrupt until sleep


}


/****************************************************************************
  Function:
    void __attribute__ ((interrupt, shadow,auto_psv)) _T1Interrupt(void)

  Summary:
    ISR for CTMU tick timer

  Description:
    ISR for CTMU tick timer, Initiates cap touch Processing.

  Precondition:
    InitCap()
    TickInit()

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void  __attribute__((interrupt, shadow, auto_psv)) _T1Interrupt(void)
{
    //Clear flag
    IFS0bits.T1IF = 0;
    _RTCIE = 0;     //disable RTCC interrupt until sleep
}


