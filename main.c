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
#ifdef __PIC24F16KA301__
    // Setup configuration bits
    _FBS(BSS_OFF & BWRP_OFF)
    _FGS(GCP_OFF & GWRP_OFF)
    _FOSCSEL(FNOSC_FRCDIV & IESO_OFF)
    _FOSC(FCKSM_CSECMD & POSCFREQ_MS & OSCIOFNC_ON & POSCMOD_NONE & SOSCSEL_SOSCLP)
    _FWDT(FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768)
    _FPOR(MCLRE_ON & BORV_LPBOR & BOREN_BOR3 & I2C1SEL_PRI & PWRTEN_OFF)
    _FICD(BKBUG_OFF & ICS_PGx3)
    _FDS(DSWDTEN_OFF & DSBOREN_ON & RTCOSC_SOSC & DSWDTOSC_SOSC & DSWDTPS_DSWDTPSF) //DSWDT SOSC = LPRC
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
   
#define MAG_ADDRESS      0x3C
#define GYRO_ADDRESS      0xD0
#define ACC_ADDRESS      0x38


#define I2C_WRITE           0x00
#define I2C_READ            0x01

#define HMC5883_ModeRegisterAddress 0x02
#define HMC5883_ContinuousModeCommand 0x00

#define HMC5883_ConfARegisterAddress 0x00
#define HMC5883_ConfBRegisterAddress 0x01
#define HMC5883_75HzCommand 0x18

#define L3G4200_CTRL_REG1 0x20
#define L3G4200_CTRL_REG1_SET 0x7F

#define L3G4200_CTRL_REG2 0x21
#define L3G4200_CTRL_REG2_SET 0x20

#define L3G4200_CTRL_REG3 0x22
#define L3G4200_CTRL_REG3_SET 0x00

#define L3G4200_CTRL_REG4 0x23
#define L3G4200_CTRL_REG4_SET 0x10

#define L3G4200_CTRL_REG5 0x24
#define L3G4200_CTRL_REG5_SET 0x00

#define L3G4200_OUT_TEMP 0x26
#define L3G4200_CONTINUOUS_READ 0x80
#define L3G4200_X_L 0x28

#define L3G4200_FIFO_CTRL_REG 0x2E
#define L3G4200_FIFO_CTRL_REG_SET 0x00


#define MMA8451Q_F_SETUP 0x09
#define MMA8451Q_F_SETUP_SET 0x00

#define MMA8451Q_TRIG_CFG 0x0A
#define MMA8451Q_TRIG_CFG_SET 0x00

#define MMA8451Q_XYZ_DATA_CFG 0x0E
#define MMA8451Q_XYZ_DATA_CFG_SET 0x02

#define MMA8451Q_FF_MT_CFG 0x15
#define MMA8451Q_FF_MT_CFG_SET 0x78

#define MMA8451Q_FF_MT_THS 0x17
#define MMA8451Q_FF_MT_THS_SET 0x00

#define MMA8451Q_CTRL_REG1 0x2A
#define MMA8451Q_CTRL_REG1_SET 0x01

#define MMA8451Q_CTRL_REG2 0x2B
#define MMA8451Q_CTRL_REG2_SET 0x02

#define MMA8451Q_CTRL_REG3 0x2C
#define MMA8451Q_CTRL_REG3_SET 0x00

#define MMA8451Q_CTRL_REG4 0x2D
#define MMA8451Q_CTRL_REG4_SET 0x00

#define MMA8451Q_CTRL_REG5 0x2E
#define MMA8451Q_CTRL_REG5_SET 0x00

#define GYRO_OUT_EN
#define MAG_OUT_EN
#define ACC_OUT_EN

/****************************************************************************
  Section: Global Variables
  ***************************************************************************/
//RTCCFORM rtcc;          //RTCC read/write structure

WORD eeAddress = 0;     //EEPROM address pointer
WORD potVal;            //POT reading in mV
WORD tempVal;           //Temperature in 0.1*C
WORD vddVal;            //Board VDD level

WORD pressedCT1 = 0;    //Cap touch button 1 pressed flag
WORD pressedCT2 = 0;    //Cap touch button 2 pressed flag
WORD pressedCT3 = 0;    //Cap touch button 3 pressed flag

LP_MODES lpMode;        //Current low power operating mode
DEMO_TASKS tasks;       //Tasks to execute flags




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

    WORD_VAL modes;

    InitIO();      //configure I/O
    InitSystem();  //Initial system Setup
    HandleReset(); //Process reset events (POR/DPSLP)

    IdleMs(500);
    
	InitI2C();
	
	IdleMs(500);

	SetupGyro();
	SetupMag();
    SetupAcc();
		
	int i;

	BYTE dataVal[6];
    BYTE *pData;
    pData = dataVal;

	BYTE gyroDataVal[6];
   	BYTE *pGyroData;
   	pGyroData = gyroDataVal;
   	
   	BYTE accDataVal[6];
   	BYTE *pAccData;
   	pAccData = accDataVal;

	while(1)
    {	
		//Send a start bit
	    I2C1CONbits.SEN = 1;
	    I2CWait();     //wait for interrupt
		
	    for(i = 0; i<6;i++)
	    {
			//Transmit the I2C address byte
		    I2C1TRN = MAG_ADDRESS | I2C_WRITE;
		    I2CWait();     //wait for interrupt
			I2C1TRN = i+3;
		    I2CWait();     //wait for interrupt
	        
			//Send a restart bit
	   		I2C1CONbits.RSEN = 1;
	    	I2CWait();     //wait for interrupt
		
			I2C1TRN = MAG_ADDRESS | I2C_READ;
		    I2CWait();     //wait for interrupt

			//Receive a data byte
	        I2C1CONbits.RCEN = 1;
	        I2CWait();     //wait for interrupt
	        //Read data
	        pData[i] = I2C1RCV;
	
	        I2C1CONbits.RSEN = 1;
	        I2CWait();     //wait for interrupt
	    }
		
	    //Send the stop bit, ending this session
	    I2C1CONbits.PEN = 1;
	    I2CWait();     //wait for interrupt
	
		
		//********************************************************************************************
		//Gyroscope code
	
		/*
	    //Send a start bit
	    I2C1CONbits.SEN = 1;
	    I2CWait();     //wait for interrupt
	
		//Transmit the I2C address byte
	    I2C1TRN = GYRO_ADDRESS | I2C_WRITE;
	    I2CWait();     //wait for interrupt
	
	    //Transmit data address
	    I2C1TRN = (L3G4200_X_L | L3G4200_CONTINUOUS_READ);
	    I2CWait();     //wait for interrupt
	
	    //Send a restart bit
	    I2C1CONbits.RSEN = 1;
	    I2CWait();     //wait for interrupt
		*/	
		
		/*
		//Transmit the I2C address byte
		I2C1TRN = GYRO_ADDRESS | I2C_READ;
		I2CWait();     //wait for interrupt		
		*/
	
		//Send a start bit
	    I2C1CONbits.SEN = 1;
	    I2CWait();     //wait for interrupt

	    //Read data packet
	    for(i = 0; i<6;i++)
	    {
			//Transmit the I2C address byte
		    I2C1TRN = GYRO_ADDRESS | I2C_WRITE;
		    I2CWait();     //wait for interrupt
			I2C1TRN = i+L3G4200_X_L;
		    I2CWait();     //wait for interrupt
	        
			//Send a restart bit
	   		I2C1CONbits.RSEN = 1;
	    	I2CWait();     //wait for interrupt
		
			I2C1TRN = GYRO_ADDRESS | I2C_READ;
		    I2CWait();     //wait for interrupt

			//Receive a data byte
	        I2C1CONbits.RCEN = 1;
	        I2CWait();     //wait for interrupt
	        //Read data
	        pGyroData[i] = I2C1RCV;
	
	        I2C1CONbits.RSEN = 1;
	        I2CWait();     //wait for interrupt

			/*
			//Receive a data byte
	        I2C1CONbits.RCEN = 1;
	        I2CWait();     //wait for interrupt

	        //Read data
	        pGyroData[i] = I2C1RCV;
	
	        //Transmit NACK on last byte, ACK on others
	        if(i==6-1)
	        {
	            I2C1CONbits.ACKDT = 1;
	        }
	        else
	        {
	            I2C1CONbits.ACKDT = 0;
	        }
	
	        I2C1CONbits.ACKEN = 1;
	        I2CWait();     //wait for interrupt
			*/
			
	    }
	
	    //Send the stop bit, ending this session
	    I2C1CONbits.PEN = 1;
	    I2CWait();     //wait for interrupt

		
		//*****************************************************
		//Acc Code
		
		//Send a start bit
	    I2C1CONbits.SEN = 1;
	    I2CWait();     //wait for interrupt
		
	    for(i = 0; i<6;i++)
	    {
			//Transmit the I2C address byte
		    I2C1TRN = ACC_ADDRESS | I2C_WRITE;
		    I2CWait();     //wait for interrupt
			I2C1TRN = i+1;
		    I2CWait();     //wait for interrupt
	        
			//Send a restart bit
	   		I2C1CONbits.RSEN = 1;
	    	I2CWait();     //wait for interrupt
		
			I2C1TRN = ACC_ADDRESS | I2C_READ;
		    I2CWait();     //wait for interrupt

			//Receive a data byte
	        I2C1CONbits.RCEN = 1;
	        I2CWait();     //wait for interrupt
	        //Read data
	        pAccData[i] = I2C1RCV;
	
	        I2C1CONbits.RSEN = 1;
	        I2CWait();     //wait for interrupt
	    }
		
	    //Send the stop bit, ending this session
	    I2C1CONbits.PEN = 1;
	    I2CWait();     //wait for interrupt
		
		
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
}//end main


void SetupGyro(void) {
	configGyro(L3G4200_CTRL_REG1, L3G4200_CTRL_REG1_SET);
	configGyro(L3G4200_CTRL_REG2, L3G4200_CTRL_REG2_SET);
	configGyro(L3G4200_CTRL_REG3, L3G4200_CTRL_REG3_SET);
	configGyro(L3G4200_CTRL_REG4, L3G4200_CTRL_REG4_SET);
	configGyro(L3G4200_CTRL_REG5, L3G4200_CTRL_REG5_SET);
	configGyro(L3G4200_FIFO_CTRL_REG, L3G4200_FIFO_CTRL_REG_SET);
}

void SetupAcc(void) {
	configAcc(MMA8451Q_F_SETUP, MMA8451Q_F_SETUP_SET);
	configAcc(MMA8451Q_TRIG_CFG, MMA8451Q_TRIG_CFG_SET);
	configAcc(MMA8451Q_XYZ_DATA_CFG, MMA8451Q_XYZ_DATA_CFG_SET);
	configAcc(MMA8451Q_FF_MT_CFG, MMA8451Q_FF_MT_CFG_SET);
	configAcc(MMA8451Q_FF_MT_THS, MMA8451Q_FF_MT_THS_SET);
	
	configAcc(MMA8451Q_CTRL_REG1, MMA8451Q_CTRL_REG1_SET);
    configAcc(MMA8451Q_CTRL_REG2, MMA8451Q_CTRL_REG2_SET);	
    configAcc(MMA8451Q_CTRL_REG3, MMA8451Q_CTRL_REG3_SET);
    configAcc(MMA8451Q_CTRL_REG4, MMA8451Q_CTRL_REG4_SET);
    configAcc(MMA8451Q_CTRL_REG5, MMA8451Q_CTRL_REG5_SET);

}
	
void SetupMag(void) {
	//Send a start bit
    I2C1CONbits.SEN = 1;
    I2CWait();     //wait for interrupt

	//Transmit the I2C address byte
    I2C1TRN = MAG_ADDRESS | I2C_WRITE;
    I2CWait();     //wait for interrupt

    //Transmit data address
    I2C1TRN = HMC5883_ConfARegisterAddress;
    I2CWait();     //wait for interrupt
    I2C1TRN = HMC5883_75HzCommand;
    I2CWait();     //wait for interrupt

    //Send a restart bit
    I2C1CONbits.RSEN = 1;
    I2CWait();     //wait for interrupt

    //Transmit the I2C address byte
    I2C1TRN = MAG_ADDRESS | I2C_WRITE;
    I2CWait();     //wait for interrupt

    //Transmit data address
    I2C1TRN = HMC5883_ModeRegisterAddress;
    I2CWait();     //wait for interrupt
    I2C1TRN = HMC5883_ContinuousModeCommand;
    I2CWait();     //wait for interrupt

	//Send the stop bit, ending this session
    I2C1CONbits.PEN = 1;
    I2CWait();     //wait for interrupt
}

void configAcc(BYTE subaddr, BYTE value) {
    //Send a start bit
    I2C1CONbits.SEN = 1;
    I2CWait();     //wait for interrupt

    //Transmit the I2C address byte
    I2C1TRN = ACC_ADDRESS | I2C_WRITE;
    I2CWait();     //wait for interrupt

    //Transmit data address
    I2C1TRN = subaddr;
    I2CWait();     //wait for interrupt
    I2C1TRN = value;
    I2CWait();     //wait for interrupt

    //Send a stop bit
    I2C1CONbits.PEN = 1;
    I2CWait();     //wait for interrupt
}    

void configGyro(BYTE subaddr, BYTE value) {
	//Send a start bit
    I2C1CONbits.SEN = 1;
    I2CWait();     //wait for interrupt

    //Transmit the I2C address byte
    I2C1TRN = GYRO_ADDRESS | I2C_WRITE;
    I2CWait();     //wait for interrupt

    //Transmit data address
    I2C1TRN = subaddr;
    I2CWait();     //wait for interrupt
    I2C1TRN = value;
    I2CWait();     //wait for interrupt

    //Send a stop bit
    I2C1CONbits.PEN = 1;
    I2CWait();     //wait for interrupt

}


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

    AD1PCFG = 0xFFFF;   //all I/O as digital

    POT_AN = 0;         //POT as analog input
    TEMP_AN = 0;        //Temp sensor as analog input
    POT_TRIS = 1; 
    TEMP_TRIS = 1;


    PWRCTL_TRIS = 0;    //ICPWR OL
    #ifdef USE_PWRCTL
    mPWR_OFF();
    #endif

    TEMP_TRIS = 1;      //AN1 TEMP IN
    POT_TRIS = 1;       //HLVD/POT IN

    SCL_TRIS = 1;       //SCL IH
    SDA_TRIS = 1;       //SDA IH

    mSetLED2Tris(1);
    mSetLED3Tris(1);

    UTX_LAT = 0;
    UTX_TRIS = 0;       //U2TX OL
    URX_TRIS = 1;       //U2RX IH

    #ifdef __PIC24FJ64GA102__
        RPINR19bits.U2RXR = 1;
        RPOR0bits.RP0R = 5;
    #endif

	#ifdef USE_HARVESTER
        #ifdef CYMBET_EVAL08
        CHARGE_TRIS = 0;    //EVAL-08 Cymbet documentation - drive Charge low until time to read
        CHARGE_LAT = 0;
        BATOFF_TRIS = 0;    //BATOFF is output, drive low to enable batteries
        BATOFF_LAT = 0;
        #elif defined(IPS_XLP_BOARD)
        REG_EN_TRIS = 0;    //Set REG_EN and UVP_EN lines to output
        UVP_EN_TRIS = 0;
        REG_EN_LAT = 1;     //enable Under voltage Protection (UVP) and 3.3V Regulator		
        UVP_EN_LAT = 1;
        #endif
	#endif

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

    CLKDIVbits.RCDIV = 0;                   //Set FRCDIV to 8 MHz operation
    __builtin_write_OSCCONL(OSCCON | 0x02); //Enable 32kHz OSC
    SRbits.IPL = 2;                         //CPU > 1, no lvl 1 ISRs occur

    tasks.Val = 0;                          //intialize device mode
    tasks.bits.transmit = DEFAULT_TX;       //Set UART TX default state
	
}


/****************************************************************************
  Function:
    void HandleReset(void)

  Summary:
    Handles Initialization based on reset condition

  Description:
    Handles device reset and Initializes RTCC.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.
  ***************************************************************************/
void HandleReset(void)
{
    WORD_VAL mode;

    #ifdef IPS_XLP_BOARD
    /*****************************************************
    * Delay for Regulator to settle
    * Note: This delay of 2ms is required befor any 
    *  peripheral devices using the same Supply line
    *  as the microcontroller can be activated to 
    *  ensure proper functionality.
    *****************************************************/
    IdleMs(2);
    #endif

    //Did wake up from Deep Sleep?
    if(RCONbits.DPSLP)
    {
        //wake up from Deep Sleep
        eeAddress = DSGPR0;
        mode.Val = DSGPR1;
        lpMode = mode.v[1];
        tasks.Val = mode.v[0];

        RCONbits.DPSLP = 0;
        DSCONbits.RELEASE = 0;      //release control and data bits for all I/Os

        if(tasks.bits.transmit)
        {
            #ifdef USE_PWRCTL
            mPWR_ON();
            IdleMs(UART_WAIT);
            #endif
            _UxMD = 0;
            UARTInit();
            UARTPrintString("Wake from Deep Sleep...");
        }

        switch(ALARM_PERIOD)
        {
            case ONE_SECOND:
                SetRTCCAlarm(1);        //Alarm Every 1 second
                break;
            case TEN_SECOND:
                SetRTCCAlarm(2);        //Alarm Every 10 seconds
                break;
            default:
                break;
        }     

        if(DSWAKEbits.DSINT0 != 0)
        {
            if(tasks.bits.transmit)
            {
                UARTPrintString("  INT0\r\n");
            }

            tasks.bits.button = 1;
        }

        if(DSWAKEbits.DSFLT != 0)
        {
            if(tasks.bits.transmit)
            {
                UARTPrintString("  DSFLT\r\n");
            }
        }

        if(DSWAKEbits.DSWDT != 0)
        {
            if(tasks.bits.transmit)
            {
                UARTPrintString("  DSWDTT\r\n");
            }
        }

        if(DSWAKEbits.DSRTC != 0)
        {
            if(tasks.bits.transmit)
            {
                 UARTPrintString("  RTCC\r\n");
            }

            tasks.bits.sample = 1;
        }

        if(DSWAKEbits.DSMCLR != 0)
        {
            if(tasks.bits.transmit)
            {
                UARTPrintString("  MCLR\r\n");
            }
        }

        if(DSWAKEbits.DSPOR != 0)  //POR
        {
            if(tasks.bits.transmit)
            {
                UARTPrintString("  POR\r\n");
            }
        }
    }
    else     //Or is this Initial power on or sleep wakeup
    {

        #ifdef USE_PWRCTL
        mPWR_ON();
        IdleMs(UART_WAIT);
        #endif

        if(RCONbits.POR)
        {
            InitRTCC();                 //Initialize RTCC
			
			#ifdef USE_HARVESTER
            InitHarvester();            //Initialize harvester battery level detection
			#endif

            if(tasks.bits.transmit)     //Transmit to PC if TX en
            {
                _UxMD = 0;
                UARTInit();
                UARTPrintString("Microchip Technology Inc......nanoWatt XLP board \r\n");
                UARTPrintString("Click S2: Forces a Sample to be taken based on the currently sensor \r\n");
                UARTPrintString("Click S3: Toggles UART on/off. Output of data and the current device state \r\n");
				
				#ifndef USE_HARVESTER
                UARTPrintString("Hold S2 over 2 sec: switches sensor type\r\n");
                UARTPrintString("Hold S3 over 2 sec: switches low power mode \r\n");
				#endif
				
				UARTPrintString("Hold S2 on reset to clear RTCC and EEPROM data \r\n");
                UARTPrintString("Hold S3 on reset to transmit logged data to PC \r\n");

            }
        }

        if(RCONbits.EXTR)               //MCLR Reset
        {
		
			#ifdef USE_HARVESTER
            InitI2C();
            RestoreHarvesterState();    //Read harvester state from EEPROM after reset
			#endif

            if(tasks.bits.transmit)     //Transmit to PC if TX en
            {
                _UxMD = 0;
                UARTInit();
                IdleMs(UART_WAIT);
                UARTPrintString("MCLR reset occurred.\r\n\r\n");
            }
        }
    
        SW2_TRIS = 1;
        SW2_PULLUP = 1;                 //Enable INT0 pullup
        SW3_TRIS = 1;
        SW3_PULLUP = 1;                 //Enable RB14 pullup
        IdleUs(5);                      //allow I/O to transition high after enabling pullup

        if(SW2_PORT == 0)               //SW2 held on startup, reset eeAddress & RTCC
        {                               
            SW2_PULLUP = 0;             //disable pullup to save current
            SW3_PULLUP = 0;             //disable pullup to save current

            eeAddress = 0x10;           //Reset eeAddress & RTCC
            InitCalendar();

            if(tasks.bits.transmit)
            {
                _UxMD = 0;
                UARTInit();
                UARTPrintString("***Datalog reset!***\n\r");
            }
        }
        else if(SW3_PORT == 0)          //SW3 held on startup, transmit datalog
        {
            SW2_PULLUP = 0;             //disable pullup to save current
            SW3_PULLUP = 0;             //disable pullup to save current

            InitI2C();                  
            GetAddress();               //Enable I2C and read Stored eeAddress & RTCC
            GetTimestamp((eeAddress>0x10)?(eeAddress-0x10):0x10); //Get previous time

            //Transmit entire EEPROM contents
            _UxMD = 0;
            UARTInit();
            UARTPrintString("Transmitting datalog:\n\r");
            TransmitEEPROM(0x10,eeAddress);
            UARTPrintString("Datalog Transmit done!\n\r");

        }
        else                            //Nothing pressed, normal startup: load eeAddress and RTCC info from EEPROM
        {
            SW2_PULLUP = 0;             //disable pullup to save current
            SW3_PULLUP = 0;             //disable pullup to save current

            InitI2C();                  //Enable I2C and read Stored eeAddress & RTCC
            GetAddress();
            GetTimestamp((eeAddress>0x10)?(eeAddress-0x10):0x10); //Get previous time
        }

        switch(ALARM_PERIOD)
        {
            case ONE_SECOND:
                SetRTCCAlarm(1);        //Alarm Every 1 second
                break;
            case TEN_SECOND:
                SetRTCCAlarm(2);        //Alarm Every 10 seconds
                break;
            default:
                break;
        }        
		
        lpMode = DEFAULT_MODE;          //Set Initial LP and sensor modes
        tasks.bits.mode = DEFAULT_SENSOR;

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

            InitCapSense();             //Init cap sensing
            CapSenseTickInit();         //Enable Timer1 for cap button detecting
        }
        #endif

        tasks.bits.sample = 1;          //Take & Store first Sample
        tasks.bits.store = 1;

        RCON=0;                         //Clear reset flags

    }//end if(RCONbits.DPSLP)

    #ifdef USE_PWRCTL
    mPWR_OFF();     //Disable external circuits and UART to save power
    #endif
    _UxMD = 1;
}//end HandleReset



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

    if(tasks.bits.transmit) //Send wakeup source information
    {
        _UxMD = 0;
        UARTInit();
        IdleUs(5);  //Allow UART to stabilize to prevent glitch on first byte
        UARTPrintString("Wake from RTCC\r\n");
        _UxMD = 1;
    }

    tasks.bits.alarm = 1;
    tasks.bits.sample = 1;
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

    tasks.bits.button = 1;
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

    tasks.bits.button = 1;

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

    tasks.bits.cap = 1;
}


