/********************************************************************
* FileName:     HardwareProfile.h
* Dependencies: none
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
* Brant Ivey       4/1/10       Modified to add Harvester support.
*                               General bug fixes.
*****************************************************************************/

#ifndef _HARDWARE_PROFILE_H
#define _HARDWARE_PROFILE_H

//Pins not defined because they are unused in demo firmware:
//RA6 -> PWM for pictal
//RA2 -> OSCI
//RA3 -> OSCO
//RA7 -> PIC24FJ device DISVREG pin

#ifdef __PIC24F16KA301__

    #define UARTNUM 1   //use UART1
    #define DSWAKEbits  DSWSRCbits    //workaround for compiler support
    #define DSRTC       DSRTCC

    //I2C
    #define SDA_LAT     LATBbits.LATB9
    #define SDA_TRIS    TRISBbits.TRISB9
    #define SCL_LAT     LATBbits.LATB8
    #define SCL_TRIS    TRISBbits.TRISB8

    //SPI
    #define SCK_LAT     LATBbits.LATB12
    #define SCK_TRIS    TRISBbits.TRISB12
    #define SDI_LAT     LATBbits.LATB14
    #define SDI_TRIS    TRISBbits.TRISB14
    #define SDO_LAT     LATBbits.LATB13
    #define SDO_TRIS    TRISBbits.TRISB13

	#define CSN1_LAT     LATAbits.LATA6
    #define CSN1_TRIS    TRISAbits.TRISA6

	#define CSN2_LAT     LATBbits.LATB15
    #define CSN2_TRIS    TRISBbits.TRISB15

	#define CE1_LAT     LATBbits.LATB0
    #define CE1_TRIS    TRISBbits.TRISB0

	#define CE2_LAT     LATBbits.LATB1
    #define CE2_TRIS    TRISBbits.TRISB1

    //UART
    #define URX_LAT     LATBbits.LATB2
    #define URX_TRIS    TRISBbits.TRISB2
    #define UTX_LAT     LATBbits.LATB7
    #define UTX_TRIS    TRISBbits.TRISB7

#elif defined __PIC24F16KA102__

    #define UARTNUM 2   //use UART2
    #define DSWAKEbits  DSWSRCbits    //workaround for compiler support
    #define DSRTC       DSRTCC

    //Pwr Control FET output
    #define PWRCTL_LAT  LATBbits.LATB2
    #define PWRCTL_TRIS TRISBbits.TRISB2
    #define PWRCTL_PORT PORTBbits.RB2
    #define mPWR_ON()   PWRCTL_LAT = 0; PWRCTL_TRIS = 0;
    #define mPWR_OFF()  PWRCTL_LAT = 1; PWRCTL_TRIS = 0;

    //HLVD/POT AN input
    #define POT_LAT     LATBbits.LATB12
    #define POT_TRIS    TRISBbits.TRISB12
    #define POT_AN      AD1PCFGbits.PCFG12
    #define POT_CHANNEL 12

    //Temp sense AN input
    #define TEMP_LAT    LATAbits.LATA1
    #define TEMP_TRIS   TRISAbits.TRISA1
    #define TEMP_AN     AD1PCFGbits.PCFG1
    #define TEMP_CHANNEL 1

    //Cap touch button AN inputs
    #define CT1_LAT     LATBbits.LATB3
    #define CT1_TRIS    TRISBbits.TRISB3
    #define CT1_AN      AD1PCFGbits.PCFG5
    #define CT1_CHANNEL 5
    #define CT2_LAT     LATAbits.LATA1
    #define CT2_TRIS    TRISAbits.TRISA1
    #define CT2_AN      AD1PCFGbits.PCFG1
    #define CT2_CHANNEL 1
    #define CT3_LAT     LATAbits.LATA0
    #define CT3_TRIS    TRISAbits.TRISA0
    #define CT3_AN      AD1PCFGbits.PCFG0
    #define CT3_CHANNEL 0

    //Cap 3 AN input
    #define CAP3_LAT    LATAbits.LATA0
    #define CAP3_TRIS   TRISAbits.TRISA0
    #define CAP3_AN     AD1PCFGbits.PCFG0
    #define CAP3_CHANNEL 0

    //Switch dig inputs
    #define SW2_LAT     LATBbits.LATB7
    #define SW2_TRIS    TRISBbits.TRISB7
    #define SW2_PORT    PORTBbits.RB7
    #define SW2_PULLUP  CNPU2bits.CN23PUE
    #define SW3_LAT     LATBbits.LATB14
    #define SW3_TRIS    TRISBbits.TRISB14
    #define SW3_PORT    PORTBbits.RB14
    #define SW3_PULLUP  CNPU1bits.CN12PUE
    #define _SW3IE      CNEN1bits.CN12IE

    //LED dig outputs
    #define LED2_LAT    LATBbits.LATB8
    #define LED2_TRIS   TRISBbits.TRISB8
    #define mSetLED2(a) LED2_LAT = a
    #define mSetLED2Tris(a) LED2_TRIS = a
    #define LED3_LAT    LATBbits.LATB15
    #define LED3_TRIS   TRISBbits.TRISB15
    #define mSetLED3(a) LED3_LAT = a
    #define mSetLED3Tris(a) LED3_TRIS = a

    //I2C
    #define SDA_LAT     LATBbits.LATB9
    #define SDA_TRIS    TRISBbits.TRISB9
    #define SCL_LAT     LATBbits.LATB8
    #define SCL_TRIS    TRISBbits.TRISB8

    //SPI
    #define SCK_LAT     LATBbits.LATB11
    #define SCK_TRIS    TRISBbits.TRISB11
    #define SDI_LAT     LATBbits.LATB10
    #define SDI_TRIS    TRISBbits.TRISB10
    #define SDO_LAT     LATBbits.LATB13
    #define SDO_TRIS    TRISBbits.TRISB13

    //UART
    #define URX_LAT     LATBbits.LATB1
    #define URX_TRIS    TRISBbits.TRISB1
    #define UTX_LAT     LATBbits.LATB0
    #define UTX_TRIS    TRISBbits.TRISB0

    //Cymbet EVAL-08 EH header
#ifdef CYMBET_EVAL08
    #define BATOFF_LAT  LATBbits.LATB6
    #define BATOFF_TRIS TRISBbits.TRISB6
    #define CHARGE_LAT  LATBbits.LATB5
    #define CHARGE_TRIS TRISBbits.TRISB5
    #define CHARGE_PORT PORTBbits.RB5
#endif 
   
    //IPS XLP EH header
#ifdef IPS_XLP_BOARD
	#define REG_EN_TRIS	TRISBbits.TRISB6  //3.3V regulator enable mask
	#define REG_EN_LAT  LATBbits.LATB6  
	#define UVP_EN_TRIS	TRISBbits.TRISB5  //Under Voltage Protection enable mask
	#define UVP_EN_LAT  LATBbits.LATB5      
#endif

    //Power Measure header
    #define SYNC_LAT    LATBbits.LATB5
    #define SYNC_TRIS   TRISBbits.TRISB5


#elif defined(__PIC24FJ64GA102__)
 
    #define UARTNUM 2   //use UART2

    //Pwr Control FET output
    #define PWRCTL_LAT  LATBbits.LATB2
    #define PWRCTL_TRIS TRISBbits.TRISB2
    #define PWRCTL_PORT PORTBbits.RB2
    #define mPWR_ON()   PWRCTL_LAT = 0; PWRCTL_TRIS = 0;
    #define mPWR_OFF()  PWRCTL_LAT = 1; PWRCTL_TRIS = 0;

    //HLVD/POT AN input
    #define POT_LAT     LATBbits.LATB12
    #define POT_TRIS    TRISBbits.TRISB12
    #define POT_AN      AD1PCFGbits.PCFG12
    #define POT_CHANNEL 12

    //Temp sense AN input
    #define TEMP_LAT    LATAbits.LATA1
    #define TEMP_TRIS   TRISAbits.TRISA1
    #define TEMP_AN     AD1PCFGbits.PCFG1
    #define TEMP_CHANNEL 1

    //Cap touch button AN inputs
    #define CT1_LAT     LATBbits.LATB3
    #define CT1_TRIS    TRISBbits.TRISB3
    #define CT1_AN      AD1PCFGbits.PCFG5
    #define CT1_CHANNEL 5
    #define CT2_LAT     LATAbits.LATA1
    #define CT2_TRIS    TRISAbits.TRISA1
    #define CT2_AN      AD1PCFGbits.PCFG1
    #define CT2_CHANNEL 1
    #define CT3_LAT     LATAbits.LATA0
    #define CT3_TRIS    TRISAbits.TRISA0
    #define CT3_AN      AD1PCFGbits.PCFG0
    #define CT3_CHANNEL 0

    //Cap 3 AN input
    #define CAP3_LAT    LATAbits.LATA0
    #define CAP3_TRIS   TRISAbits.TRISA0
    #define CAP3_AN     AD1PCFGbits.PCFG0
    #define CAP3_CHANNEL 0

    //Switch dig inputs
    #define SW2_LAT     LATBbits.LATB7
    #define SW2_TRIS    TRISBbits.TRISB7
    #define SW2_PORT    PORTBbits.RB7
    #define SW2_PULLUP  CNPU2bits.CN23PUE
    #define SW3_LAT     LATBbits.LATB14
    #define SW3_TRIS    TRISBbits.TRISB14
    #define SW3_PORT    PORTBbits.RB14
    #define SW3_PULLUP  CNPU1bits.CN12PUE
    #define _SW3IE      CNEN1bits.CN12IE

    //LED dig outputs
    #define LED2_LAT    LATBbits.LATB8
    #define LED2_TRIS   TRISBbits.TRISB8
    #define mSetLED2(a) LED2_LAT = a
    #define mSetLED2Tris(a) LED2_TRIS = a
    #define LED3_LAT    //NC
    #define LED3_TRIS   //NC
    #define mSetLED3(a)  
    #define mSetLED3Tris(a)

    //I2C
    #define SDA_LAT     LATBbits.LATB9
    #define SDA_TRIS    TRISBbits.TRISB9
    #define SCL_LAT     LATBbits.LATB8
    #define SCL_TRIS    TRISBbits.TRISB8

    //SPI
    #define SCK_LAT     LATBbits.LATB11
    #define SCK_TRIS    TRISBbits.TRISB11
    #define SDI_LAT     LATBbits.LATB10
    #define SDI_TRIS    TRISBbits.TRISB10
    #define SDO_LAT     LATBbits.LATB13
    #define SDO_TRIS    TRISBbits.TRISB13

    //UART
    #define URX_LAT     LATBbits.LATB1
    #define URX_TRIS    TRISBbits.TRISB1
    #define UTX_LAT     LATBbits.LATB0
    #define UTX_TRIS    TRISBbits.TRISB0

    //Cymbet EVAL-08 EH header
#ifdef CYMBET_EVAL08
    #define BATOFF_LAT  LATBbits.LATB6
    #define BATOFF_TRIS TRISBbits.TRISB6
    #define CHARGE_LAT  LATBbits.LATB5
    #define CHARGE_TRIS TRISBbits.TRISB5
    #define CHARGE_PORT PORTBbits.RB5
#endif 
   
    //IPS XLP EH header
#ifdef IPS_XLP_BOARD
	#define REG_EN_TRIS	TRISBbits.TRISB6  //3.3V regulator enable mask
	#define REG_EN_LAT  LATBbits.LATB6  
	#define UVP_EN_TRIS	TRISBbits.TRISB5  //Under Voltage Protection enable mask
	#define UVP_EN_LAT  LATBbits.LATB5      
#endif

    //Power Measure header
    #define SYNC_LAT  LATBbits.LATB5
    #define SYNC_TRIS TRISBbits.TRISB5

#endif


#endif //end ifdef _HARDWARE_PROFILE_H

