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

void ReadGyro(BYTE *pGyroData);
void ReadMag(BYTE *pMagData);
void ReadAcc(BYTE *pAccData);

void configGyro(BYTE subaddr, BYTE value);
void configAcc(BYTE subaddr, BYTE value);

void SetupGyro(void);
void SetupMag(void);
void SetupAcc(void);


#endif
