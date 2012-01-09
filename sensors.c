#include "system.h"


void ReadMag(BYTE *pData) {
	int i;
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

}

void ReadAcc(BYTE *pAccData) {
	int i;
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
}

void ReadGyro(BYTE *pGyroData) {
	int i;
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


	}

	//Send the stop bit, ending this session
	I2C1CONbits.PEN = 1;
	I2CWait();     //wait for interrupt
}


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
