#include "ENERGY_METER.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

/**
	* @brief Read/Write to the specified register
	* @param 
	* @retval
	*/
unsigned int RWtoRegister(unsigned char dataRW, unsigned short address, unsigned short data){
	
	uint8_t spiDataTx[4];
	uint8_t spiDataRx[2];
	unsigned short checksum;
	
	// Read Data
	if(dataRW){
		
		// Put CS low - Activate
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		// Transmit register address
		spiDataTx[0] = 0x80; 		//Read (1)
		spiDataTx[1] = address;
		HAL_SPI_Transmit(&hspi1, spiDataTx, 2, 10);
		// Read
		HAL_SPI_Receive(&hspi1, spiDataRx, 2, 10);
		// Bring CS high -  Deactivate
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
		
		checksum = (spiDataRx[0] << 8) | (spiDataRx[1] & 0xff);
		return checksum;
	}
	// Write Data
	else{
		
		//1. Bring the slave select low
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		//2. Transmit register + data
		spiDataTx[0] = 0x00; 						//Write (0)
		spiDataTx[1] = address; 				//Reg address
		spiDataTx[2] = data >> 8; 			//dataH
		spiDataTx[3] = data & 0xFFFF;   //dataL
		HAL_SPI_Transmit(&hspi1, spiDataTx, 4, 10);
		//4. Bring the slave select high
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	}
	return 0; //error
}

/**
	* @brief 
	* @param 
	* @retval
	*/
void init(void){
	RWtoRegister(WRITE, CfgRegAccEn, 0x55AA);
}

/* BEGIN FUNCTION */
/*
  - Define the pin to be used as Chip Select
  - Set serialFlag to true for serial debugging
  - Use SPI MODE 0 for the ATM90E32
*/
/**
	* @brief 
	* @param 
	* @retval
	*/
void M90E32AS_Init(int pin, unsigned short lineFreq, unsigned short pgagain, unsigned short ugain, unsigned short igainA, unsigned short igainB, unsigned short igainC){
  /* Enable SPI */

	
  //calculation for voltage sag threshold - assumes we do not want to go under 90v for split phase and 190v otherwise
  //determine proper low and high frequency threshold
  unsigned short vSagTh;
  unsigned short sagV;
  unsigned short FreqHiThresh;
  unsigned short FreqLoThresh;
  if (lineFreq == 4485 || lineFreq == 5231)
  {
    sagV = 90;
    FreqHiThresh = 61 * 100;
    FreqLoThresh = 59 * 100;
  }
  else
  {
    sagV = 190;
		FreqHiThresh = 51 * 100;
    FreqLoThresh = 49 * 100;
  }

  vSagTh = (sagV * 100 * sqrt(2)) / (2 * ugain / 32768);

  //Initialize registers
  RWtoRegister(WRITE, SoftReset, 0x789A);     // 70 Perform soft reset
  RWtoRegister(WRITE, CfgRegAccEn, 0x55AA);   // 7F enable register config access
  RWtoRegister(WRITE, MeterEn, 0x0001);       // 00 Enable Metering

  RWtoRegister(WRITE, SagPeakDetCfg, 0x143F); // 05 Sag and Voltage peak detect period set to 20ms
  RWtoRegister(WRITE, SagTh, vSagTh);         // 08 Voltage sag threshold
  RWtoRegister(WRITE, FreqHiTh, FreqHiThresh);  // 0D High frequency threshold
  RWtoRegister(WRITE, FreqLoTh, FreqLoThresh);  // 0C Lo frequency threshold
  RWtoRegister(WRITE, EMMIntEn0, 0xB76F);     // 75 Enable interrupts
  RWtoRegister(WRITE, EMMIntEn1, 0xDDFD);     // 76 Enable interrupts
  RWtoRegister(WRITE, EMMIntState0, 0x0001);  // 73 Clear interrupt flags
  RWtoRegister(WRITE, EMMIntState1, 0x0001);  // 74 Clear interrupt flags
  RWtoRegister(WRITE, ZXConfig, 0xD654);      // 07 ZX2, ZX1, ZX0 pin config - set to current channels, all polarity

  //Set metering config values (CONFIG)
  RWtoRegister(WRITE, PLconstH, 0x0861);    // 31 PL Constant MSB (default) - Meter Constant = 3200 - PL Constant = 140625000
  RWtoRegister(WRITE, PLconstL, 0xC468);    // 32 PL Constant LSB (default) - this is 4C68 in the application note, which is incorrect
  RWtoRegister(WRITE, MMode0, lineFreq);   // 33 Mode Config (frequency set in main program)
  RWtoRegister(WRITE, MMode1, pgagain);    // 34 PGA Gain Configuration for Current Channels - 0x002A (x4) // 0x0015 (x2) // 0x0000 (1x)
  RWtoRegister(WRITE, PStartTh, 0x1D4C);    // 35 All phase Active Startup Power Threshold - 50% of startup current = 0.02A/0.00032 = 7500
  RWtoRegister(WRITE, QStartTh, 0x1D4C);    // 36 All phase Reactive Startup Power Threshold
  RWtoRegister(WRITE, SStartTh, 0x1D4C);    // 37 All phase Apparent Startup Power Threshold
  RWtoRegister(WRITE, PPhaseTh, 0x02EE);    // 38 Each phase Active Phase Threshold = 10% of startup current = 0.002A/0.00032 = 750
  RWtoRegister(WRITE, QPhaseTh, 0x02EE);    // 39 Each phase Reactive Phase Threshold
  RWtoRegister(WRITE, SPhaseTh, 0x02EE);    // 3A Each phase Apparent Phase Threshold

  //Set metering calibration values (CALIBRATION)
  RWtoRegister(WRITE, PQGainA, 0x0000);     // 47 Line calibration gain
  RWtoRegister(WRITE, PhiA, 0x0000);        // 48 Line calibration angle
  RWtoRegister(WRITE, PQGainB, 0x0000);     // 49 Line calibration gain
  RWtoRegister(WRITE, PhiB, 0x0000);        // 4A Line calibration angle
  RWtoRegister(WRITE, PQGainC, 0x0000);     // 4B Line calibration gain
  RWtoRegister(WRITE, PhiC, 0x0000);        // 4C Line calibration angle
  RWtoRegister(WRITE, PoffsetA, 0x0000);    // 41 A line active power offset FFDC
  RWtoRegister(WRITE, QoffsetA, 0x0000);    // 42 A line reactive power offset
  RWtoRegister(WRITE, PoffsetB, 0x0000);    // 43 B line active power offset
  RWtoRegister(WRITE, QoffsetB, 0x0000);    // 44 B line reactive power offset
  RWtoRegister(WRITE, PoffsetC, 0x0000);    // 45 C line active power offset
  RWtoRegister(WRITE, QoffsetC, 0x0000);    // 46 C line reactive power offset

  //Set metering calibration values (HARMONIC)
  RWtoRegister(WRITE, PoffsetAF, 0x0000);   // 51 A Fund. active power offset
  RWtoRegister(WRITE, PoffsetBF, 0x0000);   // 52 B Fund. active power offset
  RWtoRegister(WRITE, PoffsetCF, 0x0000);   // 53 C Fund. active power offset
  RWtoRegister(WRITE, PGainAF, 0x0000);     // 54 A Fund. active power gain
  RWtoRegister(WRITE, PGainBF, 0x0000);     // 55 B Fund. active power gain
  RWtoRegister(WRITE, PGainCF, 0x0000);     // 56 C Fund. active power gain

  //Set measurement calibration values (ADJUST)
  RWtoRegister(WRITE, UgainA, ugain);      // 61 A Voltage rms gain
  RWtoRegister(WRITE, IgainA, igainA);     // 62 A line current gain
  RWtoRegister(WRITE, UoffsetA, 0x0000);    // 63 A Voltage offset - 61A8
  RWtoRegister(WRITE, IoffsetA, 0x0000);    // 64 A line current offset - FE60
  RWtoRegister(WRITE, UgainB, ugain);      // 65 B Voltage rms gain
  RWtoRegister(WRITE, IgainB, igainB);     // 66 B line current gain
  RWtoRegister(WRITE, UoffsetB, 0x0000);    // 67 B Voltage offset - 1D4C
  RWtoRegister(WRITE, IoffsetB, 0x0000);    // 68 B line current offset - FE60
  RWtoRegister(WRITE, UgainC, ugain);      // 69 C Voltage rms gain
  RWtoRegister(WRITE, IgainC, igainC);     // 6A C line current gain
  RWtoRegister(WRITE, UoffsetC, 0x0000);    // 6B C Voltage offset - 1D4C
  RWtoRegister(WRITE, IoffsetC, 0x0000);    // 6C C line current offset

  RWtoRegister(WRITE, CfgRegAccEn, 0x0000); // 7F end configuration
}

int Read32Register(signed short regh_addr, signed short regl_addr){
  int val, val_h, val_l;
  val_h = RWtoRegister(READ, regh_addr, 0xFFFF);
  val_l = RWtoRegister(READ, regl_addr, 0xFFFF);
  val 	= RWtoRegister(READ, regh_addr, 0xFFFF);

  val = val_h << 16;
  val |= val_l; //concatenate the 2 registers to make 1 32 bit number

  return (val);
}

unsigned short GetSysStatus0(void) {
  return RWtoRegister(READ, EMMIntState0, 0xFFFF);
}
unsigned short GetSysStatus1(void) {
  return RWtoRegister(READ, EMMIntState1, 0xFFFF);
}


//****************************

/**
	* @brief Active Power
	* @retval power * 0.00032
	*/
double GetActivePower_A(void){
  int power = Read32Register(PmeanA, PmeanALSB);
  return (double)power * 0.00032;
}

/**
	* @brief Reactive Power
	* @retval power * 0.00032
	*/
double GetReactivePower_A(void){
  int power = Read32Register(QmeanA, QmeanALSB);
  return (double)power * 0.00032;
}

/**
	* @brief Apparent Power
	* @retval power * 0.00032
	*/
double GetApparentPower_A(void){
  int power = Read32Register(SmeanA, SmeanALSB);
  return (double)power * 0.00032;
}

/**
	* @brief Fundamental Power
	* @retval power * 0.00032
	*/
double GetFundamentalPower_A(void){
  int power = Read32Register(PmeanTF, PmeanTFLSB);
  return (double)power * 0.00032;
}

/**
	* @brief Harmonic Power
	* @retval power * 0.00032
	*/
double GetHarmonicPower_A(void){
  int power = Read32Register(PmeanTH, PmeanTHLSB);
  return (double)power * 0.00032;
}

/**
	* @brief RMS for Voltage
	* @retval voltage / 100
	*/
double GetRMSVoltage_A(void){
  unsigned short voltage = RWtoRegister(READ, UrmsA, 0xFFFF);
  return (double)voltage / 100;
}

/**
	* @brief RMS for Current
	* @retval current / 1000
	*/
double GetRMSCurrent_A(void){
  unsigned short current = RWtoRegister(READ, IrmsA, 0xFFFF);
  return (double)current / 1000;
}

/**
	* @brief Power Factor
	* @retval pf / 1000
	*/
double GetActivePowerFactor_A(void){
  signed short pf = (signed short) RWtoRegister(READ, PFmeanA, 0xFFFF);
  return (double)pf / 1000;
}

/**
	* @brief Phase Angle
	* @retval phaseA / 10
	*/
double GetPhaseAngle_A(void){
	unsigned short phaseA = RWtoRegister(READ, PAngleA, 0xFFFF);
	return (double)phaseA / 10;
}

/**
	* @brief Frequency
	* @retval freq / 100
	*/
double GetFreq(void){
  unsigned short freq = RWtoRegister(READ, Freq, 0xFFFF);
  return (double)freq / 100;
}

/**
	* @brief Temperature
	* @retval temp
	*/
double GetTemp(void){
  short int temp = (short int) RWtoRegister(READ, Temp, 0xFFFF);
  return (double)temp;
}

/**
	* @brief Peak Value
	* @retval
	*/
double GetPeak_A(void){
	unsigned int Upeak;
	unsigned int UPeakRegValue = RWtoRegister(READ, Temp, 0xFFFF);
	unsigned int UgainRegValue = RWtoRegister(READ, UgainA, 0xFFFF);
	
	Upeak = UPeakRegValue * ((UgainRegValue)/(100 * 8192));
	
	return (double)Upeak;
	
}

double CalibrateVI(unsigned short reg, unsigned short actualVal){
//input the Voltage or Current register, and the actual value that it should be
//actualVal can be from a calibration meter or known value from a power supply
  uint16_t gain, val, m, gainReg;
	//sample the reading
	val = RWtoRegister(READ, reg, 0xFFFF);
	val += RWtoRegister(READ, reg, 0xFFFF);
	val += RWtoRegister(READ, reg, 0xFFFF);
	val += RWtoRegister(READ, reg, 0xFFFF);

	//get value currently in gain register
	switch (reg) {
		case UrmsA: {
			gainReg = UgainA; }
		case UrmsB: {
			gainReg = UgainB; }
		case UrmsC: {
			gainReg = UgainC; }
		case IrmsA: {
			gainReg = IgainA; }
		case IrmsB: {
			gainReg = IgainB; }
		case IrmsC: {
			gainReg = IgainC; }
	}

	gain = RWtoRegister(READ, gainReg, 0xFFFF);
	m = actualVal;
	m = ((m * gain) / val);
	gain = m;

	//write new value to gain register
	RWtoRegister(WRITE, gainReg, gain);

	return(gain);
}
