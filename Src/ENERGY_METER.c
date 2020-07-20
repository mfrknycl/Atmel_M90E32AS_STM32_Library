#include "ENERGY_METER.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;


/**
	* @brief Read/Write to the specified register.
	* @param dataRW indicates whether it's going to write or read from the specified register.
	* @param address is the address of the register to be written or read.
	* @param data to send the register. If it's wanted to read, then write 0xFFFF to data.
	* @retval It returns data from register, if it is in Reading mode. If it is in 
						Writing mode, then it does not return any value.
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
	* @brief	Read 32-bit register
	*	@param	regAdd_H is the MSB.
	*	@param	regAdd_L is the LSB.
	*	@retval returns register data.
	*/
int Read32Register(signed short regAdd_H, signed short regAdd_L){
  int val, val_h, val_l;
  val_h = RWtoRegister(READ, regAdd_H, 0xFFFF);
  val_l = RWtoRegister(READ, regAdd_L, 0xFFFF);
  val 	= RWtoRegister(READ, regAdd_H, 0xFFFF);

  val = val_h << 16;
  val |= val_l; //concatenate the 2 registers to make 1 32 bit number

  return (val);
}

//********************* System Status Registers *********************
/**
	* @brief	Gets EMM Interrupt Status 0 (R/W1C)
	*	@param	None
	*	@retval returns EMMIntState0.
	*/
unsigned short GetSysStatus0(void){
  return RWtoRegister(READ, EMMIntState0, 0xFFFF);
}

/**
	* @brief	Gets EMM Interrupt Status 1 (R/W1C)
	*	@param	None
	*	@retval returns EMMIntState1.
	*/
unsigned short GetSysStatus1(void){
  return RWtoRegister(READ, EMMIntState1, 0xFFFF);
}

/**
	* @brief	Gets EMM State 0 (R)
	*	@param	None
	*	@retval returns EMMState0.
	*/
unsigned short GetMeterStatus0(void){
  return RWtoRegister(READ, EMMState0, 0xFFFF);
}

/**
	* @brief	Gets EMM State 1 (R)
	*	@param	None
	*	@retval returns EMMState1.
	*/
unsigned short GetMeterStatus1(void){
  return RWtoRegister(READ, EMMState1, 0xFFFF);
}

//********************* Power and Power Factor Register *********************

//********************* RMS Voltage *********************
/**
	* @brief RMS for Voltage A (Unsigned, 1LSB corresponds to 0.01 V)
	*	@param	None
	* @retval voltage / 100
	*/
double GetRMSVoltage_A(void){
  unsigned short voltage = RWtoRegister(READ, UrmsA, 0xFFFF);
  return (double)voltage / 100;
}

/**
	* @brief RMS for Voltage B (Unsigned, 1LSB corresponds to 0.01 V)
	*	@param	None
	* @retval voltage / 100
	*/
double GetRMSVoltage_B(void){
  unsigned short voltage = RWtoRegister(READ, UrmsB, 0xFFFF);
  return (double)voltage / 100;
}

/**
	* @brief RMS for Voltage C (Unsigned, 1LSB corresponds to 0.01 V)
	*	@param	None
	* @retval voltage / 100
	*/
double GetRMSVoltage_C(void){
  unsigned short voltage = RWtoRegister(READ, UrmsC, 0xFFFF);
  return (double)voltage / 100;
}

//********************* RMS Current *********************

/**
	* @brief RMS for Current A (uint16_t, 1LSB corresponds to 0.001 A)
	*	@param	None
	* @retval current / 1000
	*/
double GetRMSCurrent_A(void){
  unsigned short current = RWtoRegister(READ, IrmsA, 0xFFFF);
  return (double)current / 1000;
}

/**
	* @brief RMS for Current B (uint16_t, 1LSB corresponds to 0.001 A)
	*	@param	None
	* @retval current / 1000
	*/
double GetRMSCurrent_B(void){
  unsigned short current = RWtoRegister(READ, IrmsB, 0xFFFF);
  return (double)current / 1000;
}
/**
	* @brief RMS for Current C (uint16_t, 1LSB corresponds to 0.001 A)
	*	@param	None
	* @retval current / 1000
	*/
double GetRMSCurrent_C(void){
  unsigned short current = RWtoRegister(READ, IrmsC, 0xFFFF);
  return (double)current / 1000;
}

//********************* Active Power *********************
/**
	* @brief Active Power A (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetActivePower_A(void){
  //int power = Read32Register(PmeanT, PmeanTLSB);
	int power = Read32Register(PmeanA, PmeanALSB);
  return (double)power * 0.00032;
}

/**
	* @brief Active Power B (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetActivePower_B(void){
	int power = Read32Register(PmeanB, PmeanBLSB);
  return (double)power * 0.00032;
}

/**
	* @brief Active Power C (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetActivePower_C(void){
	int power = Read32Register(PmeanC, PmeanCLSB);
  return (double)power * 0.00032;
}

/**
	* @brief Total Active Power (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetTotalActivePower(void){
	int power = Read32Register(PmeanT, PmeanTLSB);
  return (double)power * 0.00032;
}

//********************* Reactive Power *********************
/**
	* @brief Reactive Power A (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetReactivePower_A(void){
  int power = Read32Register(QmeanA, QmeanALSB);
  return (double)power * 0.00032;
}

/**
	* @brief Reactive Power B (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetReactivePower_B(void){
  int power = Read32Register(QmeanB, QmeanBLSB);
  return (double)power * 0.00032;
}

/**
	* @brief Reactive Power C (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetReactivePower_C(void){
  int power = Read32Register(QmeanC, QmeanCLSB);
  return (double)power * 0.00032;
}

/**
	* @brief Total Reactive Power (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetTotalReactivePower(void){
  int power = Read32Register(QmeanT, QmeanTLSB);
  return (double)power * 0.00032;
}
//********************* Apparent Power *********************
/**
	* @brief Apparent Power A (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetApparentPower_A(void){
  int power = Read32Register(SmeanA, SmeanALSB);
  return (double)power * 0.00032;
}

/**
	* @brief Apparent Power B (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetApparentPower_B(void){
  int power = Read32Register(SmeanB, SmeanBLSB);
  return (double)power * 0.00032;
}

/**
	* @brief Apparent Power C (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetApparentPower_C(void){
  int power = Read32Register(SmeanC, SmeanCLSB);
  return (double)power * 0.00032;
}

/**
	* @brief Total Apparent Power (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetTotalApparentPower(void){
  int power = Read32Register(SmeanT, SAmeanTLSB);
  return (double)power * 0.00032;
}
//********************* Fundamental Power *********************
/**
	* @brief  Fundamental Power A (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetFundamentalPower_A(void){
  int power = Read32Register(PmeanAF, PmeanAFLSB);
  return (double)power * 0.00032;
}

/**
	* @brief  Fundamental Power B (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetFundamentalPower_B(void){
  int power = Read32Register(PmeanBF, PmeanBFLSB);
  return (double)power * 0.00032;
}

/**
	* @brief  Fundamental Power C (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetFundamentalPower_C(void){
  int power = Read32Register(PmeanCF, PmeanCFLSB);
  return (double)power * 0.00032;
}

/**
	* @brief Total Fundamental Power (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetTotalFundamentalPower(void){
  int power = Read32Register(PmeanTF, PmeanTFLSB);
  return (double)power * 0.00032;
}
//********************* Harmonic Power *********************
/**
	* @brief  Harmonic Power A (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetHarmonicPower_A(void){
  int power = Read32Register(PmeanAH, PmeanAHLSB);
  return (double)power * 0.00032;
}

/**
	* @brief  Harmonic Power B (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetHarmonicPower_B(void){
  int power = Read32Register(PmeanBH, PmeanBHLSB);
  return (double)power * 0.00032;
}

/**
	* @brief  Harmonic Power C (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetHarmonicPower_C(void){
  int power = Read32Register(PmeanCH, PmeanCHLSB);
  return (double)power * 0.00032;
}

/**
	* @brief Total Harmonic Power (Complement, Power=32-bit register value* 0.00032 W)
	*	@param	None
	* @retval power * 0.00032
	*/
double GetTotalHarmonicPower(void){
  int power = Read32Register(PmeanTH, PmeanTHLSB);
  return (double)power * 0.00032;
}

//********************* Power Factor *********************
/**
	* @brief Power Factor A (1LSB corresponds to 0.001 degree)
	*	@param	None
	* @retval pf / 1000
	*/
double GetActivePowerFactor_A(void){
  signed short pf = (signed short) RWtoRegister(READ, PFmeanA, 0xFFFF);
  return (double)pf / 1000;
}

/**
	* @brief Power Factor B (1LSB corresponds to 0.001 degree)
	*	@param	None
	* @retval pf / 1000
	*/
double GetActivePowerFactor_B(void){
  signed short pf = (signed short) RWtoRegister(READ, PFmeanB, 0xFFFF);
  return (double)pf / 1000;
}

/**
	* @brief Power Factor C (1LSB corresponds to 0.001 degree)
	*	@param	None
	* @retval pf / 1000
	*/
double GetActivePowerFactor_C(void){
  signed short pf = (signed short) RWtoRegister(READ, PFmeanC, 0xFFFF);
  return (double)pf / 1000;
}

/**
	* @brief Total Power Factor (1LSB corresponds to 0.001 degree)
	*	@param	None
	* @retval pf / 1000
	*/
double GetTotalActivePowerFactor(void){
  signed short pf = (signed short) RWtoRegister(READ, PFmeanT, 0xFFFF);
  return (double)pf / 1000;
}

//********************* Mean Phase Angle *********************
/**
	* @brief Mean Phase Angle A (1LSB corresponds to 0.1 degree)
	*	@param	None
	* @retval phaseA / 10
	*/
double GetPhaseAngle_A(void){
	unsigned short phaseA = RWtoRegister(READ, PAngleA, 0xFFFF);
	return (double)phaseA / 10;
}

/**
	* @brief Mean Phase Angle B (1LSB corresponds to 0.1 degree)
	*	@param	None
	* @retval phaseA / 10
	*/
double GetPhaseAngle_B(void){
	unsigned short phaseA = RWtoRegister(READ, PAngleB, 0xFFFF);
	return (double)phaseA / 10;
}

/**
	* @brief Mean Phase Angle C (1LSB corresponds to 0.1 degree)
	*	@param	None
	* @retval phaseA / 10
	*/
double GetPhaseAngle_C(void){
	unsigned short phaseA = RWtoRegister(READ, PAngleC, 0xFFFF);
	return (double)phaseA / 10;
}


//********************* Frequency *********************
/**
	* @brief Frequency (1LSB corresponds to 0.01 Hz)
	*	@param	None
	* @retval freq / 100
	*/
double GetFreq(void){
  unsigned short freq = RWtoRegister(READ, Freq, 0xFFFF);
  return (double)freq / 100;
}

//********************* Temperature *********************
/**
	* @brief Temperature (1LSB corresponds to 1 °C)
	*	@param	None
	* @retval temp
	*/
double GetTemp(void){
	RWtoRegister(WRITE, 0x2FF, 0x55AA);
	RWtoRegister(WRITE, 0x216, 0x5183);
	RWtoRegister(WRITE, 0x219, 0x01C1);
	RWtoRegister(WRITE, 0x2FF, 0x0000);
	
  short int temp = (short int) RWtoRegister(READ, Temp, 0xFFFF);
  return (double)temp;
}

/**
	* @brief Peak Value NOT WORKING !!!!
	*	@param	None
	* @retval	Upeak
	*/
double GetPeak_A(void){
	unsigned int Upeak;
	unsigned int UPeakRegValue = RWtoRegister(READ, Temp, 0xFFFF);
	unsigned int UgainRegValue = RWtoRegister(READ, UgainA, 0xFFFF);
	
	Upeak = UPeakRegValue * ((UgainRegValue)/(100 * 8192));
	
	return (double)Upeak;
}

//********************* Regular Energy Measuretment *********************

/**
	* @brief	Forward Active Energy A
	*	@param	None
	* @retval	returns in Wh
	*/
double GetForwardActiveEnergy_A() {
  unsigned short ienergyA = RWtoRegister(READ, APenergyA, 0xFFFF);
  return (double)ienergyA / 0.1 / 3200; //returns Wh
  //return (double)ienergyA / 100 / 3200; //returns kWh	
}

/**
	* @brief Forward Active Energy B
	*	@param	None
	* @retval returns in Wh
	*/
double GetForwardActiveEnergy_B() {
  unsigned short ienergyB = RWtoRegister(READ, APenergyB, 0xFFFF);
  return (double)ienergyB / 0.1 / 3200; //returns Wh
  //return (double)ienergyB / 100/ 3200; //returns kWh	
}

/**
	* @brief Forward Active Energy C
	*	@param	None
	* @retval returns in Wh
	*/
double GetForwardActiveEnergy_C() {
  unsigned short ienergyC = RWtoRegister(READ, APenergyC, 0xFFFF);
	  return (double)ienergyC / 0.1 / 3200; //returns Wh
		//return (double)ienergyC / 100 / 3200; //returns kWh
}

/**
	* @brief Total Forward Active Energy
	*	@param	None
	* @retval returns in Wh
	*/
double GetTotalForwardActiveEnergy() {
  unsigned short ienergyT = RWtoRegister(READ, APenergyT, 0xFFFF);
  return (double)ienergyT / 0.1 / 3200; //returns Wh
	//return (double)ienergyT / 100 / 3200; //returns kWh
	
}

/**
	* @brief  Reverse Active Energy A
	*	@param	None
	* @retval returns in Wh
	*/
double GetReverseActiveEnergy_A(){
  unsigned short eenergyA = RWtoRegister(READ, ANenergyA, 0xFFFF);
	return (double)eenergyA / 0.1 / 3200; //returns Wh
  //return (double)eenergyA / 100 / 3200; //returns kWh
}

/**
	* @brief  Reverse Active Energy B
	*	@param	None
	* @retval returns in Wh
	*/
double GetReverseActiveEnergy_B(){
  unsigned short eenergyB = RWtoRegister(READ, ANenergyB, 0xFFFF);
  return (double)eenergyB / 0.1 / 3200; //returns Wh
	//return (double)eenergyB / 100 / 3200; //returns kWh
}
/**
	* @brief  Reverse Active Energy C
	*	@param	None
	* @retval returns in Wh
	*/
double GetReverseActiveEnergy_C(){
  unsigned short eenergyC = RWtoRegister(READ, ANenergyC, 0xFFFF);
	return (double)eenergyC / 0.1 / 3200; //returns Wh
  //return (double)eenergyC / 100 / 3200; //returns kWh
}
/**
	* @brief Total Reverse Active Energy
	*	@param	None
	* @retval returns in Wh
	*/
double GetTotalReverseActiveEnergy(){
  unsigned short eenergyT = RWtoRegister(READ, ANenergyT, 0xFFFF);
  return (double)eenergyT / 0.1 / 3200; //returns Wh
	//return (double)eenergyT / 100 / 3200; //returns kWh
}

/**
	* @brief Forward Reactive Energy A
	*	@param	None
	* @retval returns in Wh
	*/
double GetForwardReactiveEnergy_A(){
  unsigned short renergyA = RWtoRegister(READ, RPenergyA, 0xFFFF);
  return (double)renergyA / 0.1 / 3200; //returns Wh
	//return (double)renergyA / 100 / 3200; //returns kWh
}
/**
	* @brief Forward Reactive Energy B
	*	@param	None
	* @retval returns in Wh
	*/
double GetForwardReactiveEnergy_B(void){
  unsigned short renergyB = RWtoRegister(READ, RPenergyB, 0xFFFF);
  return (double)renergyB / 0.1 / 3200; //returns Wh
	//return (double)renergyB / 100 / 3200; //returns kWh
}
/**
	* @brief Forward Reactive Energy C
	*	@param	None
	* @retval returns in Wh
	*/
double GetForwardReactiveEnergy_C(void){
  unsigned short renergyC = RWtoRegister(READ, RPenergyC, 0xFFFF);
  return (double)renergyC / 0.1 / 3200; //returns Wh
	//return (double)renergyC / 100 / 3200; //returns kWh
}
/**
	* @brief Total Forward Reactive Energy
	*	@param	None
	* @retval returns in Wh
	*/
double GetTotalForwardReactiveEnergy(void){
  unsigned short renergyT = RWtoRegister(READ, RPenergyT, 0xFFFF);
  return (double)renergyT / 0.1 / 3200; //returns Wh
	//return (double)renergyT / 100 / 3200; //returns kWh
}

//Reverse Reactive Energy
/**
	* @brief Reverse Reactive Energy A
	*	@param	None
	* @retval returns in Wh
	*/
double GetReverseReactiveEnergy_A(){
  unsigned short reenergyA = RWtoRegister(READ, RNenergyA, 0xFFFF);
  return (double)reenergyA / 0.1 / 3200; //returns Wh
	//return (double)reenergyA / 100 / 3200; //returns kWh
}
/**
	* @brief Reverse Reactive Energy B
	*	@param	None
	* @retval returns in Wh
	*/
double GetReverseReactiveEnergy_B(void){
  unsigned short reenergyB = RWtoRegister(READ, RNenergyB, 0xFFFF);
  return (double)reenergyB / 0.1 / 3200; //returns Wh
	//return (double)reenergyB / 100 / 3200; //returns kWh
}
/**
	* @brief Reverse Reactive Energy C
	*	@param	None
	* @retval returns in Wh
	*/
double GetReverseReactiveEnergy_C(void){
  unsigned short reenergyC = RWtoRegister(READ, RNenergyC, 0xFFFF);
  return (double)reenergyC / 0.1 / 3200; //returns Wh  
	//return (double)reenergyC / 100 / 3200; //returns kWh
}
/**
	* @brief Total Reverse Reactive Energy
	*	@param	None
	* @retval returns in Wh
	*/
double GetTotalReverseReactiveEnergy(void){
  unsigned short reenergyT = RWtoRegister(READ, RNenergyT, 0xFFFF);
  return (double)reenergyT / 0.1 / 3200; //returns Wh
	//return (double)reenergyT / 100 / 3200; //returns kWh
}

/**
	* @brief Apparent Energy A
	*	@param	None
	* @retval returns in Wh
	*/
double GetApparentEnergy_A(){
  unsigned short senergyA = RWtoRegister(READ, SenergyA, 0xFFFF);
	return (double)senergyA / 0.1 / 3200; //returns Wh
  //return (double)senergyA / 100 / 3200; //returns kWh
}
/**
	* @brief Apparent Energy B
	*	@param	None
	* @retval returns in Wh
	*/
double GetApparentEnergy_B(void){
  unsigned short senergyB = RWtoRegister(READ, SenergyB, 0xFFFF);
  return (double)senergyB / 0.1 / 3200; //returns Wh
	//return (double)senergyB / 100 / 3200; //returns kWh
}

/**
	* @brief Apparent Energy C
	*	@param	None
	* @retval returns in Wh
	*/
double GetApparentEnergy_C(void){
  unsigned short senergyC = RWtoRegister(READ, SenergyC, 0xFFFF);
  return (double)senergyC / 0.1 / 3200; //returns Wh
	//return (double)senergyC / 100 / 3200; //returns kWh
}

/**
	* @brief Total Apparent Energy
	*	@param	None
	* @retval returns in Wh
	*/
double GetTotalApparentEnergy(void){
  unsigned short senergyT = RWtoRegister(READ, SAenergyT, 0xFFFF);
  return (double)senergyT / 0.1 / 3200; //returns Wh
	//return (double)senergyT / 100 / 3200; //returns kWh
}

/**
	* @brief	Calibration for Voltage and Current. 
	* @note		This functions is called at the beginning of program after M90E32AS_Init function
	* @param 	Channel specifies the which channel to be configured. 
	*	@param 	Vref is Voltage Reference from power supply
	*	@param 	Iref is Current Reference from power supply
	* @retval None
	*/
void CalVI(unsigned char channel, double Vref, double Iref){
	double V_Measuretment, I_Measuretment, VoltageGain, ExVoltageGain, CurrentGain, ExCurrentGain;
	
	//READ Voltage measurement value and Existing Voltage Gain
	switch(channel){
		case Channel_A :
			V_Measuretment =	GetRMSVoltage_A();
			ExVoltageGain = (double)RWtoRegister(READ, UgainA, 0xFFFF);
		
			I_Measuretment = GetRMSCurrent_A();
			ExCurrentGain = (double)RWtoRegister(READ, IgainA, 0xFFFF);
			break;
		
		case Channel_B :
			V_Measuretment = GetRMSVoltage_B();
			ExVoltageGain = RWtoRegister(READ, UgainB, 0xFFFF);		
		
			I_Measuretment = GetRMSCurrent_B();
			ExCurrentGain = RWtoRegister(READ, IgainB, 0xFFFF);
			break;
		
		case Channel_C :
			V_Measuretment = GetRMSVoltage_C();
			ExVoltageGain = RWtoRegister(READ, UgainC, 0xFFFF);		
		
			I_Measuretment = GetRMSCurrent_C();
			ExCurrentGain = RWtoRegister(READ, IgainC, 0xFFFF);
			break;
	}
	
	/*
	  Voltage/ Current Measurement Calibration
  	- Voltage Gain = (reference voltage value / Voltage measurement value) * existing voltage gain
		- Current Gain = (reference current value / Current measurement value) * existing current gain
	*/
		VoltageGain = (Vref / V_Measuretment) * 32768; //ExVoltageGain;
		CurrentGain = (Iref / I_Measuretment) * 32768; //ExCurrentGain;
	
		//WRITE Voltage Gain and Current Gain into gain registers
		switch(channel){
		case Channel_A :
			RWtoRegister(WRITE, UgainA, VoltageGain);	
			RWtoRegister(WRITE, IgainA, CurrentGain);	
			break;
				
		case Channel_B :
			RWtoRegister(WRITE, UgainB, VoltageGain);	
			RWtoRegister(WRITE, IgainB, CurrentGain);		
			break;
		
		case Channel_C :
			RWtoRegister(WRITE, UgainC, VoltageGain);	
			RWtoRegister(WRITE, IgainC, CurrentGain);			
			break;
	}
}


/**
	* @brief  prints the summary of functions
	*	@param  sum pointer to a Summary_RegTypeDef structure that contains
  *              summary of register informations.
	* @retval None
	*/ 
void printSummary(Summary_RegTypeDef *sum){
//********************* Power and Power Factor Register *********************

//RMS for Voltage
sum->GetRMSVoltage_A = GetRMSVoltage_A(); 				 //RMS for Voltage A
sum->GetRMSVoltage_B = GetRMSVoltage_B(); 				 //RMS for Voltage B
sum->GetRMSVoltage_C = GetRMSVoltage_C(); 				 //RMS for Voltage C

//RMS for Current
sum->GetRMSCurrent_A = GetRMSCurrent_A(); 				 //RMS for Current A
sum->GetRMSCurrent_B = GetRMSCurrent_B(); 				 //RMS for Current B
sum->GetRMSCurrent_C = GetRMSCurrent_C(); 				 //RMS for Current C

//Active Power
sum->GetActivePower_A = GetActivePower_A(); 			   //Active Power A
sum->GetActivePower_B = GetActivePower_B(); 			   //Active Power B
sum->GetActivePower_C = GetActivePower_C(); 			   //Active Power C
sum->GetTotalActivePower = GetTotalActivePower(); 	   //Total Active Power

//Reactive Power
sum->GetReactivePower_A = GetReactivePower_A(); 		   //Reactive Power A
sum->GetReactivePower_B = GetReactivePower_B(); 		   //Reactive Power B
sum->GetReactivePower_C = GetReactivePower_C(); 		   //Reactive Power C
sum->GetTotalReactivePower = GetTotalReactivePower();    //Total Reactive Power

//Apparent Power
sum->GetApparentPower_A = GetApparentPower_A(); 		   //Apparent Power A
sum->GetApparentPower_B = GetApparentPower_B(); 		   //Apparent Power B
sum->GetApparentPower_C = GetApparentPower_C(); 		   //Apparent Power C
sum->GetTotalApparentPower = GetTotalApparentPower();    //Total Apparent Power

//Fundamental Power
sum->GetFundamentalPower_A = GetFundamentalPower_A();    //Fundamental Power A
sum->GetFundamentalPower_B = GetFundamentalPower_B(); 	 //Fundamental Power B
sum->GetFundamentalPower_C = GetFundamentalPower_C();    //Fundamental Power C
sum->GetTotalFundamentalPower = GetTotalFundamentalPower(); //Total Fundamental Power

//Harmonic Power
sum->GetHarmonicPower_A = GetHarmonicPower_A(); 		 //Harmonic Power A
sum->GetHarmonicPower_B = GetHarmonicPower_B(); 		 //Harmonic Power B
sum->GetHarmonicPower_C = GetHarmonicPower_C(); 		 //Harmonic Power C
sum->GetTotalHarmonicPower = GetTotalHarmonicPower();  //Total Harmonic Power 

//********************* Power Factor *********************
sum->GetActivePowerFactor_A = GetActivePowerFactor_A();
sum->GetActivePowerFactor_B = GetActivePowerFactor_B();
sum->GetActivePowerFactor_C = GetActivePowerFactor_C();
sum->GetTotalActivePowerFactor = GetTotalActivePowerFactor();

//********************* Phase Angle *********************
sum->GetPhaseAngle_A = GetPhaseAngle_A();
sum->GetPhaseAngle_B = GetPhaseAngle_B();
sum->GetPhaseAngle_C = GetPhaseAngle_C();


sum->GetFreq = GetFreq(); 							 //Frequency
sum->GetTemp = GetTemp(); 							 //Temperature
//sum->GetPeak_A = GetPeak_A();						   //Peak Value

//********************* Regular Energy Measuretment *********************
//Forward Active Energy
sum->GetForwardActiveEnergy_A = GetForwardActiveEnergy_A();
sum->GetForwardActiveEnergy_B = GetForwardActiveEnergy_B();
sum->GetForwardActiveEnergy_C = GetForwardActiveEnergy_C();
sum->GetTotalForwardActiveEnergy = GetTotalForwardActiveEnergy();

//Reverse Active Energy
sum->GetReverseActiveEnergy_A = GetReverseActiveEnergy_A();
sum->GetReverseActiveEnergy_B = GetReverseActiveEnergy_B();
sum->GetReverseActiveEnergy_C = GetReverseActiveEnergy_C();
sum->GetTotalReverseActiveEnergy = GetTotalReverseActiveEnergy();
	
//Forward Reactive Energy
sum->GetForwardReactiveEnergy_A = GetForwardReactiveEnergy_A();
sum->GetForwardReactiveEnergy_B = GetForwardReactiveEnergy_B();
sum->GetForwardReactiveEnergy_C = GetForwardReactiveEnergy_C();
sum->GetTotalForwardReactiveEnergy = GetTotalForwardReactiveEnergy();

//Reverse Reactive Energy
sum->GetReverseReactiveEnergy_A = GetReverseReactiveEnergy_A();
sum->GetReverseReactiveEnergy_B = GetReverseReactiveEnergy_B();
sum->GetReverseReactiveEnergy_C = GetReverseReactiveEnergy_C();
sum->GetTotalReverseReactiveEnergy = GetTotalReverseReactiveEnergy();

//Apparent Energy
sum->GetApparentEnergy_A = GetApparentEnergy_A();
sum->GetApparentEnergy_B = GetApparentEnergy_B();
sum->GetApparentEnergy_C = GetApparentEnergy_C();
sum->GetTotalApparentEnergy = GetTotalApparentEnergy();

}

/**
	* @brief 	It sets the Baud Rate to 1000.0 Kbits/s for SPI communication for ATM90E32.
	*	@param 	None
	*	@retval None
	*/
void Energy_SetBaudRate(){
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
	* @brief  Initialization function. It uses SPI MODE 3 for the ATM90E32 - CPOL = HIGH (1) / CPHA - 2 EDGE (1).
	* @note		This functions is called at the beginning of program for initialising registers. 
	* @param  None
	* @retval None
	*/

void M90E32AS_Init(){
	
	Energy_SetBaudRate();
	

  unsigned short vSagTh;
  unsigned short sagV;
  unsigned short FreqHiThresh;
  unsigned short FreqLoThresh;


  sagV = 250;
  FreqHiThresh = 51 * 100;;
  FreqLoThresh = 49 * 100;

  //vSagTh = (sagV * 100 * sqrt(2)) / (2 * ugain / 32768);

  //Initialize registers
  RWtoRegister(WRITE, SoftReset, 0x789A);       // 70 Perform soft reset
  RWtoRegister(WRITE, CfgRegAccEn, 0x55AA);     // 7F enable register config access
  RWtoRegister(WRITE, MeterEn, 0x0001);         // 00 Enable Metering

  RWtoRegister(WRITE, SagPeakDetCfg, 0x143F);   // 05 Sag and Voltage peak detect period set to 20ms
  RWtoRegister(WRITE, SagTh, 0x0000);           // 08 Voltage sag threshold
  RWtoRegister(WRITE, FreqHiTh, FreqHiThresh);  // 0D High frequency threshold
  RWtoRegister(WRITE, FreqLoTh, FreqLoThresh);  // 0C Lo frequency threshold

  RWtoRegister(WRITE, ZXConfig, 0xD654);        // 07 ZX2, ZX1, ZX0 pin config - set to current channels, all polarity (0xD654)

	//Configuration Registers
	RWtoRegister(WRITE, PLconstH, 0x0861); // PL Constant MSB (default) - 0x0861 is the default value.
	RWtoRegister(WRITE, PLconstL, 0xC468); // PL Constant LSB (default) - this is 0x4C68 in the application note, which is incorrect.
	RWtoRegister(WRITE, MMode0, 	0x0087); // This register can be used choose different systems and metering modes.
	RWtoRegister(WRITE, MMode1, 	0x002A); // This register is used to configure PGA gain of ADC sampling channel for different current. 0x002A (x4) // 0x0015 (x2) // 0x0000 (1x)
	RWtoRegister(WRITE, PStartTh, 0x0000); // All-phase Active Startup Power Threshold.
	RWtoRegister(WRITE, QStartTh, 0x0000); // All-phase Reactive Startup Power Threshold.
	RWtoRegister(WRITE, SStartTh, 0x0000); // All-phase Apparent Startup Power Threshold.
	RWtoRegister(WRITE, PPhaseTh, 0x0000); // Each-phase Active Startup Power Threshold.
	RWtoRegister(WRITE, QPhaseTh, 0x0000); // Each-phase Reactive Startup Power Threshold.
	RWtoRegister(WRITE, SPhaseTh, 0x0000); // Each-phase Apparent Startup Power Threshold.
	
	//Calibration Registers
	RWtoRegister(WRITE, PoffsetA, 0x0000); // Phase A Active Power Offset - (default)
	RWtoRegister(WRITE, QoffsetA, 0x0000); // Phase A Reactive Power Offset - (default) 
	RWtoRegister(WRITE, PoffsetB, 0x0000); // Phase B Active Power Offset - (default)
	RWtoRegister(WRITE, QoffsetB, 0x0000); // Phase B Reactive Power Offset - (default)
	RWtoRegister(WRITE, PoffsetC, 0x0000); // Phase C Active Power Offset - (default)
	RWtoRegister(WRITE, QoffsetC, 0x0000); // Phase C Reactive Power Offset - (default)
	RWtoRegister(WRITE, PQGainA, 	0x0000); // Phase A Active/reactive Energy Calibration Gain - (default)
	RWtoRegister(WRITE, PhiA, 		0x0000); // Phase A Calibration Phase Angle - (default)
	RWtoRegister(WRITE, PQGainB, 	0x0000); // Phase B Active/reactive Energy Calibration Gain - (default)
	RWtoRegister(WRITE, PhiB, 		0x0000); // Phase B Calibration Phase Angle - (default)
	RWtoRegister(WRITE, PQGainC, 	0x0000); // Phase C Active/reactive Energy Calibration Gain - (default)
	RWtoRegister(WRITE, PhiC,		  0x0000); // Phase C Calibration Phase Angle - (default)
	
	//Fundamental/Harmonic Energy Calibration Registers
	RWtoRegister(WRITE, PoffsetAF, 0x0000); // Phase A Fundamental Active Poweroffset - (default)
	RWtoRegister(WRITE, PoffsetBF, 0x0000); // Phase B Fundamental Active Power offset - (default)
	RWtoRegister(WRITE, PoffsetCF, 0x0000); // Phase C Fundamental Active Power offset - (default)
	RWtoRegister(WRITE, PGainAF, 	 0x0000); // Phase A Fundamental Calibration Gain - (default)
	RWtoRegister(WRITE, PGainBF, 	 0x0000); // Phase B Fundamental Calibration Gain - (default)
	RWtoRegister(WRITE, PGainCF, 	 0x0000); // Phase C Fundamental Calibration Gain - (default)
	
	//Measurement Calibration Registers //0xB000
	RWtoRegister(WRITE, UgainA,	  0x8000); // Phase A Voltage RMS Gain - (default)
	RWtoRegister(WRITE, IgainA, 	0x8000); // Phase A Current RMS Gain - (default)
	RWtoRegister(WRITE, UoffsetA, 0x0000); // Phase A Voltage RMS offset - (default)
	RWtoRegister(WRITE, IoffsetA, 0x0000); // Phase A Current RMS offset - (default)
	RWtoRegister(WRITE, UgainB, 	0x8000); // Phase B Voltage RMS Gain - (default)
	RWtoRegister(WRITE, IgainB, 	0x8000); // Phase B Current RMS Gain - (default)
	RWtoRegister(WRITE, UoffsetB, 0x0000); // Phase B Voltage RMS offset - (default)
	RWtoRegister(WRITE, IoffsetB, 0x0000); // Phase B Current RMS offset - (default)
	RWtoRegister(WRITE, UgainC, 	0x8000); // Phase C Voltage RMS Gain - (default)
	RWtoRegister(WRITE, IgainC, 	0x8000); // Phase C Current RMS Gain - (default)
	RWtoRegister(WRITE, UoffsetC, 0x0000); // Phase C Voltage RMS offset - (default)
	RWtoRegister(WRITE, IoffsetC, 0x0000); // Phase C Current RMS offset - (default)
	
	//EMM Status Registers
	RWtoRegister(WRITE, SoftReset, 		0x0000); // Software Reset - (default)
	RWtoRegister(WRITE, EMMIntState0, 0x0000); // EMM Interrupt Status 0 - (default)
	RWtoRegister(WRITE, EMMIntState1, 0x0000); // EMM Interrupt Status 1 - (default)
	RWtoRegister(WRITE, EMMIntEn0, 		0x0000); // EMM Interrupt Enable 0 - (default)
	RWtoRegister(WRITE, EMMIntEn1, 		0x0000); // EMM Interrupt Enable 1 - (default)


}





