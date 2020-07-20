#ifndef __ENERGY_METER__
#define __ENERGY_METER__


/* Includes ---------------------------------------------------- */
#include "stm32f0xx.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* Private defines ---------------------------------------------------- */
#define READ   1
#define WRITE  0

#define Voltage 1
#define Current 0

#define Channel_A 1
#define Channel_B 2
#define Channel_C 3

//Status and Special Register
#define MeterEn          (0x00)   // (R/W) Metering Enable    
#define ChannelMapI      (0x01)   // (R/W) Current Channel Mapping Configuration
#define ChannelMapU      (0x02)   // (R/W) Voltage Channel Mapping Configuration 
#define SagPeakDetCfg    (0x05)   // (R/W) Sag and Peak Detector Period Configuration
#define OVth             (0x06)   // (R/W) Over Voltage Threshold
#define ZXConfig         (0x07)   // (R/W) Zero-Crossing Configuration 
#define SagTh            (0x08)   // (R/W) Voltage Sag Threshold
#define PhaseLossTh      (0x09)   // (R/W) Voltage Phase Losing Threshold
#define InWarnTh         (0x0A)   // (R/W) Neutral Current (Calculated) Warning Threshold
#define OIth         		 (0x0B)   // (R/W) Over Current Threshold
#define FreqLoTh         (0x0C)   // (R/W) Low Threshold for Frequency Detection
#define FreqHiTh         (0x0D)   // (R/W) High Threshold for Frequency Detection
#define PMPwrCtrl        (0x0E)   // (R/W) Partial Measurement Mode Power Control
#define IRQ0MergeCfg     (0x0F)   // (R/W) IRQ0 Merge Configuration

//Low Power Mode Register
#define DetectCtrl       (0x10)   // (R/W) Current Detect Control
#define DetectTh1        (0x11)   // (R/W) Channel 1 Current Threshold in Detection Mode
#define DetectTh2        (0x12)   // (R/W) Channel 2 Current Threshold in Detection Mode
#define DetectTh3        (0x13)   // (R/W) Channel 3 Current Threshold in Detection Mode
#define IDCoffsetA       (0x14)   // (R/W) Phase A Current DC offset
#define IDCoffsetB       (0x15)   // (R/W) Phase B Current DC offset
#define IDCoffsetC       (0x16)   // (R/W) Phase C Current DC offset
#define UDCoffsetA       (0x17)   // (R/W) Voltage DC offset for Channel A
#define UDCoffsetB       (0x18)   // (R/W) Voltage DC offset for Channel B
#define UDCoffsetC       (0x19)   // (R/W) Voltage DC offset for Channel C
#define UGainTAB         (0x1A)   // (R/W) Voltage Gain Temperature Compensation for Phase A/B
#define UGainTC          (0x1B)   // (R/W) Voltage Gain Temperature Compensation for Phase C
#define PhiFreqComp      (0x1C)   // (R/W) Phase Compensation for Frequency
#define LOGIrms0         (0x20)   // (R/W) Current (Log Irms0) Configuration for Segment Compensation
#define LOGIrms1         (0x21)   // (R/W) Current (Log Irms1) Configuration for Segment Compensation
#define F0               (0x22)   // (R/W) Nominal Frequency
#define T0         			 (0x23)   // (R/W) Nominal Temperature
#define PhiAIrms01       (0x24)   // (R/W) Phase A Phase Compensation for Current Segment 0 and 1
#define PhiAIrms2        (0x25)   // (R/W) Phase A Phase Compensation for Current Segment 2
#define GainAIrms01      (0x26)   // (R/W) Phase A Gain Compensation for Current Segment 0 and 1
#define GainAIrms2       (0x27)   // (R/W) Phase A Gain Compensation for Current Segment 2
#define PhiBIrms01       (0x28)   // (R/W) Phase B Phase Compensation for Current Segment 0 and 1
#define PhiBIrms2        (0x29)   // (R/W) Phase B Phase Compensation for Current Segment 2
#define GainBIrms01      (0x2A)   // (R/W) Phase B Gain Compensation for Current Segment 0 and 1
#define GainBIrms2       (0x2B)   // (R/W) Phase B Gain Compensation for Current Segment 2
#define PhiCIrms01       (0x2C)   // (R/W) Phase C Phase Compensation for Current Segment 0 and 1
#define PhiCIrms2        (0x2D)   // (R/W) Phase C Phase Compensation for Current Segment 2
#define GainCIrms01      (0x2E)   // (R/W) Phase C Gain Compensation for Current Segment 0 and 1
#define GainCIrms2       (0x2F)   // (R/W) Phase C Gain Compensation for Current Segment 2

//Configuration Registers*********

#define PLconstH      	 (0x31)   // (R/W) High Word of PL_Constant
#define PLconstL      	 (0x32)   // (R/W) Low Word of PL_Constant
#define MMode0      	   (0x33)   // (R/W) Metering Method Configuration
#define MMode1      	   (0x34)   // (R/W) PGA Gain Configuration
#define PStartTh      	 (0x35)   // (R/W) Active Startup Power Threshold
#define QStartTh      	 (0x36)   // (R/W) Reactive Startup Power Threshold
#define SStartTh      	 (0x37)   // (R/W) Apparent Startup Power Threshold
#define PPhaseTh      	 (0x38)   // (R/W) Startup Power Threshold for Any Phase (Active Energy Accumulation)
#define QPhaseTh      	 (0x39)   // (R/W) Startup Power Threshold for Any Phase (ReActive Energy Accumulation)
#define SPhaseTh      	 (0x3A)   // (R/W) Startup Power Threshold for Any Phase (Apparent Energy Accumulation)

//Calibration Registers*********
#define PoffsetA      	 (0x41)   // (R/W) Phase A Active Power offset
#define QoffsetA      	 (0x42)   // (R/W) Phase A Reactive Power offset
#define PoffsetB      	 (0x43)   // (R/W) Phase B Active Power offset
#define QoffsetB      	 (0x44)   // (R/W) Phase B Reactive Power offset
#define PoffsetC      	 (0x45)   // (R/W) Phase C Active Power offset
#define QoffsetC      	 (0x46)   // (R/W) Phase C Reactive Power offset
#define PQGainA      	   (0x47)   // (R/W) Phase A Calibration Gain
#define PhiA      	     (0x48)   // (R/W) Phase A Calibration Phase Angle
#define PQGainB      	   (0x49)   // (R/W) Phase B Calibration Gain
#define PhiB      	     (0x4A)   // (R/W) Phase B Calibration Phase Angle
#define PQGainC      	   (0x4B)   // (R/W) Phase C Calibration Gain
#define PhiC      	     (0x4C)   // (R/W) Phase C Calibration Phase Angle

//Fundamental/ Harmonic Energy Calibration Registers*********
#define PoffsetAF      	 (0x51)   // (R/W) Phase A Fundamental Active Power offset
#define PoffsetBF      	 (0x52)   // (R/W) Phase B Fundamental Active Power offset
#define PoffsetCF      	 (0x53)   // (R/W) Phase C Fundamental Active Power offset
#define PGainAF      		 (0x54)   // (R/W) Phase A Fundamental Calibration Gain
#define PGainBF      	 	 (0x55)   // (R/W) Phase B Fundamental Calibration Gain
#define PGainCF      	 	 (0x56)   // (R/W) Phase C Fundamental Calibration Gain

//Measurement Calibration Registers*********
#define UgainA      	 	 (0x61)   // (R/W) Phase A Voltage RMS Gain
#define IgainA      	 	 (0x62)   // (R/W) Phase A Current RMS Gain
#define UoffsetA      	 (0x63)   // (R/W) Phase A Voltage RMS offset
#define IoffsetA      	 (0x64)   // (R/W) Phase A Current RMS offset
#define UgainB      	 	 (0x65)   // (R/W) Phase B Voltage RMS Gain
#define IgainB      	 	 (0x66)   // (R/W) Phase B Current RMS Gain     	 
#define UoffsetB      	 (0x67)   // (R/W) Phase B Voltage RMS offset
#define IoffsetB      	 (0x68)   // (R/W) Phase B Current RMS offset
#define UgainC      	 	 (0x69)   // (R/W) Phase C Voltage RMS Gain
#define IgainC      	 	 (0x6A)   // (R/W) Phase C Current RMS Gain     	 	 
#define UoffsetC      	 (0x6B)   // (R/W) Phase C Voltage RMS offset
#define IoffsetC      	 (0x6C)   // (R/W) Phase C Current RMS offset

//EMM Status Registers
#define SoftReset      	 (0x70)   // (R/W)   Software Reset
#define EMMState0      	 (0x71)   // (R)     EMM State 0
#define EMMState1      	 (0x72)   // (R)     EMM State 1
#define EMMIntState0     (0x73)   // (R/W1C) EMM Interrupt Status 0
#define EMMIntState1     (0x74)   // (R/W1C) EMM Interrupt Status 1
#define EMMIntEn0      	 (0x75)   // (R/W)   EMM Interrupt Enable 0
#define EMMIntEn1      	 (0x76)   // (R/W)   EMM Interrupt Enable 1
#define LastSPIData      (0x78)   // (R)     Last Read/Write SPI Value
#define CRCErrStatus     (0x79)   // (R)     CRC Error Status
#define CRCDigest      	 (0x7A)   // (R/W)   CRC Digest
#define CfgRegAccEn      (0x7F)   // (R/W)   Configure Register Access Enable

//Energy Register
#define APenergyT      	 (0x80)   // (R/C) Total Forward Active Energy
#define APenergyA      	 (0x81)   // (R/C) Phase A Forward Active Energy
#define APenergyB      	 (0x82)   // (R/C) Phase B Forward Active Energy
#define APenergyC      	 (0x83)   // (R/C) Phase C Forward Active Energy
#define ANenergyT      	 (0x84)   // (R/C) Total Reverse Active Energy
#define ANenergyA      	 (0x85)   // (R/C) Phase A Reverse Active Energy
#define ANenergyB      	 (0x86)   // (R/C) Phase B Reverse Active Energy
#define ANenergyC      	 (0x87)   // (R/C) Phase C Reverse Active Energy
#define RPenergyT      	 (0x88)   // (R/C) Phase C Reverse Active Energy
#define RPenergyA      	 (0x89)   // (R/C) Phase A Forward Reactive Energy
#define RPenergyB      	 (0x8A)   // (R/C) Phase B Forward Reactive Energy
#define RPenergyC      	 (0x8B)   // (R/C) Phase C Forward Reactive Energy
#define RNenergyT      	 (0x8C)   // (R/C) Total Reverse Reactive Energy
#define RNenergyA      	 (0x8D)   // (R/C) Phase A Reverse Reactive Energy
#define RNenergyB      	 (0x8E)   // (R/C) Phase B Reverse Reactive Energy
#define RNenergyC      	 (0x8F)   // (R/C) Phase C Reverse Reactive Energy
#define SAenergyT      	 (0x90)   // (R/C) Total (Arithmetic Sum) Apparent Energy
#define SenergyA      	 (0x91)   // (R/C) Phase A Apparent Energy
#define SenergyB      	 (0x92)   // (R/C) Phase B Apparent Energy
#define SenergyC      	 (0x93)   // (R/C) Phase C Apparent Energy

//Fundamental / Harmonic Energy Register
#define APenergyTF       (0xA0)   // (R/C) Total Forward Active Fundamental Energy
#define APenergyAF       (0xA1)   // (R/C) Phase A Forward Active Fundamental Energy
#define APenergyBF       (0xA2)   // (R/C) Phase B Forward Active Fundamental Energy
#define APenergyCF       (0xA3)   // (R/C) Phase C Forward Active Fundamental Energy
#define ANenergyTF       (0xA4)   // (R/C) Total Reverse Active Fundamental Energy
#define ANenergyAF       (0xA5)   // (R/C) Phase A Reverse Active Fundamental Energy
#define ANenergyBF       (0xA6)   // (R/C) Phase B Reverse Active Fundamental Energy
#define ANenergyCF       (0xA7)   // (R/C) Phase C Reverse Active Fundamental Energy
#define APenergyTH       (0xA8)   // (R/C) Total Forward Active Harmonic Energy
#define APenergyAH       (0xA9)   // (R/C) Phase A Forward Active Harmonic Energy
#define APenergyBH       (0xAA)   // (R/C) Phase B Forward Active Harmonic Energy
#define APenergyCH       (0xAB)   // (R/C) Phase C Forward Active Harmonic Energy
#define ANenergyTH       (0xAC)   // (R/C) Total Reverse Active Harmonic Energy
#define ANenergyAH       (0xAD)   // (R/C) Phase A Reverse Active Harmonic Energy
#define ANenergyBH       (0xAE)   // (R/C) Phase B Reverse Active Harmonic Energy
#define ANenergyCH       (0xAF)   // (R/C) Phase C Reverse Active Harmonic Energy

//Power and Power Factor Registers
#define PmeanT      	 	 (0xB0)   // (R) Total (all-phase-sum) Active Power
#define PmeanA      	 	 (0xB1)   // (R) Phase A Active Power
#define PmeanB      	 	 (0xB2)   // (R) Phase B Active Power
#define PmeanC      	 	 (0xB3)   // (R) Phase C Active Power
#define QmeanT      	 	 (0xB4)   // (R) Total (all-phase-sum) Reactive Power
#define QmeanA      	 	 (0xB5)   // (R) Phase A Reactive Power
#define QmeanB      	 	 (0xB6)   // (R) Phase B Reactive Power
#define QmeanC      	 	 (0xB7)   // (R) Phase C Reactive Power
#define SmeanT      	 	 (0xB8)   // (R) Total (Arithmetic Sum) Apparent Power
#define SmeanA      	 	 (0xB9)   // (R) Phase A Apparent Power
#define SmeanB      	 	 (0xBA)   // (R) Phase B Apparent Power
#define SmeanC      	 	 (0xBB)   // (R) Phase C Apparent Power
#define PFmeanT      	 	 (0xBC)   // (R) Total Power Factor
#define PFmeanA      	 	 (0xBD)   // (R) Phase A Power Factor
#define PFmeanB      	 	 (0xBE)   // (R) Phase B Power Factor
#define PFmeanC      	 	 (0xBF)   // (R) Phase C Power Factor
#define PmeanTLSB      	 (0xC0)   // (R) Lower Word of Total (all-phase-sum) Active  Power
#define PmeanALSB      	 (0xC1)   // (R) Lower Word of Phase A Active Power
#define PmeanBLSB      	 (0xC2)   // (R) Lower Word of Phase B Active Power
#define PmeanCLSB      	 (0xC3)   // (R) Lower Word of Phase C Active Power
#define QmeanTLSB      	 (0xC4)   // (R) Lower Word of Total (all-phase-sum) Reactive Power
#define QmeanALSB      	 (0xC5)   // (R) Lower Word of Phase A Reactive Power
#define QmeanBLSB      	 (0xC6)   // (R) Lower Word of Phase B Reactive Power
#define QmeanCLSB      	 (0xC7)   // (R) Lower Word of Phase C Reactive Power
#define SAmeanTLSB       (0xC8)   // (R) Lower Word of Total (Arithmetic Sum) Apparent Power
#define SmeanALSB      	 (0xC9)   // (R) Lower Word of Phase A Apparent Power
#define SmeanBLSB      	 (0xCA)   // (R) Lower Word of Phase B Apparent Power
#define SmeanCLSB      	 (0xCB)   // (R) Lower Word of Phase C Apparent Power

//Fundamental / Harmonic Power and Voltage / Current RMS Registers
#define PmeanTF      	 	 (0xD0)   // (R) Total Active Fundamental Power
#define PmeanAF      	 	 (0xD1)   // (R) Phase A Active Fundamental Power
#define PmeanBF      	 	 (0xD2)   // (R) Phase B Active Fundamental Power
#define PmeanCF      	 	 (0xD3)   // (R) Phase C Active Fundamental Power
#define PmeanTH      	 	 (0xD4)   // (R) Phase C Active Fundamental Power
#define PmeanAH      	 	 (0xD5)   // (R) Phase A Active Harmonic Power
#define PmeanBH      	 	 (0xD6)   // (R) Phase B Active Harmonic Power
#define PmeanCH      	 	 (0xD7)   // (R) Phase B Active Harmonic Power
#define UrmsA      	 	   (0xD9)   // (R)  Phase B Active Harmonic Power     	 	 
#define UrmsB      	 	   (0xDA)   // (R) Phase B Voltage RMS
#define UrmsC      	 	   (0xDB)   // (R) Phase C Voltage RMS
#define IrmsN      	 	   (0xDC)   // (R) N Line Calculated Current RMS
#define IrmsA      	 	   (0xDD)   // (R) Phase A Current RMS
#define IrmsB      	 	   (0xDE)   // (R) Phase B Current RMS
#define IrmsC      	 	   (0xDF)   // (R) Phase C Current RMS
#define PmeanTFLSB       (0xE0)   // (R) Lower Word of Total Active Fundamental Power
#define PmeanAFLSB       (0xE1)   // (R) Lower Word of Phase A Active Fundamental Power
#define PmeanBFLSB       (0xE2)   // (R) Lower Word of Phase B Active Fundamental Power
#define PmeanCFLSB       (0xE3)   // (R) Lower Word of Phase C Active Fundamental Power
#define PmeanTHLSB       (0xE4)   // (R) Lower Word of Total Active Harmonic Power
#define PmeanAHLSB       (0xE5)   // (R) Lower Word of Phase A Active Harmonic Power
#define PmeanBHLSB       (0xE6)   // (R) Lower Word of Phase B Active Harmonic Power
#define PmeanCHLSB       (0xE7)   // (R) Lower Word of Phase C Active Harmonic Power
#define UrmsALSB       	 (0xE9)   // (R) Lower Word of Phase A Voltage RMS
#define UrmsBLSB      	 (0xEA)   // (R) Lower Word of Phase B Voltage RMS
#define UrmsCLSB         (0xEB)   // (R) Lower Word of Phase C Voltage RMS
#define IrmsALSB      	 (0xED)   // (R) Lower Word of Phase A Current RMS
#define IrmsBLSB      	 (0xEE)   // (R) Lower Word of Phase B Current RMS
#define IrmsCLSB      	 (0xEF)   // (R) Lower Word of Phase C Current RMS

//Peak, Frequency, Angle and Temperature Registers
#define UPeakA      	 	 (0xF1)   // (R) Channel A Voltage Peak
#define UPeakB      	 	 (0xF2)   // (R) Channel B Voltage Peak
#define UPeakC      	 	 (0xF3)   // (R) Channel C Voltage Peak
#define IPeakA      	 	 (0xF5)   // (R) Channel C Voltage Peak
#define IPeakB      	 	 (0xF6)   // (R) Channel C Voltage Peak
#define IPeakC      	 	 (0xF7)   // (R) Channel C Current Peak
#define Freq      	 	   (0xF8)   // (R) Frequency
#define PAngleA      	 	 (0xF9)   // (R) Phase A Mean Phase Angle
#define PAngleB      	 	 (0xFA)   // (R) Phase B Mean Phase Angle
#define PAngleC      	 	 (0xFB)   // (R) Phase C Mean Phase Angle
#define Temp      	 	   (0xFC)   // (R) Measured Temperature
#define UangleA      	 	 (0xFD)   // (R) Phase A Voltage Phase Angle
#define UangleB      	 	 (0xFE)   // (R) Phase B Voltage Phase Angle
#define UangleC      	 	 (0xFF)   // (R) Phase C Voltage Phase Angle



/* Typedef Decleration ---------------------------------------------------- */


typedef struct{
		
//********************* Power and Power Factor Register *********************

//RMS for Voltage
double GetRMSVoltage_A; 				 //RMS for Voltage A
double GetRMSVoltage_B; 				 //RMS for Voltage B
double GetRMSVoltage_C; 				 //RMS for Voltage C

//RMS for Current
double GetRMSCurrent_A; 				 //RMS for Current A
double GetRMSCurrent_B; 				 //RMS for Current B
double GetRMSCurrent_C; 				 //RMS for Current C

//Active Power
double GetActivePower_A; 			   //Active Power A
double GetActivePower_B; 			   //Active Power B
double GetActivePower_C; 			   //Active Power C
double GetTotalActivePower; 	   //Total Active Power

//Reactive Power
double GetReactivePower_A; 		   //Reactive Power A
double GetReactivePower_B; 		   //Reactive Power B
double GetReactivePower_C; 		   //Reactive Power C
double GetTotalReactivePower;    //Total Reactive Power

//Apparent Power
double GetApparentPower_A; 		   //Apparent Power A
double GetApparentPower_B; 		   //Apparent Power B
double GetApparentPower_C; 		   //Apparent Power C
double GetTotalApparentPower;    //Total Apparent Power

//Fundamental Power
double GetFundamentalPower_A;    //Fundamental Power A
double GetFundamentalPower_B; 	 //Fundamental Power B
double GetFundamentalPower_C;    //Fundamental Power C
double GetTotalFundamentalPower; //Total Fundamental Power

//Harmonic Power
double GetHarmonicPower_A; 		 //Harmonic Power A
double GetHarmonicPower_B; 		 //Harmonic Power B
double GetHarmonicPower_C; 		 //Harmonic Power C
double GetTotalHarmonicPower;  //Total Harmonic Power 

//********************* Power Factor *********************
double GetActivePowerFactor_A;
double GetActivePowerFactor_B;
double GetActivePowerFactor_C;
double GetTotalActivePowerFactor;

//********************* Phase Angle *********************
double GetPhaseAngle_A;
double GetPhaseAngle_B;
double GetPhaseAngle_C;


double GetFreq; 							 //Frequency
double GetTemp; 							 //Temperature
double GetPeak_A;						   //Peak Value

//********************* Regular Energy Measuretment *********************
//Forward Active Energy
double GetForwardActiveEnergy_A;
double GetForwardActiveEnergy_B;
double GetForwardActiveEnergy_C;
double GetTotalForwardActiveEnergy;

//Reverse Active Energy
double GetReverseActiveEnergy_A;
double GetReverseActiveEnergy_B;
double GetReverseActiveEnergy_C;
double GetTotalReverseActiveEnergy;
	
//Forward Reactive Energy
double GetForwardReactiveEnergy_A;
double GetForwardReactiveEnergy_B;
double GetForwardReactiveEnergy_C;
double GetTotalForwardReactiveEnergy;

//Reverse Reactive Energy
double GetReverseReactiveEnergy_A;
double GetReverseReactiveEnergy_B;
double GetReverseReactiveEnergy_C;
double GetTotalReverseReactiveEnergy;

//Apparent Energy
double GetApparentEnergy_A;
double GetApparentEnergy_B;
double GetApparentEnergy_C;
double GetTotalApparentEnergy;

		
}Summary_RegTypeDef;


/* Fucntion Decleration ---------------------------------------------------- */
unsigned int RWtoRegister(unsigned char dataRW, unsigned short address, unsigned short data);

void M90E32AS_Init();
int Read32Register(signed short regh_addr, signed short regl_addr);
void printSummary(Summary_RegTypeDef *sum);
void Energy_SetBaudRate(void);

void CalVI(unsigned char channel, double Vref, double Iref);

unsigned short GetSysStatus0(void);
unsigned short GetSysStatus1(void);
unsigned short GetMeterStatus0(void);
unsigned short GetMeterStatus1(void);

//******* Power and Power Factor Fucntions //*******

//RMS for Voltage
double GetRMSVoltage_A(void); 				 //RMS for Voltage A
double GetRMSVoltage_B(void); 				 //RMS for Voltage B
double GetRMSVoltage_C(void); 				 //RMS for Voltage C

//RMS for Current
double GetRMSCurrent_A(void); 				 //RMS for Current A
double GetRMSCurrent_B(void); 				 //RMS for Current B
double GetRMSCurrent_C(void); 				 //RMS for Current C

//Active Power
double GetActivePower_A(void); 			   //Active Power A
double GetActivePower_B(void); 			   //Active Power B
double GetActivePower_C(void); 			   //Active Power C
double GetTotalActivePower(void); 	   //Total Active Power

//Reactive Power
double GetReactivePower_A(void); 		   //Reactive Power A
double GetReactivePower_B(void); 		   //Reactive Power B
double GetReactivePower_C(void); 		   //Reactive Power C
double GetTotalReactivePower(void);    //Total Reactive Power

//Apparent Power
double GetApparentPower_A(void); 		   //Apparent Power A
double GetApparentPower_B(void); 		   //Apparent Power B
double GetApparentPower_C(void); 		   //Apparent Power C
double GetTotalApparentPower(void);    //Total Apparent Power

//Fundamental Power
double GetFundamentalPower_A(void);    //Fundamental Power A
double GetFundamentalPower_B(void); 	 //Fundamental Power B
double GetFundamentalPower_C(void);    //Fundamental Power C
double GetTotalFundamentalPower(void); //Total Fundamental Power

//Harmonic Power
double GetHarmonicPower_A(void); 		 //Harmonic Power A
double GetHarmonicPower_B(void); 		 //Harmonic Power B
double GetHarmonicPower_C(void); 		 //Harmonic Power C
double GetTotalHarmonicPower(void);  //Total Harmonic Power 

//******* Power Factor Fucntions //*******
double GetActivePowerFactor_A(void);
double GetActivePowerFactor_B(void);
double GetActivePowerFactor_C(void);
double GetTotalActivePowerFactor(void);

//******* Phase Angle Functions //*******
double GetPhaseAngle_A(void);
double GetPhaseAngle_B(void);
double GetPhaseAngle_C(void);


double GetFreq(void); 							 //Frequency
double GetTemp(void); 							 //Temperature
double GetPeak_A(void);						   //Peak Value

//******* Regular Energy Measurement Functions //*******
//Forward Active Energy
double GetForwardActiveEnergy_A(void);
double GetForwardActiveEnergy_B(void);
double GetForwardActiveEnergy_C(void);
double GetTotalForwardActiveEnergy(void);

//Reverse Active Energy
double GetReverseActiveEnergy_A(void);
double GetReverseActiveEnergy_B(void);
double GetReverseActiveEnergy_C(void);
double GetTotalReverseActiveEnergy(void);
	
//Forward Reactive Energy
double GetForwardReactiveEnergy_A(void);
double GetForwardReactiveEnergy_B(void);
double GetForwardReactiveEnergy_C(void);
double GetTotalForwardReactiveEnergy(void);

//Reverse Reactive Energy
double GetReverseReactiveEnergy_A(void);
double GetReverseReactiveEnergy_B(void);
double GetReverseReactiveEnergy_C(void);
double GetTotalReverseReactiveEnergy(void);

//Apparent Energy
double GetApparentEnergy_A(void);
double GetApparentEnergy_B(void);
double GetApparentEnergy_C(void);
double GetTotalApparentEnergy(void);

#endif //__ENERGY_METER__
