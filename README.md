# Atmel M90E32AS STM32F0XX Library
The [M90E32AS](http://ww1.microchip.com/downloads/en/devicedoc/Atmel-46003-SE-M90E32AS-Datasheet.pdf) is a poly-phase high performance wide-dynamic range metering IC. This library is created for STM32 microcontrollers. The M90E32AS application note can be found [here](http://ww1.microchip.com/downloads/en/AppNotes/Atmel-46103-SE-M90E32AS-ApplicationNote.pdf).

The M90E32AS provides the following measurements:
- Active / Reactive / Apparent Power
- Fundamental / Harmonic Power
- RMS for Voltage and Current
- Power Factor
- Phase Angle
- Frequency
- Temperature


## Getting Started

These instructions will get you started.

### Setup

```
#include "ENERGY_METER.h"

int main(void){
  M90E32AS_Init();
  HAL_Delay(1000);
}
```
Following code segment shows that how SPI is configurated using STM32CubeMX software.

```
static void MX_SPI1_Init(void)
{

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
```

### Prerequisites

Install HAL library for the STM32

```
#include "stm32f0xx_hal.h"
```

Install the M90E32AS Library

```
#include "ENERGY_METER.h"
```

## Running

This library is made for 3P3W system. It works under 230V/50Hz. The M90E32AS works as a slave device where STM32 works as a master device. Communication between the slave and master are established using SPI in MODE 3 - CPOL = HIGH (1) / CPHA - 2 EDGE (1).

## Built With

* [Keil MDK](http://www.keil.com/#:~:text=Keil%20MDK%20is%20the%20complete,easy%20to%20learn%20and%20use.) - STM32CubeMX used for configuration

## Functions

### Metering Functions
#### Regular Energy Functions
```
//Forward Active Energy
double GetForwardActiveEnergy_A(void);
double GetForwardActiveEnergy_B(void);
double GetForwardActiveEnergy_C(void);
double GetTotalForwardActiveEnergy(void);
```
```
//Reverse Active Energy
double GetReverseActiveEnergy_A(void);
double GetReverseActiveEnergy_B(void);
double GetReverseActiveEnergy_C(void);
double GetTotalReverseActiveEnergy(void);
```
```
//Forward Reactive Energy
double GetForwardReactiveEnergy_A(void);
double GetForwardReactiveEnergy_B(void);
double GetForwardReactiveEnergy_C(void);
double GetTotalForwardReactiveEnergy(void);
```
```
//Reverse Reactive Energy
double GetReverseReactiveEnergy_A(void);
double GetReverseReactiveEnergy_B(void);
double GetReverseReactiveEnergy_C(void);
double GetTotalReverseReactiveEnergy(void);
```
```
//Apparent Energy
double GetApparentEnergy_A(void);
double GetApparentEnergy_B(void);
double GetApparentEnergy_C(void);
double GetTotalApparentEnergy(void);
```

### Measurement Functions
#### Active / Reactive / Apparent Power
```
//Active Power
double GetActivePower_A(void); 			  
double GetActivePower_B(void); 			   
double GetActivePower_C(void); 			  
double GetTotalActivePower(void); 	   
```
```
//Reactive Power
double GetReactivePower_A(void); 		 
double GetReactivePower_B(void); 		  
double GetReactivePower_C(void); 		  
double GetTotalReactivePower(void);    
```
```
//Apparent Power
double GetApparentPower_A(void); 		   
double GetApparentPower_B(void); 		   
double GetApparentPower_C(void); 		  
double GetTotalApparentPower(void);    
```
#### Fundamental / Harmonic Power
```
//Fundamental Power
double GetFundamentalPower_A(void);    
double GetFundamentalPower_B(void); 	
double GetFundamentalPower_C(void);   
double GetTotalFundamentalPower(void);
```
```
//Harmonic Power
double GetHarmonicPower_A(void); 		 
double GetHarmonicPower_B(void); 		 
double GetHarmonicPower_C(void); 		 
double GetTotalHarmonicPower(void);  
```
#### RMS for Voltage and Current
```
//RMS for Voltage
double GetRMSVoltage_A(void); 				 
double GetRMSVoltage_B(void); 				 
double GetRMSVoltage_C(void); 				 
```
```
//RMS for Current
double GetRMSCurrent_A(void); 				 
double GetRMSCurrent_B(void); 				
double GetRMSCurrent_C(void); 				 
```
#### Power Factor
```
double GetActivePowerFactor_A(void);
double GetActivePowerFactor_B(void);
double GetActivePowerFactor_C(void);
double GetTotalActivePowerFactor(void);
```
#### Phase Angle
```
double GetPhaseAngle_A(void);
double GetPhaseAngle_B(void);
double GetPhaseAngle_C(void);
```
#### Frequency
```
double GetFreq(void); 	
```
#### Temperature
```
double GetTemp(void);
```

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
