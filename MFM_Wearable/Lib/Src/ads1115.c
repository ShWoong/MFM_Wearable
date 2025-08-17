/*
 * =====================================================================================
 * ads1115.h - Header File
 * =====================================================================================
 */

#ifndef INC_ADS1115_H_
#define INC_ADS1115_H_

#include "stm32f4xx_hal.h"

/* Definitions */
#define ADS1115_OS (0b1 << 7) // Default

#define ADS1115_MUX_AIN0 (0b100 << 4)		// Analog input 0
#define ADS1115_MUX_AIN1 (0b101 << 4)		// Analog input 1
#define ADS1115_MUX_AIN2 (0b110 << 4)		// Analog input 2
#define ADS1115_MUX_AIN3 (0b111 << 4)		// Analog input 3

#define ADS1115_PGA_TWOTHIRDS 	(0b000 << 1) 		// 2/3x Gain, FSR = ±6.144V
#define ADS1115_PGA_ONE			(0b001 << 1) 		// 1x Gain,   FSR = ±4.096V
#define ADS1115_PGA_TWO			(0b010 << 1) 		// 2x Gain,   FSR = ±2.048V
#define ADS1115_PGA_FOUR		(0b011 << 1) 		// 4x Gain,   FSR = ±1.024V
#define ADS1115_PGA_EIGHT		(0b100 << 1) 		// 8x Gain,   FSR = ±0.512V
#define ADS1115_PGA_SIXTEEN		(0b111 << 1) 		// 16x Gain,  FSR = ±0.256V

#define ADS1115_MODE (0b1) // Single-Shot Mode Bit

#define ADS1115_DATA_RATE_8		(0b000 << 5)			// 8SPS
#define ADS1115_DATA_RATE_16	(0b001 << 5)			// 16SPS
#define ADS1115_DATA_RATE_32	(0b010 << 5)			// 32SPS
#define ADS1115_DATA_RATE_64	(0b011 << 5)			// 64SPS
#define ADS1115_DATA_RATE_128	(0b100 << 5)			// 128SPS
#define ADS1115_DATA_RATE_250	(0b101 << 5)			// 250SPS
#define ADS1115_DATA_RATE_475	(0b110 << 5)			// 475SPS
#define ADS1115_DATA_RATE_860	(0b111 << 5)			// 860SPS

#define ADS1115_COMP_MODE 	(0b0 << 4) // Default
#define ADS1115_COMP_POL 	(0b0 << 3) // Default
#define ADS1115_COMP_LAT 	(0b0 << 2) // Default
#define ADS1115_COMP_QUE 	(0b11)	   // Default (Comparator Disabled)

/* ADS1115 register configurations */
#define ADS1115_CONVER_REG 0x0
#define ADS1115_CONFIG_REG 0x1

/* TIMEOUT */
#define ADS1115_TIMEOUT 10 // Timeout for HAL I2C functions.

/* Function prototypes. */
// Original Library Functions
HAL_StatusTypeDef ADS1115_Init(I2C_HandleTypeDef *handler, uint16_t setDataRate, uint16_t setPGA);
HAL_StatusTypeDef ADS1115_readSingleEnded(uint16_t muxPort, float *voltage);

// Functions Added for Continuous Mode
HAL_StatusTypeDef ADS1115_Init_Continuous(I2C_HandleTypeDef *handler, uint16_t setDataRate, uint16_t setPGA, uint16_t muxPort);
HAL_StatusTypeDef ADS1115_readContinuous(float *voltage);
HAL_StatusTypeDef ADS1115_readContinuous_Raw(int16_t *rawValue);

#endif /* INC_ADS1115_H_ */


/*
 * =====================================================================================
 * ads1115.c - Source File
 * =====================================================================================
 */

#include "stm32f4xx_hal.h"
#include "string.h"
#include "ads1115.h"
#include <stdio.h> // For printf debugging if needed

/* Variables */
uint8_t ADS1115_devAddress = 0b1001000;	// 7 bit address for ADDR -> GND (0x48)

I2C_HandleTypeDef ADS1115_I2C_Handler;	// HAL I2C handler store variable.

uint16_t ADS1115_dataRate = ADS1115_DATA_RATE_128; // Default
uint16_t ADS1115_pga = ADS1115_PGA_TWO; // Default
uint16_t ADS1115_port = ADS1115_MUX_AIN0; // Default

uint8_t ADS1115_config[2];
uint8_t ADS1115_rawValue[2];
float ADS1115_voltCoef; // Voltage coefficient.

/* Function definitions. */

/**
  * @brief  Initializes the ADS1115 library and checks for device readiness.
  */
HAL_StatusTypeDef ADS1115_Init(I2C_HandleTypeDef *handler, uint16_t setDataRate, uint16_t setPGA) {

	// Copy I2C handler
	memcpy(&ADS1115_I2C_Handler, handler, sizeof(*handler));

	// Set Data rate and PGA configurations.
	ADS1115_dataRate = setDataRate;
	ADS1115_pga = setPGA;

	// Update voltage coefficient based on PGA setting. This value is in mV per bit.
	switch (ADS1115_pga) {
		case ADS1115_PGA_TWOTHIRDS:
			ADS1115_voltCoef = 0.1875;
			break;
		case ADS1115_PGA_ONE:
			ADS1115_voltCoef = 0.125;
			break;
		case ADS1115_PGA_TWO:
			ADS1115_voltCoef = 0.0625;
			break;
		case ADS1115_PGA_FOUR:
			ADS1115_voltCoef = 0.03125;
			break;
		case ADS1115_PGA_EIGHT:
			ADS1115_voltCoef = 0.015625;
			break;
		case ADS1115_PGA_SIXTEEN:
			ADS1115_voltCoef = 0.0078125;
			break;
	}

    // Check if the device is ready on the I2C bus
	if (HAL_I2C_IsDeviceReady(&ADS1115_I2C_Handler, (uint16_t) (ADS1115_devAddress << 1), 5, ADS1115_TIMEOUT) == HAL_OK) {
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

/**
  * @brief  Initializes the ADS1115 for continuous conversion mode.
  */
HAL_StatusTypeDef ADS1115_Init_Continuous(I2C_HandleTypeDef *handler, uint16_t setDataRate, uint16_t setPGA, uint16_t muxPort)
{
	// Perform basic initialization
	if (ADS1115_Init(handler, setDataRate, setPGA) != HAL_OK)
	{
		return HAL_ERROR;
	}

	// Build the configuration bytes for continuous mode (MODE bit = 0)
	ADS1115_config[0] = ADS1115_OS | muxPort | setPGA | 0; // Set MODE bit to 0 for continuous
	ADS1115_config[1] = setDataRate | ADS1115_COMP_MODE | ADS1115_COMP_POL | ADS1115_COMP_LAT| ADS1115_COMP_QUE;

	// Write the configuration to the config register
	return HAL_I2C_Mem_Write(&ADS1115_I2C_Handler, (uint16_t) (ADS1115_devAddress << 1), ADS1115_CONFIG_REG, 1, ADS1115_config, 2, ADS1115_TIMEOUT);
}

/**
  * @brief  Reads the ADC value in continuous conversion mode and converts to Voltage.
  */
HAL_StatusTypeDef ADS1115_readContinuous(float *voltage)
{
	// Read 2 bytes directly from the conversion register
	if (HAL_I2C_Mem_Read(&ADS1115_I2C_Handler, (uint16_t) (ADS1115_devAddress << 1), ADS1115_CONVER_REG, 1, ADS1115_rawValue, 2, ADS1115_TIMEOUT) == HAL_OK)
	{
		// Combine bytes, convert to voltage using the pre-calculated coefficient, and scale from mV to V
		*voltage = (float) (((int16_t) (ADS1115_rawValue[0] << 8) | ADS1115_rawValue[1]) * ADS1115_voltCoef / 1000.0f);
		return HAL_OK;
	}

	return HAL_ERROR;
}

/**
  * @brief  Reads the pure ADC raw value in continuous conversion mode.
  */
HAL_StatusTypeDef ADS1115_readContinuous_Raw(int16_t *rawValue)
{
	// Read 2 bytes directly from the conversion register
	if (HAL_I2C_Mem_Read(&ADS1115_I2C_Handler, (uint16_t) (ADS1115_devAddress << 1), ADS1115_CONVER_REG, 1, ADS1115_rawValue, 2, ADS1115_TIMEOUT) == HAL_OK)
	{
		// Combine the two bytes into a 16-bit signed integer (raw value)
		*rawValue = (int16_t) ((ADS1115_rawValue[0] << 8) | ADS1115_rawValue[1]);
		return HAL_OK;
	}

	return HAL_ERROR;
}


/**
  * @brief  Performs a single-shot read on a specified channel. (Original library function)
  */
HAL_StatusTypeDef ADS1115_readSingleEnded(uint16_t muxPort, float *voltage) {

	// Build configuration for a single-shot conversion
	ADS1115_config[0] = ADS1115_OS | muxPort | ADS1115_pga | ADS1115_MODE;
	ADS1115_config[1] = ADS1115_dataRate | ADS1115_COMP_MODE | ADS1115_COMP_POL | ADS1115_COMP_LAT| ADS1115_COMP_QUE;
	uint8_t waiting = 1;
	uint16_t cnt = 0;

	// Write config to start a new conversion
	if(HAL_I2C_Mem_Write(&ADS1115_I2C_Handler, (uint16_t) (ADS1115_devAddress << 1), ADS1115_CONFIG_REG, 1, ADS1115_config, 2, ADS1115_TIMEOUT) == HAL_OK)
	{
		// Poll the OS bit until the conversion is complete
		while(waiting)
		{
			if(HAL_I2C_Mem_Read(&ADS1115_I2C_Handler, (uint16_t) (ADS1115_devAddress << 1), ADS1115_CONFIG_REG, 1, ADS1115_config, 2, ADS1115_TIMEOUT) == HAL_OK)
			{
				if(ADS1115_config[0] & ADS1115_OS) {
					waiting = 0; // Conversion is done
				}
			}
			else return HAL_ERROR;

			if(++cnt==100) return HAL_TIMEOUT; // Timeout protection
		}

		// Read the conversion result
		if(HAL_I2C_Mem_Read(&ADS1115_I2C_Handler, (uint16_t) (ADS1115_devAddress << 1), ADS1115_CONVER_REG, 1, ADS1115_rawValue, 2, ADS1115_TIMEOUT) == HAL_OK)
		{
			// Convert raw value to voltage (in mV)
			*voltage = (float) (((int16_t) (ADS1115_rawValue[0] << 8) | ADS1115_rawValue[1]) * ADS1115_voltCoef);
			return HAL_OK;
		}
	}

	return HAL_ERROR;
}
