/*
 * Library Name: 	ADS1115 STM32 Single-Ended, Single-Shot, PGA & Data Rate Enabled HAL Library
 * Written By:		Ahmet Batuhan Günaltay
 * Date Written:	02/04/2021
 * Description:		STM32F4 HAL-Based ADS1115 Library (Modified for Continuous Mode)
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
