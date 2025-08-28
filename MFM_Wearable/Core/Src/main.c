/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdbool.h>

#include "ads1115.h"
#include "filters.h"
#include "acs37800.h"
#include "controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART2_RX_BUFFER_SIZE 256
#define LINE_MAX              128

// ===== Heater electrical model =====
#define HEATER_R_NOM_OHM   3.2f
#define HEATER_TREF_C      25.0f
#define HEATER_TCR         0.00020f
#define R_FILTER_TAU_S     0.05f

// 상시 공급 전압 (내부 전력루프의 P↔duty 근사식에 사용)
#define VBUS_V             40.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static float Convert_Temperature(float Vin);
static void PWM_SetDuty_TIM2_CH1(float duty);
static void process_pc_line(const char* s);
static void uart2_poll_lines(void);
static inline void power_io_tick(void);
static void inner_power_tick(void);
static void PowerPI_RecalcGains(void);
static void outer_temp_tick(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c2_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
volatile bool tim1Flag = 0;
volatile uint8_t loopCount = 0;

/* UART DMA buffers & line assembler (PC↔STM32 only on USART2) */
static uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
#define RX_RING_SZ 512
static uint8_t rx_ring[RX_RING_SZ];
static volatile uint16_t rx_head=0, rx_tail=0;
static char line_pc[LINE_MAX];   static uint16_t line_pc_len = 0;

/******** ADS1115 (온도) ********/
float voltageRaw = 0.0f;
float filtered_voltage = 0.0f;

/******** ACS37800: 전력 모니터 ********/
ACS37800_Handle acs;

/* SI 단위 값 (내부루프 사용) */
volatile float P_W    = 0.0f;
volatile float Vrms_V = 0.0f;
volatile float Irms_A = 0.0f;

/******** 외부 온도루프 (IMC-PI & FF) ********/
volatile float T_set_C = 25.0f;
static ControllerParams ctrlPar = {
		.Kp_proc = 10.34f,
		.tau=4.88f,
		.theta=0.64f,
		.lambda  = 15.0f,
		.Kc=0.0f,
		.Ti=42.8f,
		.Tt=0.5f,
		.Ts=0.01f,
		.u_min_W = 0.0f,
		.u_max_W=800.0f,
		.Kff     = 4.0f,
		.lam_ref = 0.0f,
		.use_dyn_ff=true,
		.theta_samp_max=799,
		.T_soft_max=85.0f,
		.T_hard_off=90.0f,
		.T_hard_rst=87.0f,
		.track_tau_s=30.0f,
		.p_off_w=0.02f,
		.duty_off=0.02f
};
static ControllerState ctrlSt;

/* ===== 모드 FSM ===== */
typedef enum { APP_IDLE=0, APP_ISOMETRIC, APP_TSTEP } AppMode_t;
typedef enum { ISO_HEAT=0, ISO_HOLD, ISO_COOL } IsoSub_t;
typedef enum { TS_GOTO=0, TS_HOLD } TstepSub_t;
static volatile AppMode_t app_mode = APP_IDLE;
static volatile IsoSub_t iso_sub = ISO_HEAT;
static volatile uint16_t iso_hold_s = 0;
static volatile uint16_t iso_cycles = 0;
static volatile uint32_t iso_cnt = 0;    // 100 Hz ticks
static volatile uint16_t inband_counter = 0;
static float             iso_target_C = 60.0f;
#define T_BAND 0.5f
#define COOL_TARGET_C 25.0f
#define BAND_COUNT_05S 50u

static volatile TstepSub_t ts_sub = TS_GOTO;
static volatile uint16_t ts_hold_s = 0;
static volatile uint16_t ts_cycles = 0;
static volatile float    ts_start = 30.0f;
static volatile float    ts_stop  = 80.0f;
static volatile float    ts_step  = 10.0f;
static volatile float    ts_curr  = 30.0f;
static volatile int8_t   ts_dir   = +1;
static volatile uint32_t ts_cnt   = 0;

/* ===== 내부(전력) 루프 PI ===== */
typedef struct { float kp, ki, Ts, Tt, i, umin, umax; } PowerPI_t;
static PowerPI_t powPI = { .kp=2e-3f, .ki=1e-2f, .Ts=0.001f, .Tt=0.02f, .i=0.0f, .umin=0.0f, .umax=0.9990f };

static volatile float P_ref_W = 0.0f;   // 외부 루프 전력 목표 (100 Hz)
static volatile float duty_cmd = 0.0f;  // 내부 루프 듀티 (1 kHz)
static float R_est_run = HEATER_R_NOM_OHM;

/* 전력센서 읽기 상태기계 */
static uint8_t acs_phase = 0;

/* 전력루프 대표 람다(사용자 조정) */
static float lam_pwr_s = 0.02f; /* [s] */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char* p, int len){
	HAL_UART_Transmit(&huart2, (uint8_t*)p, len, 10);
	return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline bool at_target_band(float T_meas, float T_target){
  return fabsf(T_meas - T_target) <= T_BAND;
}

static void step_isometric(float T_meas){
  switch (iso_sub){
    case ISO_HEAT:
      if (at_target_band(T_meas, T_set_C)){
        if (inband_counter < 10000) inband_counter++;
      } else {
        inband_counter = 0;
      }
      if (inband_counter >= BAND_COUNT_05S){ // 0.5s in band
        iso_cnt = (uint32_t)iso_hold_s * 100u;
        iso_sub = ISO_HOLD;
      }
      break;

    case ISO_HOLD:
      if (iso_cnt > 0) {
        iso_cnt--;
      } else {
        T_set_C = COOL_TARGET_C;
        inband_counter = 0;
        iso_sub = ISO_COOL;
      }
      break;

    case ISO_COOL:
      if (T_meas <= (COOL_TARGET_C + T_BAND)){
        if (inband_counter < 10000) inband_counter++;
      } else {
        inband_counter = 0;
      }
      if (inband_counter >= BAND_COUNT_05S){
        if (iso_cycles > 0) iso_cycles--;
        if (iso_cycles == 0){
          app_mode = APP_IDLE;
        } else {
          T_set_C = iso_target_C;
          inband_counter = 0;
          iso_sub = ISO_HEAT;
        }
      }
      break;
  }
}

static void step_tstep(float T_meas){
  switch (ts_sub){
    case TS_GOTO:
      if (at_target_band(T_meas, ts_curr)){
        ts_cnt = (uint32_t)ts_hold_s * 100u;
        ts_sub = TS_HOLD;
      }
      break;

    case TS_HOLD:
      if (ts_cnt > 0){
        ts_cnt--;
        break;
      }
      if (ts_dir > 0){ // going up
        if (ts_curr < ts_stop - 1e-6f){
          float next = ts_curr + ts_step;
          if (next > ts_stop) next = ts_stop;
          ts_curr = next; T_set_C = ts_curr; ts_sub = TS_GOTO;
        } else {
          ts_dir = -1;
          float next = ts_curr + ts_dir*ts_step;
          if (next < ts_start) next = ts_start;
          ts_curr = next; T_set_C = ts_curr; ts_sub = TS_GOTO;
        }
      } else { // going down
        if (ts_curr > ts_start + 1e-6f){
          float next = ts_curr - ts_step;
          if (next < ts_start) next = ts_start;
          ts_curr = next; T_set_C = ts_curr; ts_sub = TS_GOTO;
        } else {
          if (ts_cycles > 0) ts_cycles--;
          if (ts_cycles == 0){
            app_mode = APP_IDLE;
          } else {
            ts_dir = +1;
            float next = ts_curr + ts_step;
            if (next > ts_stop) next = ts_stop;
            ts_curr = next; T_set_C = ts_curr; ts_sub = TS_GOTO;
          }
        }
      }
      break;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	/* USART2 (PC) DMA+IDLE */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buffer, sizeof(usart2_rx_buffer));
	__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

	/* ADS1115 연속 변환 */
	if (ADS1115_Init_Continuous(&hi2c1, ADS1115_DATA_RATE_860, ADS1115_PGA_TWO, ADS1115_MUX_AIN0) != HAL_OK){
		printf("ADS1115 Continuous Init Failed!\r\n");
		Error_Handler();
	}

	/* ACS37800 설정 (N=32) */
	ACS37800_Init(&acs, &hi2c2, 0x60);
	ACS37800_SetTimeout(&acs, 20);
	ACS37800_SetBoardPololu(&acs, 4);
	ACS37800_SetSampleCount(&acs, 8);          // ★ N=32
	printf("ACS37800 ready (N=8)\r\n");

	/* 외부루프 IMC 파라미터 계산 & 상태 초기화 */
	Controller_SetLambda(&ctrlPar, &ctrlSt, ctrlPar.lambda);
	Controller_Init(&ctrlPar, &ctrlSt);

	PowerPI_RecalcGains();

	/* 초기 PWM 0% */
	PWM_SetDuty_TIM2_CH1(0.0f);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/* PC 명령 처리(메인루프) */
		uart2_poll_lines();

		if (tim1Flag) {
			tim1Flag = 0;

			/* 1 kHz: 전력센서 I/O + 내부 전력 루프 */
			power_io_tick();
			inner_power_tick();

			/* 100 Hz: 외부 온도 루프 */
			if (++loopCount >= 10) {
				loopCount = 0;
				outer_temp_tick();
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 59999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static inline float clampf(float x, float lo, float hi){
	if (x < lo) return lo; if (x > hi) return hi; return x;
}

/* 1) 타이머 틱(1 kHz) 플래그 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM1){
		tim1Flag = 1;
	}
}

/* 2) UART2 RX(IDLE+DMA) → 링버퍼 적재(파싱은 메인루프) */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if (huart == &huart2) {
		for (uint16_t i=0;i<Size;i++){
			uint16_t nxt = (rx_head+1) % RX_RING_SZ;
			if (nxt != rx_tail){ rx_ring[rx_head] = usart2_rx_buffer[i]; rx_head = nxt; }
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buffer, sizeof(usart2_rx_buffer));
		__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
	}
}

/* 3) PC 명령 라인 파싱(메인루프 호출) */
static void process_pc_line(const char* s); // 후방선언
static void uart2_poll_lines(void){
	while (rx_tail != rx_head){
		char ch = (char)rx_ring[rx_tail]; rx_tail = (rx_tail+1) % RX_RING_SZ;
		if (ch=='\n' || ch=='\r'){
			if (line_pc_len>0){ line_pc[line_pc_len]='\0'; process_pc_line(line_pc); line_pc_len=0; }
		} else if (isprint((int)ch)) {
			if (line_pc_len < LINE_MAX-1) line_pc[line_pc_len++] = ch;
		}
	}
}

/* 4) 전력루프 PI 게인 자동 계산 (VBUS=40V, IMC풍) */
static void PowerPI_RecalcGains(void){
	float R = (R_est_run > 0.05f) ? R_est_run : 0.05f;
	float Kp_pwr = (VBUS_V*VBUS_V) / R;            // [W per duty]
	float theta_p = 2.0f * powPI.Ts;               // 소지연 보정(~2Ts)
	float lam = clampf(lam_pwr_s, 0.002f, 2.0f);
	float Kc = 1.0f / (Kp_pwr * (lam + theta_p));
	powPI.kp = Kc;
	powPI.ki = Kc / lam;
	/* powPI.Tt는 초기값(0.02s) 유지 */
}

/* 5) 전력센서 I/O(1 kHz 분산) + R 추정 + 게인 갱신 */
static inline void power_io_tick(void){
	switch (acs_phase){
	case 0:
		if (ACS37800_ReadActiveAndReactivePower(&acs) == HAL_OK){
			P_W = 0.001f * (float)acs.activePowerMilliwatts;
			if (P_W < 0.0f) P_W = 0.0f;
		}
		acs_phase = 1;
		break;

	case 1:
		if (ACS37800_ReadRMSVoltageAndCurrent(&acs) == HAL_OK){
			Vrms_V = 0.001f * (float)acs.rmsVoltageMillivolts;
			Irms_A = 0.001f * (float)acs.rmsCurrentMilliamps;
		}
		acs_phase = 2;
		break;

	case 2: {
		const float alpha = powPI.Ts / R_FILTER_TAU_S; // 0.001 / 2.0
		float Ttmp = Convert_Temperature(filtered_voltage);
		float R_meas = (Irms_A > 0.1f && Vrms_V > 0.3f)
                		   ? (Vrms_V/(Irms_A+1e-6f))
                				   : HEATER_R_NOM_OHM*(1.0f + HEATER_TCR*(Ttmp-HEATER_TREF_C));
		R_est_run += alpha * (R_meas - R_est_run);
		R_est_run = clampf(R_est_run, 0.5f, 20.0f);

		PowerPI_RecalcGains();
		acs_phase = 0;
	} break;
	}
}

/* 6) 내부 전력루프(1 kHz): 센서 전력(Prms) 피드백 */
static void inner_power_tick(void){
	float Pref  = P_ref_W;
	float Pmeas = P_W;
	float e = Pref - Pmeas;

	float u = /*duty_ff +*/ powPI.kp*e + powPI.i;
	float u_sat = clampf(u, powPI.umin, powPI.umax);

	float i_dot = powPI.ki*e + (u_sat - u)/powPI.Tt;
	powPI.i += powPI.Ts * i_dot;

	duty_cmd = u_sat;
	PWM_SetDuty_TIM2_CH1(duty_cmd);
}

/* 7) 외부 온도루프(100 Hz): IMC-PI+FF → 전력목표 산출 */
static void outer_temp_tick(void){
	if (ADS1115_readContinuous(&voltageRaw) == HAL_OK){
		filtered_voltage = MAFTemp(voltageRaw);
	}
	float T_meas = Convert_Temperature(filtered_voltage);

	switch (app_mode){
	case APP_ISOMETRIC: step_isometric(T_meas); break;
	case APP_TSTEP:     step_tstep(T_meas);     break;
	default: break;
	}

	float u_des_W = 0.0f, duty_hint = 0.0f;
	ctrlSt.duty_last = duty_cmd;
	Controller_Step(&ctrlPar, &ctrlSt, T_set_C, T_meas, P_W, VBUS_V, R_est_run,
			&duty_hint, &u_des_W);
	P_ref_W = u_des_W;

	printf("%.3f,%.3f,%.3f,%.1f,%.1f\r\n",T_meas, T_set_C, (double)P_W, (double)(duty_cmd*100.0f), 0.0);
}

/* 8) PC 명령 — 대표 파라미터만 튜닝 */
static void process_pc_line(const char* s){
	if (s[0]=='M'){ // Isometric
		int cyc=0, sec=0; sscanf(s+1,"%d,%d",&cyc,&sec);
		if (cyc<0) cyc=0; if (sec<0) sec=0;
		iso_cycles=(uint16_t)cyc; iso_hold_s=(uint16_t)sec;
		inband_counter=0; iso_cnt=0; iso_target_C=T_set_C; iso_sub=ISO_HEAT; app_mode=APP_ISOMETRIC; return;
	}
	if (s[0]=='T'){ // Thermal step
		int cyc=0, sec=0; float t0=30, t1=80, dT=10;
		sscanf(s+1,"%d,%d,%f,%f,%f",&cyc,&sec,&t0,&t1,&dT);
		if (cyc<0) cyc=0; if (sec<0) sec=0; if (dT<=0) dT=1;
		if (t1 < t0){ float tmp=t0; t0=t1; t1=tmp; }
		ts_cycles=(uint16_t)cyc; ts_hold_s=(uint16_t)sec;
		ts_start=t0; ts_stop=t1; ts_step=dT;
		ts_curr=ts_start; ts_dir=+1; T_set_C=ts_curr; ts_cnt=0; ts_sub=TS_GOTO; app_mode=APP_TSTEP; return;
	}
	if (s[0]=='R'){ Controller_ResetIntegrator(&ctrlSt); powPI.i=0.0f; return; }

	if (s[0]=='A' && s[1]==','){
	    float v[8]={0};
	    int n = sscanf(s+2, "%f,%f,%f,%f,%f,%f,%f,%f",
	                   &v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6],&v[7]);
	    /* 0:Tset */
	    if (n>=1){ T_set_C = v[0]; }

	    /* 1: λ_outer (IMC-PI) */
	    if (n>=2){ Controller_SetLambda(&ctrlPar, &ctrlSt, v[1]); }

	    /* 2: kff (feedforward gain) */
	    if (n>=3){ Controller_SetKff(&ctrlPar, v[2]); }

	    /* 3: Tt_outer (anti-windup time const) */
	    if (n>=4){ ctrlPar.Tt = clampf(v[3], 0.05f, 10.0f); }

	    /* 4: λ_power (inner power-loop) */
	    if (n>=5){
	      lam_pwr_s = clampf(v[4], 0.002f, 2.0f);
	      PowerPI_RecalcGains();
	      powPI.i = 0.0f;
	    }

	    /* 5,6: Duty min/max (GUI가 ratio 또는 % 둘 다 허용) */
	    if (n>=6){
	      float umin = v[5];
	      float umax = (n>=7) ? v[6] : powPI.umax;
	      /* 퍼센트로 왔으면 0~100 → 0~1 변환, ratio면 그대로 */
	      if (umin > 1.0f || umax > 1.0f) { umin *= 0.01f; umax *= 0.01f; }
	      powPI.umin = clampf(umin, 0.0f, 0.9990f);
	      powPI.umax = clampf(umax, powPI.umin, 0.9990f);
	    }

	    /* 7: Tt_power (inner anti-windup) */
	    if (n>=8){
	      powPI.Tt = clampf(v[7], 0.005f, 5.0f);
	    }
	    return;
	  }

	if (s[0]=='U' && s[1]==','){ // 듀티 한계
		float umin=powPI.umin, umax=powPI.umax;
		if (sscanf(s+2, "%f,%f", &umin, &umax) >= 1){
			powPI.umin = clampf(umin, 0.0f, 0.9990f);
			powPI.umax = clampf(umax, powPI.umin, 0.9990f);
		}
		return;
	}

	if (s[0]=='G' && s[1]==','){ // 공정모델
		float Kp=ctrlPar.Kp_proc, tau=ctrlPar.tau, th=ctrlPar.theta;
		int n = sscanf(s+2, "%f,%f,%f", &Kp, &tau, &th);
		if (n>=1) ctrlPar.Kp_proc = Kp;
		if (n>=2) ctrlPar.tau     = tau;
		if (n>=3) ctrlPar.theta   = th;
		Controller_UpdatePlant(&ctrlPar, &ctrlSt, ctrlPar.Kp_proc, ctrlPar.tau, ctrlPar.theta);
		return;
	}
}

/* 9) 변환함수 & PWM 설정 */
static float Convert_Temperature(float Vin) {
	const float Vref = 1.25f;
	const float sensitivity = 0.005f; // 5 mV/°C
	float Temp_Celsius = (Vin - Vref) / sensitivity + 3.9f;
	return Temp_Celsius;
}

static void PWM_SetDuty_TIM2_CH1(float duty){
	if (duty < 0.0f) duty = 0.0f;
	if (duty > 0.9999f) duty = 0.9999f;
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
	uint32_t ccr = (uint32_t)((arr + 1) * duty);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
