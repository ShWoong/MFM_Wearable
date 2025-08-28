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
#define R_FILTER_TAU_S     2.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static float Convert_Temperature(float Vin);
static void PWM_SetDuty_TIM2_CH1(float duty);
static void process_pc_line(const char* s);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
volatile bool tim1Flag = 0;
volatile bool tim2Flag = 0;

/* UART DMA buffers & line assembler (PC↔STM32 only on USART2) */
static uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
static char line_pc[LINE_MAX];   static uint16_t line_pc_len = 0;

/******** ADS1115 (예: AD8495 온도 프런트엔드) ********/
float voltageRaw = 0.0f;
float filtered_voltage = 0.0f;

/******** ACS37800: 전력 모니터 ********/
ACS37800_Handle acs;

/* 정수(raw milli 단위) 저장용 */
volatile int32_t PACT_mW = 0;
volatile int32_t VRMS_mV = 0;
volatile int32_t IRMS_mA = 0;

/* SI 단위 값 */
volatile float P_W    = 0.0f;
volatile float Vrms_V = 0.0f;
volatile float Irms_A = 0.0f;

/******** Controller (Outer PI only) ********/
volatile float T_set_C = 25.0f;    // 기본 타겟
static ControllerParams ctrlPar = {
  .Kp_proc = 162.0f,
  .tau     = 42.8f,
  .theta   = 1.0f,
  .lambda  = 10.0f,   // 초기 λ
  .Kc      = 0.0f,    // 런타임에서 λ로 계산
  .Ti      = 42.8f,    // 런타임에서 τ로 설정
  .Tt      = 0.3f,
  .Ts      = 0.01f,   // 100 Hz
  .u_min_W = 0.0f,
  .u_max_W = 800.0f,
  .Kff     = 1.0f,
  .lam_ref = 0.0f,      // 0 → lambda를 참조 성형에도 사용
  .use_dyn_ff= true,      // 역모델 FF 켜기
  /* Safety */
  .T_soft_max = 90.0f, .T_hard_off = 95.0f, .T_hard_rst = 92.0f,

  /* Ambient estimation */
  .track_tau_s = 30.0f,
  .p_off_w     = 0.2f,
  .duty_off    = 0.02f,
  .latch_samples = 50
};
static ControllerState ctrlSt;

/* ===== Mode FSM ===== */
typedef enum { APP_IDLE=0, APP_ISOMETRIC, APP_TSTEP } AppMode_t;
typedef enum { ISO_HEAT=0, ISO_HOLD, ISO_COOL } IsoSub_t;
typedef enum { TS_GOTO=0, TS_HOLD } TstepSub_t;

static volatile AppMode_t app_mode = APP_IDLE;

/* Isometric state */
static volatile IsoSub_t iso_sub = ISO_HEAT;
static volatile uint16_t iso_hold_s = 0;
static volatile uint16_t iso_cycles = 0;
static volatile uint32_t iso_cnt = 0;    // 100 Hz ticks
static volatile uint16_t inband_counter = 0;
static float             iso_target_C = 60.0f;

#define T_BAND 0.5f
#define COOL_TARGET_C 25.0f
#define BAND_COUNT_05S 50u

/* Thermal stepping state */
static volatile TstepSub_t ts_sub = TS_GOTO;
static volatile uint16_t ts_hold_s = 0;
static volatile uint16_t ts_cycles = 0;
static volatile float    ts_start = 30.0f;
static volatile float    ts_stop  = 80.0f;
static volatile float    ts_step  = 10.0f;
static volatile float    ts_curr  = 30.0f;
static volatile int8_t   ts_dir   = +1;   // +1 up, -1 down
static volatile uint32_t ts_cnt   = 0;    // 100 Hz ticks
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
static void MX_USART3_UART_Init(void);
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1); // 500 Hz (ADS 등)
  HAL_TIM_Base_Start_IT(&htim2); // 100 Hz (제어/CSV)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  /* USART2 (PC) DMA+IDLE */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buffer, sizeof(usart2_rx_buffer));
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

  if (ADS1115_Init_Continuous(&hi2c1, ADS1115_DATA_RATE_860, ADS1115_PGA_TWO, ADS1115_MUX_AIN0) != HAL_OK){
    printf("ADS1115 Continuous Init Failed!\r\n");
    Error_Handler();
  }
  ACS37800_Init(&acs, &hi2c2, 0x60);
  ACS37800_SetTimeout(&acs, 20);
  ACS37800_SetBoardPololu(&acs, 4);
  ACS37800_SetSampleCount(&acs, 320);
  printf("ACS37800 ready (N=320)\r\n");

  /* 초기 λ로 Kc/Ti 계산 */
  ctrlPar.Kc = ctrlPar.tau / (ctrlPar.Kp_proc * (ctrlPar.lambda + ctrlPar.theta));
  ctrlPar.Ti = ctrlPar.tau;
  Controller_Init(&ctrlPar, &ctrlSt);

  PWM_SetDuty_TIM2_CH1(0.0f);
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
  /* USER CODE BEGIN WHILE */
    if (tim1Flag) {
      tim1Flag = 0;
      if (ADS1115_readContinuous(&voltageRaw) == HAL_OK){
        filtered_voltage = MAFTemp(voltageRaw);
      }
    }

    if (tim2Flag) {
      tim2Flag = 0;

      /* (1) Power/V/I from ACS */
      if (ACS37800_ReadActiveAndReactivePower(&acs) == HAL_OK) PACT_mW = acs.activePowerMilliwatts;
      if (ACS37800_ReadRMSVoltageAndCurrent(&acs) == HAL_OK){
        VRMS_mV = acs.rmsVoltageMillivolts;
        IRMS_mA = acs.rmsCurrentMilliamps;
      }
      P_W    = 0.001f * (float)PACT_mW; if (P_W < 0.0f) P_W = 0.0f;
      Vrms_V = 0.001f * (float)VRMS_mV;
      Irms_A = 0.001f * (float)IRMS_mA;

      /* (1b) Heater R estimate (filtered) */
      float R_est_ohm;
      {
        static float R_hat = HEATER_R_NOM_OHM;
        const float Ts = ctrlPar.Ts;            // 0.01 s
        const float alpha = (Ts / R_FILTER_TAU_S);
        const int   meas_ok = (Irms_A > 0.1f && Vrms_V > 0.3f);

        if (meas_ok) {
            const float R_meas = Vrms_V / (Irms_A + 1e-6f);
            R_hat += alpha * (R_meas - R_hat);
        } else {
            float T_meas_tmp = Convert_Temperature(filtered_voltage);
            const float R_fallback = HEATER_R_NOM_OHM * (1.0f + HEATER_TCR * (T_meas_tmp - HEATER_TREF_C));
            R_hat += alpha * (R_fallback - R_hat);
        }
        if (R_hat < 0.5f)  R_hat = 0.5f;
        if (R_hat > 20.0f) R_hat = 20.0f;
        R_est_ohm = R_hat;
      }

      /* (2) Temperature */
      float T_meas = Convert_Temperature(filtered_voltage);

      /* (3) App mode FSM (may update T_set_C) */
      switch (app_mode){
        case APP_ISOMETRIC: step_isometric(T_meas); break;
        case APP_TSTEP:     step_tstep(T_meas);     break;
        default: break;
      }

      /* (4) Controller step (outer PI → power → duty) */
      float duty = 0.0f, u_des_W = 0.0f;
      Controller_Step(&ctrlPar, &ctrlSt, T_set_C, T_meas, P_W, Vrms_V, R_est_ohm, &duty, &u_des_W);
      PWM_SetDuty_TIM2_CH1(duty);

      /* (5) CSV to PC: T,Tset,Power,duty%,fan%  */
      float duty_pct = duty * 100.0f;
      float fan_pct = 0.0f;
      printf("%.3f,%.3f,%.3f,%.1f,%.1f\r\n", T_meas, T_set_C, (double)P_W, (double)duty_pct, (double)fan_pct);
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
  htim1.Init.Prescaler = 5;
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
  htim2.Init.Period = 899999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance==TIM1){
    tim1Flag = 1;
  } else if(htim->Instance==TIM2){
    tim2Flag = 1;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
  if (huart == &huart2) {  // PC
    for (uint16_t i=0;i<Size;i++){
      char ch = (char)usart2_rx_buffer[i];
      if (ch=='\n' || ch=='\r'){
        if (line_pc_len>0){ line_pc[line_pc_len]='\0'; process_pc_line(line_pc); line_pc_len=0; }
      } else if (isprint((int)ch)) {
        if (line_pc_len < LINE_MAX-1) line_pc[line_pc_len++] = ch;
      }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buffer, sizeof(usart2_rx_buffer));
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  }
}

/* Commands from PC:
 * A,<TsetC>,<lambda>[,<kff>[,<Tt>]]
 * M<cycle>,<hold_s>
 * T<cycle>,<hold_s>,<Tstart>,<Tstop>,<dT>
 */
static void process_pc_line(const char* s){
  if (s[0]=='M'){
    int cyc=0, sec=0;
    sscanf(s+1,"%d,%d",&cyc,&sec);
    if (cyc<0) cyc=0;
    if (sec<0) sec=0;
    iso_cycles = (uint16_t)cyc;
    iso_hold_s = (uint16_t)sec;
    inband_counter = 0;
    iso_cnt = 0;
    iso_target_C = T_set_C;   // 현재 목표 저장
    iso_sub = ISO_HEAT;
    app_mode = APP_ISOMETRIC;
    return;
  }
  if (s[0]=='T'){
    int cyc=0, sec=0;
    float t0=30.0f, t1=80.0f, dT=10.0f;
    sscanf(s+1,"%d,%d,%f,%f,%f",&cyc,&sec,&t0,&t1,&dT);
    if (cyc<0) cyc=0;
    if (sec<0) sec=0;
    if (dT<=0.0f) dT = 1.0f;
    if (t1 < t0){ float tmp=t0; t0=t1; t1=tmp; }
    ts_cycles = (uint16_t)cyc;
    ts_hold_s = (uint16_t)sec;
    ts_start  = t0;
    ts_stop   = t1;
    ts_step   = dT;
    ts_curr   = ts_start;
    ts_dir    = +1;
    T_set_C   = ts_curr;
    ts_cnt    = 0;
    ts_sub    = TS_GOTO;
    app_mode  = APP_TSTEP;
    return;
  }
  if (s[0]=='A' && s[1]==','){
      float Tset=0.0f, lam=ctrlPar.lambda, kff=-1.0f;
      int n = sscanf(s+2, "%f,%f,%f", &Tset, &lam, &kff);

      if (n >= 1) T_set_C = Tset;

      if (n >= 2 && lam > 0.05f){
        ctrlPar.lambda = lam;
        ctrlPar.Kc = ctrlPar.tau / (ctrlPar.Kp_proc * (ctrlPar.lambda + ctrlPar.theta));
        ctrlPar.Ti = ctrlPar.tau;                // Ti 고정(=tau)
        ctrlPar.lam_ref = 0.0f;                  // 0 → lambda를 참조 성형에도 사용
      }
      if (n >= 3 && kff >= 0.0f){
        ctrlPar.Kff = kff;
      }

      Controller_ResetIntegrator(&ctrlSt);       // ★ 새 파라미터/목표 시 적분기 리셋
      return;
    }
}

static float Convert_Temperature(float Vin) {
    const float Vref = 1.25f;
    const float sensitivity = 0.005f; // 5 mV/°C
    float Temp_Celsius = (Vin - Vref) / sensitivity + 3.9f;
    return Temp_Celsius;
}

static void PWM_SetDuty_TIM2_CH1(float duty){
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 0.9999f) duty = 0.9999f;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2); // = 899999
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
