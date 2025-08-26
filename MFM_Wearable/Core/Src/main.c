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
#define USART3_RX_BUFFER_SIZE 128
#define LINE_MAX              128

// ===== Heater electrical model (user-provided) =====
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
static void process_arduino_line(const char* s);
static void uart3_send(const char* s);
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

/* UART DMA buffers & line assemblers */
static uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
static uint8_t usart3_rx_buffer[USART3_RX_BUFFER_SIZE];
static char line_pc[LINE_MAX];   static uint16_t line_pc_len = 0;
static char line_ard[LINE_MAX];  static uint16_t line_ard_len = 0;

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

/******** Control: two-loop (Temp→Power, Power→Duty) ********/
volatile float T_set_C = 25.0f;    // 기본 타겟
static ControllerParams ctrlPar = {
  .Kp_proc = 162.0f,
  .tau     = 42.8f,
  .theta   = 1.0f,
  .lambda  = 10.0f,
  .Kc      = 0.0f,
  .Ti      = 0.0f,
  .Tt      = 0.0f,
  .lam_ff  = 2.0f,
  .Ts      = 0.01f,  // 100 Hz
  .u_min_W = 0.0f,
  .u_max_W = 800.0f,

  /* Inner (Power→Duty) */
  .KcP = 0.0060f, .TiP = 1.0f, .TtP = 0.5f,
  .duty_min = 0.0f, .duty_max = 0.95f, .du_max = 1.0f,

  /* Safety */
  .T_soft_max = 90.0f, .T_hard_off = 95.0f, .T_hard_rst = 92.0f,

  /* Ambient estimation */
  .track_tau_s = 30.0f,
  .p_off_w     = 0.2f,
  .duty_off    = 0.02f,
  .latch_samples = 50
};
static ControllerState ctrlSt;

/* Arduino → STM32 latest telem */
static volatile float Force_N = 0.0f;
static volatile float Length_mm = 0.0f;

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
static float             iso_target_C = 60.0f; // ★ 이소메트릭 가열 목표(Apply로 설정된 값 저장)

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
static void step_isometric(float T_meas);
static void step_tstep(float T_meas);
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

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

  /* UART DMA+IDLE */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buffer, sizeof(usart2_rx_buffer));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_rx_buffer, sizeof(usart3_rx_buffer));
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);

  if (ADS1115_Init_Continuous(&hi2c1, ADS1115_DATA_RATE_860, ADS1115_PGA_TWO, ADS1115_MUX_AIN0) != HAL_OK){
    printf("ADS1115 Continuous Init Failed!\r\n");
    Error_Handler();
  }
  ACS37800_Init(&acs, &hi2c2, 0x60);
  ACS37800_SetTimeout(&acs, 20);
  ACS37800_SetBoardPololu(&acs, 4);
  ACS37800_SetSampleCount(&acs, 320);
  printf("ACS37800 ready (N=320)\r\n");

  Controller_Init(&ctrlPar, &ctrlSt);
  PWM_SetDuty_TIM2_CH1(0.0f);
  /* USER CODE END 2 */

  while (1)
  {
    if (tim1Flag) {
      tim1Flag = 0;
      if (ADS1115_readContinuous(&voltageRaw) == HAL_OK){
        filtered_voltage = MAFTemp(voltageRaw);
      }
    }

    if (tim2Flag) {
      tim2Flag = 0;

      /* (1) Power/V/I */
      if (ACS37800_ReadActiveAndReactivePower(&acs) == HAL_OK) PACT_mW = acs.activePowerMilliwatts;
      if (ACS37800_ReadRMSVoltageAndCurrent(&acs) == HAL_OK){
        VRMS_mV = acs.rmsVoltageMillivolts;
        IRMS_mA = acs.rmsCurrentMilliamps;
      }
      P_W    = 0.001f * (float)PACT_mW; if (P_W < 0.0f) P_W = 0.0f;
      Vrms_V = 0.001f * (float)VRMS_mV;
      Irms_A = 0.001f * (float)IRMS_mA;
      float R_est_ohm;

      /* --- Heater resistance estimate (for duty feedforward) --- */
      {
        static float R_hat = HEATER_R_NOM_OHM;
        const float Ts = ctrlPar.Ts;            // 0.01 s
        const float alpha = (Ts / R_FILTER_TAU_S);
        const int   meas_ok = (Irms_A > 0.3f && Vrms_V > 1.0f);

        if (meas_ok) {
            const float R_meas = Vrms_V / (Irms_A + 1e-6f);
            R_hat += alpha * (R_meas - R_hat);
        } else {
            /* BUGFIX: fallback는 T_meas 기반 */
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

      /* (3) App mode steppers (may update T_set_C) */
      switch (app_mode){
        case APP_ISOMETRIC: step_isometric(T_meas); break;
        case APP_TSTEP:     step_tstep(T_meas);     break;
        default: break; // APP_IDLE → T_set_C 유지
      }

      /* (4) Controller step */
      float duty = 0.0f, u_des_W = 0.0f;
      Controller_Step(&ctrlPar, &ctrlSt, T_set_C, T_meas, P_W, Vrms_V, R_est_ohm, &duty, &u_des_W);
      PWM_SetDuty_TIM2_CH1(duty);

      /* (5) CSV to PC: T,Tset,Power,duty%,fan%  */

      float duty_pct = duty * 100.0f;
      float fan_pct = 0.0f;
      printf("%.3f,%.3f,%.3f,%.1f,%.1f\r\n",T_meas, T_set_C, (double)P_W, (double)duty_pct, (double)fan_pct);
    }
  }
}

/* ======= Mode helpers ======= */
static inline bool at_target_band(float T_meas, float T_target){
  return fabsf(T_meas - T_target) <= T_BAND;
}

/* Isometric: heat→hold→cool(25°C) → repeat */
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
        // go cool
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
        // one cycle done
        if (iso_cycles > 0) iso_cycles--;
        if (iso_cycles == 0){
          app_mode = APP_IDLE;
        } else {
          // 다음 사이클: 다시 가열 목표로 복귀
          T_set_C = iso_target_C;   // ★ 저장해둔 이소메트릭 목표로 복원
          inband_counter = 0;
          iso_sub = ISO_HEAT;
        }
      }
      break;
  }
}

/* Thermal stepping: up→hold→...→top hold→down→hold→...→bottom hold → cycles-- */
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

      // decide next target after hold
      if (ts_dir > 0){ // going up
        if (ts_curr < ts_stop - 1e-6f){
          float next = ts_curr + ts_step;
          if (next > ts_stop) next = ts_stop;
          ts_curr = next; T_set_C = ts_curr; ts_sub = TS_GOTO;
        } else {
          // at top → flip direction
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
          // reached bottom → one full up&down cycle completed
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

/* System init functions (generated by CubeMX) --------------------------------*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) { Error_Handler(); }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 5;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 59999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 899999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
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
  } else if (huart == &huart3) { // Arduino
    for (uint16_t i=0;i<Size;i++){
      char ch = (char)usart3_rx_buffer[i];
      if (ch=='\n' || ch=='\r'){
        if (line_ard_len>0){ line_ard[line_ard_len]='\0'; process_arduino_line(line_ard); line_ard_len=0; }
      } else if (isprint((int)ch)) {
        if (line_ard_len < LINE_MAX-1) line_ard[line_ard_len++] = ch;
      }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_rx_buffer, sizeof(usart3_rx_buffer));
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }
}

/* Apply / Modes parser
 * Apply: "Tset,Disp,P,I,D,Lambda,g_inner"
 * M : "M<cycle>,<time>"
 * T : "T<cycle>,<time>,<Tstart>,<Tstop>,<dT>"
 * H : (Isothermal) → STM32에서는 무시 (Arduino 전용)
 * C : from GUI → forward "Z\n" to Arduino (home/zero)
 */
static void process_pc_line(const char* s){
  if (s[0]=='C' && s[1]=='\0') {
    uart3_send("Z\n");
    return;
  }
  if (s[0]=='M'){
    int cyc=0, sec=0;
    sscanf(s+1,"%d,%d",&cyc,&sec);
    if (cyc<0) cyc=0;
    if (sec<0) sec=0;
    iso_cycles = (uint16_t)cyc;
    iso_hold_s = (uint16_t)sec;
    inband_counter = 0;
    iso_cnt = 0;
    // 이소메트릭 가열 목표는 현재 T_set_C를 저장
    iso_target_C = T_set_C;
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
  if (s[0]=='H'){
    // Isothermal은 Arduino 전용 → STM32는 무시
    return;
  }

  // Apply (7 floats)
  float Tset=0, Disp=0, P=0, I=0, D=0, Lam=0, g_inner=1.0f;
  int n = sscanf(s, "%f,%f,%f,%f,%f,%f,%f", &Tset,&Disp,&P,&I,&D,&Lam,&g_inner);
  if (n==7){
    T_set_C       = Tset;
    ctrlPar.lambda= Lam;
    ctrlPar.Kc    = P;
    ctrlPar.Ti    = I;
    ctrlPar.KcP   = 0.0060f * g_inner;

    // Arduino로 변위 set (수동 이동/홈은 별도 명령)
    char msg[32];
    snprintf(msg, sizeof(msg), "D%.2f\n", Disp);
    uart3_send(msg);
  }
}

static void process_arduino_line(const char* s){
  // Force,Length or "DONE"
  if (strcmp(s,"DONE")==0){
    return;
  }
  float f=0, l=0;
  if (sscanf(s, "%f,%f", &f, &l) == 2){
    Force_N  = f;
    Length_mm= l;
  }
}

static void uart3_send(const char* s){
  HAL_UART_Transmit(&huart3, (uint8_t*)s, (uint16_t)strlen(s), 10);
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

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif /* USE_FULL_ASSERT */
