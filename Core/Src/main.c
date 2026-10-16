/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include "stm32f103xb.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "projdefs.h"
#include "semphr.h"

#include "Modbus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CW_PIN           GPIO_PIN_10   // Both are from bank B
#define EN_PIN           GPIO_PIN_11
#define MOTOR_EN         (1u << 0u)    // Coil 0
#define MOTOR_CW         (1u << 1u)    // Coil 1
#define MOTION_TRACK_EN  (1u << 2u)    // Coil 2
#define MOTION_ZERO      (1u << 3u)    // Coil 3
#define WHEEL_RADIUS_MM 30.0
// shortcut: full stepping and 1:1 gearing assumed; update these after driver/gear calibration.
#define MOTOR_PULSES_PER_REV 200.0
#define MOTOR_REVS_PER_WHEEL_REV 3.7
#define MOTION_MM_PER_PULSE (2.0 * 3.141592653589793 * WHEEL_RADIUS_MM / \
                             (MOTOR_PULSES_PER_REV * MOTOR_REVS_PER_WHEEL_REV))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
osThreadId_t motorControlTaskHandle;

const osThreadAttr_t motorControlTask_attributes = {
  .name = "motorControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t motionTrackingTaskHandle;
const osThreadAttr_t motionTrackingTask_attributes = {
  .name = "motionTrackingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static volatile bool motionTrackingEnabled;
static volatile bool motionMotorRunning;
static volatile int32_t motionDirection = 1;
static volatile uint64_t motionPosition; // Unsigned storage preserves two's-complement wrap.
static volatile int32_t motionIntervalPulses;
static volatile uint64_t travelRemainingPulses;
static volatile bool travelComplete;
static volatile bool travelStopFailed;

modbusHandler_t modbusHandle;
uint16_t modbusCoilsData[1];            // 16 coils, Modbus addresses 0-15
uint16_t modbusDiscreteInputsData[1];   // 16 discrete inputs, Modbus addresses 100-115
uint16_t modbusInputRegistersData[6];   // Speed mm/s 200-201, position mm 202-205
uint16_t modbusHoldingRegistersData[5] = {100}; // Speed mm/s at 300; signed relative mm at 301-304

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void MotorControlTask(void *argument);
void MotionTrackingTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}

void ModbusComm_Init(void)
{
  modbusHandle.uModbusType = MB_SLAVE;
  modbusHandle.port = &huart1;
  modbusHandle.u8id = 17;
  modbusHandle.u16timeOut = 1000;
  modbusHandle.EN_Port = NULL;
  modbusHandle.u16regs = NULL;
  modbusHandle.u16regsize = 0;
  modbusHandle.xTypeHW = USART_HW;

  // Coils — FC1, FC5, FC15
  // 0th coil is EN for motor
  // 1st coil is CW for motor
  // 2nd coil enables pulse speed/position tracking
  // 3rd coil requests zeroing; cleared after completion
  modbusHandle.u16coils            = modbusCoilsData;
  modbusHandle.u16coilsStartAdd    = 0;
  modbusHandle.u16coilsNregs       = 1;

  // Discrete Inputs — FC2
  modbusHandle.u16discreteInputs          = modbusDiscreteInputsData;
  modbusHandle.u16discreteInputsStartAdd  = 100;
  modbusHandle.u16discreteInputsNregs     = 1;

  // Input Registers — FC4
  modbusHandle.u16inputRegs          = modbusInputRegistersData;
  modbusHandle.u16inputRegsStartAdd  = 200;
  modbusHandle.u16inputRegsNregs     = 6;

  // Holding Registers — FC3, FC6, FC16
  modbusHandle.u16holdingRegs          = modbusHoldingRegistersData;
  modbusHandle.u16holdingRegsStartAdd  = 300;
  modbusHandle.u16holdingRegsNregs     = 5;

  ModbusInit(&modbusHandle);
  ModbusStart(&modbusHandle);
}

static void RecordMotionPulse(void)
{
  if (!motionMotorRunning) return;
  if (motionTrackingEnabled)
  {
    motionPosition += motionDirection;
    motionIntervalPulses += motionDirection;
  }
  if (travelRemainingPulses && --travelRemainingPulses == 0)
  {
    HAL_GPIO_WritePin(GPIOB, EN_PIN, GPIO_PIN_SET);
    travelStopFailed = HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1) != HAL_OK;
    motionMotorRunning = false;
    travelComplete = true;
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    RecordMotionPulse();
  }
}

// Called with TIM3 masked: account for an edge awaiting interrupt service.
static void FlushMotionPulse(void)
{
  if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC1) != RESET)
  {
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1);
    RecordMotionPulse();
  }
}

static void SetMotionTracking(uint16_t coils)
{
  taskENTER_CRITICAL();
  FlushMotionPulse();
  motionTrackingEnabled = (coils & MOTION_TRACK_EN) != 0;
  taskEXIT_CRITICAL();
}

static uint16_t SpeedFrequency(uint16_t speed)
{
  if (speed == 0) return 0;
  double frequency = speed / MOTION_MM_PER_PULSE;
  if (frequency < 100.0) frequency = 100.0;
  if (frequency > 700.0) frequency = 700.0;
  return (uint16_t)(frequency + 0.5);
}

static uint64_t ReadTravelCommand(void)
{
  uint64_t distance = 0;
  for (unsigned int i = 1; i < 5; ++i)
    distance = (distance << 16) | modbusHoldingRegistersData[i];
  return distance;
}

void MotorControlTask(void *argument)
{
  bool running = false;
  bool wasEnabled = false;
  bool finiteMove = false;
  GPIO_PinState moveDirection = GPIO_PIN_SET;
  uint16_t frequency = 0;
  // APB1 divider is 1: TIM3 clock equals PCLK1; PWM period stays 100 counts.
  const uint32_t timerClock = HAL_RCC_GetPCLK1Freq();
  GPIO_PinState direction = GPIO_PIN_RESET;

  while (true)
  {
    if (travelStopFailed) Error_Handler(__FILE__, __LINE__);
    if (xSemaphoreTake(modbusHandle.ModBusSphrHandle, 0) == pdTRUE)
    {
      // Acknowledge completion before accepting another enable edge.
      taskENTER_CRITICAL();
      if (travelComplete)
      {
        modbusCoilsData[0] &= ~MOTOR_EN;
        travelComplete = false;
        running = false;
      }
      taskEXIT_CRITICAL();
      uint16_t coils = modbusCoilsData[0];
      uint16_t speed = modbusHoldingRegistersData[0];
      uint64_t distance = ReadTravelCommand();
      xSemaphoreGive(modbusHandle.ModBusSphrHandle);

      SetMotionTracking(coils);
      bool enabled = (coils & MOTOR_EN) != 0;
      GPIO_PinState requestedDirection = (coils & MOTOR_CW) ? GPIO_PIN_RESET : GPIO_PIN_SET;
      uint16_t requestedFrequency = SpeedFrequency(speed);
      if (enabled && !wasEnabled)
      {
        finiteMove = distance != 0;
        bool backward = (distance >> 63) != 0;
        uint64_t magnitude = backward ? UINT64_C(0) - distance : distance;
        moveDirection = backward ? GPIO_PIN_RESET : GPIO_PIN_SET;
        taskENTER_CRITICAL();
        travelRemainingPulses = (uint64_t)(magnitude / MOTION_MM_PER_PULSE + 0.5);
        if (finiteMove && !travelRemainingPulses) travelRemainingPulses = 1;
        taskEXIT_CRITICAL();
      }
      wasEnabled = enabled;
      if (finiteMove) requestedDirection = moveDirection;

      if (running && (!enabled || !requestedFrequency || requestedDirection != direction || requestedFrequency != frequency))
      {
        taskENTER_CRITICAL();
        if (motionMotorRunning)
        {
          HAL_GPIO_WritePin(GPIOB, EN_PIN, GPIO_PIN_SET);
          if (HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
            Error_Handler(__FILE__, __LINE__);
        }
        // Stop the timer before flushing so an edge during shutdown is counted.
        FlushMotionPulse();
        motionMotorRunning = false;
        taskEXIT_CRITICAL();
        running = false;
      }
      if (!enabled)
      {
        taskENTER_CRITICAL();
        travelRemainingPulses = 0;
        taskEXIT_CRITICAL();
      }

      if (enabled && requestedFrequency && !running && !travelComplete)
      {
        HAL_GPIO_WritePin(GPIOB, CW_PIN, requestedDirection);
        osDelay(10);
        // Re-read live commands after the pause without blocking Modbus writes.
        if (xSemaphoreTake(modbusHandle.ModBusSphrHandle, 0) == pdTRUE)
        {
          uint16_t latestCoils = modbusCoilsData[0];
          uint16_t latestSpeed = modbusHoldingRegistersData[0];
          xSemaphoreGive(modbusHandle.ModBusSphrHandle);
          SetMotionTracking(latestCoils);
          if ((latestCoils & MOTOR_EN) && latestSpeed == speed &&
              (finiteMove || ((latestCoils ^ coils) & MOTOR_CW) == 0))
          {
            taskENTER_CRITICAL();
            __HAL_TIM_SET_PRESCALER(&htim3,
                (timerClock / 100 + requestedFrequency / 2) / requestedFrequency - 1);
            if (HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
              Error_Handler(__FILE__, __LINE__);
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1);
            HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
            motionDirection = requestedDirection == GPIO_PIN_SET ? 1 : -1;
            if (HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
              Error_Handler(__FILE__, __LINE__);
            HAL_GPIO_WritePin(GPIOB, EN_PIN, GPIO_PIN_RESET);
            motionMotorRunning = true;
            taskEXIT_CRITICAL();
            direction = requestedDirection;
            frequency = requestedFrequency;
            running = true;
          }
          else if (!(latestCoils & MOTOR_EN))
          {
            wasEnabled = false;
            taskENTER_CRITICAL();
            travelRemainingPulses = 0;
            taskEXIT_CRITICAL();
          }
        }
      }
    }
    osDelay(10);
  }
}

void MotionTrackingTask(void *argument)
{
  uint32_t previousTime = HAL_GetTick();
  const uint32_t period = osKernelGetTickFreq() / 10;
  while (true)
  {
    osDelay(period);
    taskENTER_CRITICAL();
    FlushMotionPulse();
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - previousTime;
    uint64_t position = motionPosition;
    int32_t pulses = motionIntervalPulses;
    bool active = motionTrackingEnabled && motionMotorRunning;
    if (elapsed != 0)
    {
      motionIntervalPulses = 0;
    }
    taskEXIT_CRITICAL();
    if (elapsed == 0)
    {
      continue;
    }
    previousTime = now;
    // shortcut: current calibration and 700 Hz PWM fit int32 speed; revisit if either changes.
    int32_t speed = active ? (int32_t)(pulses * MOTION_MM_PER_PULSE * 1000.0 / elapsed) : 0;
    double distanceMm = (int64_t)position * MOTION_MM_PER_PULSE;
    // Saturate physical distance rather than overflowing the signed Modbus value.
    int64_t distance = distanceMm >= (double)INT64_MAX ? INT64_MAX :
                       distanceMm <= (double)INT64_MIN ? INT64_MIN : (int64_t)distanceMm;
    if (xSemaphoreTake(modbusHandle.ModBusSphrHandle, 0) == pdTRUE)
    {
      if (modbusCoilsData[0] & MOTION_ZERO)
      {
        taskENTER_CRITICAL();
        // Discard all pulses through the reset boundary, including a pending edge.
        FlushMotionPulse();
        motionPosition = 0;
        motionIntervalPulses = 0;
        previousTime = HAL_GetTick();
        taskEXIT_CRITICAL();
        distance = 0;
        speed = 0;
      }
      modbusInputRegistersData[0] = (uint32_t)speed >> 16;
      modbusInputRegistersData[1] = (uint32_t)speed;
      for (unsigned int i = 0; i < 4; ++i)
      {
        modbusInputRegistersData[2 + i] = (uint64_t)distance >> (48 - 16 * i);
      }
      modbusCoilsData[0] &= ~MOTION_ZERO;
      xSemaphoreGive(modbusHandle.ModBusSphrHandle);
    }
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  ModbusComm_Init();
  /* add threads, ... */
  motorControlTaskHandle = osThreadNew(MotorControlTask, NULL, &motorControlTask_attributes);
  if (!motorControlTaskHandle)
  {
    Error_Handler(__FILE__, __LINE__);
  }
  motionTrackingTaskHandle = osThreadNew(MotionTrackingTask, NULL, &motionTrackingTask_attributes);
  if (!motionTrackingTaskHandle)
  {
    Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(const char* file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  printf("Error occured at %s, line %d\n", file, line);
  HAL_GPIO_WritePin(GPIOB, EN_PIN, GPIO_PIN_SET);
  HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
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
