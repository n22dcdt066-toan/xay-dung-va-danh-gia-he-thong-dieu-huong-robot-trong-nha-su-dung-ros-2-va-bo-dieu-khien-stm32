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
#include <math.h>
#include "kinematics_odometry.h"
#include "pid_controller.h"
#include "safety_fsm.h"
#include "uart_comm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Nhãn chân GPIO của cầu H (Bạn có thể đổi cổng/chân cho đúng mạch thực tế) */
#define IN1_FL_PORT GPIOA
#define IN1_FL_PIN  GPIO_PIN_3
#define IN2_FL_PORT GPIOA
#define IN2_FL_PIN  GPIO_PIN_4

#define IN1_FR_PORT GPIOA
#define IN1_FR_PIN  GPIO_PIN_12
#define IN2_FR_PORT GPIOB
#define IN2_FR_PIN  GPIO_PIN_2

#define IN1_RL_PORT GPIOB
#define IN1_RL_PIN  GPIO_PIN_10
#define IN2_RL_PORT GPIOB
#define IN2_RL_PIN  GPIO_PIN_12

#define IN1_RR_PORT GPIOB
#define IN1_RR_PIN  GPIO_PIN_13
#define IN2_RR_PORT GPIOB
#define IN2_RR_PIN  GPIO_PIN_14

#define STBY_PORT   GPIOB
#define STBY_PIN    GPIO_PIN_15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId ControlTaskHandle;
osThreadId SafetyTaskHandle;
osThreadId CommTaskHandle;
/* USER CODE BEGIN PV */
Robot_Odometry_t robot_odom;
Safety_FSM_t robot_fsm;

PID_Controller_t pid_FL, pid_FR, pid_RL, pid_RR;

uint16_t last_cnt_FL = 0, last_cnt_FR = 0, last_cnt_RL = 0, last_cnt_RR = 0; // UNUSED: kept for compatibility
/* BUG FIX: Dùng uint32_t để hỗ trợ TIM2/TIM5 (32-bit) và TIM3/TIM4 (16-bit) không bị cắt ngắn */
uint32_t last_cnt32_FL = 0, last_cnt32_FR = 0, last_cnt32_RL = 0, last_cnt32_RR = 0;
int32_t total_enc_FL = 0, total_enc_FR = 0, total_enc_RL = 0, total_enc_RR = 0;

uint8_t rx_buffer[RX_PACKET_SIZE];
uint8_t tx_buffer[TX_PACKET_SIZE];
Master_Cmd_Packet_t master_cmd;
Robot_Feedback_Packet_t robot_fb;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
void StartControlTask(void const * argument);
void StartSafetyTask(void const * argument);
void StartCommTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Motor_Set_PWM(float u_FL, float u_FR, float u_RL, float u_RR) {
    /* Đảm bảo STBY của TB6612FNG luôn bật */
    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);

    /* ----- 1. ĐIỀU KHIỂN CHIỀU QUAY (TB6612FNG IN1/IN2) ----- */
    if (u_FL >= 0) {
        HAL_GPIO_WritePin(IN1_FL_PORT, IN1_FL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN2_FL_PORT, IN2_FL_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(IN1_FL_PORT, IN1_FL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_FL_PORT, IN2_FL_PIN, GPIO_PIN_SET);
    }

    if (u_FR >= 0) {
        HAL_GPIO_WritePin(IN1_FR_PORT, IN1_FR_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN2_FR_PORT, IN2_FR_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(IN1_FR_PORT, IN1_FR_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_FR_PORT, IN2_FR_PIN, GPIO_PIN_SET);
    }

    if (u_RL >= 0) {
        HAL_GPIO_WritePin(IN1_RL_PORT, IN1_RL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN2_RL_PORT, IN2_RL_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(IN1_RL_PORT, IN1_RL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_RL_PORT, IN2_RL_PIN, GPIO_PIN_SET);
    }

    if (u_RR >= 0) {
        HAL_GPIO_WritePin(IN1_RR_PORT, IN1_RR_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN2_RR_PORT, IN2_RR_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(IN1_RR_PORT, IN1_RR_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_RR_PORT, IN2_RR_PIN, GPIO_PIN_SET);
    }

    /* ----- 2. ĐIỀU KHIỂN TỐC ĐỘ (PWM) ----- */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)fabs(u_FL));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)fabs(u_FR));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)fabs(u_RL));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)fabs(u_RR));
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Odometry_Init(&robot_odom);
  FSM_Init(&robot_fsm);
  
  /* BUG FIX: PID gains phù hợp với đầu vào m/s và đầu ra PWM ticks (0-4999)
   * v_max L-Type 520 @ 12V = (520/40)*2pi*0.0325 ≈ 2.64 m/s
   * Kp ≈ 1800 để sai lệch 2.64m/s → 4752 ticks (gần full range) */
  PID_Init(&pid_FL, 1800.0f, 200.0f, CONTROL_DT, 4999.0f, -4999.0f);
  PID_Init(&pid_FR, 1800.0f, 200.0f, CONTROL_DT, 4999.0f, -4999.0f);
  PID_Init(&pid_RL, 1800.0f, 200.0f, CONTROL_DT, 4999.0f, -4999.0f);
  PID_Init(&pid_RR, 1800.0f, 200.0f, CONTROL_DT, 4999.0f, -4999.0f);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // PA8  → TIM1_CH1  → PWM motor FL
  /* .ioc: PB0=TIM1_CH2N, PB1=TIM1_CH3N → đây là kênh bù (complementary), dùng PWMN là ĐÚNG */
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);   // PB0  → TIM1_CH2N → PWM motor FR
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);   // PB1  → TIM1_CH3N → PWM motor RL
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);       // PA11 → TIM1_CH4  → PWM motor RR

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  /* Bắt đầu nhận UART bằng DMA */
  HAL_UART_Receive_DMA(&huart1, rx_buffer, RX_PACKET_SIZE);
  /* USER CODE END 2 */

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
  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, StartControlTask, osPriorityHigh, 0, 256);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of SafetyTask */
  osThreadDef(SafetyTask, StartSafetyTask, osPriorityAboveNormal, 0, 128);
  SafetyTaskHandle = osThreadCreate(osThread(SafetyTask), NULL);

  /* definition and creation of CommTask */
  osThreadDef(CommTask, StartCommTask, osPriorityNormal, 0, 256);
  CommTaskHandle = osThreadCreate(osThread(CommTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4999;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB12 PB13
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Ngắt gọi khi DMA nhận xong 14 bytes */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        /* Báo hiệu (Signal) cho CommTask dậy xử lý (dùng CMSIS_v1 API) */
        osSignalSet(CommTaskHandle, 0x01);
        
        /* Bật lại DMA nhận luồng tiếp theo */
        HAL_UART_Receive_DMA(&huart1, rx_buffer, RX_PACKET_SIZE);
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief  Function implementing the ControlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    /* Đọc Encoder - BUG FIX: dùng last_cnt32_* (uint32_t) để không bị cắt ngắn 16-bit cao */
    int32_t delta_FL, delta_FR, delta_RL, delta_RR;
    Encoder_Update(&total_enc_FL, &delta_FL, __HAL_TIM_GET_COUNTER(&htim2), &last_cnt32_FL);
    Encoder_Update(&total_enc_FR, &delta_FR, __HAL_TIM_GET_COUNTER(&htim3), &last_cnt32_FR);
    Encoder_Update(&total_enc_RL, &delta_RL, __HAL_TIM_GET_COUNTER(&htim4), &last_cnt32_RL);
    Encoder_Update(&total_enc_RR, &delta_RR, __HAL_TIM_GET_COUNTER(&htim5), &last_cnt32_RR);
    
    /* Cập nhật Odometry */
    Odometry_Update(&robot_odom, delta_FL, delta_FR, delta_RL, delta_RR);
    
    /* Tính vận tốc thực tế từng bánh (m/s) */
    float actual_v_FL = (delta_FL * 2.0f * MATH_PI * ROBOT_WHEEL_RADIUS) / (ENCODER_RESOLUTION * CONTROL_DT);
    float actual_v_FR = (delta_FR * 2.0f * MATH_PI * ROBOT_WHEEL_RADIUS) / (ENCODER_RESOLUTION * CONTROL_DT);
    float actual_v_RL = (delta_RL * 2.0f * MATH_PI * ROBOT_WHEEL_RADIUS) / (ENCODER_RESOLUTION * CONTROL_DT);
    float actual_v_RR = (delta_RR * 2.0f * MATH_PI * ROBOT_WHEEL_RADIUS) / (ENCODER_RESOLUTION * CONTROL_DT);
    
    /* Tính toán PID nếu đang hoạt động */
    if (FSM_Is_Running(&robot_fsm)) {
        float ref_FL, ref_FR, ref_RL, ref_RR;
        Kinematics_Inverse(master_cmd.v_ref, master_cmd.w_ref, &ref_FL, &ref_FR, &ref_RL, &ref_RR);
        
        float u_FL = PID_Compute(&pid_FL, ref_FL, actual_v_FL);
        float u_FR = PID_Compute(&pid_FR, ref_FR, actual_v_FR);
        float u_RL = PID_Compute(&pid_RL, ref_RL, actual_v_RL);
        float u_RR = PID_Compute(&pid_RR, ref_RR, actual_v_RR);
        
        Motor_Set_PWM(u_FL, u_FR, u_RL, u_RR);
    } else {
        Motor_Set_PWM(0, 0, 0, 0);
        PID_Reset(&pid_FL); PID_Reset(&pid_FR); PID_Reset(&pid_RL); PID_Reset(&pid_RR);
    }

    osDelay(10); /* Chu kỳ 10ms (100Hz) */
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSafetyTask */
/**
* @brief Function implementing the SafetyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSafetyTask */
void StartSafetyTask(void const * argument)
{
  /* USER CODE BEGIN StartSafetyTask */
  /* Infinite loop */
  for(;;)
  {
    /* Kiểm tra Timeout Watchdog 300ms */
    FSM_Update(&robot_fsm, HAL_GetTick());
    
    osDelay(10); /* Kiểm tra mỗi 10ms */
  }
  /* USER CODE END StartSafetyTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void const * argument)
{
  /* USER CODE BEGIN StartCommTask */
  /* Infinite loop */
  for(;;)
  {
      /* Chờ cờ Signal 0x01 từ hàm ngắt UART báo hiệu đã nhận xong gói tin */
      osEvent event = osSignalWait(0x01, osWaitForever);
      
      if (event.status == osEventSignal) {
          Unpack_Master_Command(rx_buffer, &master_cmd);
          
          if (master_cmd.is_valid) {
              /* Nhận đúng -> Báo Watchdog */
              FSM_Feed_Watchdog(&robot_fsm, HAL_GetTick());
              
              /* BUG FIX: Xử lý đầy đủ các cờ CONTROL_FLAG */
              if (master_cmd.control_flag & 0x02) { // Bit 1: E-STOP khẩn cấp
                  FSM_Trigger_Fault(&robot_fsm);
              } else if (master_cmd.control_flag & 0x01) { // Bit 0: Reset lỗi
                  FSM_Reset_Fault(&robot_fsm);
              }
              if (master_cmd.control_flag & 0x04) { // Bit 2: Reset Odometry
                  Odometry_Init(&robot_odom);
              }
              
              /* Đóng gói và gửi phản hồi */
              robot_fb.seq_id = master_cmd.seq_id;
              robot_fb.timestamp_ms = HAL_GetTick();
              robot_fb.x = robot_odom.x;
              robot_fb.y = robot_odom.y;
              robot_fb.theta = robot_odom.theta;
              robot_fb.linear_v = robot_odom.linear_v;
              robot_fb.angular_w = robot_odom.angular_w;
              robot_fb.enc_FL = total_enc_FL;
              robot_fb.enc_FR = total_enc_FR;
              robot_fb.enc_RL = total_enc_RL;
              robot_fb.enc_RR = total_enc_RR;
              robot_fb.state = (uint8_t)robot_fsm.state;
              
              Pack_Robot_Feedback(&robot_fb, tx_buffer);
              
              /* Gửi phản hồi qua DMA TX */
              HAL_UART_Transmit_DMA(&huart1, tx_buffer, TX_PACKET_SIZE);
          }
      }
  }
  /* USER CODE END StartCommTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9)
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
