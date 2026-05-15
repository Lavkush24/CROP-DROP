/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "stdbool.h"
#include "stdint.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SENSORS        5
#define BASE_SPEED         60
#define TURN_SPEED         BASE_SPEED * 2.2
#define GAP_SPEED          60
#define MAX_PWM            150
#define MIN_ACTIVE_PWM     10
#define LEFT_TRIM   	   0.92f
#define RIGHT_TRIM 		   1.00f

#define MAX_CORRECTION     30.0f
#define CORNER_TH          0.7f
#define TURN_TIME_MS       140

#define LINE_LOST_ENTER    0.45f
#define LINE_LOST_EXIT     0.65f
#define PIVOT_POS_TH       0.65f


// color detect
#define LED_R_PORT GPIOC
#define LED_R_PIN  GPIO_PIN_6

#define LED_G_PORT GPIOC
#define LED_G_PIN  GPIO_PIN_8

#define LED_B_PORT GPIOC
#define LED_B_PIN  GPIO_PIN_9

// obstacle detection
#define IR_OUT_PORT GPIOA
#define IR_OUT_PIN  GPIO_PIN_15

// magnet
#define EM_PORT GPIOA
#define EM_PIN  GPIO_PIN_4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* helper function required */

typedef struct {
	float kp;
	float ki;
	float kd;
	float prev_error;
	float integral;
} PID_t;

typedef enum {
    MODE_TRACK,
    MODE_PIVOT,
    MODE_GAP,
	PICK_BOX,
	NODE_MODE
} ControlMode;

//static int slow_hold = 0;
volatile uint16_t adcBuffer[5];
char buffer[64];

float black[5];
float white[5];


// color detection
volatile uint32_t capture1 = 0, capture2 = 0;
volatile uint8_t  captureDone = 0;
volatile uint32_t frequency = 0;

typedef enum {
    COLOR_UNKNOWN,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE
} DetectedColor;

static uint32_t led_last_tick = 0;
static uint8_t  led_on = 0;
static DetectedColor led_color = COLOR_UNKNOWN;

//static float prev_pos = 0.0f;
float speed_scale = 1.0f;
static float last_pos = 0.0f;
static bool in_gap = false;
static bool box_picked = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);


/* USER CODE BEGIN PFP */

// hardware control functions
void custom_control(int left,int right);

// helper functions
int clamp(int val,int min,int max);
float clampFloat(float v, float min, float max);
void pidInit(PID_t *pid,float kp,float ki,float kd);
void ir_state(const uint16_t *adc, uint8_t *state);
void readIR(uint16_t *dst);
void computeReflectance(uint16_t* adcBuffer, float *ir_reflectance);
float position_reflectance(const float *ir);
float position_states(uint8_t *state);
float pid_compute(PID_t *pid, float error, float dt);
void set_threshold(uint16_t* adcBuffer);
void calibrate_sensor(uint32_t duration_ms);



float compute_line_conf(float *r);
void control_track(float steer, float speed,float left_scale, float right_scale,int *left, int *right);
void control_pivot(float pos, int *left, int *right);
void control_gap(float last_pos, int *left, int *right);

// colord erection
DetectedColor TCS3200_DetectColor(void);
void LED_BlinkColor(DetectedColor color, uint32_t period_ms);
uint32_t Read_Color_Frequency(uint8_t s2, uint8_t s3);
uint8_t IR_ObjectDetected(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void Electromagnet_ON(void)
{
    HAL_GPIO_WritePin(EM_PORT, EM_PIN, GPIO_PIN_SET);
}

static inline void Electromagnet_OFF(void)
{
    HAL_GPIO_WritePin(EM_PORT, EM_PIN, GPIO_PIN_RESET);
}

static inline void LED_AllOff(void)
{
    HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_SET);
}

static inline void LED_SetColor(DetectedColor color)
{
    LED_AllOff();  // important

    switch (color)
    {
    case COLOR_RED:
        HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, GPIO_PIN_RESET);
        break;

    case COLOR_GREEN:
        HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, GPIO_PIN_RESET);
        break;

    case COLOR_BLUE:
        HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_RESET);
        break;

    default:
        // unknown → all off
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adcBuffer,5);
  HAL_Delay(200);

  calibrate_sensor(5000);

  // motor control pin configuration
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);


  // color freq
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

  PID_t linePID;
  pidInit(&linePID, 15.0f, 0.0f, 0.0f);

  uint32_t lastTick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // -------------------- Time --------------------
	  uint32_t now = HAL_GetTick();
	  float dt = (now - lastTick) / 1000.0f;
	  lastTick = now;

	  if (dt <= 0.0f || dt > 0.1f)
	      dt = 0.01f;

	  // -------------------- Sensors --------------------
	  uint16_t ir[NUM_SENSORS];
	  float reflectance[NUM_SENSORS];
	  readIR(ir);
	  computeReflectance(ir, reflectance);
	  // -------------------- Position --------------------
	  float pos = position_reflectance(reflectance);   // pos < 0 : left, pos > 0 : right

	  // color detection
	  DetectedColor color = TCS3200_DetectColor();
	  bool box = IR_ObjectDetected();

	  if(color != COLOR_UNKNOWN && !box_picked) {
		  LED_SetColor(color);
	  }
	  else {
		  LED_SetColor(COLOR_UNKNOWN);
	  }

	  bool stop_condition = reflectance[0] == 1.0 && reflectance[4] == 1.0 && reflectance[3] != 1.0;

	  float line_conf = compute_line_conf(reflectance);

	  if (line_conf > LINE_LOST_EXIT)
		  last_pos = pos;

	  if (!in_gap && line_conf < LINE_LOST_ENTER)
		  in_gap = true;
	  else if (in_gap && line_conf > LINE_LOST_EXIT)
		  in_gap = false;


	  // ---------- PID ----------
	  float error = -pos;
	  float steer = pid_compute(&linePID, error, dt);
	  steer = clampFloat(steer, -MAX_CORRECTION, MAX_CORRECTION);

	  // ---------- Corner detection ----------
	  static bool iscorner = false;
	  static uint32_t corner_time = 0;
	  static uint8_t corner_cnt = 0;

	  bool left_edge  = reflectance[0] > CORNER_TH;
	  bool right_edge = reflectance[4] > CORNER_TH;


	  if (!iscorner) {
		  if (left_edge || right_edge) {
			  corner_cnt++;
			  if (corner_cnt >= 2) {
				  iscorner = true;
				  corner_time = HAL_GetTick();
			  }
		  } else {
			  corner_cnt = 0;
		  }
	  }

	  if (iscorner && (HAL_GetTick() - corner_time > TURN_TIME_MS)) {
		  iscorner = false;
		  corner_cnt = 0;
	  }

	  ControlMode mode ;
	  if(stop_condition) {
		  mode = NODE_MODE;
	  }
	  else if(box && color != COLOR_UNKNOWN && !box_picked) {
		  mode = PICK_BOX;
	  }
	  else
	  if (iscorner && fabs(pos) > PIVOT_POS_TH) {
		  mode = MODE_PIVOT;
	  }
	  else if (in_gap && !iscorner && fabs(pos) < 0.4f) {
		  mode = MODE_GAP;
	  }
	  else {
		  mode = MODE_TRACK;
	  }

//	   debug
//	  int len = 0;
//	  for(int i=0; i<5; i++) {
//		  len += snprintf(buffer+len,sizeof(buffer)-len,"ir%d: %.2f",i+1,reflectance[i]);
//	  }
//
//	  buffer[len++] = '\r';
//	  buffer[len++] = '\n';
//
//	  HAL_UART_Transmit(&huart2,(uint8_t*) buffer, len, HAL_MAX_DELAY);
//	  HAL_Delay(500);

	  // ---------- Speed & scaling ----------
	  float speed = BASE_SPEED;
	  float left_scale = 1.0f;
	  float right_scale = 1.0f;

	  if (iscorner && mode == MODE_TRACK) {
		  steer = clampFloat(steer, -5.0f, 5.0f);
		  speed = TURN_SPEED;

		  if (pos < 0) {
			  left_scale  = 0.1f;
			  right_scale = 1.5f;
		  } else {
			  left_scale  = 1.5f;
			  right_scale = 0.1f;
		  }

		  if (steer < 0)
			  steer *= 1.3f;   // right-turn boost only
	  }


	  // ---------- Actuation ----------
	  int left = 0, right = 0;

	  switch (mode) {
	  case PICK_BOX:
		  custom_control(40,40);
		  box_picked = true;
		  HAL_Delay(300);
		  Electromagnet_ON();
		  break;
	  case NODE_MODE:
		  custom_control(-100,100);
//		  LED_BlinkColor(color, 300);
//		  Electromagnet_OFF();
		  HAL_Delay(1100);
//		  box_picked = false;
		  break;
	  case MODE_PIVOT:
		  control_pivot(pos, &left, &right);
		  break;

	  case MODE_GAP:
		  control_gap(last_pos, &left, &right);
		  break;

	  case MODE_TRACK:
	  default:
		  control_track(steer, speed, left_scale, right_scale, &left, &right);
		  break;
	  }


	  left = (int)(left*LEFT_TRIM);
	  right = (int)(right*RIGHT_TRIM);
	  custom_control(left, right);


	  // motor check
//	  custom_control(0,100);


//	  // color detection
//
//	  DetectedColor color = TCS3200_DetectColor();
//	  LED_SetColor(color);
//
//
//	  // object detection
//	  uint8_t box = IR_ObjectDetected();
//
//	  // magner pickup
//	  if(box) {
//		  Electromagnet_ON();
//	  }
//	  else {
//		  Electromagnet_OFF();
//	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, S0_Pin|S1_Pin|S2_Pin|S3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RED_Pin|GREEN_Pin|BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Electromagnet_Pin LD2_Pin */
  GPIO_InitStruct.Pin = Electromagnet_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : S0_Pin S1_Pin S2_Pin S3_Pin */
  GPIO_InitStruct.Pin = S0_Pin|S1_Pin|S2_Pin|S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_Pin GREEN_Pin BLUE_Pin */
  GPIO_InitStruct.Pin = RED_Pin|GREEN_Pin|BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Box_detect_Pin */
  GPIO_InitStruct.Pin = Box_detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Box_detect_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//void custom_control(int left, int right) {
//	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,right);
//	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
//	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
//	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,left);
//}



void custom_control(int left, int right)
{
    // clamp
    if (left > MAX_PWM)  left = MAX_PWM;
    if (left < -MAX_PWM) left = -MAX_PWM;
    if (right > MAX_PWM)  right = MAX_PWM;
    if (right < -MAX_PWM) right = -MAX_PWM;

    // -------- LEFT MOTOR --------
    if (left >= 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left);   // forward
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);      // reverse off
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);      // forward off
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -left);  // reverse
    }

    // -------- RIGHT MOTOR --------
    if (right >= 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right);  // forward
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);      // reverse off
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);      // forward off
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, -right); // reverse
    }
}




// helper function implementation

//void set_threshold(uint16_t* adcBuffer) {
//	for(int i=0; i<5; i++) {
//		if(adcBuffer[i] < )
//	}
//}
void pidInit(PID_t *pid,float kp,float ki,float kd) {
	pid->kp = kp;
	pid->kd = kd;
	pid->ki = ki;
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;
}

int clamp(int v, int min, int max)
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

float clampFloat(float v, float min, float max)
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

void readIR(uint16_t *dst)
{
    for (int i = 0; i < NUM_SENSORS; i++)
        dst[i] = adcBuffer[i];
}


//void ir_state(const uint16_t *adc, uint8_t *state)
//{
//    for (int i = 0; i < NUM_SENSORS; i++) {
//        if (adc[i] > WHITE_TH + MARGIN)
//            state[i] = 2; // WHITE
//        else if (adc[i] < BLACK_TH - MARGIN)
//            state[i] = 0; // BLACK
//        else
//            state[i] = 1; // EDGE
//    }
//}

void calibrate_sensor(uint32_t duration_ms)
{
    for (int i = 0; i < 5; i++) {
        black[i] = 4095.0f;
        white[i] = 0.0f;
    }

    uint32_t start = HAL_GetTick();

    while (HAL_GetTick() - start < duration_ms)
    {
        for (int i = 0; i < 5; i++)
        {
            uint16_t v = adcBuffer[i];

            if (v < black[i]) black[i] = v;
            if (v > white[i]) white[i] = v;
        }

        HAL_Delay(2);
    }

    // Safety margin to avoid divide_zero and noise
    for (int i = 0; i < 5; i++) {
        if (white[i] - black[i] < 200) {
            white[i] = black[i] + 200;
        }
    }
}


//void adaptiveCalibration(const uint16_t *adc)
//{
//    for (int i = 0; i < NUM_SENSORS; i++)
//    {
//        if (adc[i] > white[i]) {
//            white[i] = white[i] * (1.0f - CAL_ALPHA)
//                     + adc[i]   * CAL_ALPHA;
//        }
//
//        if (adc[i] < black[i]) {
//            black[i] = black[i] * (1.0f - CAL_ALPHA)
//                     + adc[i]   * CAL_ALPHA;
//        }
//
//        if (white[i] - black[i] < MIN_RANGE) {
//            white[i] = black[i] + MIN_RANGE;
//        }
//    }
//}

void computeReflectance(uint16_t* adcBuffer, float *ir_reflectance)
{
    for (int i = 0; i < 5; i++)
    {
        float denom = white[i] - black[i];
        if (denom < 1.0f)
            denom = 1.0f;

        float r = ((float)adcBuffer[i] - black[i]) / denom;

        if (r < 0.0f) r = 0.0f;
        if (r > 1.0f) r = 1.0f;

        ir_reflectance[i] = r;
    }
}


float position_reflectance(const float *ir)
{
    static float last_pos = 0.0f;
    const float w[5] = { -2, -1, 0, 1, 2 };

    float sum = 0.0f;
    float weighted = 0.0f;

    for (int i = 0; i < 5; i++) {
        float r = clampFloat(ir[i], 0.0f, 1.0f);
        sum += r;
        weighted += r * w[i];
    }

    if (sum < 0.5f) {
        return last_pos;   // HOLD
    }

    float pos = weighted / (sum + 0.3f);  // soft floor
    pos = clampFloat(pos, -2.0f, 2.0f);

    last_pos = pos;
    return pos;
}

float position_states(uint8_t *state)
{
	const int weight[5] = { -2, -1, 0, 1, 2 };
	float sum = 0.0f;
	float count = 0.0f;

	for (int i = 0; i < NUM_SENSORS; i++) {
		if (state[i] == 2) {
			sum += weight[i];
			count++;
		}
	}

	if (count == 0)
		return 0.0f;

	return sum / count;
}

bool cornerDetection(int *ir) {
	if((ir[0] == 0 && ir[1] == 0) || (ir[3] == 0 && ir[4] == 0)) {
		if((ir[3] == 100 && ir[4] == 100) || (ir[0] == 100 && ir[1] == 100)) {
			return true;
		}
	}
	return false;
}


float pid_compute(PID_t *pid, float error, float dt)
{
    pid->integral += error * dt;
    pid->integral = clampFloat(pid->integral, -10.0f, 10.0f);

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return pid->kp * error +
           pid->ki * pid->integral +
           pid->kd * derivative;
}



float compute_line_conf(float *r)
{
    float s = 0.0f;
    for (int i = 0; i < NUM_SENSORS; i++) s += r[i];
    return s;
}

void control_track(float steer, float speed,float left_scale, float right_scale,int *left, int *right)
{
    if (steer > 0) {
        *left  = speed * left_scale;
        *right = speed * right_scale + steer;
    } else {
        *left  = speed * left_scale - steer;
        *right = 0;
    }

    *left  = clamp(*left,  MIN_ACTIVE_PWM, MAX_PWM);
    *right = clamp(*right, MIN_ACTIVE_PWM, MAX_PWM);
}


void control_pivot(float pos, int *left, int *right)
{
    if (pos > 0) {
        *left  = TURN_SPEED;
        *right = 0;
    } else {
        *left  = 0;
        *right = TURN_SPEED;
    }
}


void control_gap(float last_pos, int *left, int *right)
{
    if (last_pos > 0) {
        *left  = GAP_SPEED;
        *right = GAP_SPEED * 0.6f;
    } else {
        *left  = GAP_SPEED * 0.6f;
        *right = GAP_SPEED;
    }
}

// color detection]


DetectedColor TCS3200_DetectColor(void)
{
    uint32_t red, green, blue;
    float r, g, b, sum;

    red   = Read_Color_Frequency(GPIO_PIN_RESET, GPIO_PIN_RESET); // RED
    green = Read_Color_Frequency(GPIO_PIN_SET,   GPIO_PIN_SET);   // GREEN
    blue  = Read_Color_Frequency(GPIO_PIN_RESET, GPIO_PIN_SET);   // BLUE

    if (red == 0 || green == 0 || blue == 0)
        return COLOR_UNKNOWN;

    sum = red + green + blue;
    r = red   / sum;
    g = green / sum;
    b = blue  / sum;

    if (r > 0.45f && r > g && r > b)
        return COLOR_RED;

    // BLUE: strong, absolute threshold OK
    if (b > 0.40f && b > r && b > g)
        return COLOR_BLUE;

    // GREEN: weak channel → relative dominance only
    if (g > r * 1.08f && g > b * 1.08f)
        return COLOR_GREEN;

    return COLOR_UNKNOWN;
}



void LED_BlinkColor(DetectedColor color, uint32_t period_ms)
{
    uint32_t now = HAL_GetTick();

    // Color changed → reset blink state
    if (color != led_color) {
        led_color = color;
        led_on = 0;
        LED_AllOff();
        led_last_tick = now;
    }

    if (color == COLOR_UNKNOWN)
        return;

    if (now - led_last_tick >= period_ms) {
        led_last_tick = now;
        led_on ^= 1;

        if (led_on)
            LED_SetColor(color);
        else
            LED_AllOff();
    }
}

uint32_t Read_Color_Frequency(uint8_t s2, uint8_t s3)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, s2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, s3);

    captureDone = 0;
    while (captureDone < 2);

    return frequency;
}

uint8_t IR_ObjectDetected(void)
{
    // ACTIVE LOW sensor
    if (HAL_GPIO_ReadPin(IR_OUT_PORT, IR_OUT_PIN) == GPIO_PIN_RESET)
        return 1;   // object detected
    else
        return 0;   // no object
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
