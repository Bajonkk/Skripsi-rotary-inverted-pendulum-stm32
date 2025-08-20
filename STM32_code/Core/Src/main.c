/* USER CODE BEGIN Header */
/**
 * RIP_swing stable
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
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pi 3.14159265358979323846
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
int32_t printt;
int32_t printt_then = 0;
int32_t printt_then2 = 0;
int32_t counterValue_1 = 0;
int16_t counterValue_2 = 0;
int16_t cnt_trns = 0;

double ctrl_sgn_1;
double prev_ctrl_sgn_1 = 0;
int32_t ctrl_sgn_2;
int32_t ctrl_sgn_k;

float x1 = 0;
float x2 = 0;
float prev_x1 = -180;
float prev_x2 = 0;
float dx1, dx2, fdx1, fdx2, fdx1_then, fdx2_then;
float sp1 = 0;
float sp2 = 0;
double rad_angle_1 = 0;
double rad_angle_2 = 0;
double current_error_1 = 0;
double prev_error_1 = 0;
float current_error_2 = 0;
float prev_error_2 = 0;
float derivative = 0;
float integral_1 = 0;
float integral_2 = 0;
float drv_error_1, drv_error_2, itg_error_1, itg_error_2;
float fdrv_error_1, fdrv_error_2;
float fdrv_error_1_then, fdrv_error_2_then;

float drv_angle_1, drv_angle_2;
float fdrv_angle_1, fdrv_angle_2;
float fdrv_angle_1_then, fdrv_angle_2_then;
float prev_angle_1 = 0;
float prev_angle_2 = 0;
float momentum_angle = 0;

float Ts = 0.000064;

//float Kp_1 = 40000;
//float Ki_1 = 10000;
//float Kd_1 = 0;
//float Kp_2 = 22000;
//float Ki_2 = 12000;
//float Kd_2 = 9000;

float Kp_1 = 20000;
float Ki_1 = 5000;
float Kd_1 = 0;
float Kp_2 = 30000;
float Ki_2 = 7000;
float Kd_2 = 0;

int8_t mode = 0;
int8_t chg = 1;
int8_t a = 1;

volatile uint8_t tx_ready = 1;

char MSG[100];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
float wrap_angle_deg(float angle_deg, float rtt) {
    while (angle_deg > rtt) angle_deg -= 2.0f * rtt;
    while (angle_deg < (-1) * rtt) angle_deg += 2.0f * rtt;
    return angle_deg;
}

float filter(float now, float* filtered_then, float alpha) {
    float filtered = alpha * now + (1.0f - alpha) * (*filtered_then);
    *filtered_then = filtered;  // update the stored value
    return filtered;
}
float compute_derivative(float now, float then, float dt){
	float derivative = ((float)(now - then)) / dt;
	return derivative;
}

float compute_integral(float now, float then, float dt, float* integral_accum) {
    float avg_error = (now + then) / 2.0f;

    // Round off tiny noise
    if (fabs(avg_error) < 0.005f) {
        avg_error = 0.0f;
    }

    // Integrate only if error is significant
    if (avg_error != 0.0f) {
    	*integral_accum += dt * avg_error;
    }

    // Anti-windup clamp
    if (*integral_accum > 100.0f) *integral_accum = 100.0f;
	if (*integral_accum < -100.0f) *integral_accum = -100.0f;

	return *integral_accum;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        tx_ready = 1; // Allow next message to be sent
    }
}

void send_uart_dma() {
//	sprintf(MSG, "%.4f, %.4f, %.4f, %.4f, %.4f, %d\r\n", current_error_2, current_error_1, itg_error_2, itg_error_1, 90.0, ctrl_sgn_k);
//	sprintf(MSG, "%.4f, %.4f, %.4f, %.4f, %.4f, %d\r\n", rad_angle_1, rad_angle_2, itg_error_2, itg_error_1, 90.0, cnt_trns);
	sprintf(MSG, "%.4f, %.4f, %.4f, %.4f, %d\r\n", x2, x1, fdx2, fdx1, ctrl_sgn_k);
//	sprintf(MSG, "%.4f, %.4f\r\n", x2, 90.0);
//	sprintf(MSG, "%.4f, %.4f\r\n", x1, -180.0);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)MSG, strlen(MSG));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
    HAL_Delay(50); // Debounce
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
    {
    	SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk; // Full software reset
    }
  }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_Delay(500);
  TIM3->CCR1 = 16000;
  TIM3->CCR2 = 0;
  HAL_Delay(100);
  TIM3->CCR1 = 0;
  TIM3->CCR2 = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  printt = HAL_GetTick();
	  counterValue_1 = (int32_t)(TIM2->CNT);
	  counterValue_2 = (int16_t)(TIM4->CNT);
	  x1 = (roundf(100 * counterValue_1 * 360.0 / 4000.0) / 100.0) - 180;
	  dx1 = compute_derivative(x1, prev_x1, Ts);
	  fdx1 = filter(dx1, &fdx1_then, 0.0005);
	  x2 = roundf(100 * counterValue_2 * 360.0 / 300.0)/100.0;
	  dx2 = compute_derivative(x2, prev_x2, Ts);
	  fdx2 = filter(dx2, &fdx2_then, 0.0005);
	  prev_x2 = x2;
	  prev_x1 = x1;
	  if(fabs(x2) > 540){
		  mode = 3;
	  }
	  switch(mode){
	  case 0:
		  rad_angle_1 = roundf(1000 * counterValue_1 * pi * 2.0 / 4000.0) / 1000.0;
		  rad_angle_1 = wrap_angle_deg(rad_angle_1, pi);
		  rad_angle_2 = roundf(1000 * counterValue_2 * pi * 2.0 / 300.0) / 1000.0;
		  current_error_1 = sp1 - rad_angle_1;
		  drv_error_1 = compute_derivative(current_error_1, prev_error_1, Ts);
		  fdrv_error_1 = filter(drv_error_1, &fdrv_error_1_then, 0.001);
		  itg_error_1 = compute_integral(current_error_1, prev_error_1, Ts, &integral_1);
		  ctrl_sgn_1 = Kp_1 * current_error_1 + Ki_1 * itg_error_1 + Kd_1 * drv_error_1;
		  current_error_2 = rad_angle_2;
		  drv_error_2 = compute_derivative(current_error_2, prev_error_2, Ts);
		  fdrv_error_2 = filter(drv_error_2, &fdrv_error_2_then, 0.001);
		  itg_error_2 = compute_integral(current_error_2, prev_error_2, Ts, &integral_2);
		  ctrl_sgn_2 = Kp_2 * current_error_2 + Ki_2 * itg_error_2 + Kd_2 * drv_error_2;
		  ctrl_sgn_k = ctrl_sgn_1 + ctrl_sgn_2;
		  if (ctrl_sgn_k > 50000) ctrl_sgn_k = 50000;
		  if (ctrl_sgn_k < -50000) ctrl_sgn_k = -50000;
		  if(ctrl_sgn_k >= 0){
			  TIM3->CCR1 = 0;
			  TIM3->CCR2 = ctrl_sgn_k; //CCW
		  }
		  if(ctrl_sgn_k <= 0){
			  TIM3->CCR1 = (-1) * ctrl_sgn_k; //CW
			  TIM3->CCR2 = 0;
		  }
		  if(rad_angle_1 <= -0.9 * pi || rad_angle_1 >= 0.9 * pi){
			  mode = 2;
			  if (rad_angle_1 >= 0.9 * pi){
		  		prev_angle_1 = -0.1 * pi;
		  		drv_angle_1 = pi;
		  		fdrv_angle_1_then = pi;
			  } else {
				prev_angle_1 = 0.1 * pi;
				drv_angle_1 = -pi;
				fdrv_angle_1_then = -pi;
			  }
			  fdrv_angle_2_then = 0;
			  prev_angle_2 = rad_angle_2;
		  }

		  prev_error_1 = current_error_1;
		  prev_ctrl_sgn_1 = ctrl_sgn_1;
		  prev_error_2 = current_error_2;
		  break;
	  case 1:
		  rad_angle_1 = roundf(1000 * counterValue_1 * pi * 2.0 / 4000.0) / 1000.0;
		  rad_angle_1 = wrap_angle_deg(rad_angle_1, pi);
		  rad_angle_2 = roundf(1000 * counterValue_2 * pi * 2.0 / 300.0) / 1000.0;
		  ctrl_sgn_k = 10000 * rad_angle_1 + 10000 * rad_angle_2;
		  if (ctrl_sgn_k > 50000) ctrl_sgn_k = 50000;
		  if (ctrl_sgn_k < -50000) ctrl_sgn_k = -50000;
		  if(ctrl_sgn_k >= 0){
			  TIM3->CCR1 = 0;
			  TIM3->CCR2 = ctrl_sgn_k; //CCW
		  }
		  if(ctrl_sgn_k <= 0){
			  TIM3->CCR1 = (-1) * ctrl_sgn_k; //CW
			  TIM3->CCR2 = 0;
		  }
		  if (fabs(rad_angle_1) <= 0.2){
			  cnt_trns++;
		  } else {
			  cnt_trns = 0;
		  }
		  if (cnt_trns >= 1000){
			  mode = 0;
			  prev_error_1 = 0;
			  prev_error_2 = 0;
			  itg_error_1 = 0;
			  itg_error_2 = 0;
			  integral_1 = 0;
			  integral_2 = 0;
			  ctrl_sgn_2 = 0;
			  ctrl_sgn_k = 0;
		  }
		  break;
	  case 2:
		  rad_angle_1 = (roundf(1000 * counterValue_1 * pi * 2.0 / 4000.0) / 1000.0) - pi;
		  rad_angle_1 = wrap_angle_deg(rad_angle_1, pi);
		  rad_angle_2 = roundf(1000 * counterValue_2 * pi * 2.0 / 300.0) / 1000.0;
		  drv_angle_1 = compute_derivative(rad_angle_1, prev_angle_1, Ts);
		  fdrv_angle_1 = filter(drv_angle_1, &fdrv_angle_1_then, 0.0012);
		  drv_angle_2 = compute_derivative(rad_angle_2, prev_angle_2, Ts);
		  fdrv_angle_2 = filter(drv_angle_2, &fdrv_angle_2_then, 0.0012);
		  ctrl_sgn_k = 100000 * ((0.1412)* (rad_angle_2) + (1.8605)* rad_angle_1 + (0.0781)* fdrv_angle_2 + (0.2341)* fdrv_angle_1);
		  if (ctrl_sgn_k > 70000) ctrl_sgn_k = 70000;
		  if (ctrl_sgn_k < -70000) ctrl_sgn_k = -70000;
		  if(ctrl_sgn_k >= 0){
			  TIM3->CCR1 = ctrl_sgn_k;
			  TIM3->CCR2 = 0; //CCW
		  }
		  if(ctrl_sgn_k <= 0){
			  TIM3->CCR1 = 0; //CW
			  TIM3->CCR2 = (-1) * ctrl_sgn_k;
		  }
		  if(fabs(rad_angle_1) > 0.3 * pi){
		  		  mode = 1;
		  }
		  prev_angle_1 = rad_angle_1;
		  prev_angle_2 = rad_angle_2;
		  break;

	  case 3:
		  TIM3->CCR1 = 0;
		  TIM3->CCR2 = 0;
		  x1 = 0;
		  x2 = 0;
		  fdx1 = 0;
		  fdx2 = 0;
		  itg_error_1 = 0;
		  itg_error_2 = 0;
		  ctrl_sgn_2 = 0;
		  ctrl_sgn_k = 0;
		  break;
	  }
	  printt_then = printt;
	  if (printt - printt_then2 > 5){
//		  if (printt < 20000){
			  if (tx_ready) {
				  send_uart_dma();
				  tx_ready = 0;
//			  }
		  } else {

		  }
		  printt_then2 = printt;
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
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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
  huart1.Init.BaudRate = 921600;
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
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
