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
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/** UART **/
uint8_t rx_byte;
char rx_buffer[16];
uint8_t idx = 0;

volatile bool pong_received = false;
volatile uint32_t last_ping_time = 0;
const uint32_t TIMEOUT_MS = 2000;

/** MOTORES **/
typedef struct {
    int last_direction;
    int last_pwm;
} motor_state_t;

motor_state_t motorA = {0, 0};
motor_state_t motorB = {0, 0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (rx_byte != '\n' && idx < sizeof(rx_buffer)-1)
        {
            rx_buffer[idx++] = rx_byte;
        }
        else
        {
            rx_buffer[idx] = '\0';
            idx = 0;

            if (strncmp((char*)rx_buffer, "PING", 4) == 0)
            {
                char *pong = "PONG\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)pong, strlen(pong), 20);
            }
            else if (strncmp((char*)rx_buffer, "PONG", 4) == 0)
            {
                pong_received = true;
                last_ping_time = HAL_GetTick(); // Reinicia timeout
            }
        }

        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

void motor_control(uint8_t motor_id, int pwm, int direction)
{
    TIM_HandleTypeDef *htim = &htim2;

    // Selección de canales y pines EN
    uint32_t ch1, ch2;
    uint16_t en_pin;
    GPIO_TypeDef *en_port = GPIOB;
    motor_state_t *motor;

    if (motor_id == 0) {
        // Motor Right --> PA0 (CH1) y PA1 (CH2)
        ch1 = TIM_CHANNEL_1;   // MRF
        ch2 = TIM_CHANNEL_2;   // MRB
        en_pin = MENR_Pin;
        motor = &motorA;
    } else {
        // Motor Left --> PB10 (CH3) y PB11 (CH4)
        ch1 = TIM_CHANNEL_3;   // MLF
        ch2 = TIM_CHANNEL_4;   // MLB
        en_pin = MENL_Pin;
        motor = &motorB;
    }

    // Si PWM = 0 --> apagar motor
    if (pwm <= 0) {
        HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_RESET);

        __HAL_TIM_SET_COMPARE(htim, ch1, 0);
        __HAL_TIM_SET_COMPARE(htim, ch2, 0);

        motor->last_direction = 0;
        motor->last_pwm = 0;
        return;
    }

    // Detectar cambio de sentido --> aplicar Dead Time
    if (direction != motor->last_direction && motor->last_direction != 0) {
        __HAL_TIM_SET_COMPARE(htim, ch1, 0);
        __HAL_TIM_SET_COMPARE(htim, ch2, 0);
        HAL_Delay(1); // Dead time 1 ms por seguridad
    }

    // Activar enable del motor
    HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_SET);

    // Aplicar PWM según dirección
    if (direction > 0) {
        // Forward
    	__HAL_TIM_SET_COMPARE(htim, ch2, 0);
        __HAL_TIM_SET_COMPARE(htim, ch1, pwm);
    } else {
        // Backward
        __HAL_TIM_SET_COMPARE(htim, ch1, 0);
        __HAL_TIM_SET_COMPARE(htim, ch2, pwm);
    }

    motor->last_direction = direction;
    motor->last_pwm = pwm;
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Inicializar PWM apagados
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  last_ping_time = HAL_GetTick();

  // UART INICIALIZADO, APAGO LOS PWM Y LOS PINES DE ENABLE MOTOR

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	uint32_t now = HAL_GetTick();
//
//	// Si llegó PONG → enviar otro PING
//	if (pong_received)
//	{
//	  char *ping = "PING\n";
//	  HAL_UART_Transmit(&huart2, (uint8_t*)ping, strlen(ping), 20);
//	  pong_received = false;
//	  last_ping_time = now;
//	}
//
//	// Timeout → reintentar PING
//	if ((now - last_ping_time) >= TIMEOUT_MS)
//	{
//	  char *ping = "PING\n";
//	  HAL_UART_Transmit(&huart2, (uint8_t*)ping, strlen(ping), 20);
//	  last_ping_time = now;
//	}

	    // --- Paso 1: Motores apagados ---
	    motor_control(0, 0, 0);
	    motor_control(1, 0, 0);
	    HAL_Delay(500);

	    // --- Paso 2: Ambos motores al 30% adelante ---
	    motor_control(0, 300, 1);
	    motor_control(1, 300, 1);
	    HAL_Delay(1200);

	    // --- Paso 3: Ambos al 100% adelante ---
	    motor_control(0, 999, 1);
	    motor_control(1, 999, 1);
	    HAL_Delay(1000);

	    // --- Paso 4: Motor Right apagado ---
	    motor_control(0, 0, 0);
	    HAL_Delay(500);

	    // --- Paso 5: Motor Left retroceso 40% ---
	    motor_control(1, 400, -1);
	    HAL_Delay(800);

	    // --- Paso 6: Ambos retroceso 100% ---
	    motor_control(0, 999, -1);
	    motor_control(1, 999, -1);
	    HAL_Delay(1200);

	    // --- Paso 7: Motores apagados ---
	    motor_control(0, 0, 0);
	    motor_control(1, 0, 0);
	    HAL_Delay(600);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MENL_Pin|MENR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MENL_Pin MENR_Pin */
  GPIO_InitStruct.Pin = MENL_Pin|MENR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
