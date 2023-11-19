/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "channelConfig.h"
#include <string.h>
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
const uint8_t lengthTransmit = 38; //16 entradas y salidas digitales, 8 entradas analógicas, 2 salidas analógicas y 3 bytes en incio
const uint8_t lengthReceive = 14; //Longitud que espera la uart del dato a recibir, son 8 salidas digitales y 2 analógicas y 2 bytes de inicio y fin
const uint8_t lengthDigInOut = 16; //Son 8 entradas digitales y 8 salidas digitales
const uint8_t lengthAnalogInOut = 10;
const char *errorTransmision = "reenviar Tx"; //código de error a recibir desde QT
uint8_t flagChange = 0; //variable para cambiar el valor de channelMux cuando el dato se haya guardado
uint8_t channelMux     =  0;  //variable para swtichear entre los canales del mux, va del 0 al 7
uint16_t analogInOut[lengthAnalogInOut]   = {}; //array para guardar las lecturas de las entradas analógicas, 8 entradas y 2 salidas
uint8_t digitalInOut[lengthDigInOut]   = {}; //array para guardar las lecturas de las entradas digitales, 8 entradas y salidas
uint8_t dataSend[lengthTransmit]   = {}; //array para el envio de datos
uint8_t dataReceive[lengthReceive] = {}; //array para el recibo de dato, no se si es necesario, lo sabremos luego
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void envioDeDatos(void);
void analogToByte(uint16_t * analogData);
void ADC_Select_CH1 (void);
void ADC_Select_CH1 (void);


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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3,dataReceive, lengthReceive); //receive data from data buffer interrupt mode
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 71;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart3.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(GPIOB, S0_Pin|S1_Pin|S2_Pin|D_OUT1_Pin
                          |D_OUT8_Pin|D_OUT4_Pin|D_OUT5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D_OUT2_Pin|D_OUT6_Pin|D_OUT3_Pin|D_OUT7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IN_MUX_Pin */
  GPIO_InitStruct.Pin = IN_MUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(IN_MUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D_IN7_Pin D_IN6_Pin D_IN5_Pin D_IN4_Pin
                           D_IN3_Pin D_IN2_Pin */
  GPIO_InitStruct.Pin = D_IN7_Pin|D_IN6_Pin|D_IN5_Pin|D_IN4_Pin
                          |D_IN3_Pin|D_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D_IN1_Pin D_IN8_Pin */
  GPIO_InitStruct.Pin = D_IN1_Pin|D_IN8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : S0_Pin S1_Pin S2_Pin D_OUT1_Pin
                           D_OUT8_Pin D_OUT4_Pin D_OUT5_Pin */
  GPIO_InitStruct.Pin = S0_Pin|S1_Pin|S2_Pin|D_OUT1_Pin
                          |D_OUT8_Pin|D_OUT4_Pin|D_OUT5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D_OUT2_Pin D_OUT6_Pin D_OUT3_Pin D_OUT7_Pin */
  GPIO_InitStruct.Pin = D_OUT2_Pin|D_OUT6_Pin|D_OUT3_Pin|D_OUT7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * Función para manejar las interrupciones del timer 1 dedicado a la lectura de entradas
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1){
		//leo las entrada digital correspondiente al case ejecutandose
		digitalInOut[channelMux] = HAL_GPIO_ReadPin(channelsIn[channelMux].portD, channelsIn[channelMux].digitalPin);

		if ( channelMux == 7){
			ADC_Select_CH0(); //selecciono el canal del adc a leer
			HAL_ADC_Start_IT(&hadc1); //inicio el acc en modo interrupcion
			if (flagChange == 1){
				channelMux = 0; //incrementa antes de pasar a envio de datos por si vuelve a activarse el timer antes de terminar envioDeDatos()
				flagChange = 0 ;
				envioDeDatos();
			}
		}else{
			//Configuro las llaves selectoras del MUX
			for (uint8_t i=0; i<3; i++){
				HAL_GPIO_WritePin(channelsIn[channelMux].port, channelsIn[channelMux].pin[i], (GPIO_PinState)channelsIn[channelMux].value[i]); //casteamos el valor para que sea un estado de pin admitido
			}
			ADC_Select_CH1(); //selecciono el canal del mux para leer
			HAL_ADC_Start_IT(&hadc1);
			if (flagChange == 1) {
				channelMux = (channelMux + 1) % 8; //para mantener el número entre 0 y 7
				flagChange = 0 ;
			}
		}
	}
}

/*
 * Funcion para cuando la conversión del ADC se termina
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	analogInOut[channelMux] = HAL_ADC_GetValue(&hadc1); //reemplazar por la lectura del ADC
	HAL_ADC_Stop_IT(&hadc1);
	flagChange = 1;

}

/*
 * Función para manejar los datos cuando la recepción esta completa
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	// Verifica si los datos recibidos son iguales a "reenviar transmision"
	if (strncmp((char*)dataReceive, errorTransmision, strlen(errorTransmision)) == 0) {
		HAL_UART_Transmit_IT(&huart3,dataSend, lengthTransmit); //envio de datos a través de la uart por interrupción
	}else if ( dataReceive[0] == 'I' && dataReceive[lengthReceive-1] == 'E'){
		//Escribo los valores obtenidos en las salidas digitales
		for (uint8_t i = 0; i<=8 ; i++){
			HAL_GPIO_WritePin(channelsOut[i].port, channelsOut[i].digitalPin, dataReceive[i+1]);
		}
		//Modifico el formato de la salida analógica recibida a tipo uint_16
		uint16_t A_OUT1 = (uint16_t)(dataReceive[9] << 8 | dataReceive[10]);
		uint16_t A_OUT2 = (uint16_t)(dataReceive[11] << 8 | dataReceive[12]);
		//Lo guardo en el array de entradas y salidas analógicas
		analogInOut[8] = A_OUT1;
		analogInOut[9] = A_OUT2;
		// Asigno los valores a CCR1 y CCR2, previamente casteados a uint32, que manejan el duty del PWM
		TIM4->CCR1 = (uint32_t)A_OUT1;
		TIM4->CCR2 = (uint32_t)A_OUT2;
		//Envío los datos para actualizar los valores de QT
		envioDeDatos();

	}
}


/*
 * Funcion que se encarga de leer las salidas y almacenar los datos de las entradas en el array sendData para después enviarlo por uart
 */
void envioDeDatos(void){
	uint8_t Outs = 0;
	for (uint8_t i=0; i< lengthDigInOut + 3; i++){
		if(i == 0) dataSend[i] = 'x';
		else if (i == 1) dataSend[i] = 'f';
		else if (i == 2) dataSend[i] = 'x';
		else if(i>2 && i<=10) dataSend[i] = digitalInOut[i]; // guardo en el array dataSend las entradas digitales
		else {
			//Guardo en el array sendData las salidas digitales
			dataSend[i] = HAL_GPIO_ReadPin(channelsOut[Outs].port, channelsOut[Outs].digitalPin);
			Outs = (Outs + 1) % 8;
		}
	}
	analogToByte(analogInOut);
	HAL_UART_Transmit_IT(&huart3,dataSend, lengthTransmit); //envio de datos a través de la uart por interrupción

}

/*Función que se encarga de transformar los valores obtenidos del ADC (0 al 4095) en bytes y almacenarlos en sendData
 * ocupan los lugres del 19 al 38 en el array
 */
void analogToByte(uint16_t * analogData) {
  uint8_t k = 19;
  for (uint8_t i = 0; i < lengthAnalogInOut; ++i) {
	  uint16_t numero = analogData[i];
	  // cada entero tipo uin16_t ocupa 2 bytes (uint8_t)
      for (uint8_t j=0; j<2; j++){
        dataSend[k] = (numero >> (j * 8)) & 0xFF;
        k++;
      }
  }
}

void ADC_Select_CH0 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}


void ADC_Select_CH1 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = ADC_REGULAR_RANK_2;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
		Error_Handler();
	  }
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
