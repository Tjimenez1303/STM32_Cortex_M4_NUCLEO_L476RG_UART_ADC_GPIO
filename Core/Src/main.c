/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/****************************************************PROTOTIPADO VARIABLES*********************************************************/

/****************************************Prototipado de banderas de botones*****************************/
uint8_t ban_button_up;
uint8_t ban_button_low;
uint8_t ban_button_TS;


/***********************************Prototipado variables del temperature sensor*************************/
uint8_t Rx_data_lim[lenght_Rx];
uint8_t Presence;
uint8_t Temp_byte1;
uint8_t Temp_byte2;
float temp_dec;

/****************************************Prototipado variables del UART******************************/
uint8_t Rx_data[lenght_Rx];
uint8_t Rx_data_color[lenght_Rx];
uint8_t ban_recibido_msj;
uint8_t ban_mismo_msj;

uint8_t transmit_lenght;
uint8_t transmit_text[200];

uint32_t li_red;
uint32_t li_blue;
uint32_t li_green;
uint32_t ls_red;
uint32_t ls_blue;
uint32_t ls_green;

uint8_t pos_in;
uint8_t pos_fin;


/*********************Prototipado variables control encendido Leds touch******************************/
uint8_t lecture;
uint16_t cont_time_offline;

/*********************Prototipado variables del ADC**************************************************/
float vin;
uint16_t raw;
uint8_t Rx_data_ADC[lenght_Rx];

/*************************Prototipado variables del SPI FLASH***************************************/
uint8_t read_flash_Byte[255];
uint8_t things_to_write[255];
uint8_t size_to_send;
volatile uint8_t address_to_write[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/*****************************************************PROTOTIPADO FUNCIONES*********************************************************/


/****************************************Prototipado de funciones del temperature sensor*****************************/
void delay (uint32_t us);
void Set_Pin_Input(GPIO_TypeDef *Port, uint32_t Pin);
static void Set_Pin_Output(GPIO_TypeDef *Port, uint32_t Pin);

uint8_t DS18B20_Start (void);
void DS18B20_Write (uint8_t data);
uint8_t DS18B20_Read (void);

float convert_temperature(uint8_t byte_1, uint8_t byte_2 );


/****************************************Prototipado de funciones del UART****************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
static  int pow (int num, int exp);
static int Get_number(void);

static void Led_RGB_Limits_Temp (int temp_li_green ,int temp_li_blue, int temp_li_red, int temp_ls_green ,int temp_ls_blue, int temp_ls_red);



/*******************************Prototipado de funciones control encendido Leds touch************************/

//static void Time_LED_Control (int estado);


/****************************************Prototipado de SPI Flash****************************************/
void flash_csn( uint8_t val);
void Write_flash_enable();
void Flash_read_identification_id();
void Flash_read_page(uint8_t address_1, uint8_t address_2, uint8_t address_3, uint8_t aux_length);
void Flash_write_page(uint8_t address_1, uint8_t address_2, uint8_t address_3, uint8_t *value_to_write, uint8_t length);
void erase_all ();
uint8_t read_status_register();


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
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /*******************************************************INICIALIZACIONES********************************************************/


  /****************************************Inicialización banderas de botones*****************************/

  ban_button_up = 0;
  ban_button_low = 0;
  ban_button_TS = 0;

  /****************************************Inicialización banderas de UART********************************/
  ban_recibido_msj = 0;
  ban_mismo_msj = 0;

  /******************************Inicialización control encendido Leds touch******************************/
  lecture = 0;
  cont_time_offline = 0;



  /************************************Inicialización variables del temperature sensor*******************/
  Presence = 0;
  Temp_byte1 = 0;
  Temp_byte2 = 0;

  temp_dec = 0;


  /****************************************Inicialización variables del UART****************************/
  pos_in = 0;
  pos_fin = 0;

  li_red = 0;
  li_blue = 0;
  li_green = 0;
  ls_red = 0;
  ls_blue = 0;
  ls_green = 0;

  /**************************************Inicialización variables del ADC******************************/
  vin = 0;
  raw = 0;


  /****************************************Inicialización del UART IT**********************************/
  HAL_UART_Receive_IT(&huart1, Rx_data, 13);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  /****************************************Inicialización del la FLASH********************************/
//	  flash_csn(1);
//	  Flash_read_identification_id();
//
//	  Flash_read_page(0x00, 0x01, 0x02, 255);
//	  HAL_Delay(100);

//
//	  //Inicialización
//	  flash_csn(1); //Pone el CS en 1
//	  memset(&read_flash_Byte, 0x00,255); //Limpia el vector read_flash_Byte con 0s en all
//	  Flash_read_identification_id();
//
//
//	  //	  size_to_send = sprintf( &transmit_text, "value read: %d \r\n", read_flash_Byte[0] );
//	  //	  HAL_UART_Transmit(&huart1, &transmit_text, size_to_send, 1000);
//
//	  memset(&things_to_write, 0x00,255); //Pone all en 0 (limpia)
//	  address_to_write[0] = 0x00;
//	  address_to_write[2] = 0x02;
//	  Flash_read_page(0x00, 0x01, 0x02, 255);
//	  erase_all();
//	  HAL_Delay(100);
//
//
//	  memset(&things_to_write, 0x20,255);
//	  Flash_write_page( 0x00, 0x01, 0x02 , &things_to_write[0] , 255 );
//	  HAL_Delay(100);
//
//	  //Lee la página POS1	POS2   POS3 Size
//	  Flash_read_page(0x00, 0x01, 0x02, 255);
//	  erase_all();
//	  HAL_Delay(100);
//
//
//
//
//	  //	  size_to_send = sprintf( &things_to_write, "the temperature is: %d \r\n", 30 );
//
//
//
//
//	  memset(&things_to_write, 0x30,255);
//	  Flash_write_page(0x00, 0x01, 0x02 , &things_to_write[1] , size_to_send );
//	  HAL_Delay(100);
//	  //
//	 //Lee la página POS1	POS2   POS3 Size
//	  Flash_read_page(0x00, 0x01, 0x02, 255);
//	  HAL_Delay(1000);







/***************************************************PROGRAMACIÓN del ADC*************************************************************/
	  HAL_ADC_Start(&hadc);

	  HAL_ADC_PollForConversion(&hadc, 300);
	  raw = HAL_ADC_GetValue(&hadc);

	  vin = (raw * (MAX_VALUE_ADC_V/MAX_VALUE_ADC_BITS))+OFFSET;





	  /*****************************************Comando de impresión ADC*********************************/
	  if (ban_recibido_msj == 1){
		  strncpy (&Rx_data_ADC, &Rx_data, 10);

		  if (strcmp(&Rx_data_ADC,"#LECTURE_A") == 0){
			  transmit_lenght = sprintf(&transmit_text[0], "El voltaje de entrada es: %f \r\n", vin);
			  HAL_UART_Transmit(&huart1, transmit_text, transmit_lenght, 1000);
			  ban_recibido_msj = 0;

		  }
	  }

/****************************************PROGRAMACIÓN TEMPERATURE SENSOR**************************************************************/
	  Presence = DS18B20_Start ();
	  HAL_Delay (1);
	  DS18B20_Write (0xCC);  // skip ROM				//Inicialización
	  DS18B20_Write (0x44);  // convert t
	  HAL_Delay (800);

	  Presence = DS18B20_Start ();
	  HAL_Delay(1);
	  DS18B20_Write (0xCC);  // skip ROM				//Inicialización
	  DS18B20_Write (0xBE);  // Read Scratch-pad


	  Temp_byte1 = DS18B20_Read ();						//Lectura del sensor
	  Temp_byte2 = DS18B20_Read ();

	  temp_dec = convert_temperature(Temp_byte1, Temp_byte2); //Conversión de hexadecimal a decimal



	  /*****************************************Impresión temperatura*********************************/

	  transmit_lenght = sprintf(&transmit_text[0], "La temperatura es: %f \r\n", temp_dec);
	  HAL_UART_Transmit(&huart1, transmit_text, transmit_lenght, 1000);


	  /****************************************PROGRAMACIÓN UART**********************************************************************/



	  /*********************************Obtención de límites de temperatura***************************************/

	  if (ban_recibido_msj == 1){ //Mira si se recibe algún mensaje

		  strncpy (&Rx_data_color, &Rx_data, 8); //Recorta para comparar el comando

		  /*********************LI Y LS GREEN************************/

		  if (strcmp(&Rx_data_color,"#TEMPLIG") == 0){
			  li_green = Get_number();
			  ban_recibido_msj = 0;
		  }

		  if (strcmp(&Rx_data_color,"#TEMPLSG") == 0){
			  ls_green = Get_number();
			  ban_recibido_msj = 0;
		  }

		  /*********************LI Y LS BLUE************************/

		  if (strcmp(&Rx_data_color,"#TEMPLIB") == 0){
			  li_blue = Get_number();
			  ban_recibido_msj = 0;
		  }

		  if (strcmp(&Rx_data_color,"#TEMPLSB") == 0){
			  ls_blue = Get_number();
			  ban_recibido_msj = 0;
		  }

		  /*********************LI Y LS RED************************/

		  if (strcmp(&Rx_data_color,"#TEMPLIR") == 0){
			  li_red = Get_number();
			  ban_recibido_msj = 0;
		  }

		  if (strcmp(&Rx_data_color,"#TEMPLSR") == 0){
			  ls_red = Get_number();
			  ban_recibido_msj = 0;
		  }

	  }


	  /*********************Impresión por UART de límites************************/
	  if (ban_recibido_msj == 1){
		  strncpy (&Rx_data_ADC, &Rx_data, 10);

		  if (strcmp(&Rx_data_ADC,"#LECTURE_L") == 0){
			  transmit_lenght = sprintf(&transmit_text[0], "VERDE-> LI: %d LS: %d \r\n",li_green,ls_green);
			  HAL_UART_Transmit(&huart1, transmit_text, transmit_lenght, 1000);

			  transmit_lenght = sprintf(&transmit_text[0], "AZUL-> LI: %d LS: %d \r\n",li_blue,ls_blue);
			  HAL_UART_Transmit(&huart1, transmit_text, transmit_lenght, 1000);

			  transmit_lenght = sprintf(&transmit_text[0], "ROJO-> LI: %d LS: %d \r\n",li_red,ls_red);
			  HAL_UART_Transmit(&huart1, transmit_text, transmit_lenght, 1000);

			  ban_recibido_msj = 0;

		  }
	  }








	  /*********************************Control encendido LED RGB***************************************/
	  Led_RGB_Limits_Temp(li_green ,li_blue, li_red,ls_green ,ls_blue, ls_red );


	  /**************************************CONTROL ENCENDIDO LEDS TOUCH *******************************************************/
	  if (ban_button_up == 1){
		  cont_time_offline ++;
		  ban_button_up = 0;
	  }

	  if (ban_button_low == 1){
		  if (cont_time_offline > 0){
			  cont_time_offline --;
		  }

		  ban_button_low = 0;
	  }

	  /********************************Comando de impresión Tiempo de desconexion******************************/
	  if (ban_recibido_msj == 1){
		  strncpy (&Rx_data_ADC, &Rx_data, 10);

		  if (strcmp(&Rx_data_ADC,"#LECTURE_T") == 0){
		  	  transmit_lenght = sprintf(&transmit_text[0], "Tiempo de desconexion= %d \r\n",cont_time_offline);
		  	  HAL_UART_Transmit(&huart1, transmit_text, transmit_lenght, 1000);
			  ban_recibido_msj = 0;

		  }
	  }



	  lecture = HAL_GPIO_ReadPin(TTP223_GPIO_Port, TTP223_Pin);
	  if (lecture == 1){
		  LED_GREEN_ON;
		  LED_RED_OFF;
		  HAL_Delay(cont_time_offline *value_ms_to_s);
	  }
	  else{
		  LED_RED_ON;
		  LED_GREEN_OFF;
	  }







	  /***************************************************PROGRAMACIÓN FLASH *******************************************************/


	  /**********************************Guarda los valores en flash************************************************/
	  things_to_write [0] = temp_dec;
	  things_to_write [1] = li_green;
	  things_to_write [2] = li_blue;
	  things_to_write [3] = li_red;
	  things_to_write [4] = ls_green;
	  things_to_write [5] = ls_blue;
	  things_to_write [6] = ls_red;
	  things_to_write [7] = cont_time_offline;

	  //memset(&things_to_write, 0x30,255);

	  /**********************************Escribe valores en flash************************************************/
//	  Flash_write_page(0x00, 0x01, 0x02 , &things_to_write , size_to_send );
//	  HAL_Delay(100);


	  //Lee la página POS1	POS2   POS3 Size
//	  Flash_read_page(0x00, 0x01, 0x02, 255);


	  /****************************Enviamos por UART los valores de la flash**********************************/
//	  size_to_send = sprintf( &transmit_text, "La temperatura es: %d \r\n", read_flash_Byte[0] );
//	  HAL_UART_Transmit(&huart1, &transmit_text, size_to_send, 1000);
//
//	  size_to_send = sprintf( &transmit_text, "El limite inferior verde es: %d \r\n", read_flash_Byte[1] );
//	  HAL_UART_Transmit(&huart1, &transmit_text, size_to_send, 1000);
//
//	  size_to_send = sprintf( &transmit_text, "El limite inferior azul es: %d \r\n", read_flash_Byte[2] );
//	  HAL_UART_Transmit(&huart1, &transmit_text, size_to_send, 1000);
//
//	  size_to_send = sprintf( &transmit_text, "El limite inferior rojo es: %d \r\n", read_flash_Byte[3] );
//	  HAL_UART_Transmit(&huart1, &transmit_text, size_to_send, 1000);

//	  HAL_Delay(1000);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 48-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
   	{
   		Error_Handler();
   	}

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_CS_Pin|LED_BLUE_RGB_Pin|LED_GREEN_RGB_Pin|LED_RED_RGB_Pin
                          |LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI_CS_Pin LED_BLUE_RGB_Pin LED_GREEN_RGB_Pin LED_RED_RGB_Pin
                           LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin|LED_BLUE_RGB_Pin|LED_GREEN_RGB_Pin|LED_RED_RGB_Pin
                          |LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TTP223_Pin BUTTON_UP_Pin BUTTON_LOW_Pin */
  GPIO_InitStruct.Pin = TTP223_Pin|BUTTON_UP_Pin|BUTTON_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/****************************************FUNCIONES DE CALLBACKS BOTONES***************************************************++*******/


/****************************************Función de Callback botón UP ,LOW Y TOUCH SENSOR*****************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == PIN_BUTTON_UP_IT){ // 1024
		ban_button_up = 1;
	}

	if (GPIO_Pin == PIN_BUTTON_LOW_IT){ //2048
		ban_button_low = 1;
	}

	if (GPIO_Pin == GPIO_PIN_1)
	{
		ban_button_TS = 1;
	}

}

/****************************************FUNCIONES DEL UART***************************************************++*******/


/****************************************Función Callback de UART*****************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(&huart1, Rx_data, 13);
	ban_recibido_msj = 1;
	ban_mismo_msj = 0;
}


/****************************************Función para obtener el número del UART***************/
static int Get_number(void){
	uint32_t result;

	for (int i = 0; i<lenght_Rx + 1; i++){
		if (Rx_data[i] == '$'){
			pos_in = i;
			for (int j = i+1; j<lenght_Rx+1; j++){
				if (Rx_data[j] == '$'){
					pos_fin = j;
					break;
				}
			}
			break;
		}
	}

	result = 0;
	for (int z = pos_in+1; z<=pos_fin-1; z++ ){
		result += pow(Rx_data[z]-48, pos_fin-1-z);
	}

	return result;

}

/****************************************Función para potencia*******************************/
static  int pow (int num, int exp ){

	int result = 1;

	if (exp == 0){
		return num;
	}
	else{
		for (int i = 0; i<exp; i++){
			result = result * num;
		}
		return 10*result;
	}
}


/*************************************Función para Control encendido LED RGB*******************************/
static void Led_RGB_Limits_Temp (int temp_li_green ,int temp_li_blue, int temp_li_red, int temp_ls_green ,int temp_ls_blue, int temp_ls_red){

	LED_RED_RGB_OFF;
	LED_BLUE_RGB_OFF;
	LED_GREEN_RGB_OFF;

	if ((temp_dec>temp_li_green) && (temp_dec <= temp_ls_green)){
		LED_GREEN_RGB_ON;
	}

	if ((temp_dec>temp_li_blue) && (temp_dec <= temp_ls_blue)){
		LED_BLUE_RGB_ON;
	}

	if ((temp_dec>temp_li_red) && (temp_dec <= temp_ls_red)){
		LED_RED_RGB_ON;
	}
}



/****************************************FUNCIONES DEL TEMPERATURE SENSOR**********************************************************/


/****************************************Función de delay del temperature sensor******************************/
void delay (uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim6,0);  // set the counter value a 0
		while (__HAL_TIM_GET_COUNTER(&htim6) < us);  // wait for the counter to reach the us input in the parameter
}




/*************************************Función de establecer pin como entrada del temperature sensor*********/
void Set_Pin_Input(GPIO_TypeDef *Port, uint32_t Pin){
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	 /*Configure GPIO pin : DS18B20_Pin */
	  GPIO_InitStruct.Pin = Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(Port, &GPIO_InitStruct);
}




/****************************************Función de establecer pin como salida del temperature sensor********/
void Set_Pin_Output(GPIO_TypeDef *Port, uint32_t Pin){
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	 /*Configure GPIO pin : DS18B20_Pin */
	  GPIO_InitStruct.Pin = Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(Port, &GPIO_InitStruct);
}





/****************************************Función de inicialización del temperature sensor******************/
uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;

	Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);   // set the pin as output
	HAL_GPIO_WritePin (DS18B20_GPIO_Port, DS18B20_Pin, 0);  // pull the pin low
	delay (480);   // delay according to datasheet (480)

	Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);    // set the pin as input
	delay (80);    // delay according to datasheet (80)

	if (!(HAL_GPIO_ReadPin (DS18B20_GPIO_Port, DS18B20_Pin))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	delay (480); // 480 us delay totally. (400)

	return Response;
}



/****************************************Función para escribir un byte en el temperature sensor**************/
void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);  // set as output
			HAL_GPIO_WritePin (DS18B20_GPIO_Port, DS18B20_Pin, 0);  // pull the pin LOW
			delay (1);  // wait for 1 us (1)

			Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);  // set as input
			delay (50);  // wait for 60 us (50)
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);
			HAL_GPIO_WritePin (DS18B20_GPIO_Port, DS18B20_Pin, 0);  // pull the pin LOW
			delay (50);  // wait for 60 us (50)

			Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);
		}
	}
}



/****************************************Función para leer bits del temperature sensor**************************/
uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);

	for (int i=0;i<8;i++)
	{

		Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);   // set as output

		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, 0);  // pull the data pin LOW
		delay (2);  // wait for 2 us (2)

		Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);  // set as input
		//delay(17);
		if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_2))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (60);  // wait for 60 us (60)
	}
	return value;
}




/****************************************Función para convertir los bytes del temperature sensor********************/
float convert_temperature(uint8_t byte_1, uint8_t byte_2 ){
	uint16_t tempval = byte_2 << 8 | byte_1;
	float result_temp = (125.0 / 2048)*tempval;

	return  result_temp;

}






/****************************************FUNCIONES DEL TOUCH SENSOR LEDS**************************************************************/






/*************************************************FUNCIONES DEL SPI FLASH************************************************************/


/**********************************Función para poner el CS en LOW o HIGH********************/
void flash_csn( uint8_t val){
	if ( val == 0 )
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

}

/*********************************Función para Write Enable en la FLASH********************/
void Write_flash_enable(){
	uint8_t spiBuf = WREN;
	//Put CSN low
	flash_csn(0);
	//Transmit register address
	HAL_SPI_Transmit(&hspi1, &spiBuf, 1, 1000);
	flash_csn(1);
}

/*********************************Función para inicializr ID único en la FLASH********************/
void Flash_read_identification_id()
{
	uint8_t spiBuf[5];
	//Put CSN low
	spiBuf[0] = READ_ID;
	spiBuf[1] = 0;
	spiBuf[2] = 0;
	spiBuf[3] = 0;
	spiBuf[4] = 0;
	flash_csn(0);
	//Transmit register address

	HAL_SPI_Transmit(&hspi1, &spiBuf, 5, 1000);
	//Receive data
	HAL_SPI_Receive(&hspi1, &read_flash_Byte, 16, 1000);
	//Bring CSN high
	flash_csn(1);
}


/*********************************Función para escrbir en una página en la FLASH********************/
void Flash_write_page(uint8_t address_1, uint8_t address_2, uint8_t address_3, uint8_t *value_to_write, uint8_t length)
{
	Write_flash_enable();
	uint8_t spiBuf[length + 4];
	spiBuf[0] = PAGE_PROGRAM;
	// concatenate all the data to be sent
	sprintf( &spiBuf[1], "%c%c%c", address_1, address_2, address_3 );
	strncpy( &spiBuf[4], value_to_write, length );
	//Put CSN low
	flash_csn(0);
	//Transmit register address and data
	HAL_SPI_Transmit(&hspi1, &spiBuf, length+4 , 100);
	//Bring CSN high
	flash_csn(1);
}



/*********************************Función para leer una página en la FLASH***********************/
void Flash_read_page(uint8_t address_1, uint8_t address_2, uint8_t address_3, uint8_t aux_length)
{
	uint8_t spiBuf[4];
	//Put CSN low
	flash_csn(0);
	// concatenate all the data to be sent
	spiBuf[0] = READ_PAGE;
	sprintf( &spiBuf[1], "%c%c%c", address_1, address_2, address_3 );
	//Transmit register address
	HAL_SPI_Transmit(&hspi1, &spiBuf, 4, 1000);
	//Receive data
	HAL_SPI_Receive(&hspi1, &read_flash_Byte[0], aux_length, 1000);
	//Bring CSN high
	flash_csn(1);
}

/*********************************Función para borrar all en la FLASH***********************/
void erase_all (){
	// Activate WREN
	Write_flash_enable();
	uint8_t spiBuf[4];

	//Transmit register address
	spiBuf[0] = ERASE_ALL_COMAND;
	//sprintf( &spiBuf[1], "%c%c%c", address_1, address_2, address_3 );
	//Put CSN low
	flash_csn(0);
	HAL_SPI_Transmit(&hspi1, &spiBuf[0], 1, 1000);
	flash_csn(1);
}

/*********************************Función para ? en la FLASH*****************************/
uint8_t read_status_register(){
	uint8_t spiBuf[5];
	uint8_t retData;
	//Put CSN low
	flash_csn(0);
	//Transmit register address
	spiBuf[0] = READ_STATUS_REG;
	HAL_SPI_Transmit(&hspi1, &spiBuf[0], 1, 1000);
	//Receive data
	HAL_SPI_Receive(&hspi1, &retData, 1, 1000);
	//Bring CSN high
	flash_csn(1);
	return retData;
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
