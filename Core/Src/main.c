/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 202
  * 4 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "tinygps.h"
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
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

//GPS Variables//************************
char RX_BufferGPS[RX_BUFFER_SIZE_GPS];
uint8_t GPS_encode = 0;
uint32_t GPS_timer= 0;
extern  bool _is_gps_data_good;
uint8_t gps_sleepFlag = 0;
uint8_t gps_wakeFlag = 0;
//***************************************

//Debug Variables//**********************
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
//***************************************

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//GPS control functions//*****************************************
void gps_processNMEA(char *buffer,uint16_t length);
void gps_init();
int gps_command(uint8_t *cmd, char *response);
uint8_t get_checksum(char *cmd);
void gps_updateMode(uint8_t mode,uint16_t on, uint16_t off);
//******************************************************

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

	if(huart->Instance == USART1){
		//char output[Size + 1];
		char* result = {0};
		result = strstr((char *)RX_BufferGPS,"$PMTK,001");

		if(result == NULL){ //Prevents command responses from being NMEA processed
			for(int i = 0; i < Size; i++ ){
				if(RX_BufferGPS[i] == '\n'){
					//snprintf(output, i+2 ,"%s\r\n",RX_BufferGPS);//print gps incoming message
					//CDC_Transmit_FS((uint8_t*) output, strlen(output));
					gps_processNMEA(RX_BufferGPS, i+1);
					memset(RX_BufferGPS,0,RX_BUFFER_SIZE_GPS);
				}
			}
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t *) RX_BufferGPS, RX_BUFFER_SIZE_GPS);
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
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);


  gps_init();

  float latitude = 0;
  float longitude = 0;
  unsigned long fixAgeGPS = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //GPS Data code//****************************************************************
		  if(GPS_encode == 1){
			  if(_is_gps_data_good){ // update current coordinates if data is good
			  	  gps_f_get_position(&latitude, &longitude,&fixAgeGPS );
			  	  char output[50];
			  	  snprintf(output, sizeof(output),"Lat: %.6f, Lon: %.6f, \r\n",latitude,longitude);
		 		  CDC_Transmit_FS((uint8_t *)output, strlen(output));
		 		  GPS_encode = 0;
		 	  }
		  else{
			  char output[25];
			  snprintf(output, sizeof(output),"GPS data is not good\r\n");
			  //CDC_Transmit_FS((uint8_t *)output, strlen(output));
			  GPS_encode = 0;
		  	  }

			  if((gps_sleepFlag == 1)&&(gps_wakeFlag == 1)){
				  char output[] = "Leaving periodic power mode";
				  CDC_Transmit_FS((uint8_t *)output, strlen(output));
				  gps_command((uint8_t *) gps_Test, "PMTK001,0,3");
				  HAL_Delay(3000);
				  gps_command((uint8_t *) gps_FullPower, "PMTK001,225,3");
				  gps_command((uint8_t *) LOW_UPDATE_RATE, "PMTK001,220,3");
				  gps_sleepFlag = 0;
				  gps_wakeFlag = 0;
			  }
		  }
		  //**************************************************************************
		      if(UserRxBufferFS[0] == '1'){
		    	  gps_command((uint8_t *) gps_Test, "PMTK001,0,3");
		    	  gps_command((uint8_t *) gps_FullPower, "PMTK225,0,3");
		 		  memset(UserRxBufferFS,'\0',strlen((char *)UserRxBufferFS));
		 	  }

		 	  else if(UserRxBufferFS[0] == '2'){
		 		 gps_updateMode(3,10000,30000);
		 		  memset(UserRxBufferFS,'\0',strlen((char *)UserRxBufferFS));
		 	  }

		 	  else if(UserRxBufferFS[0] == '3'){
		 		  uint8_t TEST[] = "$PMTK000*32\r\n"; //Message wakes up the gps module
		 		   gps_command(TEST,"PMTK001,0,3");
		 		   HAL_Delay(5000);
		 		  gps_command((uint8_t *) LOW_UPDATE_RATE, "PMTK001,220,3");
		 		   memset(UserRxBufferFS,'\0',strlen((char *)UserRxBufferFS));
		 	  }

		 	  else if(UserRxBufferFS[0] == '4'){
		 		  uint8_t rmcCommand[] = "$PMTK161,0*28\r\n"; //enable sleep mode only
		 		  	gps_command(rmcCommand, "PMTK001,161,3");
		 		  memset(UserRxBufferFS,'\0',strlen((char *)UserRxBufferFS));
		 	  }

		 	 else if(UserRxBufferFS[0] == '5'){
		 			 gps_wakeFlag = 1;
		 			 memset(UserRxBufferFS,'\0',strlen((char *)UserRxBufferFS));
		 			 }

		 	else if(UserRxBufferFS[0] == '6'){
		 		        gps_command((uint8_t *) LOW_UPDATE_RATE, "PMTK001,220,3");
		 			 	memset(UserRxBufferFS,'\0',strlen((char *)UserRxBufferFS));
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
	CDC_Transmit_FS((uint8_t *)"UART Error\r\n", 13);
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
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GSM_PWX_GPIO_Port, GSM_PWX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : User_Button___KEY_Pin */
  GPIO_InitStruct.Pin = User_Button___KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(User_Button___KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_RST_Pin */
  GPIO_InitStruct.Pin = GPS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPS_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GSM_PWX_Pin */
  GPIO_InitStruct.Pin = GSM_PWX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GSM_PWX_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void gps_processNMEA(char *buffer,uint16_t length){ //parse full message to gps encode

	for(int j = 0; j<(length); j++){
		if (gps_encode(buffer[j])){
			GPS_encode = 1;  //Get GPS information
		}
		}
	}

void gps_init(){
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t *) RX_BufferGPS, RX_BUFFER_SIZE_GPS);

	if(gps_command((uint8_t *) gps_Test, "PMTK001,0,3") != Gpsok){ //hard reset
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0);
				HAL_Delay(1000);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,1);
			}

	gps_command((uint8_t *) gps_FullPower, "PMTK001,225,0,3");

	uint8_t baudRateGPS[] = "$PMTK251,9600*17\r\n"; //set UART baud rate to 9600
	gps_command(baudRateGPS,"PMTK001,251");

	//set update position report to every 10 seconds (works)
	gps_command((uint8_t *) LOW_UPDATE_RATE, "PMTK001,220,3");

	uint8_t rmcCommand[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; //enable RMC mode only
	gps_command(rmcCommand, "PMTK001,314,3");
}


int gps_command(uint8_t *cmd, char *response) //Send Commands to GPS and check response
{

	HAL_Delay(100);

	HAL_UART_Transmit_DMA(&huart1, (uint8_t *) cmd, strlen((char *)cmd));
    CDC_Transmit_FS((uint8_t *)cmd,  strlen((char *)cmd));

    char *P = NULL;
    char *Failed = NULL;
    GPS_timer = HAL_GetTick();

    while(*P != response[0]){
    	P = strstr(RX_BufferGPS, response);
    	Failed = strstr(RX_BufferGPS, ",2*");
    	Failed = strstr(RX_BufferGPS, ",1*");
    	Failed = strstr(RX_BufferGPS, ",0*");


    	if((HAL_GetTick() - GPS_timer) > 2000){ //2s time-out error
    		memset(RX_BufferGPS, 0, RX_BUFFER_SIZE_GPS);
    		CDC_Transmit_FS((uint8_t *)"Time Out Error\r\n", 15); //debug terminal
    		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t *) RX_BufferGPS, RX_BUFFER_SIZE_GPS);
    		return GpsError;
    	}

    	if(Failed != NULL){
    		memset(RX_BufferGPS, 0, RX_BUFFER_SIZE_GPS);
    		CDC_Transmit_FS((uint8_t *)"Failed\r\n", 15); //debug terminal
    		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t *) RX_BufferGPS, RX_BUFFER_SIZE_GPS);
    		return GpsError;
    	}

    }

    memset(RX_BufferGPS, 0, RX_BUFFER_SIZE_GPS);
    CDC_Transmit_FS((uint8_t *)response,  strlen(response));
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t *) RX_BufferGPS, RX_BUFFER_SIZE_GPS);
	return Gpsok;
}


void gps_updateMode(uint8_t mode,uint16_t on, uint16_t off){ // implement mode changes from messages or timing
	switch(mode){
	case(1): //Full power continuous mode
			gps_command((uint8_t *)LOW_UPDATE_RATE, "PMTK001,220,3");
			uint8_t cmd1[] = "$PMTK225,0*2B";
	        gps_command(cmd1,"PMTK001,225,3");
			break;
	case(2): //Standby low power mode
			uint8_t cmd2[] = "$PMTK161,0*28";
			gps_command(cmd2,"PMTK001,161,3");
			break;
	case(3): //Periodic on/off mode -> need to implement a function that handles timing length changes (maybe use presets)
			if((on < 65536) && (off < 65536)){//check under max limit
			char cmd3[60] = {0};
			snprintf(cmd3,sizeof(cmd3) ,"$PMTK225,2,%u,%u,%u,%d*", on, off, on, off);
			uint8_t checksum = get_checksum(cmd3);
			char finalcmd3[70] = {0};
			snprintf(finalcmd3, sizeof(finalcmd3),"%s%X\r\n",cmd3,checksum);
			gps_command((uint8_t *) HIGH_UPDATE_RATE,"PMTK001,220,3");
			gps_command((uint8_t *)finalcmd3,"PMTK001,225,3");
			gps_sleepFlag = 1;
			}
			break;
	case(4): //Standby mode -> deep power saving need to implement wake up pin

			break;
	case(5): //Always locate mode: advanced adaptive power periodic mode
			uint8_t cmd5[] = "$PMKT225,8*23";//Always locate standby
			gps_command(cmd5,"$PMKT001,225,3");
			break;
	}

}

uint8_t get_checksum(char *cmd){ // determines checksum of command and returns .
	uint8_t checksum = 0;
	for(int j = 0; j < strlen((char *)cmd)+1; j++){
		if(cmd[j] == '*' ){
			return(checksum);
		}
		else if(cmd[j]!= '$'){
			checksum ^= cmd[j];
		}
	}
	return(0);
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
