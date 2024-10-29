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
#include "gsm.h"
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

//GPS Variables//************************
//internal
char RX_BufferGPS[RX_BUFFER_SIZE_GPS];
uint8_t gps_sleepFlag = 0;
uint8_t GPS_encode = 0;
uint32_t GPS_timer= 0;
uint8_t GPS_StartUP = 0;

//external
extern  bool _is_gps_data_good;
//***************************************

//GSM Variables//************************
//internal
uint8_t gsm_sleepFlag = 0;
uint8_t gsm_sleepReturnFlag = 0;
char phone[20];
extern char RX_Buffer_GSM[RX_BUFFER_SIZE_GSM];
uint8_t gsm_TimerCounter = 0;
//external
extern uint8_t newMessageFlag;

//***************************************

//General Variables//********************
uint8_t LowBatFlag = 0;
uint8_t sendCoordinateFlag = 0;
uint8_t currentModeState = Mode3;
uint8_t newModeFlag = 0;
//Variables for Mode 1
float geoFenceLat = 0;
float geoFenceLong = 0;
float geoFenceSize = 100;
//Variables for Mode 2
uint32_t updatePeriod = 10*60*1000; //10 minutes
//***************************************

//Debug Variables//**********************
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint32_t messageTIM = 0;
//uint32_t TTFF = 0;

//***************************************

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
//GPS control functions//*****************************************
void gps_processNMEA(char *buffer,uint16_t length);
void gps_init();
int gps_command(uint8_t *cmd, char *response);
uint8_t get_checksum(char *cmd);
void gps_updateMode(uint8_t mode,uint16_t on, uint16_t off);
void mode_update(uint8_t newMode, float param1, float param2, float param3);
//******************************************************

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

	if(huart->Instance == USART1){
		//char output[40];
		char* result = {0};
		char* coordinate = {0};
		result = strstr((char *)RX_BufferGPS,"PMTK");
		coordinate = strstr((char *)RX_BufferGPS,"RMC");
		if(((result == NULL)&&(coordinate != NULL))||(GPS_StartUP != 1)){ //Prevents command responses from being NMEA processed
			for(int i = 0; i < Size; i++ ){
				if(RX_BufferGPS[i] == '\n'){
					//if(TTFF_Flag == 1){
					//TTFF = HAL_GetTick() - TTFF;
					//TTFF = TTFF/1000;
					//snprintf(output,40,"Time=%u\r\n",(uint16_t)TTFF);
					//snprintf(output, i+2 ,"%s\r\n",RX_BufferGPS);//print gps incoming message
					//CDC_Transmit_FS((uint8_t*) output, strlen(output));
					//}
					gps_processNMEA(RX_BufferGPS, i+1);
					memset(RX_BufferGPS,0,RX_BUFFER_SIZE_GPS);
				}
			}
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t *) RX_BufferGPS, RX_BUFFER_SIZE_GPS);
	}

	if(huart->Instance == USART2){

			char *result = {0};
			result =strstr(RX_Buffer_GSM,"+CMTI:"); //strstr function searches for 1st occurrence of the substring in the main string

			if(result != NULL){ //check for a new message notification
			 newMessageFlag = 1;
			 messageTIM = HAL_GetTick();
			}
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t*)RX_Buffer_GSM, RX_BUFFER_SIZE_GSM);
		}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){ //GPS 10 minute status check


	if(newMessageFlag == 0){

	if(htim->Instance == TIM2){//GPS maintenance
		char output[] = "GPS Status Check:\r\n";
		CDC_Transmit_FS((uint8_t *)output, strlen(output));
			  if(gps_sleepFlag == 1 ){
				  gps_updateMode(1,0,0); //Message wakes up the gps module
				  HAL_Delay(50);
			  }

			  if (gps_command((uint8_t *)gps_Test, "PMTK001,0,3") == GpsError){//Double error check
				  if(gps_command((uint8_t *)gps_Test,"PMTK001,0,3") != Gpsok){
				  				  gps_init(); //reset and initialize
				  			  }
			  }

			  if(gps_sleepFlag == 1){ //return to sleep mode
				  gps_updateMode(3,10000,30000);
			  }
	}

	if(htim->Instance == TIM3){//GSM maintenance

		if(gsm_TimerCounter == 4){ //Set to be called every 2.5 minutes
			gsm_TimerCounter = 0;
		char output[] = "GSM Status Check:\r\n";
		CDC_Transmit_FS((uint8_t *)output, strlen(output));

			if(gsm_sleepFlag == 1){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);//Wake up SIM800C
				gsm_sleepFlag = 0;
				gsm_sleepReturnFlag = 1;
				HAL_Delay(250);
			}

			if(gsm_init() == GsmError){//Check if startup failed then therefore device is off (Evaluates proper running and if not resets device)
				gsm_init(); //turn the device back on
			}

			if(gsm_batteryCheck() == 0){
				//gsm_sendSMS("Battery Low");
				//mode_update(Mode4,0,0,0);
				LowBatFlag = 1; // Sets system to mode 4
			}

			if((gsm_sleepFlag == 0)&&(gsm_sleepReturnFlag == 1)){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);//Sleep GSM
			}
		}
		else{
			gsm_TimerCounter++;
		}

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);

  gsm_init();
  HAL_TIM_Base_Start_IT(&htim3); //GSM check tim


  HAL_Delay(500);

  GPS_StartUP = 1;
  gps_init();
  HAL_TIM_Base_Start_IT(&htim2);//GPS check timer
  GPS_StartUP = 0;

  HAL_Delay(500);

  GPS_timer= HAL_GetTick();

  //gsm_sendSMS("Hello World");
  snprintf(phone,strlen(MY_PHONE)+1,"%s",MY_PHONE);

  float latitude = 0;
  float longitude = 0;
  unsigned long fixAgeGPS = 0;

  uint32_t mode2Timer = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //GPS Data code//****************************************************************
		  if(GPS_encode == 1){
			  if(_is_gps_data_good){ // update current coordinates if data is good
			  	  gps_f_get_position(&latitude, &longitude,&fixAgeGPS );
			  	  //char output[50];
			  	  //snprintf(output, sizeof(output),"Latitude: %.6f, Longitude: %.6f \r\n",latitude,longitude);
		 		  //CDC_Transmit_FS((uint8_t *)output, strlen(output));
			  	  GPS_timer= HAL_GetTick();
		 		  GPS_encode = 0;
		 	  }
		  else{
			  //char output[25];
			  //snprintf(output, sizeof(output),"GPS data is not good\r\n");
			  //CDC_Transmit_FS((uint8_t *)output, strlen(output));
			  GPS_encode = 0;
		  	  }

		  }
		  //**************************************************************************

		  //MISC//********************************************************************
		  if(newMessageFlag == 1){ //Handle new message received
			  if(gsm_sleepFlag == 1){//Ensure module is awake
			  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);//Wake up SIM800C
			  		HAL_Delay(250);
			  		gsm_sleepFlag = 0;
			  		gsm_sleepReturnFlag = 1;
			  		}

			  gsm_readSMS();
			  gsm_sendCommand(ClearMemory,"OK\r\n");

			  if(gsm_sleepReturnFlag == 1){
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);//SIm800C Sleep
				  gsm_sleepFlag = 1;
				  gsm_sleepReturnFlag = 0;
			  }

			  newMessageFlag = 0;
		  }

		  if(sendCoordinateFlag == 1){ //Send location pin to user (uses most recent 'good' location data.
			  	  gsm_sendLocation(latitude, longitude);
			  	  CDC_Transmit_FS((uint8_t *)Location_Sent,strlen(Location_Sent) );
		  		  sendCoordinateFlag = 0;
		  	  }

		  if(LowBatFlag == 1){
			  currentModeState = Mode4;
			  newModeFlag = 1;
		  }
		  //**************************************************************************


		  //State Machine code//******************************************************
		   switch(currentModeState){

		   	   case(Mode1):

				 if(newModeFlag == 1){//Set new mode parameters
					 gsm_sendSMS("Mode 1 Set");
				   	 gsm_sendCommand("AT+CSCLK=1\r\n","OK\r\n");
				   	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);//set sleep mode
				   	 gps_updateMode(3,1000,3000); //Period
					 CDC_Transmit_FS((uint8_t*)"GPS IDLE & GSM SLEEP SET\r\n",26);
				   	 gsm_sleepFlag = 1;
				   	 gps_sleepFlag = 1;
				   	 newModeFlag = 0;
				 }

				if(((gps_distance_between(latitude, longitude, geoFenceLat, geoFenceLong) - geoFenceSize) > 0) && ((longitude != 0) || (latitude != 0))){
					 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);//Wake up SIM800C
					 HAL_Delay(250);
					 gsm_sleepFlag = 0;
					 gsm_sendSMS("GEO-Fence breached!\r\n");
					 gsm_sendLocation(latitude, longitude);
					 mode_update(Mode2,5,0,0);
				 }
		   	   	   //currentModeState = Mode1;

		   	   	 break;

		   	   case(Mode2):
					if((HAL_GetTick() - mode2Timer) > updatePeriod){
						if(gsm_sleepFlag == 1){
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);//Wake up SIM800C
							gsm_sleepFlag = 0;
							gsm_sleepReturnFlag = 1;
							HAL_Delay(100);
						}

						gsm_sendLocation(latitude, longitude);
						mode2Timer = HAL_GetTick();

			    if(gsm_sleepReturnFlag == 1){
			    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);//set sleep SIM800C
			    	gsm_sleepFlag = 1;
					gsm_sleepReturnFlag = 1;
			    	}
					}

		   	   if(newModeFlag == 1){//Set new mode parameters
		   		   gsm_sendSMS("Mode 2 Set");
		   		   gsm_sendCommand("AT+CSCLK=1\r\n","OK\r\n");
		   		   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);//set sleep mode
		   		   gps_updateMode(3,1000,3000); //Period
		   		   CDC_Transmit_FS((uint8_t*)"GPS IDLE & GSM SLEEP SET\r\n",26);
		   		   gsm_sleepFlag = 1;
		   		   gps_sleepFlag = 1;
		   		   newModeFlag = 0;
		   	   }
		   	   //currentModeState = Mode2;

		   	   break;

		   	   case(Mode3):
	  			  currentModeState = Mode3;

		   	   if(newModeFlag == 1){
		   		   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);//turn off sleep mode
		   		   HAL_Delay(250);
		   		   gsm_sendCommand("AT+CSCLK=0","OK");
		   		   HAL_Delay(100);
		   		   gsm_sendSMS("Mode 3 Set");
		   		   gps_updateMode(1,0,0); //Period
		   		   HAL_Delay(100);
		   		   newModeFlag = 0;
		   		   gsm_sleepFlag = 0;
		   		   gps_sleepFlag = 0;
		   	   }

		   	   break;

		   	   case(Mode4)://Low power mode GPS remains in standby and GSM stays in sleep unless message is received
					currentModeState = Mode4;

		   	   if(newModeFlag == 1){
		   		   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);//Wake up SIM800C
		   		   HAL_Delay(250);
		   		   gsm_sendSMS("Mode 4 Set");
		   		   gps_updateMode(2,0,0);
		   		   gsm_sendCommand("AT+CSCLK=1\r\n","OK\r\n");
		   		   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);//turn on sleep mode
		   		   newModeFlag = 0;
		   		   gsm_sleepFlag = 1;
		   		   gps_sleepFlag = 1;
		   	   }

		   }

		  //**************************************************************************

		      if(UserRxBufferFS[0] == '1'){
		    	  gps_command((uint8_t *) gps_Test, "PMTK001,0,3");
		    	  gps_sleepFlag = 0;
		 		  memset(UserRxBufferFS,'\0',strlen((char *)UserRxBufferFS));
		 	  }

		 	  else if(UserRxBufferFS[0] == '2'){
		 		 gps_updateMode(3,10000,30000);
		 		 gps_sleepFlag = 1;
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
		 		  	gps_sleepFlag = 1;
		 		  memset(UserRxBufferFS,'\0',strlen((char *)UserRxBufferFS));
		 	  }

		 	 else if(UserRxBufferFS[0] == '5'){
		 		 	 gps_updateMode(3,15000,30000);
		 			 memset(UserRxBufferFS,'\0',strlen((char *)UserRxBufferFS));
		 			 }

		 	else if(UserRxBufferFS[0] == '6'){
		 		char url[100];
		 				gsm_sendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n","OK\r\n");
		 				gsm_sendCommand("AT+SAPBR=3,1,\"APN\",\"CMNET\"\r\n","OK\r\n");
		 				gsm_sendCommand("AT+SAPBR=1,1\r\n","OK\r\n");
		 				if (gsm_sendCommand("AT+SAPBR=2,1\r\n","OK\r\n") == Gsmok){ //Network connection secured?
		 				gsm_sendCommand("AT+CLBSCFG=0,1\r\n","OK\r\n");
		 				gsm_sendCommand("AT+CLBSCFG=0,2\r\n","OK\r\n");
		 				gsm_sendCommand("AT+CLBSCFG=1,3,\"lbs-simcom.com:3002\"\r\n","OK\r\n");
		 				HAL_Delay(100);

		 				if (gsm_sendCommand("AT+CLBS=1,1\r\n","OK\r\n") == Gsmok){ //Location returned?
		 				//char *token = NULL;
		 					char *messageCopy = NULL;
		 					size_t length = strlen(RX_Buffer_GSM);
		 					char *token1 = NULL;
		 					char *token2 = NULL;


		 					messageCopy = (char *)malloc(length);
		 					strncpy(messageCopy, RX_Buffer_GSM, length);
		 					messageCopy[length + 1 ] = '\0';
		 					memset(RX_Buffer_GSM,'\0', length); //clear
		 					token1 = strtok(messageCopy, ","); //function splits up the string.
		 					token1 = strtok(NULL, ","); //longitude
		 					token2 = strtok(NULL, ","); //latitude
		 					free(messageCopy);
		 					snprintf(url, sizeof(url),"LBS: Latitude:%s, Longitude:%s",token2,token1);
		 					CDC_Transmit_FS((uint8_t*) url, strlen(url));
		 				}

		 				else{
		 					//snprintf(url, sizeof(url),"No Location Available"); //Send if Network location fails

		 					}
		 				//gsm_sendSMS(url);
		 			}
		 				gsm_sendCommand("AT+SAPBR=0,1\r\n","OK\r\n");

		 			 	memset(UserRxBufferFS,'\0',strlen((char *)UserRxBufferFS));
		 			 }
		 	else if(UserRxBufferFS[0] == '7'){
		 		 	 gsm_sendCommand(test,"OK\r\n");
		 		 	 memset(UserRxBufferFS,'\0',strlen((char *)UserRxBufferFS));
		 	}

		 	else if(UserRxBufferFS[0] == '8'){
		 			gsm_sendSMS("Hello World");
		 			memset(UserRxBufferFS,'\0',strlen((char *)UserRxBufferFS));
		 	}

		 	else if(UserRxBufferFS[0] == '9'){
		 			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
		 			HAL_Delay(2000);
		 			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 249;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000000;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 60000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 62500;
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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DTR_GPIO_Port, DTR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GSM_PWX_GPIO_Port, GSM_PWX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : User_Button___KEY_Pin */
  GPIO_InitStruct.Pin = User_Button___KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(User_Button___KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DTR_Pin GPS_RST_Pin */
  GPIO_InitStruct.Pin = DTR_Pin|GPS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
	//TTFF = HAL_GetTick();

	//gps_command((uint8_t *) gps_FullPower, "PMTK001,225,0,3");//Returns a different message

	HAL_Delay(200);

	uint8_t baudRateGPS[] = "$PMTK251,115200*1F\r\n"; //set UART baud rate to 9600
	gps_command(baudRateGPS,"PMTK001,251,3");

	HAL_Delay(200);

	//set update position report to every 10 seconds (works)
	gps_command((uint8_t *) LOW_UPDATE_RATE, "PMTK001,220,3");

	HAL_Delay(200);

	uint8_t rmcCommand[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; //enable RMC mode only
	gps_command(rmcCommand, "PMTK001,314,3");

}


int gps_command(uint8_t *cmd, char *response) //Send Commands to GPS and check response
{
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *) cmd, strlen((char *)cmd));

    CDC_Transmit_FS((uint8_t *)cmd,  strlen((char *)cmd));

    char *P = NULL;
    char *Failed = NULL;
    uint32_t GPS_timer2 = HAL_GetTick();

    while(*P != response[0]){
    	P = strstr(RX_BufferGPS, response);
    	Failed = strstr(RX_BufferGPS, ",2*");
    	Failed = strstr(RX_BufferGPS, ",1*");
    	Failed = strstr(RX_BufferGPS, ",0*");


    	if((HAL_GetTick() - GPS_timer2) > 2000){ //2s time-out error
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
	        gps_command((uint8_t *)gps_Test,"PMTK001,0,3");
			gps_command((uint8_t *)gps_FullPower, "PMTK001,220,3");
	        HAL_Delay(1000);
			gps_command((uint8_t *)LOW_UPDATE_RATE, "PMTK001,220,3");
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
			gps_command((uint8_t *)HIGH_UPDATE_RATE,"PMTK001,220,3");
			gps_command((uint8_t *)finalcmd3,"PMTK001,225,3");
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


void mode_update(uint8_t newMode, float param1, float param2, float param3){

	if(newMode == Mode1){ //update Mode parameters
		geoFenceLat = param1;
		geoFenceLong = param2;
		geoFenceSize = param3 + 10; //added tolerance in m

		if(currentModeState != Mode1){//If not currently in mode switch to new mode.
			currentModeState = Mode1;
			newModeFlag = 1;
		}
	}

	if(newMode == Mode2){
		updatePeriod = (uint32_t)param1*60*1000;//Set to minutes
		if(updatePeriod == 0){
			updatePeriod = 10;
		}

		if(currentModeState != Mode2){
			currentModeState = Mode2;
			newModeFlag = 1;
		}
	}

	if(newMode == Mode3){

		if(currentModeState != Mode3){
			currentModeState = Mode3;
			newModeFlag = 1;
		}

	}

	if(newMode == Mode4){
		if(currentModeState != Mode4){
			currentModeState = Mode4;
			newModeFlag = 1;
			}
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
