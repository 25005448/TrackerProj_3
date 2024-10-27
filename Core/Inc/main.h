/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef enum{
	Gpsok=0,
	GpsError,
	GpsReady
}GpsStatus;

enum{ //Mode state of system
	Mode1 =1,//Geo-fence
	Mode2,   //Timed update mode
	Mode3,	//On Request
	Mode4 //Standby
};
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define User_Button___KEY_Pin GPIO_PIN_0
#define User_Button___KEY_GPIO_Port GPIOA
#define DTR_Pin GPIO_PIN_4
#define DTR_GPIO_Port GPIOA
#define GPS_RST_Pin GPIO_PIN_8
#define GPS_RST_GPIO_Port GPIOA
#define GSM_PWX_Pin GPIO_PIN_9
#define GSM_PWX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//GPS defines
#define RX_BUFFER_SIZE_GPS 200
#define gps_Test "$PMTK000*32\r\n"
#define HIGH_UPDATE_RATE "$PMTK220,2000*1C\r\n"
#define VERYHIGH_UPDATE_RATE "$PMTK220,1000*1F\r\n"
#define LOW_UPDATE_RATE "$PMTK220,10000*2F\r\n"
#define gps_FullPower "$PMTK225,0*2B\r\n"
//

//GSM defines


//





/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
