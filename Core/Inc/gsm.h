/*
 * gms.h
 *
 *  Created on: Sep 10, 2024
 *      Author: djabe
 */

#ifndef INC_GSM_H_
#define INC_GSM_H_

#include "stm32f4xx_hal.h"

//General AT commands
#define  test           "AT\r\n"
#define  NetworkTest	"AT+CREG=?\r\n"
#define  EnableMessageNotification "AT+CNMI=2,1,0,0,0\r\n" // Buffer results and send when available, send memory location and index
#define  setBaudRate "AT+IPR=115200\r\n"
#define	 keepSetting "AT&W\r\n"
#define  ClearMemory "AT+CMGD=1,2\r\n"

//IP Network AT commands
#define  SetAPN         "AT+SAPBR=3,1,\"APN\",\"www.safaricom.com\"\r\n"
#define  ActGPRS        "AT+SAPBR =1,1\r\n"
#define  GetIP         "AT+SAPBR=2,1\r\n"

//message AT commands
#define SmsTextMode "AT+CMGF=1\r\n"
#define SmsAlphabet "AT+CSCS=\"GSM\"\r\n" //set GSM vocabulary

#define RX_BUFFER_SIZE_GSM 500

typedef enum{
	Gsmok=0,
	GsmError,
	GsmReady,
	GsmMessage
}GsmStatus;

int gsm_init();
int gsm_sendCommand(char *Command,char *response);
//int gms_setInternetConnectivity(void);
int gsm_sendSMS(char *Message);
//int gms_HttpRequest(char *UrlLink,GpsInfo * Gps);
int gsm_batteryCheck();
int gsm_getIndex();
void gsm_readSMS();
void gsm_sendLocation(float lat, float longa);
//char* extractContent(const char *message);
void processMode(char *messageContents);
#endif /* SRC_SIM900_H_ */
