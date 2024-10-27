/*
 * gsm.c
 *
 *  Created on: Sep 10, 2024
 *      Author: djabe
 *
 *      Some code borrowed from  https://github.com/ENG-EDISON/STM32_Tracker/blob/main/Core/Src/SIM900.c#L58
 *
 *      Phone number of module: 077 360 4180
 *
 */
//includes
#include "gsm.h"
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

//
extern void mode_update(uint8_t newMode, float param1, float param2, float param3);
extern void gps_f_get_position(float *latitude, float *longitude, unsigned long *fix_age);

//Internal Variables//********************
uint32_t timer1 = 0;
uint32_t timer2 = 0;
uint8_t readMessageFlag = 0;
uint8_t sendMessageFlag = 0;
uint8_t newMessageFlag = 0;
char RX_Buffer_GSM[RX_BUFFER_SIZE_GSM];
//****************************************

//External Variables//********************
extern char RX_Buffer_GSM[RX_BUFFER_SIZE_GSM];
extern char *phone;
extern uint8_t gsm_sleepFlag;
extern UART_HandleTypeDef huart2;
extern uint8_t sendCoordinateFlag;
extern uint32_t GPS_timer;
//****************************************


int gsm_init(){
	//Can setup power pin to switch on the sim800
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t*)RX_Buffer_GSM, RX_BUFFER_SIZE_GSM);
	HAL_Delay(100);

	if((gsm_sendCommand(test, "OK\r\n"))==GsmError){ //This switches the sim800 ON if it is off (Also used as a reset)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
	HAL_Delay(2000);

	if(gsm_sendCommand(test, "OK\r\n")==GsmError){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
			HAL_Delay(2000);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
			HAL_Delay(2000);
	}
		gsm_sendCommand("AT+CSCLK=0","OK");
		gsm_sendCommand(setBaudRate, "OK\r\n");
		gsm_sendCommand(EnableMessageNotification, "OK\r\n" ); //enable SMS message notifications
		gsm_sendCommand(SmsAlphabet,"OK\r\n");
		gsm_sendCommand(keepSetting, "OK\r\n");
		gsm_sendCommand(NetworkTest, "OK\r\n"); //check that messages can be sent an received on the network
		return Gsmok;
	}

	else{
	gsm_sendCommand("AT+CSCLK=0","OK");
	gsm_sendCommand(setBaudRate, "OK\r\n");
	gsm_sendCommand(EnableMessageNotification, "OK\r\n" ); //enable SMS message notifications
	gsm_sendCommand(SmsAlphabet,"OK\r\n");
	gsm_sendCommand(keepSetting, "OK\r\n");
	gsm_sendCommand(NetworkTest, "OK\r\n"); //check that messages can be sent an received on the network
	return Gsmok;
	}
}


//borrowed and modified code: https://github.com/ENG-EDISON/STM32_Tracker/blob/main/Core/Src/SIM900.c#L58
int gsm_sendCommand(char *Command,char *response) //Sends command via UART and checks response
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t*)RX_Buffer_GSM, RX_BUFFER_SIZE_GSM);

	char *P= NULL;
	char *err= NULL;
	char *GsmRead= NULL;
	char *NewMsg= NULL;
	char *Batt = NULL;
	char *GPRS = NULL;

	HAL_UART_Transmit_DMA(&huart2,(uint8_t*)Command,strlen(Command)); //transmit command

	timer2 = HAL_GetTick();
	while(*P!=response[0])	//Waits 7 second for response therefor delays are not needed
	{
		P=strstr(RX_Buffer_GSM,response); //What is the message returned type?
		err=strstr(RX_Buffer_GSM,"ERROR\r\n");
		NewMsg=strstr(RX_Buffer_GSM,"+CMGR:");
		Batt=strstr(RX_Buffer_GSM,"+CBC:");
		GsmRead=strstr(RX_Buffer_GSM,"PSUTTZ");
		GPRS = strstr(RX_Buffer_GSM,"+CLBS:");

		if(*err=='E'){ //SIM800 error
			if(newMessageFlag == 0){
				memset(RX_Buffer_GSM,'\0',strlen(RX_Buffer_GSM));
			}
		    CDC_Transmit_FS((uint8_t *)"Error\r\n", 8); //debug terminal
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t*)RX_Buffer_GSM, RX_BUFFER_SIZE_GSM);
			return GsmError;
		}

		if(HAL_GetTick() - timer2 > 7000){ //7s time-out error
			if(newMessageFlag == 0){
			memset(RX_Buffer_GSM,'\0',strlen(RX_Buffer_GSM));
			}
			CDC_Transmit_FS((uint8_t *)"Time Out Error\r\n", 15); //debug terminal
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t*)RX_Buffer_GSM, RX_BUFFER_SIZE_GSM);
			return GsmError;
		}

		if(*GsmRead=='P')
				{
				HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t*)RX_Buffer_GSM, RX_BUFFER_SIZE_GSM);
				return GsmReady;
				}
	}

	if((readMessageFlag == 1)&&(*NewMsg == '+')){
		readMessageFlag = 0;
		return GsmMessage;
	}

	else if(*Batt == '+'){
		CDC_Transmit_FS((uint8_t *)RX_Buffer_GSM,  strlen(RX_Buffer_GSM));
		return Gsmok;
	}

	else if(*GPRS == '+'){
		return Gsmok;
	}

	else{
		CDC_Transmit_FS((uint8_t *)RX_Buffer_GSM,  strlen(RX_Buffer_GSM)); //debug terminal
		if(newMessageFlag == 0){
			memset(RX_Buffer_GSM,'\0',strlen(RX_Buffer_GSM));
			} //clear buffer
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t*)RX_Buffer_GSM, RX_BUFFER_SIZE_GSM);
		return Gsmok;
	}
}


int gsm_sendSMS(char *Message){ //Send an SMS to a chosen phone number
    char SetPara[100] = {0};

    snprintf(SetPara, sizeof(SetPara), "AT+CMGS=\"%s\"\r\n", phone);
    gsm_sendCommand(SmsTextMode, "OK\r\n");
    gsm_sendCommand(SetPara, ">");  // Start SMS sending
    sendMessageFlag = 1;
    gsm_sendCommand(Message, ">");  // Send the SMS message content

    char ch[1] = {0};
    int n = 26;
    ch[0] = n;
    ch[1] = '\r';

    gsm_sendCommand(ch, "OK\r\n"); // send <CTRL + Z>
    return Gsmok;
}


int gsm_getIndex() //get the index of the new stored sms message
	{
	char *cmti = strstr(RX_Buffer_GSM, "+CMTI: \"SM\",");
	int sms_index = atoi(cmti + strlen("+CMTI: \"SM\","));
	memset(RX_Buffer_GSM,'\0',strlen(RX_Buffer_GSM)); //clear buffer
	readMessageFlag = 1;
	return(sms_index);
}


void gsm_readSMS() //extract and copy message into a buffer -> Categories message
{
	char readCommand[20] = {0};
	int smsIndex = 0;
	smsIndex = gsm_getIndex(); //get the messages storage index

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t*)RX_Buffer_GSM, RX_BUFFER_SIZE_GSM);

	sprintf(readCommand, "AT+CMGR=%d\r\n", smsIndex); //read the message stored at the index

	if(gsm_sendCommand(readCommand, "OK\r\n" ) == GsmMessage){			//Extract and copy message contents

	char *messageCopy = NULL;
	char *messageStart = NULL;
	char *messageEnd = NULL;

	size_t length = 0;

	messageStart = strchr(RX_Buffer_GSM, '$') + 1;

	if(messageStart != NULL){
		messageEnd = strchr(messageStart, '#');

		if((messageEnd != NULL)&&(messageEnd > messageStart)){
			length = messageEnd - messageStart;
			messageCopy = (char *)malloc(length + 1);
			strncpy(messageCopy, messageStart, length);
			messageCopy[length] = '\0';
		}
	}
	newMessageFlag = 0;
	memset(RX_Buffer_GSM,'\0', strlen(RX_Buffer_GSM)); //clear buff
	CDC_Transmit_FS((uint8_t*) messageCopy, strlen(messageCopy));

	char *mode = NULL;
	char *req = NULL;
	char *number = NULL;

	mode = strstr(messageCopy, "MD");
	req = strstr(messageCopy, "REQ");
	number = strstr(messageCopy, "NN");

	if(mode != NULL){
		processMode(messageCopy);
	}

	else if(req != NULL){
		 sendCoordinateFlag = 1;
	}

	else if(number != NULL){
		char *code= NULL;

		code = strstr(messageCopy, "1234");

			if(code != NULL){ //Check for correct Code
				char *token;
				token = strtok(RX_Buffer_GSM, ","); //function splits up the string.
				token = strtok(NULL, ",");
				token = strtok(NULL, ",");
				*phone = *token;
				gsm_sendSMS("Phone number updated"); //Send an SMS to a chosen phone number
			}
	}
	else{
		gsm_sendSMS("Command not recognized!");
	}


	free(messageCopy);
		}
}

void processMode(char *messageContents){
	char *modeSet = NULL;
	uint8_t term = 0;

	char *gsmParam1 = NULL;
	char *gsmParam2 = NULL;
	char *gsmParam3 = NULL;

	float gsmParam1_int;
	float gsmParam2_int;
	float gsmParam3_int;

	uint8_t newModeState = 3;

	char *token;

	token = strtok(messageContents, ","); //function splits up the string.
	token = strtok(NULL, ",");

	while(token != NULL){

		if(term == 0){
			modeSet = token;
			term++;
		}

		else if(term == 1){
				gsmParam1 = token;
				term++;
		}

		else if(term == 2){
				gsmParam1 = token;
				term++;
		}

		else if(term == 3){
				gsmParam1 = token;
				term++;
		}
		token = strtok(NULL, ",");
	}


	newModeState = modeSet[0] - '0';
	char modeChange[20];
	snprintf(modeChange, 20 ,"Mode %d set\r\n", newModeState);
	CDC_Transmit_FS((uint8_t *) modeChange, strlen(modeChange));
					//read and update variables for different mode states.

	if(newModeState == 1){//Geo-fence Mode
		newModeState = Mode1;
		char *coordSetting = {0};
		coordSetting = strstr(gsmParam1, "HERE");

		if(coordSetting != NULL){ //Check for device set co-ordinates.
			unsigned long fixAgeGPS;

			gps_f_get_position(&gsmParam1_int,&gsmParam2_int,&fixAgeGPS);
			gsmParam3_int = strtol(gsmParam3, NULL, 10);//radius
		}

		else{//Manually set coordinates

			if((strlen(gsmParam1) > 8)&&(strlen(gsmParam1) < 11)){
				gsmParam1_int = strtol(gsmParam1, NULL, 10);//latitude
			}
			else{
				gsmParam1_int = 0;
			}

			if((strlen(gsmParam2)>8)&&(strlen(gsmParam2) < 11)){
				gsmParam2_int = strtol(gsmParam2, NULL, 10);//longitude
			}
			else{
				gsmParam2_int = 0;
			}
			gsmParam3_int = strtol(gsmParam3, NULL, 10);//radius
		}
	}

	else if(newModeState == 2){//Period update Mode
			newModeState = Mode2;
			gsmParam1_int = strtol(gsmParam1, NULL, 10);//Time Period
			gsmParam2_int = 0;
			gsmParam3_int = 0;
	}

	else if(newModeState == 4){//Sleep mode
			newModeState = Mode4;

	}

	else {//Full power mode
		newModeState = Mode3;
	}

		mode_update(newModeState,gsmParam1_int,gsmParam2_int,gsmParam3_int);
}


void gsm_sendLocation(float lat, float longa){
	char url[100];
	if((lat==0 || longa == 0)||(HAL_GetTick() - GPS_timer) > 180000) { //only if lat or long not corrcet or more than 30 min passed without good gps data

		char *token1 = NULL;
		char *token2 = NULL;

		gsm_sendSMS("Bad GPS fix!");
		gsm_sendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n","OK\r\n");
		gsm_sendCommand("AT+SAPBR=3,1,\"APN\",\"internet\"\r\n","OK\r\n");
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
			messageCopy = (char *)malloc(length);
			strncpy(messageCopy, RX_Buffer_GSM, length);
			messageCopy[length + 1 ] = '\0';
			memset(RX_Buffer_GSM,'\0', length); //clear
			token1 = strtok(messageCopy, ","); //function splits up the string.
			token1 = strtok(NULL, ","); //longitude
			token2 = strtok(NULL, ","); //latitude
			free(messageCopy);
			snprintf(url, sizeof(url),"Estimated GSM Location: https://www.google.com/maps?q=%s,%s\r\n",token2,token1);
		}

		else{
			snprintf(url, sizeof(url),"No Location Available"); //Send if Network location fails
			}
		gsm_sendCommand("AT+SAPBR=0,1\r\n","OK\r\n");
	}

	else{
		snprintf(url, sizeof(url),"No Location Available"); //Send if Network location fails
		}
	}

	else{
	snprintf(url, sizeof(url),"https://www.google.com/maps?q=%f,%f\r\n",lat,longa);
	}

	gsm_sendSMS(url);
}




int gsm_batteryCheck(){//check battery level (return = 0 if the battery is critically low)
	uint16_t batVoltage = 4000;

	if(gsm_sendCommand("AT+CBC\r\n", "OK\r\n") == Gsmok){ //return current voltage and battery life level
		char *messageCopy = NULL;
		size_t length = strlen(RX_Buffer_GSM);

		messageCopy = (char *)malloc(length);
		strncpy(messageCopy, RX_Buffer_GSM, length);
		messageCopy[length + 1 ] = '\0';
		memset(RX_Buffer_GSM,'\0', length); //clear buffer

		char* token = NULL;

		token = strtok(messageCopy, ","); //function splits up the string.
		token = strtok(NULL, ",");
		token = strtok(NULL, ",");
		token[4] = '\0';
		batVoltage = (token[0] - '0')*1000 + (token[1] - '0')*100 + (token[2] - '0')*10 + (token[3] - '0');

		free(messageCopy);

		if(batVoltage >= 3500 ){
			return 1;
		}
		else if(batVoltage < 3500){
			return 0;
		}
	}
	return 1;
}

