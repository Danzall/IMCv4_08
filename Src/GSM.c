/*
 * GSM.c
 *
 *  Created on: Apr 24, 2019
 *      Author: Jeefo
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
//#include "cmsis_os.h"
#include "string.h"
#include "GSM.h"
#include "ADE7953.h"
#include "myString.h"

uint16_t onTimer = 0;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

State gsmState = GSM_Off;	//restart
SMSflags smsFlags;
SMSinfo smsInfo;
GSMinfo gsmInfo;

#define recBuffSize 300
#define procBuffSize 230
char recBuff[recBuffSize];
char procBuff[procBuffSize];
int recBuffPointerWrite = 0;
int recBuffPointerRead = 0;
int procBuffpointer = 0;
char temp[100];
char temp1[20];

//Alarm alarm;
char SMScontent[160];
char GPRSresult[6];
char tempGPRS[40];
char quality[10];
int gsmTimer;
int smsTimer;
int gprsTimer;
int errorTimer = 0;
int errorCounter = 0;
int restart = 1;
int restartTimer = 0;
int tempState = 0;
//extern uint16_t adcResult;
int vthTimer = 0;
int v1 = 0;
int v2 = 0;
int gprs = 0;
char v1s[6];
char v2s[6];
char dummy[50];
extern volt;
int voltage;
int current1;

void pec_Update(char* pec, char index)
{
	char index1;
	static const  char lookup[256][3] =		//256 rows each having 3 columns
	{
	"21", "22", "23", "24", "25", "26", "27", "28",
	"29", "2A", "2B", "2C", "2D", "2E", "2F",
	"30", "31", "32", "33", "34", "35", "36", "37",
	"38", "39", "3A", "3B", "3C", "3D", "3E", "3F",
	"40", "41", "42", "43", "44", "45", "46", "47",
	"48", "49", "4A", "4B", "4C", "4D", "4E", "4F",
	"50", "51", "52", "53", "54", "55", "56", "57",
	"58", "59", "5A", "5B", "5C", "5D", "5E", "5F",
	"60", "61", "62", "63", "64", "65", "66", "67",
	"68", "69", "6A", "6B", "6C", "6D", "6E", "6F",
	"70", "71", "72", "73", "74", "75", "76", "77",
	"78", "79", "7A", "7B", "7C", "7D", "7E", "7F"

	};
	//pec = lookup[pec];
	index1 = index - 33;

	*pec = lookup[index1][0] ;
	//sendByte(* pec, UART0);
	pec++;
	*pec = lookup[index1][1] ;
	//sendByte(* pec, UART0);
	pec++;
	*pec = 0 ;
	//strcpy(pec,lookup[index1]);
	/*char temp[15];
	sprintf (temp,"\nIndex:%i\r\n", index1);
	sendData(temp,UART0);*/
	//sendData(pec,UART0);
	//sendData("\r\n",UART0);
	//return pec;
}


void GSM_Init(){
	//gsmState = AutoBaud;
	Debug_Send("GSM Init\r\n");
	gsmInfo.GPRSinterval = 300;
	strcpy(gsmInfo.urlport,"5008");
	//strcpy(gsmInfo.urlport,"80");
	strcpy(gsmInfo.url,"escorsocket.ddns.net");
	strcpy(gsmInfo.url,"196.40.108.169");
	//strcpy(gsmInfo.url,"197.94.242.65");
	//strcpy(gsmInfo.url,"102.129.99.123");
	restartTimer = 0;
	onTimer = 0;
	HAL_UART_Receive_IT(&huart2, temp1, 20);
	//SET_BIT(huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
	SET_BIT(huart2.Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
	HAL_UART_Receive_IT(&huart1, temp, 50);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
	smsFlags.gprsActive = 0;
	gsmInfo.socket = 0;
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
	gsmTimer = 0;
}

void GSM_Service(){
	//GSM_Send("AT\r");
	//sprintf(temp, "GSM state %i\r\n",gsmState);
	//Debug_Send(temp);
	char temp[30];
	int tempI;

	voltage = getVolt();
	myLongStr(voltage,temp1,10,10);
	strcat(temp,",");
	strcat(temp,temp1);
	current1 = getCurrent();
	/*myLongStr(current,temp1,10,10);
	strcat(temp,",");
	strcat(temp,temp1);*/
	//sprintf(temp, "gprs %i state %i v:%i c %i\r\n",gsmInfo.GPRStimer, gsmState, voltage, current);
	//Debug_Send(temp);
	if (gsmInfo.socket == 1) HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
	else  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
	//ClearScreenF();
	//LineSelect(0x80);
	//LCD_Print(temp);
	tempI = getVolt();
	tempI = getCurrent();
	switch(gsmState){
	case GSM_Off:
		Debug_Send("GSM off1\r\n");

		if (onTimer < 4){
			HAL_GPIO_WritePin(GSM_Enable_GPIO_Port, GSM_Enable_Pin, GPIO_PIN_RESET);
			onTimer++;
		}
		else{
			HAL_GPIO_WritePin(GSM_On_GPIO_Port, GSM_On_Pin, GPIO_PIN_RESET);
			gsmState = GSM_On;
			//gsmState = AutoBaud;
		}

		break;
	case GSM_On:
		Debug_Send("GSM on\r\n");
		HAL_GPIO_WritePin(GSM_Enable_GPIO_Port, GSM_Enable_Pin, GPIO_PIN_SET);
		//HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		onTimer = 0;
		gsmState = GSM_Enable;
		break;
	case GSM_Enable:
		Debug_Send("GSM enable\r\n");
		if (onTimer < 4){
			HAL_GPIO_WritePin(GSM_On_GPIO_Port, GSM_On_Pin, GPIO_PIN_SET);
			onTimer++;
			Debug_Send("GSM on set\r\n");
		}
		else{
			HAL_GPIO_WritePin(GSM_On_GPIO_Port, GSM_On_Pin, GPIO_PIN_RESET);
			gsmState = Search;
			//onTimer = 0;
			Debug_Send("GSM reset\r\n");
			//gsmState = AutoBaud;
		}
		break;
	case Search:
		//smsFlags.send = 1;	//test sms function
		//Debug_Send("Search\r\n");
		//GSM_Send("AT+CREG?\r");
		GSM_Send("AT+CREG=1\r");
		gsmState = GPRS_Search;
		break;
	case GPRS_Search:
		GSM_Send("AT+CGREG=1\r");
		gsmState = 0;
		break;
	case SMSconfig:
		Debug_Send("testing\r\n");
		GSM_Send("AT+CMGF=1\r\n");
		smsFlags.config = 1;
		smsFlags.signal = 1;
		//strcpy(gsmInfo.vth,"11.5");
		//sendData(gsmInfo.vth,UART0);
		//gsmInfo.vth_l = myStrLong(gsmInfo.vth, 10);
		//gsmInfo.vth_l *= 100;

		//gsmState = Imei;
		gsmState = Operator;
		break;
	case Operator:
		GSM_Send("AT+COPS?\r");
		gsmState = Imei;
		//gsmState = GPRS_Send;
		//gsmState = SMS_Send;
		break;
	case Imei:
		GSM_Send("AT+CGSN\r\n");
		gsmInfo.imeiFlag = 5;
		gsmState = Ccid;

		//gsmState = GPRS_Send;
		gsmState = GprsContext;
		//gsmState = 0;
		break;
	case Ccid:
		GSM_Send("AT+CCID\r\n");
		gsmState = GprsContext;
		gsmState = GPRS_Attach;
		break;
	case GprsContext:
		GSM_Send("AT+CGDCONT=1,\"IP\",\"INTERNET\"\r\n");
		//strcpy(gsmInfo.url,"www.google.com");
		//strcpy(gsmInfo.urlport,"80");
		gsmState = DataMode;

		//sendSMS("0720631005", smsInfo.build);
		//gsmState = CallerID;
		//gsmState = USSD_Config;
		//gsmState = 0;
		break;
	case DataMode:

		//GSM_Send("AT+QIMODE=0\r\n");
		//GSM_Send("AT+QIMODE?\r\n");
		//Debug_Send("Build SMS\r\n");
		BuildPower();
		sendSMS("0720631005", smsInfo.build);

		gsmState = CallerID;
		gsmState = 0;
		break;
	case CallerID:
		GSM_Send("AT+CLIP=1\r\n");
		//+CLIP: "+27733507155",145,,,"",0
		gsmState = USSD;		//test
		gsmState = SMS_Check;
		break;
	case USSD:
		if(strncmp(gsmInfo.network,"VodaCom",7)==0)GSM_Send("AT+CUSD=1,\"*100#\"\r\n");
		else if(strncmp(gsmInfo.network,"MTN",3)==0)GSM_Send("AT+CUSD=1,\"*141#\"\r\n");
		//tempState = USSD;
		gsmInfo.balanceFlag = 0;
		gsmInfo.balancePending = 1;
		gsmState = 0;

		break;

	case SMS_Check:
		GSM_Send("AT+CMGL=\"ALL\"\r\n");
		smsInfo.count = 0;
		//if (gsmInfo.registered == 1) sendData("Unit registered!!!\r\n",UART0);
		gsmState = Signal;		//check signal
		break;
	case Signal:
		GSM_Send("AT+CSQ\r\n");
		//if (gsmInfo.registered == 1) gsmState = SocketOpen;
		//if (smsFlags.gprsActive == 0) gsmState = GPRS_On;
		//else if (smsFlags.gprsActive == 1)gsmState = SocketOpen;
		//else if (smsFlags.gprsPending == 1) gsmState = GPRS_Off;		//switch off gsm if previous socket dial failed
		//else if (gsmInfo.gprsDataPending == 1) gsmInfo.gprsDataPending = 0;
		//else if (gsmInfo.gprsDataPending == 0) gsmState = SocketOpen;
		//else if (gsmInfo.registered == 1) gsmState = SocketOpen;
		/*else */gsmState = 0;
		if (gsmInfo.balanceFlag == 1) gsmState = USSD;

		//buildInfo();
		break;
	case SMS_Send:
		smsFlags.send = 1;
		Debug_Send("Send SMS\r\n");
		if (CheckUser() == 0){ //if there is no user registered
			strcpy(smsInfo.recMSISDN, "0720631005");		//return all sms to admin
		}
		else strcpy(smsInfo.recMSISDN, gsmInfo.msisdn);
		strcpy(smsInfo.recMSISDN, "0720631005");	//debug
		Debug_Send("MSISDN:");
		Debug_Send(smsInfo.recMSISDN);
		Debug_Send("\r\n");
		//sendSMS(smsInfo.recMSISDN, smsInfo.build);
		sendSMS("0720631005", smsInfo.build);
		gsmInfo.balanceFlag = 1;
		//sendSMS("0720631005", "TEST");
		gsmState = 0;
		break;
	case SMS_Text:
		//sendData("Insert SMS text\r\n",UART0);
		strcpy(temp,SMScontent);
		strcat(temp,"\032\r");
		GSM_Send(temp);
		//GSM_Send(SMScontent);
		//sendByte('1A', UART1);
		//GSM_Send("\032\r");		//1A HEX
		smsFlags.reply = 0;
		gsmState = 0;
		break;
	case GPRS_Attach:
		GSM_Send("AT+CGATT=1\r\n");
		gsmState = 0;
		gsmState = GprsContext;
		break;
	case GPRS_On:
		GSM_Send("AT+CGACT=1,1\r\n");
		smsFlags.gprsPending = 1;
		gsmInfo.socket = 0;
		//gsmState = 0;
		//gsmState = 10;
		gsmState = GPRS_IP;
		break;
	case GPRS_IP:
		GSM_Send("AT+CGPADDR=1\r\n");
		gsmState = 0;
		break;
	case GPRS_Off:
		//GSM_Send("AT+MIPCALL=0\r\n");
		GSM_Send("AT+CGACT=0,1\r\n");
		gsmInfo.socket = 0;
		smsFlags.gprsActive = 0;
		gsmInfo.gprsDataPending = 0;
		gsmState = 0;
		break;
	case SocketOpen:
		gsmState = 0;
		//sendData("AT+MIPOPEN=1,10,\"139.130.4.5\",80,0\r\n",UART1);		//connect to remote server
		//if (gsmInfo.socket == 0) sendData("AT+MIPOPEN=1,10,\"216.58.223.4\",80,0\r\n",UART1);		//connect to remote server
		//if (gsmInfo.socket == 0) sendData("AT+MIPOPEN=1,10,\"197.159.49.78\",5001,0\r\n",UART1);
		gsmInfo.gprsDataPending = 1;
		//if (gsmInfo.socket == 0) sendData("AT+MIPOPEN=1,10,\"196.40.183.39\",5001,0\r\n",UART1);
		//if (gsmInfo.socket == 0) sendData("AT+MIPOPEN=1,10,\"102.129.99.123\",5001,0\r\n",UART1);
		char temp[50];
		//strcpy(temp,"AT+MIPOPEN=1,5001,\"");
		strcpy(temp,"AT+QIOPEN=1,\"TCP\",\"");
		strcpy(temp,"AT+QIOPEN=\"TCP\",\"");
		strcat(temp,gsmInfo.url);
		strcat(temp,"\",");
		strcat(temp,gsmInfo.urlport);
		strcat (temp,"\r\n");
		//strcat (temp,",0\r\n");
		if (gsmInfo.socket == 0){
			//sendData("AT+MIPOPEN=1,5001,\"102.129.99.123\",5000,0\r\n",UART1);
			GSM_Send(temp);
		}
		else{
			//else if (gsmInfo.gprsDataPending == 1)
			gsmState = SocketClose;
			//sendData("Socket open\r\n",UART0);
		}
		//sendData("AT+MIPOPEN=?\r\n",UART1);
		//gsmState = GPRS_Write;
		//if (gsmInfo.gprsDataPending == 0) gsmState = 0;
		break;
	case SocketClose:
		//GSM_Send("AT+MIPCLOSE=1\r\n");
		GSM_Send("AT+QICLOSE\r\n");
		gsmInfo.gprsDataPending = 0;
		gsmState = 0;
		break;
	case SMS_Del:
		//smsFlags.send = 1;
		//GSM_Send("AT+CMGD=");
		//GSM_Send(smsInfo.index);
		strcpy(temp,"AT+CMGD=");
		strcat(temp,smsInfo.index);
		strcat(temp,"\r\n");
		//sendData("4",UART1);
		//GSM_Send("\r\n");
		GSM_Send(temp);
		smsFlags.del = 0;
		gsmState = 0;
		break;

	/*case SocketListen:
		sendData("AT+MIPOPEN=1,1100,\"0.0.0.0\",0,1\r\n",UART1);	//listen for incoming connections
		//sendData("AT+MIPOPEN=?\r\n",UART1);
		gsmState = 0;
		break;
	case 100:
		sendData("AT+MIPSETS=1,200\r\n",UART1);	//listen for incoming connections
		//sendData("AT+MIPOPEN=?\r\n",UART1);
		gsmState = 16;
		break;
	case SocketCheck:
		sendData("AT+MIPOPEN?\r\n",UART1);	//listen for incoming connections
		//sendData("AT+MIPOPEN=?\r\n",UART1);

		gsmState = 0;
		break;*/
	case GPRS_Write:
		gsmInfo.dataFlag = 1;
		Debug_Send("GPRS write\r\n");
		if (gsmInfo.registered == 0){
			strcpy(tempGPRS,"imei:");
			strcat(tempGPRS,(char*)gsmInfo.imei);

			int temp3;
			char temp2[20];
			temp3 = getVolt();
			myLongStr(voltage,temp2,10,10);
			strcat(tempGPRS,",v:");
			strcat(tempGPRS,temp2);
			temp3 = getCurrent();
			myLongStr(current1,temp2,10,10);
			strcat(tempGPRS,",c:");
			strcat(tempGPRS,temp2);
		}
		else {
			strcpy(tempGPRS,"user:");
			strcat(tempGPRS,(char*)gsmInfo.user);

			/*strcat(tempGPRS,",");
			char owData[7];
			char tempv[30];
			getOW_Data(owData);
			sprintf (tempv,"humidity:%d", owData[0]);
			strcat(tempGPRS,tempv);
			strcat(tempGPRS,",");
			sprintf (tempv,"temp:%d", owData[2]);
			strcat(tempGPRS,tempv);
			if (owData[3] != 0){
				sprintf (tempv,".%d", owData[3]);
				strcat(tempGPRS,tempv);
			}*/
		}
		//GPRS_SendData(tempGPRS);
		if (gsmInfo.userFlag == 1){
			gsmInfo.userFlag = 0;
			strcpy(tempGPRS,",");
			strcat(tempGPRS,"user:");
			strcat(tempGPRS,(char*)gsmInfo.user);


			//strcat(tempGPRS,",");
			//sendData(tempGPRS,UART0);
			GPRS_SendData(tempGPRS);
		}

		if (gsmInfo.msisdnFlag == 1){
			gsmInfo.msisdnFlag = 0;
			strcpy(tempGPRS,",");
			strcat(tempGPRS,"msisdn:");
			strcat(tempGPRS,(char*)gsmInfo.msisdn);
			//sendData(tempGPRS,UART0);
			GPRS_SendData(tempGPRS);
		}

		if (gsmInfo.passwordFlag == 1){
			gsmInfo.passwordFlag = 0;
			strcpy(tempGPRS,",");
			strcat(tempGPRS,"password:");
			strcat(tempGPRS,(char*)gsmInfo.password);
			//sendData(tempGPRS,UART0);
			GPRS_SendData(tempGPRS);
		}
		if (gsmInfo.relayFlag == 1){
			gsmInfo.relayFlag = 0;
			strcpy(tempGPRS,",");
			strcat(tempGPRS,"relay:");
			//if (RELAY == 1) strcat(tempGPRS,"ON");
			//else strcat(tempGPRS,"OFF");
			GPRS_SendData(tempGPRS);
		}
		if (gsmInfo.vthFlag == 1){
			gsmInfo.vthFlag = 0;
			strcpy(tempGPRS,",");
			strcat(tempGPRS,"vth:");
			strcat(tempGPRS,(char*)gsmInfo.vth);
			GPRS_SendData(tempGPRS);
		}
		if (gsmInfo.ccidFlag == 1){
			gsmInfo.ccidFlag = 0;
			strcpy(tempGPRS,",");
			strcat(tempGPRS,"ccid:");
			strcat(tempGPRS,(char*)gsmInfo.ccid);
			GPRS_SendData(tempGPRS);
		}
		strcat(tempGPRS,"\32\r");
		GSM_Send(tempGPRS);
		//GSM_Send("\032\r");
		gsmState = 0;
		/*GPRS_SendData(" ");
		GSM_Send("\"\r\n");
		gsmState = GPRS_Send;*/
		//sprintf (tempGPRS,"GPRS Write:GSM state-%d\r\n", gsmState);
		//sendData(tempGPRS,UART0);
		//sendData("-gprs\r\n",UART0);
		break;
	case GPRS_Send:
		//GSM_Send("AT+MIPPUSH=1\r\n");	//listen for incoming connections
		GSM_Send("AT+QISEND\r");
		//Debug_Send("Push GPRS data!\n");
		/*GSM_Send(tempGPRS);
		GSM_Send("\032\r");*/
		gsmInfo.dataFlag = 0;
		gsmState = 0;
		break;
	case GPRS_SendMode:
		GSM_Send("AT+ATO\r");

		break;
	case GPRS_SendModeTRans:
		GSM_Send("test\r");

		break;
	case AutoBaud:
		Debug_Send("Auto baud\r\n");
		GSM_Send("AT\r\n");

		break;
		/*case GSM_On:
		sendData("GSM power on\r\n",UART0);
		GSM_PWR = 1;
		gsmState++;
		smsFlags.restartCount++;
		break;
	case GSM_Enable:
		sendData("GSM enable\r\n",UART0);
		restartTimer++;
		if (restartTimer >= 3){
			restartTimer = 0;
			gsmState = Search;
			GSM_ON = 0;
		}
		break;*/
	default:

		break;
	}
	//Debug_Send("Done\r\n");
	//if (gsmInfo.socket == 1) gsmState = 17;		//if context is open send gprs data

	//if (smsFlags.prompt == 1){

	//	smsFlags.prompt = 0;
	//}
	//if ((gsmTimer < 20)&&(smsFlags.signal == 1)){			//restarts cycle
		gsmTimer++;
		//sendData("Incr\n",UART0);
	//}
	//else {
		//gsmTimer = 0;
		//sendData("Recy\n",UART0);
		//if (smsFlags.config == 1) gsmState = SMS_Check;
	//}
		//char counted[5];
		//myLongStr(gsmTimer,counted,10,10);
		//strcat(count,"count:");
		//strcat(count,counted);
		//strcat(count,"\r\n");
		//sprintf(count, "%d", gsmTimer);
		//sprintf(count, "%d", gsmState);
		//sendData(count,UART0);
		//sendData("\r\n",UART0);
		//if ((gsmTimer >= 20)&&(smsFlags.config == 1)&&(gsmInfo.socket == 0)){
		if ((gsmTimer >= 20)&&(smsFlags.config == 1)){
			gsmTimer = 0;
			Debug_Send("Recy\n");
			//gsmState = SMS_Check;
			gsmState = USSD;		//test
			gsmState = SMS_Check;


		}
		gsmInfo.GPRStimer++;

		if ((gsmInfo.GPRStimer >= gsmInfo.GPRSinterval)&&(gsmState == 0)&&(smsFlags.send == 0)&&(smsFlags.signal == 1)){
			gsmInfo.GPRStimer = 0;
			if (smsFlags.gprsActive == 0) gsmState = GPRS_On;
			else if (smsFlags.gprsActive == 1)gsmState = SocketOpen;
			else if (smsFlags.gprsPending == 1) gsmState = GPRS_Off;		//switch off gsm if previous
		}

		//if (gsmState == 0) gsmInfo.Timeout++;
		if(gsmInfo.Timeout >= 60){
			gsmInfo.Timeout = 0;
			GSM_Init();
			gsmState = GSM_Off;

		}
		//smsTimer++;
		if ((smsTimer >= 3600)&&(gsmState == 0)&&(gsmInfo.socket == 0)&&(smsFlags.signal == 1)){
			smsTimer = 0;
			strcpy(smsInfo.build,"test");
			BuildPower();
			gsmState = SMS_Send;
		}
		//sprintf (temp,"GSM state after: %d\r\n", gsmState);
		//sendData(temp,UART0);
	//if (errorTimer >= 55){
		//errorTimer = 0;
		//strcpy(smsInfo.recMSISDN,"0720631005");
		//buildInfo();
		//smsFlags.send = 1;
		//restart = 1;
		//restartTimer = 0;
		//gsmState = 30;
	//}
		//buildInfo();
	//}


}

void GSM_Send(char* data){
	int size;
	int timeout = 5;
	size = strlen(data);
	//HAL_UART_Transmit(&huart2, (uint8_t*)data, size, timeout);

	HAL_UART_Transmit_IT(&huart2, data, size);
	HAL_UART_Receive_IT(&huart2, (uint8_t *) dummy, 50);
}

void Debug_Send(char* data){
	int size;
	int timeout = 5;
	size = strlen(data);
	//HAL_UART_Transmit(&huart1, (uint8_t*)data, size, timeout);
	HAL_UART_Transmit_IT(&huart1, data, size);
}

void SendChar(char data){
	int tickstart;
	tickstart = HAL_GetTick();
	if(UART_WaitOnFlagUntilTimeout(&huart1, UART_FLAG_TXE, RESET, tickstart, 10) != HAL_OK)
	  {
		//return HAL_TIMEOUT;
	  }
	huart1.Instance->TDR = (data & (uint8_t)0xFFU);

}

void recData(){
	//HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	//while (recBuffPointerWrite != recBuffPointerRead){
	/*if (gsmInfo.Prompt == 1) {
		gsmInfo.Prompt = 0;
		Debug_Send("got prompt2\r\n");
		if (gsmInfo.socket == 1) gsmState = GPRS_Write;
		else gsmState = SMS_Text;
	}*/
		if (recBuffPointerWrite != recBuffPointerRead){
			//if (recBuff[recBuffPointerRead] != 0x0A){
			procBuff[procBuffpointer] = recBuff[recBuffPointerRead];
			//HAL_UART_Transmit(&huart1, procBuff[procBuffpointer], 1, 10);
			/*if (recBuff[recBuffPointerRead] != 0x0A)*/ //SendChar(procBuff[procBuffpointer]);
			recBuffPointerRead++;
			if (recBuffPointerRead > recBuffSize) recBuffPointerRead = 0;


			procBuffpointer++;
			if (procBuffpointer > procBuffSize) procBuffpointer = 0;
			//}

			//if (procBuff[procBuffpointer - 1] == '>'){
			if (procBuff[procBuffpointer - 1] == 62){
			//if (procBuff[procBuffpointer] == '>'){
				//Debug_Send("got prompt\r\n");
				if (gsmInfo.socket == 1) gsmState = GPRS_Write;
				else gsmState = SMS_Text;
			}

			if (procBuff[procBuffpointer - 1] == 0x0A){
				procBuff[procBuffpointer - 1] = 0;
				procBuffpointer = 0;
				//sendData(procBuff,UART0);
				//sendData("\r\n",UART0);
				//Debug_Send("A");
			}
			if (procBuff[procBuffpointer - 1] == 0x0D){
				procBuff[procBuffpointer - 1] = 0;
				procBuffpointer = 0;
				//Debug_Send(procBuff);
				//Debug_Send("\r\n");
				//Debug_Send("D");
				procData();
				HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
				//("0x0D\r\n");

			}
		}
	//}
}

void procData(){		//process line
	//Debug_Send("Rx:");
	//Debug_Send(procBuff);
	//Debug_Send("\r\n");
	strcpy(temp,"Rx:");
	strcat(temp,procBuff);
	strcat(temp,"\r\n");
	Debug_Send(temp);
	if (smsFlags.content == 1){
		smsFlags.content = 0;
		smsFlags.del = 1;
		//if(strncmp((char*)smsInfo.index,"1",2)==0){
			//sendData("Got index: ",UART0);
			//sendData(smsInfo.index,UART0);
			//sendData("\r\n",UART0);
			strcpy(smsInfo.content1,procBuff);	//store SMS content
			//caps(smsInfo.content1);
			//procSMS();
		/*}
		if(strncmp((char*)smsInfo.index,"2",2)==0){
			sendData("Got index2\r\n",UART0);
			strcpy(smsInfo.content2,procBuff);
			procSMS();
		}*/
		//sendData("SMS conent:",UART0);
		//sendData(smsInfo.content,UART0);
		//sendData("\r\n",UART0);
		//sendData("Got SMS content\r\n",UART0);
			Debug_Send("SMS Process Done\r\n");
	}
	if (gsmInfo.imeiFlag == 1)getIMEI();
	else if (gsmInfo.imeiFlag > 1){
		//Debug_Send("DEC imei\r\n");
		gsmInfo.imeiFlag--;
	}
	else if(strncmp((char*)procBuff,"AT+CGSN",7)==0){
			//Debug_Send("Capture IMEI!\r\n");
			gsmInfo.imeiFlag = 1;
		}
	else if (gsmInfo.GPRS_Rec == 1){
		//Debug_Send("Data!\r\n");
		DataDecrypt();
	}
	else if (gsmInfo.GPRS_Rec > 1) gsmInfo.GPRS_Rec--;
	else if(strncmp((char*)procBuff,"+CREG: 0,1",10)==0)gsmState = SMSconfig;
	else if(strncmp((char*)procBuff,"+CREG: 1",8)==0)gsmState = SMSconfig;
	else if(strncmp((char*)procBuff,"+CGREG: 1",9)==0) Debug_Send("GOT GPRS!!!!!!!\r\n");
	else if(strncmp((char*)procBuff,"+CSQ",4)==0)getSignal();
	//else if(strncmp((char*)procBuff,"+CGSN:",6)==0)getIMEI();
	else if(strncmp((char*)procBuff,"+CMGS:",6)==0)smsFlags.send = 0;
	else if(strncmp((char*)procBuff,"+CCID:",6)==0)getCCID();
	else if(strncmp((char*)procBuff,"OK",3)==0)	OK();
	else if(strncmp((char*)procBuff,"+CGPADDR",8)==0)Context();
	//else if(strncmp((char*)procBuff,"SEND FAIL",9)==0)Socket1();
	else if(strncmp((char*)procBuff,"CONNECT",6)==0){
		gsmInfo.gprsDataPending = 0;
		gsmInfo.socket = 1;
		gsmState = GPRS_Send;
		//gsmState = GPRS_SendMode;
	}
	else if(strncmp((char*)procBuff,"ALREADY CONNECT",15)==0){
		gsmState = SocketClose;
		gsmInfo.socket = 0;
	}
	else if(strncmp((char*)procBuff,"CONNECT FAIL",13)==0){
		gsmState = GPRS_Off;
	}
	else if(strncmp((char*)procBuff,"SEND OK",7)==0){	//data sent
		//gsmInfo.GPRS_Rec = 1;
		//gsmState = SocketClose;
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	}
	else if(strncmp((char*)procBuff,"CLOSED",7)==0){	//socket closed by remote
		gsmInfo.socket = 0;
		gsmTimer = 0;
		Debug_Send("Socket closed\r\n");
		gsmState = GPRS_Off;
		gsmState = 0;
	}
	else if(strncmp((char*)procBuff,"CLOSE OK",8)==0){
		gsmInfo.socket = 0;
		gsmTimer = 0;
		Debug_Send("Socket forced closed\r\n");
		gsmState = GPRS_Off;
		gsmState = 0;
	}
	else if(strncmp((char*)procBuff,"+MIPCLOSE",9)==0)gsmState = GPRS_Off;
	else if(strncmp((char*)procBuff,"SEND FAIL",8)==0)gsmState = GPRS_Off;
	//else if(strncmp((char*)procBuff,"+MIPRTCP",8)==0)DataDecrypt();
	//else if(strncmp(procBuff,"+SIM READY",10)==0)sendData("SIM is ready!!!\r\n",UART0);
	//else if(strncmp((char*)procBuff,"+CLIP:",5)==0)Call();
	//else if(strncmp((char*)procBuff,"+COPS:",5)==0)Network();
	//else if(strncmp((char*)procBuff,"+SIM DROP",8)==0)gsmState = GSM_Off;
	//else if(strncmp((char*)procBuff,"+CUSD:",5)==0)Balance();
	else if(strncmp((char*)procBuff,"+CMGL",5)==0) GotSMS();
	else if(strncmp((char*)procBuff,"+CPIN: NOT READY",16)==0){
		GSM_Init();
	}
	else if(strncmp((char*)procBuff,"ERROR",5)==0){
		//gsmState = GSM_Off;
		onTimer = 0;
		smsFlags.config = 0;
		//restartTimer = 0;
		smsFlags.gprsActive = 0;
		if (gsmInfo.balancePending == 1){
			gsmInfo.balancePending = 0;
			gsmState = 0;
		}
	}
	else if (gsmInfo.GPRS_Rec == 1){
		Debug_Send("Socket forced closed\r\n");
	}
	//else if(strncmp((char*)procBuff,"ERROR",5)==0)if (tempState == USSD) gsmState = SMS_Check;	//restart

	//else if(strncmp((char*)procBuff,"+CGDCONT",8)==0)checkGPRS();


	//sendData(procBuff,UART0);
	//sendData("\r\n",UART0);
	//else if(strncmp((char*)procBuff,"+SIM READY1",8)==0);		//unsolicited
}

void sendSMS(char* num, char* msg){
	if (smsFlags.signal == 0) return;
	if (smsFlags.config == 0) return;
	if (gsmInfo.low == 1) return;
	//char sms
	//sendData("Send SMS\r\n",UART0);
	strcpy (temp, "AT+CMGS=\"");
	strcat(temp,num);
	strcat(temp,"\"\r\n");
	GSM_Send(temp);

	//GSM_Send("AT+CMGS=\"");
	//GSM_Send(num);,
	//GSM_Send("\"\r\n");
	//sendData("AT+CMGS=\"0720631005\"\r\n",UART1);
	strcpy(SMScontent,msg);
	//smsFlags.send = 0;
}

void Network(){
	//sendData("Network\r\n",UART0);
	myStrSection(procBuff, gsmInfo.network,30,'"',1);
	Debug_Send(gsmInfo.network);
	//sendData("\r\n",UART0);
}

void Balance(){
	char tmp[15];
	gsmInfo.balancePending = 0;
	//sendData("Balance\r\n",UART0);
	myStrSection(procBuff, tmp,9,'R',1);
	myStrSection(tmp, gsmInfo.balance,9,' ',0);
	gsmState = SMS_Check;
	if (gsmInfo.balance[0] == 0x30) gsmInfo.low = 1;
	else gsmInfo.low = 0;
	//sendData("R",UART0);
	//sendData(gsmInfo.balance,UART0);
	//sendData("\r\n",UART0);
}


void Call(){
	char tmp[30];
	myStrSection(procBuff, tmp,30,'"',1);
	if(strncmp((char*)tmp,gsmInfo.msisdn,8)==0){
		//RELAY ^= 1;

	}
}
/*
void Error(){
	errorCounter++;
	if (errorCounter >= 3){
		errorCounter = 0;
		gsmState = 20;
	}
}*/

void getIMEI(){
	//myStrSection(procBuff, gsmInfo.imei,30,'"',1);
	/*Debug_Send("Procbuff:");
	Debug_Send(procBuff);
	Debug_Send("\r\n");*/
	strcpy(gsmInfo.imei,procBuff);
	Debug_Send("IMEI:");
	Debug_Send(gsmInfo.imei);
	Debug_Send("\r\n");
	gsmInfo.imeiFlag = 0;
	//sendData("IMEI:",UART0);
	//sendData(gsmInfo.imei,UART0);
	//sendData("\r\n",UART0);
}

void getCCID(){
	myStrSection(procBuff, gsmInfo.ccid,30,' ',1);
	//sendData("CCID:",UART0);
	//sendData(gsmInfo.ccid,UART0);
	//sendData("\r\n",UART0);
}

void getSignal(){
	myStrSection(procBuff, quality,3,' ',1);
	int temp2;
	int temp3;
	temp2 = atoi(quality);
	temp2 *= 2;
	temp3 = 113 - temp2;
	sprintf (quality," -%ddBm", temp3);
}

void procSMS(){
	Debug_Send("Process SMS\r\n");
	//if (gsmInfo.registered == 1)
		//if(strncmp(gsmInfo.msisdn,smsInfo.recMSISDN,5)=0) return;
	//caps((char*)smsInfo.content1);		//convert to capital letters
	int index;
	index = 0;
	char temp[30];
	//char temp1[20];
	//strcpy(temp,"+279602005555439");	//test
	//strcpy(temp,"+27720631005");
	//if(strncmp(gsmInfo.msisdn,temp,20)!=0) return;
	//sendData("Numbers matched!\r\n",UART0);
	//return;
	//strcat(smsInfo.content1,"pass:Kayleigh");
	caps(smsInfo.content1);
	//sendData(smsInfo.content1,UART0);
	//sendData("\r\n",UART0);
	if(strncmp((char*)smsInfo.content1,"USER",4)==0){
		Register();
		//Confirmation();
	}

	else if(strncmp((char*)smsInfo.content1,"INFO",4)==0){
		buildInfo();
		//RELAY = 1;
	}
	//if(strncmp((char*)gsmInfo.msisdn,smsInfo.recMSISDN,20)!=0) return;
	else if(strncmp((char*)smsInfo.content1,"TEST",4)==0){
		buildInfo();
		//RELAY = 0;
	}
	else if(strncmp((char*)smsInfo.content1,"ON",2)==0){
		//myStrSection((char*)smsInfo.content1, (char*)temp[,30,',',0);
		if ((gsmInfo.activeFlag == 1)&&(gsmInfo.ccidStatus == 0)){
			//RELAY = 1;
			gsmInfo.relayFlag = 1;
			Confirmation();
			gsmInfo.updateFlag = 1;
		}
		else Deny();

	}
	else if(strncmp((char*)smsInfo.content1,"OFF",3)==0){
		if ((gsmInfo.activeFlag == 1)&&(gsmInfo.ccidStatus == 0)){
			//RELAY = 0;
			gsmInfo.relayFlag = 1;
			Confirmation();
			gsmInfo.updateFlag = 1;
		}
		else Deny();

	}
	if(strncmp((char*)smsInfo.content1,"URL",3)==0){
		myStrSection(smsInfo.content1, temp,20,':',1);
		myStrSection(temp, gsmInfo.url,20,',',0);
		myStrSection(temp, gsmInfo.urlport,10,',',1);
		Debug_Send("URL\r\n");
		Debug_Send(gsmInfo.url);
		Debug_Send(gsmInfo.urlport);
	}
	/*if(strncmp((char*)smsInfo.content1,"user1",5)==0){
		myStrSection(smsInfo.content1, gsmInfo.user1,20,':',1);
	}*/
	if(strncmp((char*)smsInfo.content1,"RESET",5)==0){
		//myStrSection(smsInfo.content1, gsmInfo.user2,20,':',1);
		gsmState = GSM_Off;
		Debug_Send("Unit reset\r\n");
	}
	if(strncmp((char*)smsInfo.content1,"VTH",3)==0){
		myStrSection(smsInfo.content1, gsmInfo.vth,20,':',1);
		//gsmInfo.vth_f = atof(gsmInfo.vth);
		gsmInfo.vth_l = myStrLong(gsmInfo.vth, 10);
		gsmInfo.vth_l *= 100;
		gsmInfo.vthFlag = 1;
	}
	if(strncmp((char*)smsInfo.content1,"REBOOT",3)==0){

	}
	//gsmState = SMS_Del;			//del sms

}

void Register(){
	char pair[20];
	char name[20];
	int index = 0;
	myStrSection((char*)smsInfo.content1, (char*)pair,100,',',0);
	//sendData(pair,UART0);
	while (strlen(pair) > 1){
		Debug_Send(pair);
		Debug_Send("-p\r\n");
		myStrSection((char*)pair, name,30,':',0);			//get first param pair
		//sendData(name,UART0);
		//sendData("-n\r\n",UART0);
		if(strncmp((char*)name,"USER",4)==0){
			if (gsmInfo.userFlag == 0){
				myStrSection(pair, gsmInfo.user,30,':',1);		//get param value
				gsmInfo.userFlag = 1;
				Debug_SendsendData("Got user\r\n");
			}
			else {
				if (CheckUser() == 1){
					myStrSection(pair, gsmInfo.user,30,':',1);		//get param value
					gsmInfo.userFlag = 0;
				}
				gsmInfo.userFlag = 0;		//debug only
			}
			//sendData(gsmInfo.user,UART0);
			//sendData("\r\n",UART0);
		}
		if(strncmp((char*)name,"MSISDN",6)==0){
			//myStrSection(pair, gsmInfo.msisdn,30,':',1);
			char tempnum[20];
			myStrSection(pair, tempnum,30,':',1);
			if (tempnum[0] == 0x30){
				strcpy(gsmInfo.msisdn,"+2");
				strcat(gsmInfo.msisdn,tempnum);
				gsmInfo.msisdn[2] = 0x37;
			}
			else strcpy((char*)gsmInfo.msisdn,tempnum);
			gsmInfo.msisdnFlag = 1;
			//sendData(gsmInfo.msisdn,UART0);
			//sendData("\r\n",UART0);
		}
		if(strncmp((char*)name,"PASS",4)==0){
			myStrSection(pair, gsmInfo.password,30,':',1);
			gsmInfo.passwordFlag = 1;
			//sendData(gsmInfo.password,UART0);
			//sendData("\r\n",UART0);
		}
		gsmInfo.ccidFlag = 1;
		gsmInfo.vthFlag = 1;
		index++;
		myStrSection(smsInfo.content1, pair,100,',',index);
	}
}

void RegisterConfirm(){
	char temps[30];
	strcpy(SMScontent,"Unit registered\n");
	sprintf(temps,"User: %s\n",gsmInfo.user);
	strcat(SMScontent,temps);
	sprintf(temps,"MSISDN: %s\n",gsmInfo.msisdn);
	strcat(SMScontent,temps);
	strcat(SMScontent,"URL: ");
	strcat(SMScontent,gsmInfo.url);
	strcat(SMScontent,":");
	strcat(SMScontent,gsmInfo.urlport);
	strcpy (smsInfo.build, SMScontent);
	smsFlags.reply = 1;

}

char CheckUser(){
	Debug_Send("Check user\r\n");
	if (strlen(gsmInfo.msisdn) < 2) return 0;
	if (strncmp(smsInfo.recMSISDN, gsmInfo.msisdn, 16) == 0){
		Debug_Send("Found user\r\n");
		return 1;
	}
	else {
		Debug_Send("No user found\r\n");
		return 0;
	}
}

void Confirmation(){
	strcpy(smsInfo.build,"Command accepted\n");
	smsFlags.reply = 1;
}

void Deny(){
	strcpy(smsInfo.build,"Command ignored, unit suspended\n");
	smsFlags.reply = 1;
}

void VoltageAlarm(){
	char alarm[50];
	char voltage[20];
	strcpy(smsInfo.build,"Alarm!\r\nVoltage:");
	GetADC(voltage);
	strcat(smsInfo.build,voltage);
	smsFlags.reply = 1;
	//strcpy(smsInfo.recMSISDN, gsmInfo.msisdn);
	//strcpy(smsInfo.recMSISDN, "0733507155");
	//sendSMS(gsmInfo.msisdn, alarm);
	//sendData("Alarm generated\r\n",UART0);
}
/*
void alarmFunc(Alarm input){
	char alarm[50];
	char voltage1[20];
	if (input == voltage){
		strcpy(smsInfo.build,"Alarm!\r\nVoltage:");
		GetADC(voltage1);
		strcat(smsInfo.build,voltage1);
		smsFlags.reply = 1;
	}
	if (input == ccid){
		sendData("CCID Alarm generated\r\n",UART0);
		strcpy(smsInfo.build,"Alarm!\r\nCCID mismatch:");
		smsFlags.reply = 1;
	}
}*/

void caps(char* str){
	char result[160];
	int pointer = 0;
	while (*str != 0){
		if ((*str >= 'a') && (*str <= 'z')){
			//sendByte(* str, UART0);
			//*str -= 32;
			result[pointer] = *str - 32;
			*str = result[pointer];
			//sendByte(result[pointer], UART0);
			str++;
			pointer++;
		}
		else{
			result[pointer] = *str;
			str++;
			pointer++;
		}
	}
	*str = 0;
	//pointer++;
	result[pointer] = 0;
	//sendData(result,UART0);
	//strcpy(str, result);

}

void buildInfo(){
	Debug_Send("Build INFO SMS\r\n");
	//strcpy(SMScontent,"Powerbox is active\n ");
	char temps[40];
	char adcv[10];
	GetADC(adcv);
	strcpy(SMScontent,"GSM active\n");
	sprintf(temps,"V: %s\n",adcv);
	strcpy(SMScontent,temps);
	sprintf(temps,"Thresh: %sV\n",gsmInfo.vth);
	strcat(SMScontent,temps);
	//sprintf(temps,"CCID: %s\n",gsmInfo.ccid);
	if (gsmInfo.ccidStatus == 1) strcat(SMScontent,"CCID miss-matched!\r\n");
	//sprintf(temps,"mode: %s\n",smsInfo.IP);
	//strcat(SMScontent,temps);
	if (gsmInfo.modeFlag == 1) strcat(SMScontent,"Mode:Pulse\r\n");
	else strcat(SMScontent,"Mode: Latch\r\n");
	if (gsmInfo.activeFlag == 1) strcat(SMScontent,"Status:Active\r\n");
	else strcat(SMScontent,"Status: Suspended\r\n");
	//if (RELAY == 1) strcat(SMScontent,"Relay: ON\r\n");
	//else strcat(SMScontent,"Relay: OFF\r\n");
	sprintf(temps,"Airtime: %s\n",gsmInfo.balance);
	strcat(SMScontent,temps);
	sprintf(temps,"User: %s\n",gsmInfo.user);
	strcat(SMScontent,temps);
	sprintf(temps,"MSISDN: %s\n",gsmInfo.msisdn);
	strcat(SMScontent,temps);
	quality[0] = 0x20;
	sprintf(temps,"Signal:%s\n",quality);
	strcat(SMScontent,temps);
	strcat(SMScontent,"URL: ");
	strcat(SMScontent,gsmInfo.url);
	strcat(SMScontent,":");
	strcat(SMScontent,gsmInfo.urlport);
	//sprintf(temps,"URL:%s:%s\n",gsmInfo.url, gsmInfo.urlport);
	//strcat(SMScontent,temps);
	//strcat(SMScontent,quality);
	//strcat(SMScontent,"\n");
	/*rtc time;
	time = getUptime();
	char temp1[30];
	sprintf (temp1,"Uptime: %.2dw %.2dd %.2dh %.2dm %.2ds\n", time.weeks, time.days, time.hours, time.minutes, time.seconds);
	strcat(SMScontent,temp1);*/
	strcpy (smsInfo.build, SMScontent);
	//sendData("\r\nSMS content:",UART0);
	//sendData(SMScontent,UART0);
	//sendData("\r\n",UART0);
	//sendData("\r\nSMSinfo build:",UART0);
	Debug_Send(smsInfo.build);
	Debug_Send("\r\n");
	smsFlags.reply = 1;			//generates reply sms
	//strcpy(smsInfo.recMSISDN,"0720631005");	//debug for sms from rtc
}

void BuildPower(){
	/*int temp3;
	char temp2[20];
	temp3 = getVolt();
	myLongStr(voltage,temp2,10,10);
	strcpy (smsInfo.build, "IMC v4.08\n");
	strcat (smsInfo.build, "Voltage:");
	strcat (smsInfo.build, temp2);
	strcat (smsInfo.build, "\n");
	temp3 = getCurrent();
	myLongStr(current,temp2,10,10);
	strcat (smsInfo.build, "Current:");
	strcat (smsInfo.build, temp2);
	strcat (smsInfo.build, "\n");*/

	sprintf(smsInfo.build, "IMC4.08\n v:%i c %i\r\n",voltage, current1);

	Debug_Send(smsInfo.build);
}

void Context(){	//MIPCALL

	myStrSection(procBuff, smsInfo.IP,15,'"',1);
	if (strlen(smsInfo.IP) > 3){
		smsFlags.gprsActive = 1;		//if gotten IP
		smsFlags.gprsPending = 0;
		gsmState = SocketOpen;
		Debug_Send(smsInfo.IP);
		Debug_Send("\r\n");
		//sendData(smsInfo.IP,UART0);
		//sendData("\r\n",UART0);
		//sendData("Context opened\r\n",UART0);
	}
	else{
		smsFlags.gprsActive = 0;
		//gsmState = USSD;
		//gsmState = SMS_Check;
		if (gsmInfo.alarm == 1){
			//gsmInfo.alarm = 0;
			gsmState = SMS_Send;
		}
		//sendData("Context closed\r\n",UART0);
	}
}

/*void Socket(){
	char temp[10];
	char temp1[5];
	int index = 0;
	gsmInfo.socket1 = 1;
	gsmInfo.socket2 = 1;
	gsmInfo.socket3 = 1;
	gsmInfo.socket4 = 1;
	gsmInfo.socket = 0;
	myStrSection(procBuff, temp ,15,' ',1);
	for (index = 0; index < 4; index++){
		myStrSection(temp, temp1 ,15,',',index);
		if (strncmp((char*)temp1,"1",1)==0) gsmInfo.socket1 = 0;
		if (strncmp((char*)temp1,"2",1)==0) gsmInfo.socket2 = 0;
		if (strncmp((char*)temp1,"3",1)==0) gsmInfo.socket3 = 0;
		if (strncmp((char*)temp1,"4",1)==0) gsmInfo.socket4 = 0;
	}
	char sock[20];
	sprintf(sock,"Socket status: 1:%d 2:%d 3:%d 4:%d \r\n",gsmInfo.socket1, gsmInfo.socket2, gsmInfo.socket3, gsmInfo.socket4);
	sendData(sock,UART0);
	if ((gsmInfo.socket1 == 1)||(gsmInfo.socket2 == 1)||(gsmInfo.socket3 == 1)||(gsmInfo.socket4 == 1)){
		gsmInfo.socket = 1;
		gsmState = 17;
	}
	else gsmInfo.socket = 0;
}*/
/*
void Socket1(){
	char temp[10];
	char temp1[5];
	char temp2[5];
	int index = 0;
	gsmInfo.socket1 = 0;
	gsmInfo.socket2 = 0;
	gsmInfo.socket3 = 0;
	gsmInfo.socket4 = 0;
	gsmInfo.socket = 0;
	myStrSection(procBuff, temp ,15,' ',1);		//1,1
	myStrSection(temp, temp1 ,15,',',0);		//socket
	myStrSection(temp, temp2 ,15,',',1);		//socket status
	if (strncmp((char*)temp2,"1",1)==0){
		if (strncmp((char*)temp1,"1",1)==0) gsmInfo.socket1 = 1;
		if (strncmp((char*)temp1,"2",1)==0) gsmInfo.socket2 = 1;
		if (strncmp((char*)temp1,"3",1)==0) gsmInfo.socket3 = 1;
		if (strncmp((char*)temp1,"4",1)==0) gsmInfo.socket4 = 1;
	}
	//char sock[20];
	//sprintf(sock,"Socket status: 1:%d 2:%d 3:%d 4:%d \r\n",gsmInfo.socket1, gsmInfo.socket2, gsmInfo.socket3, gsmInfo.socket4);
	//sendData(sock,UART0);
	if ((gsmInfo.socket1 == 1)||(gsmInfo.socket2 == 1)||(gsmInfo.socket3 == 1)||(gsmInfo.socket4 == 1)){
		gsmInfo.socket = 1;
		gsmState = GPRS_Write;			//send data
	}
	else gsmInfo.socket = 0;
}*/

void Socket2(){

}

void GotSMS(){
	if (smsFlags.reply == 1) return;
	char section[15];
	//char tempData[90];
	//strcpy(tempData,procBuff);
	//sendData("SMS info:\r\n",UART0);
	myStrSection(procBuff, section,10,',',0);
	myStrSection(section, smsInfo.index,3,' ',1);
	myStrSection(procBuff, smsInfo.recMSISDN,20,'"',3);
	//strcpy(smsInfo.recMSISDN,"0733507155");					//return SMS to default number
	//sendData(smsInfo.recMSISDN,UART0);
	//sendData(smsInfo.index,UART0);
	//sendData("\r\n",UART0);
	//sendData(smsInfo.index,UART0);
	//sendData("\r\n",UART0);
	smsFlags.content = 1;
}

void OK(){

	//Debug_Send("Got OK\r\n");
	errorTimer = 0;
	gsmTimer = 0;
	//if (gsmState == GPRS_On) gsmState = SocketOpen;
	if (smsFlags.reply == 1){		//sent when sms built
	//if (smsFlags.send == 1){
		if ((smsFlags.config == 1)&&(smsFlags.gprsActive == 0)) gsmState = SMS_Send;
		//sendData("Send OK\r\n",UART0);
	}
	if (gsmState == GprsContext) smsFlags.context = 1;
	if (smsFlags.del == 1) gsmState = SMS_Del;	//set when sms content read
	if (gsmState == AutoBaud) gsmState = Search;
	if (gsmState == GPRS_SendMode) gsmState = GPRS_SendModeTRans;
	//if (gsmState == 12) gsmState = 6;
	//sprintf(count, "GSM state=%d", gsmState);
	//sendData(count,UART0);
	//sendData("\r\n",UART0);
}

void GPRS_SendData(char* gprs){
	int len;
	len = strlen(gprs);
	Debug_Send(gprs);
	Debug_Send("-send\r\n");
	GSM_Send(gprs);
	//sprintf (temp,"Len: %d\r\n", len);
	//sendData(temp,UART0);
	/*int ind = 0;
	char byt[5];
	strcpy((char*)GPRSresult,"");
	len++;
	while (len > 0){
		pec_Update(byt,*gprs);
		gprs++;
		strcat(GPRSresult,byt);
		strcat(GPRSresult,0);			//test
		GSM_Send(GPRSresult);		//test
		GPRSresult[0] = 0;				//test
		len--;
		ind++;
	}*/
	//strcat(GPRSresult,"0A0D");
	//sendData(GPRSresult,UART1);			//test remove
	//sendData("0A0D",UART1);				//test
	//sendData("\r\n",UART1);
	//sendData(GPRSresult,UART0);
	//sendData("\r\n",UART0);
	//sprintf (temp,"GPRS send:GSM state-%d\r\n", gsmState);
	//	sendData(temp,UART0);
	GPRSresult[0] = 0;
}

void DataStatus(){
	char temp[10];
	char temp1[5];
	char temp2[5];
	myStrSection(procBuff, temp ,15,' ',1);		//1,1
	myStrSection(temp, temp1 ,15,',',0);		//socket
	myStrSection(temp, temp2 ,15,',',1);		//socket data status
	if (strncmp((char*)temp2,"0",1)==0){

			//sendData("ACK indication\r\n",UART0);
		}
	if (strncmp((char*)temp2,"1",1)==0){
		gsmInfo.socket = 0;
		Debug_Send("Broken protocol stack\r\n");
		gsmState = GPRS_Off;  		//close socket
	}
	if (strncmp((char*)temp2,"2",1)==0){

		gsmInfo.socket = 0;
		//sendData("Connection closed automatically due to non � fatal alert\r\n",UART0);
		gsmState = GPRS_Off;  		//close socket
	}
}

void DataDecrypt(){
	Debug_Send("Decrypt\r\n");
	Debug_Send(procBuff);
	Debug_Send("\r\n");
	int value;
	//char test[20];
	//char test1[10];
	char temp[180];
	char temp1[50];
	//strcpy(procBuff,"696E672F");
	int len;
	int index = 0;
	char hex[4];
	int hexIndex = 0;
	int count = 0;
	char receive[180];
	/*myStrSection(procBuff, temp ,180,',',2);
	//len = strlen(procBuff);
	len = strlen(temp);
	//sprintf(test1, "len:%d\r\n", len);
	//sendData(test1,UART0);
	//sendData(temp,UART0);
	//sendData("\r\n",UART0);

	int recIndex = 0;
	len += 2;
	while (index < len){
		//sendData("D\r\n",UART0);
		//sprintf(test1, "i:%d\r\n", index);
		//sendData(test1,UART0);
		//sprintf(test1, "h:%d\r\n", hexIndex);
		//sendData(test1,UART0);

		if (hexIndex < 2){
			hex[hexIndex] = temp[index];
			//hex[hexIndex] = procBuff[index];
			hexIndex++;

		}
		else {			//hex index == 2
			hexIndex = 0;
			hex[2] = 0;
			//sendData("2\r\n",UART0);
			//sendByte(hex[0], UART0);
			//sendByte(hex[1], UART0);
			value = Getvalue(hex[0]) * 16;
			value += Getvalue(hex[1]);
			receive[recIndex] = value;
			recIndex++;
			//sprintf(test1, "VALUE:%d\r\n", value);
			//sendData(test1,UART0);
			//if (value == 105) sendData("First byte 105\r\n",UART0);
			hex[hexIndex] = temp[index];
			//hex[hexIndex] = procBuff[index];
			hexIndex++;
		}
		//if (index == 7){
			//hex[2] = 0;
			//sendData("3\r\n",UART0);
			//sendByte(hex[0], UART0);
			//sendByte(hex[1], UART0);
			//value = Getvalue(hex[0]) * 16;
			//value += Getvalue(hex[1]);
			//receive[recIndex] = value;
			//recIndex++;
			//if (strncmp((char*)hex,"6",1)==0) sendData("First nibble 6\r\n",UART0);
		//}
		index++;
	}
	//recIndex++;
	receive[recIndex] = 0;
	index = 0;
	myStrSection(receive, temp ,160,';',index);*/
	strcpy(receive,procBuff);
	myStrSection(receive, temp ,160,';',index);

	while (strlen(temp) > 1){
		Debug_Send(temp);
		Debug_Send("\r\n");
		if(strncmp((char*)temp,"user",4)==0){
			myStrSection(temp, temp1 ,40,':',1);
			myStrSection(temp1, gsmInfo.user ,30,',',0);
			myStrSection(temp1, gsmInfo.msisdn ,30,',',1);
			Debug_Send(gsmInfo.user);
			Debug_Send(" - ");
			Debug_Send(gsmInfo.msisdn);
			//LED = 0;
			Debug_Send("\r\n");
			gsmInfo.registered = 1;
		}
		if(strncmp((char*)temp,"output",5)==0){
			char state[5];
			myStrSection(receive, state ,4,':',1);
			if(strncmp(state,"ON",2)==0){
				if (gsmInfo.updateFlag == 0){
					//if (gsmInfo.activeFlag == 1) RELAY = 1;
				}
			}
			if(strncmp(state,"OFF",3)==0){
				Debug_Send("Off\r\n");
				if (gsmInfo.updateFlag == 0){
					//if (gsmInfo.activeFlag == 1) RELAY = 0;
				}
			}
			//CheckUser();
		}
		if(strncmp((char*)temp,"mode",4)==0){
			Debug_Send("GOT MODE!!!!!\r\n");
			myStrSection(temp, temp1 ,40,':',1);
			if (strncmp(temp1,"latch",5)==0){
				gsmInfo.modeFlag = 0;
			}
			if (strncmp(temp1,"pulse",5)==0){
				gsmInfo.modeFlag = 1;
			}
			Debug_Send(temp1);
			Debug_Send("\r\n");
		}
		if(strncmp((char*)temp,"ccid",4)==0){
			char ccid1[30];
			myStrSection(temp, ccid1 ,40,':',1);
			if (strncmp(gsmInfo.ccid,ccid1,25)==0){
				Debug_Send("CCID same\r\n");
				gsmInfo.ccidStatus = 0;
			}
			else{
				//alarmFunc(ccid);
				//RELAY = 0;
				Debug_Send("CCID differ\r\n");
				gsmInfo.ccidStatus = 1;
			}
		}
		if(strncmp((char*)temp,"status",6)==0){

		}
		if(strncmp((char*)temp,"vth1",4)==0){
			if (gsmInfo.vthFlag == 0){
				Debug_Send("GOT THV!!!!!\r\n");
				myStrSection(temp, gsmInfo.vth ,5,':',1);
				gsmInfo.vth_l = myStrLong(gsmInfo.vth, 10);
				gsmInfo.vth_l *= 100;
			}
			Debug_Send(gsmInfo.vth);
			Debug_Send("\r\n");

		}
		if(strncmp((char*)smsInfo.content1,"vth",5)==0){
			myStrSection(smsInfo.content1, gsmInfo.vth,4,':',1);
			//gsmInfo.vth_f = atof(gsmInfo.vth);
			gsmInfo.vth_l = myStrLong(gsmInfo.vth, 10);
			gsmInfo.vth_l *= 100;
		}
		if(strncmp((char*)temp,"mode",4)==0){

		}
		if(strncmp((char*)temp,"status",5)==0){
			myStrSection(temp, temp1 ,4,':',1);
			if(strncmp(temp1,"1",1)==0){
				Debug_Send("Unit active\r\n");
				gsmInfo.activeFlag = 1;
			}
			else{
				gsmInfo.activeFlag = 0;
				Debug_Send("Unit not active\r\n");
			}
		}
		if(strncmp((char*)temp,"sms",3)==0){
			myStrSection(temp, temp1 ,4,':',1);
			if(strncmp(temp1,"1",1)==0){
				Debug_Send("Generate SMS\r\n");
				buildInfo();
			}
			else Debug_Send("Dont generate SMS\r\n");
		}
		if(strncmp((char*)temp,"sync",4)==0){
			myStrSection(temp, temp1 ,4,':',1);
			if(strncmp(temp1,"1",1)==0){
				Debug_Send("Unit active\r\n");
			}
			else Debug_Send("Unit not active\r\n");
		}
		if(strncmp((char*)temp,"vthresh",7)==0){
			myStrSection(temp, temp1 ,4,':',1);
			if(strncmp(temp1,"1",1)==0){
				Debug_Send("Threshold active\r\n");
				gsmInfo.vThreshold = 1;
			}
			else{
				Debug_Send("Threshold not active\r\n");
				gsmInfo.vThreshold = 0;
			}
		}
		if(strncmp((char*)temp,"saved",5)==0){
			Debug_Send("Generate registration SMS\r\n");
			RegisterConfirm();
			Debug_Send("Registration SMS generated\r\n");
		}
		if(strncmp((char*)temp,"tmg",3)==0){
			myStrSection(temp, gsmInfo.GPRSint,20,':',1);
			gsmInfo.GPRSinterval = myStrLong(gsmInfo.GPRSint, 10);
			gsmInfo.GPRSinterval *= 60;
			/*sprintf(temp, "tmg %i\r\n",gsmInfo.GPRSinterval);
			Debug_Send(temp);*/
			/*Debug_Send("Got tmg:");
			Debug_Send(gsmInfo.GPRSint);
			Debug_Send("\r\n");*/
			//gsmInfo.vth_f = atof(gsmInfo.vth);


			//gsmInfo.vthFlag = 1;
		}
		if(strncmp((char*)temp,"no user",7)==0){
			//LED = 1;
			gsmInfo.registered = 0;
		}
		index++;
		myStrSection(receive, temp ,160,';',index);
	}
	//if(strncmp((char*)temp,"vth",3)==0){
	gsmInfo.updateFlag = 0;
	//}
	//Debug_Send(receive);
	//Debug_Send("\r\n");
	receive[0] = 0;
	gsmState = SocketClose;
	gsmInfo.GPRS_Rec = 0;
	gsmState = 0;
}

int Getvalue(char in){
	int count = 0;

	int compare = 0x30;
	compare = 0x30;
	if ((in >= 0x30) && (in <= 0x39)){
		 //sendData("Number\r\n",UART0);
		 while(compare != in){
			 //sendByte(compare, UART0);
			 compare++;
			 count++;
		 }
	 }
	compare = 0x41;
	 if ((in >= 0x41) && (in <= 0x46)){
		 //sendData("Letter\r\n",UART0);
		 while(compare != in){
			 //sendByte(compare, UART0);
			 compare++;
			 count++;
		 }
		 count += 10;
	 }
	 //char testS[5];
	 //sprintf(testS, "c:%d\r\n", count);
	 //sendData(testS,UART0);
	return count;
}


void GSM_Receive(char in){
	gsmInfo.Timeout = 0;
	//HAL_UART_Receive_IT(&huart2, temp, 50);
	if (in == '>') gsmInfo.Prompt = 1;
	recBuff[recBuffPointerWrite] = in;
	//HAL_UART_Transmit(&huart1, recBuff[recBuffPointerWrite], 2, 10);
	//huart1.Instance->TDR = in;
	recBuffPointerWrite++;
	if (recBuffPointerWrite > recBuffSize) recBuffPointerWrite = 0;

}
