/*
 * motor.c
 *
 *  Created on: Apr 25, 2019
 *      Author: Jeefo
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
//#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "GSM.h"
#include "motor.h"
#include "Relay.h"
#include "ADC.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

uint16_t  mode = 1;
uint32_t* adc_;
void Motor_Service(){

	switch(mode){
	case 1:	//brake
		Debug_Send("Motor brake\r\n");
		HAL_GPIO_WritePin(Valve_Phase_GPIO_Port, Valve_Phase_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Valve_Enable_GPIO_Port, Valve_Enable_Pin, GPIO_PIN_RESET);
		mode++;
		break;
	case 2:	//forward
		Debug_Send("Motor forward!!!!!!!\r\n");
		RelayOn();
		HAL_GPIO_WritePin(Valve_Phase_GPIO_Port, Valve_Phase_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Valve_Enable_GPIO_Port, Valve_Enable_Pin, GPIO_PIN_SET);
		mode++;
		break;
	case 3:	//brake
		Debug_Send("Motor brake\r\n");
		HAL_GPIO_WritePin(Valve_Phase_GPIO_Port, Valve_Phase_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Valve_Enable_GPIO_Port, Valve_Enable_Pin, GPIO_PIN_RESET);
		mode++;
		break;
	case 4: //reverse
		Debug_Send("Motor reverse!!!!!!!\r\n");
		RelayOff();
		HAL_GPIO_WritePin(Valve_Phase_GPIO_Port, Valve_Phase_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Valve_Enable_GPIO_Port, Valve_Enable_Pin, GPIO_PIN_RESET);
		mode = 1;
		break;
	}
}

void ValveOpen(){
	uint32_t tempval;
	uint8_t temp[20];
	uint8_t temp1[10];
	uint8_t open = 0;
	HAL_Delay(200);
	Debug_Send("Valve open start\r\n");
	HAL_GPIO_WritePin(Valve_Phase_GPIO_Port, Valve_Phase_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Valve_Enable_GPIO_Port, Valve_Enable_Pin, GPIO_PIN_SET);
	while(!open){
		adc_ = GetADC1();
		adc_+=ValveLimit;
		tempval = *adc_;
		strcpy(temp, "Valve I:"); //myStrLong(char *p_str,char p_base)
		itoa(tempval,temp1,10);
		strcat(temp,temp1);
		Debug_Send(temp);
		//if(tempval < 1100) open =1;
		HAL_Delay(50);
	}
	HAL_GPIO_WritePin(Valve_Phase_GPIO_Port, Valve_Phase_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Valve_Enable_GPIO_Port, Valve_Enable_Pin, GPIO_PIN_RESET);
	Debug_Send("Valve open done\r\n");
}

void ValveClose(){
	HAL_Delay(50);
}




