/*
 * LCD.c
 *
 *  Created on: Feb 12, 2020
 *      Author: Jeefo
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
//#include "cmsis_os.h"

#include "LCD1.h"
#include "myString.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
extern I2C_HandleTypeDef hi2c1;
uint8_t phase = 1;
char lcd[20];
uint8_t counter=0;
#define RS 0x01
#define WR 0x02
#define EN 0x04
#define BL 0x08


void LCD_Init1(){
	LCD_Instruction_Write(0x20);
	osDelay(40);
	LCD_Instruction_Write(0x20);
	osDelay(1);
	LCD_Instruction_Write(0x20);
	osDelay(1);
	LCD_Instruction_Write(0x20);
	osDelay(1);

	LCD_Instruction_Write4(0x28);		//Func set, 4 bits
	//LCD_Instruction_Write4(0x20);		//Func set, 4 bits, 1line
	//LCD_Instruction_Write4(0x30);		//Func set, 8 bits

	osDelay(1);
	LCD_Instruction_Write4(0x08);		//Display off, cursor off, blink off
	osDelay(1);
	LCD_Instruction_Write4(0x01);
	osDelay(1);
	LCD_Instruction_Write4(0x06);
	osDelay(1);
	LCD_Instruction_Write4(0x0C);/**///Display on, cursor off, blink off
	//LCD_Instruction_Write4(0x0E);	//Display on, cursor on, blink off
	//LCD_Instruction_Write4(0x0F);	//Display on, cursor on, blink on
	//LCD_String("lcd done");
	LCD_Instruction_Write4(0x03);
}

void Byte_Write(uint8_t data){
	uint8_t i2c[5];
	i2c[0] = data;
	HAL_I2C_Master_Transmit(&hi2c1, 0x4F, &i2c, 1, 10);
}

void LCD_Instruction_Write(uint8_t data){
	uint8_t temp;
	temp = data;
	temp &= ~RS;
	temp &= ~WR;
	temp &= ~EN;
	temp |= BL;
	Enable(temp);
	/*Byte_Write(temp);
	temp |= EN;
	Byte_Write(temp);*/
}
void LCD_Instruction_Write4(uint8_t data){
	uint8_t temp;
	temp = data;
	temp &= ~RS;
	temp &= ~WR;
	temp &= ~EN;
	temp |= BL;
	Enable(temp);
	temp = data;
	temp = temp << 4;
	temp &= ~RS;
	temp &= ~WR;
	temp &= ~EN;
	temp |= BL;
	Enable(temp);
}


void LCD_Data_Write(uint8_t data){
	uint8_t temp;
	temp = data;
	temp |= RS;
	temp &= ~WR;
	temp &= ~EN;
	temp |= BL;
	Enable(temp);
}

void LCD_Data_Write4(uint8_t data){
	uint8_t temp;
	temp = data;
	temp |= RS;
	temp &= ~WR;
	temp &= ~EN;
	temp |= BL;
	Enable(temp);
	temp = data;
	temp = temp << 4;
	temp |= RS;
	temp &= ~WR;
	temp &= ~EN;
	temp |= BL;
	Enable(temp);
	//osDelay(10);
}

void Enable(uint8_t en){
	uint8_t temp;
	temp = en;
	temp &= ~EN;
	temp |= BL;
	Byte_Write(temp);
	osDelay(1);
	temp |= EN;
	Byte_Write(temp);
	osDelay(1);
	temp &= ~EN;
	Byte_Write(temp);
}

void SetLine(uint8_t line){
	uint8_t temp;
	temp = line;
	//temp |= 0x80;
	LCD_Instruction_Write4(temp);
	osDelay(100);
}

void LCD_String(char* input){
	uint8_t length;
	length = strlen(input);
	//while (*input != 0){
	while (length != 0){
		LCD_Data_Write4(*input);
		input++;
		length--;
	}
}

void ClearScreen1(){
	LCD_Instruction_Write4(0x01);
}

void LCD_Service1(){
	Debug_Send("LCD1 service\r\n");
	uint8_t phase1;
	char tempstr[5];
	switch(phase){
	case 1:
		//LCD_String("test");
		//if (counter == 0 ) LCD_Instruction_Write4(0x01);
		SetLine(One);
		osDelay(1);
		myLongStr(counter,tempstr,10,10);
		counter++;

		strcpy(lcd,"Count - ");
		strcat(lcd,tempstr);
		LCD_String(lcd);
		LCD_String("Test Counter");
		phase = 2;
		break;
	case 2:
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		if (counter == 0) LCD_Instruction_Write4(0x01);
		SetLine(Two);
		LCD_String("test1");
		//SetLine(One);
		myLongStr(counter,tempstr,10,10);
		counter++;
		strcpy(lcd,"Count - ");
		strcat(lcd,tempstr);
		LCD_String(lcd);
		phase = 1;
		break;
	case 3:
		LCD_String("test1");
		phase = 4;
		break;
	case 4:
		SetLine(One);
		phase = 1;
		break;
	default:

		break;

	}
}
