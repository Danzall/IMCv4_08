/*
 * LCD.h
 *
 *  Created on: Feb 12, 2020
 *      Author: Jeefo
 */

#ifndef LCD_H_
#define LCD_H_

void LCD_Init();
void Byte_Write(uint8_t data);
void LCD_Instruction_Write(uint8_t data);
void LCD_Instruction_Write4(uint8_t data);
void LCD_Data_Write(uint8_t data);
void LCD_Data_Write4(uint8_t data);
void Enable(uint8_t en);
void SetLine(uint8_t line);
void LCD_String(char* input);

#define One 0x80
#define Two 0xC0
#endif /* LCD_H_ */
