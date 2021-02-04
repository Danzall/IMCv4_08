/*
 * LCD.h
 *
 *  Created on: Feb 12, 2020
 *      Author: Jeefo
 */

#ifndef LCD1_H_
#define LCD1_H_

void LCD_Init1();
void Byte_Write(uint8_t data);
void LCD_Instruction_Write(uint8_t data);
void LCD_Instruction_Write4(uint8_t data);
void LCD_Data_Write(uint8_t data);
void LCD_Data_Write4(uint8_t data);
void Enable(uint8_t en);
void SetLine(uint8_t line);
void LCD_String(char* input);
void LCD_Service1();
void ClearScreen1();

#define One 0x80
#define Two 0xC0
#endif /* LCD_H_ */
