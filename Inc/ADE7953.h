/*
 * ADE7953.h
 *
 *  Created on: Oct 29, 2019
 *      Author: Jeefo
 */

#ifndef ADE7953_H_
#define ADE7953_H_

typedef enum{
	InitalizeADE = 1,
	ACCMODE,
	Voltage,
	Current,
	ActivePower,
	ReactivePower,
	ApparentPower
} ADE_State;

typedef enum{
	Tx = 1,
	Rx,
	Idle
} ADE_direction;

void ADE_Service();
void ADE_Receive(int addr);
uint8_t* ADE_Get();
uint32_t ADE_Rx(uint16_t DevAddress, uint8_t *pData, uint8_t size);
void I2C_Timeout();
void i2cFlags();
//void ADE_TxRx();
void ADE_TxRx(int addr);
void TxDone();
HAL_StatusTypeDef ADE_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
int getVolt();
int getCurrent();
#endif /* ADE7953_H_ */
