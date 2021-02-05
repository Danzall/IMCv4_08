/*
 * AE7953.c
 *
 *  Created on: Oct 29, 2019
 *      Author: Jeefo
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
//#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "GSM.h"
#include "LCD.h"
#include "LCD1.h"
#include "ADE7953.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_i2c.h"
/* USER CODE END Includes */

static void My_I2C_TransferConfig(I2C_HandleTypeDef *hi2c,  uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request);

ADE_State adeState = Current;
ADE_direction adeDir = Idle;
uint8_t I2C[10];
uint8_t I2Ct[10];
int recIndex;
extern I2C_HandleTypeDef hi2c1;
char temp1[20];
char LCD_1[40];
char LCD_2[20];
int dataValid = 0;
int volt;
int current;

extern HAL_StatusTypeDef I2C_Master_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);

void ADE_Service(){
	//Debug_Send("ADE service\r\n");
	/*ClearScreenF();
	LineSelect(0x80);
	LCD_Print("Start ADE service");*/

	switch(adeState){

	case InitalizeADE:
		adeState = ACCMODE;
		break;
	case ACCMODE:
		//ADE_Receive(0x201);
		//ADE_Receive(0x008);

		//ADE_Receive(0x102);
		//adeDir = Tx;
		//ADE_TxRx(0x102);


		strcpy(LCD_1,"Config: ");
		Debug_Send("ACCMODE\r\n");
		myLongStr(I2C[0],temp1,10,16);
		Debug_Send("1st byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");
		strcat(LCD_1,temp1);
		strcat(LCD_1," ");
		myLongStr(I2C[1],temp1,10,16);
		Debug_Send("2nd byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");
		strcat(LCD_1,temp1);
		/*strcat(LCD_1," ");
		myLongStr(I2C[2],temp1,10,10);
		Debug_Send("3rd byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");
		strcat(LCD_1,temp1);*/

		Debug_Send(LCD_1);
		Debug_Send("\r\n");
		adeState = Voltage;
		//adeState = Current;
		break;
	case Voltage:
		ADE_Receive(0x21F);
		strcpy(LCD_1,"Active: ");
		//Debug_Send("Active\r\n");
		myLongStr(I2C[0],temp1,10,16);
		volt = I2C[2];
		/*Debug_Send("1st byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");*/
		strcat(LCD_1,temp1);
		strcat(LCD_1," ");
		myLongStr(I2C[1],temp1,10,16);
		volt = volt + (256 * I2C[1]);
		/*Debug_Send("2nd byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");*/
		strcat(LCD_1,temp1);
		strcat(LCD_1," ");
		myLongStr(I2C[2],temp1,10,16);
		volt = volt + (65536 * I2C[2]);
		/*Debug_Send("3rd byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");*/
		strcat(LCD_1,temp1);
		strcat(LCD_1," ");

		ADE_Receive(0x223);
		strcpy(LCD_1,"Apparent: ");
		//Debug_Send("Apparent\r\n");
		myLongStr(I2C[0],temp1,10,16);
		/*Debug_Send("1st byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");*/
		strcat(LCD_1,temp1);
		strcat(LCD_1," ");
		myLongStr(I2C[1],temp1,10,16);
		/*Debug_Send("2nd byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");*/
		strcat(LCD_1,temp1);
		strcat(LCD_1," ");
		myLongStr(I2C[2],temp1,10,16);
		/*Debug_Send("3rd byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");*/
		strcat(LCD_1,temp1);
		strcat(LCD_1," ");

		/*ClearScreenF();
		LineSelect(0x80);
		LCD_Print(LCD_1);*/
		Debug_Send("Active ");
		Debug_Send(LCD_1);
		Debug_Send("\r\n");

		/*ClearScreen1();
		SetLine(One);
		LCD_String(LCD_1);

		LineSelect(0xC0);
		LCD_Print(LCD_2);*/
		Debug_Send("Apparent ");
		Debug_Send(LCD_1);
		Debug_Send("\r\n");

		/*SetLine(Two);
		LCD_String(LCD_2);*/

		/*Debug_Send(LCD_2);
		Debug_Send("\r\n");*/
		adeState = Current;
		//adeState = ACCMODE;
		//RelayOn();
		break;
	case Current:
		ADE_Receive(0x21C);
		strcpy(LCD_1,"Voltage: ");
		//Debug_Send("Voltage\r\n");
		myLongStr(I2C[0],temp1,10,16);
		volt = I2C[2];
		/*Debug_Send("1st byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");*/
		strcat(LCD_1,temp1);
		strcat(LCD_1," ");
		myLongStr(I2C[1],temp1,10,16);
		volt = volt + (256 * I2C[1]);
		/*Debug_Send("2nd byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");*/
		strcat(LCD_1,temp1);
		strcat(LCD_1," ");
		myLongStr(I2C[2],temp1,10,16);
		volt = volt + (65536 * I2C[0]);
		/*Debug_Send("3rd byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");*/
		strcat(LCD_1,temp1);
		strcat(LCD_1,"\r\n");

		ADE_Receive(0x21B);
		strcpy(LCD_2,"Current: ");
		//strcat(LCD_2,"Current: ");
		//Debug_Send("Current\r\n");
		myLongStr(I2C[0],temp1,10,16);

		/*Debug_Send("1st byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");*/
		strcat(LCD_2,temp1);
		strcat(LCD_2," ");
		myLongStr(I2C[1],temp1,10,16);
		//
		/*Debug_Send("2nd byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");*/
		strcat(LCD_2,temp1);
		strcat(LCD_2," ");
		current = I2C[2];
		current = current + (256 * I2C[1]);
		current = current + (65536 * I2C[0]);
		myLongStr(I2C[2],temp1,10,16);
		//
		/*Debug_Send("3rd byte: ");
		Debug_Send(temp1);
		Debug_Send("\r\n");*/
		strcat(LCD_2,temp1);
		strcat(LCD_2,"\r\n");
		//Debug_Send("LCD service\r\n");

		//sprintf(LCD_2, "v:%i c %i\r\n",volt, current);
		//Debug_Send(LCD_2);

		//if (dataValid == 1){
		/*ClearScreenF();
		LineSelect(0x80);
		LCD_Print(LCD_1);*/

		//ClearScreen1();
		//SetLine(One);
		//LCD_String(LCD_1);
		//Debug_Send("Voltage ");

		//Debug_Send(LCD_1);
		//Debug_Send("\r\n");

		/*LineSelect(0xC0);
		LCD_Print(LCD_2);*/

		//SetLine(Two);
		//LCD_String(LCD_2);
		//Debug_Send("Current ");
		//Debug_Send(LCD_2);
		//Debug_Send("\r\n");
		//}

		//Debug_Send(LCD_2);
		//adeState = Voltage;
		//adeState = ActivePower;
		//RelayOff();
		break;
	case ActivePower:
		ADE_Receive(0x21B);
		Debug_Send("ActivePower\r\n");
		//adeState = Voltage;
		break;
	case ReactivePower:

		Debug_Send("ReactivePower\r\n");
		adeState = Voltage;
		break;
	case ApparentPower:

		Debug_Send("ApparentPower\r\n");
		adeState = Voltage;
		break;
	}
}

int getVolt(){
	int temp;
	volt = volt /30000;
	return volt;
}

int getCurrent(){
	int temp6;
	if (current > 1200){
		current = current / 6;
	}
	else current = 0;
	return current;
}

HAL_StatusTypeDef ADE_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  uint32_t xfermode = 0U;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode        = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    //hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Master_ISR_IT;

    /*if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = I2C_RELOAD_MODE;
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      xfermode = I2C_AUTOEND_MODE;
    }*/

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
    My_I2C_TransferConfig(hi2c, DevAddress, hi2c->XferSize, xfermode, I2C_GENERATE_START_WRITE);

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, TXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
    //I2C_Enable_IRQ(hi2c, 0x00000001U);	//I2C_XFER_TX_IT
    __HAL_I2C_ENABLE_IT(hi2c, 0x0001);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

void ADE_TxRx(int addr){


	switch(adeDir){
	case Tx:
		I2C[0] = (addr & 0xFF00) >> 8;
		I2C[1] = addr & 0xFF;
		dataValid = 0;
		//HAL_I2C_Master_Sequential_Transmit_IT(&hi2c1, 0x70, I2C, 2, I2C_RELOAD_MODE);
		HAL_I2C_Master_Transmit_IT(&hi2c1, 0x70, I2C, 2);
		//ADE_Transmit_IT(&hi2c1, 0x70, I2C, 2);

		break;
	case Rx:
		//Debug_Send("TXRX\r\n");
		recIndex = 0;
		I2Ct[0] = 0x00;
		I2Ct[1] = 0x00;
		I2Ct[2] = 0x00;
		//HAL_I2C_Master_Sequential_Receive_IT(&hi2c1, 0x70, I2C, 3, I2C_AUTOEND_MODE);
		HAL_I2C_Master_Receive_IT(&hi2c1, 0x70, I2C, 3);
		//HAL_I2C_Master_Receive_IT(&hi2c1, 0x7c, I2C, 3);
		adeDir = Idle;

		break;
	case Idle:


		break;
	}
}

void i2cFlags(){

	uint32_t itsources = READ_REG(hi2c1.Instance->ISR);//I2C_FLAG_BUSY I2C_FLAG_AF I2C_FLAG_TXIS I2C_FLAG_RXNE I2C_ISR_TC
	if ((itsources&I2C_FLAG_BUSY) != I2C_FLAG_BUSY){	//received not busy
		/*HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		if (adeDir == Tx){
			adeDir = Rx;
			ADE_TxRx(I2C);
		}*/
	}
	if ((itsources&I2C_ISR_TC) == I2C_ISR_TC){	//still busy, transfer complete
		Debug_Send("RX\r\n");
		adeDir = Rx;
		ADE_TxRx(0x109);
	}

	if ((itsources&I2C_ISR_TCR) == I2C_ISR_TCR){
		Debug_Send("Reload\r\n");

		if (adeDir == Tx){
			//adeDir = Rx;
			//hi2c1.State = HAL_I2C_STATE_READY;
			//ADE_TxRx(0x7c);
		}
	}
	if ((itsources&I2C_FLAG_RXNE) == I2C_FLAG_RXNE){
		//Debug_Send("Recv\r\n");
		//I2Ct[recIndex] = hi2c1.Instance->RXDR;
		//I2Ct[recIndex] = I2C[recIndex];
		//recIndex++;
	}
	if ((itsources&I2C_FLAG_AF) == I2C_FLAG_AF){		//received NACK
		Debug_Send("No ACK\r\n");
	}
	if ((itsources&I2C_FLAG_STOPF) == I2C_FLAG_STOPF){
		//Debug_Send("Stop\r\n");
		dataValid = 1;
		//__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);
		//MX_I2C1_Init();
	}

	if ((itsources&I2C_FLAG_OVR) == I2C_FLAG_OVR){
		Debug_Send("OVR\r\n");
		hi2c1.State = HAL_I2C_STATE_READY;
		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_OVR);
	}
	if ((itsources&I2C_FLAG_ARLO) == I2C_FLAG_ARLO){
		Debug_Send("Abr\r\n");
		hi2c1.State = HAL_I2C_STATE_READY;
		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_ARLO);
	}


	/*myLongStr(itsources,temp1,10,10);
	Debug_Send("int: ");
	Debug_Send(temp1);
	Debug_Send("\r\n");*/
	/*uint32_t itsources1 = READ_REG(hi2c1.Instance->CR1);
	myLongStr(itsources1,temp1,10,10);
	Debug_Send("intm: ");
	Debug_Send(temp1);
	Debug_Send("\r\n");*/
}

void TxDone(){
	adeDir = Rx;
	ADE_TxRx(0x7c);
}

void ADE_Receive(int addr){
	I2C[0] = (addr & 0xFF00) >> 8;
	I2C[1] = addr & 0xFF;
	//I2C[0] = 0x00;
	//I2C[1] = 0x08;
	//I2C[2] = 0xA0;
	//HAL_I2C_Master_Transmit(&hi2c1, 0x70, I2C, 3, 10);
	//HAL_I2C_Master_Receive(&hi2c1, 0x7C, I2C, 2, 10);
	//Debug_Send("Send I2C\r\n");
	ADE_Rx(0x70, I2C, 3);

	//ADE_TxRx(0x102);
}

uint8_t* ADE_Get(){
	return I2C;
}


uint32_t ADE_Rx(uint16_t DevAddress, uint8_t *pData, uint8_t size){
	//uint32_t tickstart = 0U;
	/* Process Locked */
	__HAL_LOCK(&hi2c1);
	uint16_t timeout;

	timeout = HAL_GetTick();

	while (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == SET){	//wait for busy flag to reset
		//HAL_GetTick();
		//if  (((HAL_GetTick() - timeout) > 4) || (HAL_GetTick() < timeout)) return 0;
	}

	hi2c1.State     = HAL_I2C_STATE_BUSY_TX;
	hi2c1.Mode      = HAL_I2C_MODE_MASTER;
	hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;

	/* Prepare transfer parameters */
	hi2c1.pBuffPtr  = pData;
	hi2c1.XferCount = size;
	hi2c1.XferCount = 2;
	hi2c1.XferISR   = NULL;

    hi2c1.XferSize = hi2c1.XferCount;
    My_I2C_TransferConfig(&hi2c1, DevAddress, hi2c1.XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_WRITE);

    //while (__HAL_I2C_GET_FLAG(hi2c1, I2C_FLAG_TXIS) == RESET);

    while (hi2c1.XferCount > 0U)
    {
      /* Wait until TXIS flag is set */

      /*if (I2C_WaitOnTXISFlagUntilTimeout(hi2c1, Timeout, tickstart) != HAL_OK)
      {

    	  if (hi2c1.ErrorCode == HAL_I2C_ERROR_AF)
        {

          return HAL_ERROR;
        }
        else
        {
        	HAL_GPIO_TogglePin(Relay_Open_GPIO_Port, Relay_Open_Pin);
          return HAL_TIMEOUT;
        }
      }*/
    	timeout = HAL_GetTick();
      while (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_TXIS) == RESET){
    	  /*if  (((HAL_GetTick() - timeout) > 4) || (HAL_GetTick() < timeout)){
			  I2C_Timeout();
			  return 0;
		  }*/
      }
    	//I2C_WaitOnTXISFlagUntilTimeout(&hi2c1, 5, 0);
    	/*timeout = HAL_GetTick();
      while (__HAL_I2C_GET_FLAG(&hi2c1, I2C_ISR_TXE) == RESET){		//wait till data buffer is empty
    	  if  (((HAL_GetTick() - timeout) > 4) || (HAL_GetTick() < timeout)){
    		  I2C_Timeout();
    		  return 0;
    	  }
      }*/

      /* Write data to TXDR */
      hi2c1.Instance->TXDR = (*hi2c1.pBuffPtr++);
      hi2c1.XferCount--;
      hi2c1.XferSize--;


      /*if ((hi2c1.XferSize == 0U) && (hi2c1.XferCount != 0U))
      {
        /* Wait until TCR flag is set */
        /*if (I2C_WaitOnFlagUntilTimeout(hi2c1, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }*/
        /*while (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_TCR) == RESET);
        if (hi2c1.XferCount > 255)
        {
          hi2c1.XferSize = 255;
          My_I2C_TransferConfig(&hi2c1, DevAddress, hi2c1.XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
        }
        else
        {
          hi2c1.XferSize = hi2c1.XferCount;
          My_I2C_TransferConfig(&hi2c1, DevAddress, hi2c1.XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
        }
      }*/

    }

    /*My I2C receive Function*/
    /*if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }*/
    timeout = HAL_GetTick();
    //while (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == SET);	//wait for busy flag to reset
    while (__HAL_I2C_GET_FLAG(&hi2c1, I2C_ISR_TXE) == RESET){	//wait till data buffer is empty
    	/*if  (((HAL_GetTick() - timeout) > 4) || (HAL_GetTick() < timeout)){
    		I2C_Timeout();
    		return 0;
    	}*/
    }

    hi2c1.State     = HAL_I2C_STATE_BUSY_RX;
    hi2c1.Mode      = HAL_I2C_MODE_MASTER;
    hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c1.pBuffPtr  = pData;
    hi2c1.XferCount = size;
    hi2c1.XferISR   = NULL;

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    if (hi2c1.XferCount > 255)
    {
      hi2c1.XferSize = 255;
      My_I2C_TransferConfig(&hi2c1, DevAddress, hi2c1.XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_READ);
    }
    else
    {
      hi2c1.XferSize = hi2c1.XferCount;
      My_I2C_TransferConfig(&hi2c1, DevAddress, hi2c1.XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);
    }
    //hi2c->Instance->TXDR = 0x00;		//instruction reg
    while (hi2c1.XferCount > 0U)
    {
      /* Wait until RXNE flag is set */
      /*if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
      {
        if (hi2c->ErrorCode == HAL_I2C_ERROR_AF)
        {
          return HAL_ERROR;
        }
        else
        {
          return HAL_TIMEOUT;
        }
      }*/
    	timeout = HAL_GetTick();
      while (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_RXNE) == RESET){	//wait for rxne flag to set
    	  //if  (((HAL_GetTick() - timeout) > 4) || (HAL_GetTick() < timeout)) return 0;
      }

      /* Read data from RXDR */
      (*hi2c1.pBuffPtr++) = hi2c1.Instance->RXDR;
      hi2c1.XferSize--;
      hi2c1.XferCount--;

      if ((hi2c1.XferSize == 0U) && (hi2c1.XferCount != 0U))
      {
        /* Wait until TCR flag is set */
        /*if (I2C_WaitOnFlagUntilTimeout(hi2c1, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }*/
    	  timeout = HAL_GetTick();
        while (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_TCR) == RESET){
        	//if  (((HAL_GetTick() - timeout) > 4) || (HAL_GetTick() < timeout))  return 0;
        }

        if (hi2c1.XferCount > 255)
        {
          hi2c1.XferSize = 255;
          My_I2C_TransferConfig(&hi2c1, DevAddress, hi2c1.XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
        }
        else
        {
          hi2c1.XferSize = hi2c1.XferCount;
          My_I2C_TransferConfig(&hi2c1, DevAddress, hi2c1.XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
        }
      }
    }
    //Debug_Send("ADE end\r\n");


    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
	/* Wait until STOPF flag is set */
	/*if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
	{
	  if (hi2c->ErrorCode == HAL_I2C_ERROR_AF)
	  {
		return HAL_ERROR;
	  }
	  else
	  {
		return HAL_TIMEOUT;
	  }
	}*/
	while (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_STOPF) == RESET);

	/* Clear STOP Flag */
	__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);



	/* Clear Configuration Register 2 */
	I2C_RESET_CR2(&hi2c1);

	hi2c1.State = HAL_I2C_STATE_READY;
	hi2c1.Mode  = HAL_I2C_MODE_NONE;

	/* Process Unlocked */
	__HAL_UNLOCK(&hi2c1);

	return 0;
}

void I2C_Timeout(){
	Debug_Send("ADE timeout\r\n");
	/* Clear STOP Flag */
	__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);

	__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_TXE);

	/* Clear Configuration Register 2 */
	I2C_RESET_CR2(&hi2c1);

	hi2c1.State = HAL_I2C_STATE_READY;
	hi2c1.Mode  = HAL_I2C_MODE_NONE;

	/* Process Unlocked */
	__HAL_UNLOCK(&hi2c1);
}

static void My_I2C_TransferConfig(I2C_HandleTypeDef *hi2c,  uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request)
{
	uint32_t tmpreg = 0U;

	/* Check the parameters */
	assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));
	assert_param(IS_TRANSFER_MODE(Mode));
	assert_param(IS_TRANSFER_REQUEST(Request));

	/* Get the CR2 register value */
	tmpreg = hi2c->Instance->CR2;

	/* clear tmpreg specific bits */
	tmpreg &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP));

	/* update tmpreg */
	tmpreg |= (uint32_t)(((uint32_t)DevAddress & I2C_CR2_SADD) | (((uint32_t)Size << 16) & I2C_CR2_NBYTES) | \
					   (uint32_t)Mode | (uint32_t)Request);

	/* update CR2 register */
	hi2c->Instance->CR2 = tmpreg;
}
