/*
 * airTempTask.c
 *
 *  Created on: Sep 20, 2022
 *      Author: Abhinav Tyagi
 */

#include "farmTask.h"
#include "main.h"
#include <stdio.h>
#include "liquidcrystal_i2c.h"

/************************ Variable Declarations **************************************/

uint8_t airTemp = 0;
uint8_t airTemp_byte1, airTemp_byte2;

/*********************** SELECT PINS FOR DS18B20 & DHT ************************************/

void Set_Pin_Output1 (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input1 (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/*********************************** DS18B20 FUNCTIONS ****************************************/
uint8_t DS18B20_Start2 (void)
{
	uint8_t Response = 0;
	Set_Pin_Output1(airTemp_PORT, airTemp_PIN);  		 	// set the pin as output
	HAL_GPIO_WritePin (airTemp_PORT, airTemp_PIN, 0);  		// pull the pin low
	DelayUS(480);   										// delay according to datasheet

	Set_Pin_Input1(airTemp_PORT, airTemp_PIN);    			// set the pin as input
	DelayUS (80);    										// delay according to datasheet

	if (!(HAL_GPIO_ReadPin (airTemp_PORT, airTemp_PIN)))
		Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	DelayUS(480); // 480 us delay totally.

	return Response;
}

void DS18B20_Write2 (uint8_t data)
{
	Set_Pin_Output1(airTemp_PORT, airTemp_PIN);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1
			Set_Pin_Output1(airTemp_PORT, airTemp_PIN);  // set as output
			HAL_GPIO_WritePin (airTemp_PORT, airTemp_PIN, 0);  // pull the pin LOW
			DelayUS(1);  // wait for 1 us

			Set_Pin_Input1(airTemp_PORT, airTemp_PIN);  // set as input
			DelayUS(50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0
			Set_Pin_Output1(airTemp_PORT, airTemp_PIN);
			HAL_GPIO_WritePin (airTemp_PORT, airTemp_PIN, 0);  // pull the pin LOW
			DelayUS(50);	// wait for 60 us
			Set_Pin_Input1(airTemp_PORT, airTemp_PIN);
		}
	}
}

uint8_t DS18B20_Read2 (void)
{
	uint8_t value=0;

	Set_Pin_Input1(airTemp_PORT, airTemp_PIN);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output1(airTemp_PORT, airTemp_PIN);  		// set as output

		HAL_GPIO_WritePin (airTemp_PORT, airTemp_PIN, 0);  // pull the data pin LOW
		DelayUS(1);
		//delay (1);  // wait for > 1us

		Set_Pin_Input1(airTemp_PORT, airTemp_PIN);  			// set as input
		if (HAL_GPIO_ReadPin (airTemp_PORT, airTemp_PIN))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		DelayUS(50); 		// wait for 60 us
	}
	return value;
}


/****************** TASK DEFINITIONS *********************************/

int airTemp_Task ()					// Check Soil Temperature
{
	uint8_t TEMP = 0;
	char lcdBuffer[20];

//	while(1)
	{
        DS18B20_Start2();
        HAL_Delay (1);
	    DS18B20_Write2 (0xCC);  					// skip ROM
	    DS18B20_Write2 (0x44);  					// convert t
	    HAL_Delay (800);

	    DS18B20_Start2();
	    HAL_Delay(1);
	    DS18B20_Write2 (0xCC);  					// skip ROM
	    DS18B20_Write2 (0xBE);  					// Read Scratch-pad

	    airTemp_byte1 = DS18B20_Read2();
	    airTemp_byte2 = DS18B20_Read2();
	    TEMP = (airTemp_byte2<<8)|airTemp_byte1;
	    airTemp = TEMP/16;

	    /***** Display on LCD ****/

	    lcd_Clear();
	    lcd_SetCursor(0,0);
	    lcd_PrintStr("Air Temperature");
	    lcd_SetCursor(0,1);
	    sprintf(lcdBuffer,"Value: %d C", airTemp);
	    lcd_PrintStr(lcdBuffer);
	    HAL_Delay(2000);
	}
	return airTemp;
}
