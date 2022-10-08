/*
 * soilTempTask.c
 *
 *  Created on: Sep 20, 2022
 *      Author: Abhinav Tyagi
 */

#include "farmTask.h"
#include "main.h"
#include <stdio.h>
#include "liquidcrystal_i2c.h"
extern xTaskHandle soilTempTaskHandler;

/************************ Structure & Variable Declarations **************************************/
Sensor sTemp;

float soilTemp = 0;
uint8_t soil_byte1, soil_byte2;
/*********************************** DS18B20 FUNCTIONS ****************************************/
uint8_t DS18B20_Start1 (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(soilTemp_PORT, soilTemp_PIN);   // set the pin as output
	HAL_GPIO_WritePin (soilTemp_PORT, soilTemp_PIN, 0);  // pull the pin low
	DelayUS(480);   // delay according to datasheet

	Set_Pin_Input(soilTemp_PORT, soilTemp_PIN);    // set the pin as input
	DelayUS (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (soilTemp_PORT, soilTemp_PIN)))
		Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	DelayUS(400); // 480 us delay totally.

	return Response;
}

void DS18B20_Write1 (uint8_t data)
{
	Set_Pin_Output(soilTemp_PORT, soilTemp_PIN);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1
			Set_Pin_Output(soilTemp_PORT, soilTemp_PIN);  // set as output
			HAL_GPIO_WritePin (soilTemp_PORT, soilTemp_PIN, 0);  // pull the pin LOW
			DelayUS(1);  // wait for 1 us

			Set_Pin_Input(soilTemp_PORT, soilTemp_PIN);  // set as input
			DelayUS(50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(soilTemp_PORT, soilTemp_PIN);
			HAL_GPIO_WritePin (soilTemp_PORT, soilTemp_PIN, 0);  // pull the pin LOW
			//delay (50);  // wait for 60 us
			DelayUS(50);
			Set_Pin_Input(soilTemp_PORT, soilTemp_PIN);
		}
	}
}

uint8_t DS18B20_Read1 (void)
{
	uint8_t value=0;

	Set_Pin_Input(soilTemp_PORT, soilTemp_PIN);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(soilTemp_PORT, soilTemp_PIN);   // set as output

		HAL_GPIO_WritePin (soilTemp_PORT, soilTemp_PIN, 0);  // pull the data pin LOW
		DelayUS(1);
		//delay (1);  // wait for > 1us

		Set_Pin_Input(soilTemp_PORT, soilTemp_PIN);  // set as input
		if (HAL_GPIO_ReadPin (soilTemp_PORT, soilTemp_PIN))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		DelayUS(50);  // wait for 60 us
	}
	return value;
}
/* USER CODE END 0 */


/****************** TASK DEFINITIONS *********************************/

void soilTemp_Task (void *arg)					// Check Soil Temperature
{
	uint16_t TEMP;

//	while(1)
	{
        DS18B20_Start1();
        HAL_Delay (1);
	    DS18B20_Write1 (0xCC);  					// skip ROM
	    DS18B20_Write1 (0x44);  					// convert t
	    HAL_Delay (800);

	    DS18B20_Start1();
	    HAL_Delay(1);
	    DS18B20_Write1 (0xCC);  					// skip ROM
	    DS18B20_Write1 (0xBE);  					// Read Scratch-pad

	    soil_byte1 = DS18B20_Read1();
	    soil_byte2 = DS18B20_Read1();
	    TEMP = (soil_byte2<<8)|soil_byte1;
	    soilTemp = (float)TEMP/16;

		/***** LOAD THE DATA ****/
		sTemp.id = '2';							// Soil Temperature ID
		sTemp.data = soilTemp;				// Soil Temperature Data
		sTemp.sep = '|';

		/***** send to the queue ****/
		xQueueSendToBack(sensorQHandler, &sTemp, 100);
//		xQueueSend(sensorQHandler, &sTemp, 100);
//		xQueueSend(loraQHandler, &sTemp, portMAX_DELAY);

//		BaseType_t res = xQueueSend(sensorQHandler, &sTemp, portMAX_DELAY);
//		if(res != pdPASS)
//		{
//			lcd_SetCursor(0,0);
//			lcd_PrintStr("Soil Temperature not Sent.");
//			for(int x=0; x<40; x=x+1)
//			   {
//				 lcd_ScrollDisplayLeft();
//				 HAL_Delay(500);
//			   }
//		}
	  vTaskDelay(pdMS_TO_TICKS(1000));
	}
	vTaskDelete(NULL);
}
