/*
 * airHumidityTask.c
 *
 *  Created on: Sep 20, 2022
 *      Author: Abhinav Tyagi
 */

#include "farmTask.h"
#include "main.h"
#include <stdio.h>
#include "liquidcrystal_i2c.h"

extern xTaskHandle humidityTaskHandler;

/************************ Structure & Variable Declarations **************************************/
Sensor aHum;

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
float dhtHumidity = 0;

/*********************************** DHT11 FUNCTIONS ********************************************/

void DHT11_Start (void)
{
	Set_Pin_Output (DHT11_Port, DHT11_Pin);  // set the pin as output

	HAL_GPIO_WritePin(DHT11_Port, DHT11_Pin, 1); //initialize with data pin high
	HAL_Delay(1000); //wait for 1000 milliseconds

	HAL_GPIO_WritePin (DHT11_Port, DHT11_Pin, 0);   // pull the pin low
	DelayUS(18000);   // wait for 18ms
    HAL_GPIO_WritePin (DHT11_Port, DHT11_Pin, 1);   // pull the pin high
    DelayUS(20);   // wait for 20us
	Set_Pin_Input(DHT11_Port, DHT11_Pin);    // set as input
}

uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;
	DelayUS(40);
	if (!(HAL_GPIO_ReadPin (DHT11_Port, DHT11_Pin)))
	{
		DelayUS(80);
		if (HAL_GPIO_ReadPin (DHT11_Port, DHT11_Pin))
			Response = 1;
		else Response = -1; // 255
	}
	while (HAL_GPIO_ReadPin (DHT11_Port, DHT11_Pin));   // wait for the pin to go low

	return Response;
}

uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_Port, DHT11_Pin)));   // wait for the pin to go high
		DelayUS(40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_Port, DHT11_Pin)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while (HAL_GPIO_ReadPin (DHT11_Port, DHT11_Pin));  // wait for the pin to go low
	}
	return i;
}


/****************** TASK DEFINITIONS *********************************/

void humidity_Task (void *arg)					// DHT11 -Check Humidity in Air
{
	uint16_t RH;

	//float dhtTemp = 0;
//	while(1)
		{
		  DHT11_Start();
		  DHT11_Check_Response();

  		  Rh_byte1 = DHT11_Read ();
		  Rh_byte2 = DHT11_Read ();
		  Temp_byte1 = DHT11_Read ();
		  Temp_byte2 = DHT11_Read ();

//		  SUM = DHT11_Read();

//		  TEMP = Temp_byte1;
		  RH = Rh_byte1;

//		  Temperature = (float) TEMP;
//		  dhtHumidity = (float) RH;

		  aHum.id = '4';					// Battery ID
		  aHum.data = RH;					// Present Humidity in the Air
		  aHum.sep = '|';

		  /***** send to the queue ****/
		  xQueueSendToBack(sensorQHandler, &aHum, 100);
   // 	  xQueueSend(sensorQHandler, &aHum, 100);
   // 	  xQueueSend(loraQHandler, &aHum, portMAX_DELAY);
		}
}
