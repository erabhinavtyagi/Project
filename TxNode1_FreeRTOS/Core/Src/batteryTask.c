/*
 * batteryTask.c
 *
 *  Created on: Sep 20, 2022
 *      Author: Abhinav Tyagi
 */

#include "farmTask.h"
#include "main.h"

#include <stdio.h>
#include "liquidcrystal_i2c.h"

extern xTaskHandle batteryTaskHandler;

float adcValue2 = 0;
float voltage = 0;

/************************ Structure & Variable Declarations **************************************/

//struct sensor{
//	char id;
//	float data;
//	char sep;
//}bVol;
Sensor bVol;
/****************** TASK DEFINITIONS *********************************/

void battery_Task (void *arg)					// Check Battery Level at Node1
{
//	while(1)
	{
		selectADC2();

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);			// Get Soil Moisture Sensor Reading
		adcValue2 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		/********** LOAD THE DATA ***********/
		voltage = (adcValue2/4095)*16.5;		//for 3.3v systems : 3.3*5  = 16.5
	//	voltage = (adcValue2/4095)*25;
		bVol.id = '5';							// Battery ID
		bVol.data = voltage;					// Present Battery Voltage
		bVol.sep = '|';

		/***** send to the queue ****/
		xQueueSendToBack(sensorQHandler, &bVol, 100);
	//	xQueueSend(sensorQHandler, &bVol, 100);
	//	xQueueSend(loraQHandler, &bVol, portMAX_DELAY);

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	vTaskDelete(NULL);
}
