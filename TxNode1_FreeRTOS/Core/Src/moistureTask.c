/*
 * moistureTask.c
 *
 *  Created on: Sep 20, 2022
 *      Author: Abhinav Tyagi
 */


#include "farmTask.h"
#include "main.h"
#include <stdio.h>
#include "liquidcrystal_i2c.h"

extern xTaskHandle soilMoistTaskHandler;
int moisture = 0;
float adcValue1 = 0;
/************************ Structure & Variable Declarations **************************************/

//struct sensor{
//	char id;
//	float data;
//	char sep;
//}sMoist;

Sensor sMoist;
/****************** TASK DEFINITIONS *********************************/

void soilMoist_Task (void *arg)					// Check Soil Moisture
{
	//while(1)
	{
		selectADC1();

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);			// Get Soil Moisture Sensor Reading
		adcValue1 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		moisture = 100-(adcValue1/2783)*100;				// Max Value of Sensor without water = 2783

		sMoist.id = '1';
		sMoist.data = moisture;				// As Voltage is inversely proportional to Capacitance Hence Value subtracted from 100.
		sMoist.sep ='|';

		/***** send to the queue ****/
		xQueueSend(sensorQHandler, &sMoist, 100);

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	vTaskDelete(NULL);
}
