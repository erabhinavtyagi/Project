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

int moisture = 0;
float adcValue1 = 0;

//Sensor sMoist;
/****************** TASK DEFINITIONS *********************************/

int soilMoist_Task ()					// Check Soil Moisture
{
	char lcdBuffer[20];
	//while(1)
	{
		selectADC1();

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);			// Get Soil Moisture Sensor Reading
		adcValue1 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		moisture = 100-(adcValue1/2783)*100;				// Max Value of Sensor without water = 2783

		/***** Display on LCD ****/

		lcd_Clear();
		lcd_SetCursor(0,0);

		lcd_PrintStr("Soil Moisture");

		lcd_SetCursor(0,1);
		sprintf(lcdBuffer,"Value: %d ", moisture);
		lcd_PrintStr(lcdBuffer);
		lcd_PrintStr("%");
		HAL_Delay(2000);
	}
	return moisture;
}
