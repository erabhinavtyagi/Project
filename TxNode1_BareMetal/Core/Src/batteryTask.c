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

float adcValue2 = 0;
uint8_t voltage = 0;

/****************** TASK DEFINITIONS *********************************/

int battery_Task ()					// Check Battery Level at Node1
{
	char lcdBuffer[20];
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

		/***** Display on LCD ****/

		lcd_Clear();
		lcd_SetCursor(0,0);
		lcd_PrintStr("Battery Voltage");
		lcd_SetCursor(0,1);
		sprintf(lcdBuffer,"Value: %d V", voltage);
		lcd_PrintStr(lcdBuffer);
		HAL_Delay(2000);
	}
	return voltage;
}
