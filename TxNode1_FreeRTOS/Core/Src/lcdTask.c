/*
 * lcdTask.c
 *
 *  Created on: Sep 20, 2022
 *      Author: Abhinav Tyagi
 */

#include "farmTask.h"
#include "main.h"
#include "liquidcrystal_i2c.h"
#include <stdio.h>

extern xTaskHandle lcdTaskHandler;

/****************** TASK DEFINITIONS *********************************/

void lcd_Task (void *arg)						// Display Values on LCD
{
	int buffer[30];
	char lcdbuffer[20];
	while(1)
	{
		/**** RECEIVE FROM QUEUE *****/
	//	if (xQueueIsQueueFullFromISR(sensorQHandler))
		{
			if(xQueuePeek(sensorQHandler, &buffer, portMAX_DELAY) == pdPASS)
			{
				lcd_Clear();
				HAL_Delay(1000);
				lcd_SetCursor(0,0);
				lcd_PrintStr("Farm Node01");
				lcd_SetCursor(0,1);
				lcd_PrintStr("Data Received");
				HAL_Delay(2000);
				{
					lcd_Clear();
					lcd_SetCursor(0,0);

					sprintf(lcdbuffer,"ID: %c", buffer[0]);
					lcd_PrintStr(lcdbuffer);

					lcd_SetCursor(0,1);
					sprintf(lcdbuffer,"Value: %d", buffer[1]);
					lcd_PrintStr(lcdbuffer);
					HAL_Delay(2000);

//					for(int x=0; x<10; x=x+1)
//				    {
//					 lcd_ScrollDisplayLeft();
//					 HAL_Delay(500);
//				    }
				}
			}
			else
			{
				lcd_PrintStr("Data NOT Received");
				for(int x=0; x<40; x=x+1)
				   {
					 lcd_ScrollDisplayLeft();
					 HAL_Delay(500);
				   }
			}
			vTaskDelay(pdMS_TO_TICKS(1000));
		 }
	vTaskDelete(NULL);
	}
}
