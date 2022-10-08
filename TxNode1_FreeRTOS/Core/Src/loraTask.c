/*
 * loraTask.c
 *
 *  Created on: Sep 20, 2022
 *      Author: Abhinav Tyagi
 */

#include "LoRa.h"
#include "farmTask.h"
#include "liquidcrystal_i2c.h"
#include "main.h"
#include <stdio.h>

extern xTaskHandle loraTaskHandler;
extern UART_HandleTypeDef huart1;

/****************** TASK DEFINITIONS *********************************/
// Transmit/Receive using LoRa
// Global Variables For LORA
LoRa myLoRa;
uint8_t read_data[128];
//uint8_t send_data[128];
//int	RSSI;


void lora_Task (void *arg)
{
	int lrBuffer[3];
	while(1)
	{
		//if (xQueueIsQueueFullFromISR(loraQHandler))
		{
			if(xQueueReceive(sensorQHandler, &lrBuffer, portMAX_DELAY) == pdPASS)
			{
				lcd_SetCursor(0,0);
				lcd_PrintStr("Data Received by LoRa");
				for(int x=0; x<20; x++)
				   {
					 lcd_ScrollDisplayLeft();
					 HAL_Delay(250);
				   }
			}
			else
			{
				lcd_PrintStr("Data NOT Received");
				for(int x=0; x<40; x++)
				   {
					 lcd_ScrollDisplayLeft();
					 HAL_Delay(500);
				   }
			}
		}

		// SENDING DATA - - - - - - - - - - - - - - - - - - - - - - - - -

		//send_data[0] = 0x3B; 					// MY ADDRESS
		//send_data[0] = 0xBB; 					// MY ADDRESS
		//send_data[0] = 0xFF; 					// MY ADDRESS

		for(int i=0; i<=6; i++)
		{
			LoRa_transmit(&myLoRa, (void *)lrBuffer, 128, 1000);
	//		HAL_UART_Transmit(&huart1, (uint8_t *)lrBuffer, 128, 1000);
		}

		HAL_Delay(1500);



		// RECEIVING DATA - - - - - - - - - - - - - - - - - - - - - - - -
				//*********************** SWITCH "ON/OFF" SOLENOID VALVE OF NODE_01.**************************

		LoRa_receive(&myLoRa, read_data, 128);
		if (read_data[0] == 1)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		}
	}
	vTaskDelete(NULL);
}
