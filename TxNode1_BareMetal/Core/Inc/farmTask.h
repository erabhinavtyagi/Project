/*
 * farmTask.h
 *
 *  Created on: Sep 20, 2022
 *      Author: Abhinav Tyagi
 */

#include "liquidcrystal_i2c.h"

#define soilTemp_PORT GPIOC
#define soilTemp_PIN GPIO_PIN_6

#define airTemp_PORT GPIOC
#define airTemp_PIN GPIO_PIN_7

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/************************ TASK FUNCTIONS **************************************/
//typedef struct sensor{
//	char id;
//	int data;
//	char sep;
//}Sensor;

void lora_Task ();
void lcd_Task ();

int soilMoist_Task ();
int soilTemp_Task ();
int humidity_Task ();
int airTemp_Task ();
int battery_Task ();

extern ADC_HandleTypeDef hadc1;
/*****************************************************************************/


