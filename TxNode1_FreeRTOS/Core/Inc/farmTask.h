/*
 * farmTask.h
 *
 *  Created on: Sep 20, 2022
 *      Author: Abhinav Tyagi
 */

#include "liquidcrystal_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#define soilTemp_PORT GPIOC
#define soilTemp_PIN GPIO_PIN_6

#define airTemp_PORT GPIOC
#define airTemp_PIN GPIO_PIN_7

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
/************************ TASK FUNCTIONS **************************************/
typedef struct sensor{
	char id;
	int data;
	char sep;
}Sensor;

//int adcConvert(void);
//float adcConvert(int adc_PIN);

void lora_Task (void *arg);
void lcd_Task (void *arg);

void soilMoist_Task (void *arg);
void soilTemp_Task (void *arg);
void humidity_Task (void *arg);
void airTemp_Task (void *arg);
void battery_Task (void *arg);

/************************ QUEUE HANDLER **************************************/
extern xQueueHandle sensorQHandler;
//extern xQueueHandle loraQHandler;
extern ADC_HandleTypeDef hadc1;
//extern SemaphoreHandle_t bin_sem;     // Waits for parameter to be read
//extern SemaphoreHandle_t mutex;       // Lock access to Serial resource
/*****************************************************************************/


