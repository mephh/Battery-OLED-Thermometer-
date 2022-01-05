/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "i2c.h"
#include "sht3x.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float temp, hum;

sht3x_handle_t handle = { .i2c_handle = &hi2c3, .device_address =
		SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW };
/* USER CODE END Variables */
osThreadId InitDevicesTaskHandle;
osThreadId ReadSHTTaskHandle;
osThreadId DisplayDataTaskHandle;
osThreadId StoreSDHandle;
osSemaphoreId i2cSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void InitDevices_task(void const * argument);
void ReadSHT_task(void const * argument);
void DisplayData_task(void const * argument);
void StoreSD_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	 called if a stack overflow is detected. */
	for (int var = 0; var < 20; ++var) {
					HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
					osDelay(900);
					HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
					osDelay(900);
				}
}
/* USER CODE END 4 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of i2cSem */
  osSemaphoreDef(i2cSem);
  i2cSemHandle = osSemaphoreCreate(osSemaphore(i2cSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of InitDevicesTask */
  osThreadDef(InitDevicesTask, InitDevices_task, osPriorityRealtime, 0, 128);
  InitDevicesTaskHandle = osThreadCreate(osThread(InitDevicesTask), NULL);

  /* definition and creation of ReadSHTTask */
  osThreadDef(ReadSHTTask, ReadSHT_task, osPriorityLow, 0, 128);
  ReadSHTTaskHandle = osThreadCreate(osThread(ReadSHTTask), NULL);

  /* definition and creation of DisplayDataTask */
  osThreadDef(DisplayDataTask, DisplayData_task, osPriorityHigh, 0, 512);
  DisplayDataTaskHandle = osThreadCreate(osThread(DisplayDataTask), NULL);

  /* definition and creation of StoreSD */
  osThreadDef(StoreSD, StoreSD_task, osPriorityLow, 0, 256);
  StoreSDHandle = osThreadCreate(osThread(StoreSD), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_InitDevices_task */
/**
 * @brief  Function implementing the InitDevicesTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_InitDevices_task */
void InitDevices_task(void const * argument)
{
  /* USER CODE BEGIN InitDevices_task */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		ssd1306_Init();
//		 Create the handle for the sensor.
		if (!sht3x_init(&handle)) {
		    printf("SHT3x access failed.\n\r");
		    for (int var = 0; var < 50; ++var) {
				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
				osDelay(50);
				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
				osDelay(50);
			}
		}
//		ssd1306_TestAll();
//		ssd1306_Fill(0x00);
//		osThreadSuspend(InitDevicesTaskHandle);
		osThreadTerminate(InitDevicesTaskHandle);
//    osDelay(1);
	}
  /* USER CODE END InitDevices_task */
}

/* USER CODE BEGIN Header_ReadSHT_task */
/**
 * @brief Function implementing the ReadSHTTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ReadSHT_task */
void ReadSHT_task(void const * argument)
{
  /* USER CODE BEGIN ReadSHT_task */

	/* Infinite loop */
	for (;;) {
		//read sensor data and store values
		osSemaphoreWait(i2cSemHandle, osWaitForever);
		sht3x_read_temperature_and_humidity(&handle, &temp, &hum);
		osSemaphoreRelease(i2cSemHandle);
		osDelay(3000);
	}
  /* USER CODE END ReadSHT_task */
}

/* USER CODE BEGIN Header_DisplayData_task */
/**
 * @brief Function implementing the DisplayDataTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_DisplayData_task */
void DisplayData_task(void const * argument)
{
  /* USER CODE BEGIN DisplayData_task */
	char temp_text[11];
	char hum_text[11];
	uint8_t indx = 0;
	/* Infinite loop */
	for (;;) {
		snprintf(temp_text, sizeof(temp_text), "Temp: %.2f", temp);
		snprintf(hum_text, sizeof(hum_text), "Hum:  %.2f", hum);
		osSemaphoreWait(i2cSemHandle, osWaitForever);
		ssd1306_SetCursor(3, 3);
		ssd1306_WriteString(temp_text, Font_11x18, White);
		ssd1306_SetCursor(3, 45);
		ssd1306_WriteString(hum_text, Font_11x18, White);
		ssd1306_DrawRectangle(0, 1, 127, 63, White);
		if (indx != 0){
			ssd1306_DrawPixel(125, indx+4, Black);
		}
		ssd1306_DrawPixel(125, indx+5, White);
		ssd1306_UpdateScreen();
		osSemaphoreRelease(i2cSemHandle);
		indx++;
		indx = indx%5;
		osDelay(100);
	}
  /* USER CODE END DisplayData_task */
}

/* USER CODE BEGIN Header_StoreSD_task */
/**
* @brief Function implementing the StoreSD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StoreSD_task */
void StoreSD_task(void const * argument)
{
  /* USER CODE BEGIN StoreSD_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StoreSD_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
