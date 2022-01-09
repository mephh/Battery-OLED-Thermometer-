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
#include "usart.h"
#include <stdbool.h>

#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
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

sht3x_handle_t handle = { .i2c_handle = &hi2c1, .device_address =
		SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW };

FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; //Result after operations
char logfileName[] = "LOGDATA.TXT";
/* USER CODE END Variables */
osThreadId InitDevicesTaskHandle;
osThreadId ReadSHTTaskHandle;
osThreadId DisplayDataTaskHandle;
osThreadId StoreSDHandle;
osSemaphoreId i2cSemHandle;
osSemaphoreId spiSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void myprintf(const char *fmt, ...);
void GetSdProperties(void);
void CreateSDCardLogFile(void);
void UpdateLogFile(void);
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
		osDelay(2000);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		osDelay(2000);
	}
}

void myprintf(const char *fmt, ...) {
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&huart2, (uint8_t*) buffer, len, -1);

}

void GetSdProperties(void) {
	//some variables for FatFs
	FATFS FatFs; 	//Fatfs handle
	FIL fil; 		//File handle
	FRESULT fres; //Result after operations

	//Open the file system
	fres = f_mount(&FatFs, "", 1); //1=mount now
	if (fres != FR_OK) {
		while (1) {
			myprintf("f_mount error (%i)\r\n", fres);
		}
	}

	//		//Let's get some statistics from the SD card
	DWORD free_clusters, free_sectors, total_sectors;
	FATFS *getFreeFs;
	fres = f_getfree("", &free_clusters, &getFreeFs);
	if (fres != FR_OK) {
		while (1) {
			myprintf("f_getfree error (%i)\r\n", fres);
		}
	}

	//Formula comes from ChaN's documentation
	total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
	free_sectors = free_clusters * getFreeFs->csize;
	myprintf(
			"SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n",
			total_sectors / 2, free_sectors / 2);
}

void CreateSDCardLogFile(void) {
	//Open the file system
	osSemaphoreWait(spiSemHandle, osWaitForever);
	fres = f_mount(&FatFs, "/", 1); //1=mount now
	if (fres != FR_OK) {
		while (1) {
			myprintf("f_mount error (%i)\r\n", fres);
		}
	}
	//create file
	fres = f_open(&fil, logfileName,
			FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
	if (fres == FR_OK) {
		myprintf("I was able to open 'LogData.txt' for writing\r\n");
	} else {
		myprintf("f_open error during file creation (%i)\r\n", fres);
	}

	//Close your file!
	f_close(&fil);
	osSemaphoreRelease(spiSemHandle);
}

void UpdateLogFile(void) {
	//some variables for FatFs

	BYTE sensorData[30];
	memset(sensorData, 0, sizeof sensorData);
	UINT bytesWrote;
	osSemaphoreWait(spiSemHandle, osWaitForever);
	//store sht data
	snprintf((char*) sensorData, sizeof(sensorData), "Temp:%.3f | Hum:%.3f\n",
			temp, hum);
	fres = f_mount(&FatFs, "/", 1); //1=mount now
	if (fres != FR_OK) {
//		while (1) {
		myprintf("f_mount error (%i)\r\n", fres);
//		}
	}
	fres = f_open(&fil, logfileName, FA_OPEN_APPEND | FA_WRITE);
	if (fres != FR_OK) {
		myprintf("can't open file to append. Err code:%i", fres);
	}
	myprintf("file to store in:%s", &logfileName);
	fres = f_write(&fil, sensorData, 30, &bytesWrote);
	if (fres == FR_OK) {
		myprintf("Wrote %i bytes to 'logdata.txt'!\r\n", bytesWrote);
	} else {
		myprintf(
				"f_write error during sht data storing (err code:%i | byteswritten:%i)\r\n",
				fres, bytesWrote);
	}
	//Close your file!
	f_close(&fil);

	osSemaphoreRelease(spiSemHandle);
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

  /* definition and creation of spiSem */
  osSemaphoreDef(spiSem);
  spiSemHandle = osSemaphoreCreate(osSemaphore(spiSem), 1);

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
  osThreadDef(InitDevicesTask, InitDevices_task, osPriorityRealtime, 0, 512);
  InitDevicesTaskHandle = osThreadCreate(osThread(InitDevicesTask), NULL);

  /* definition and creation of ReadSHTTask */
  osThreadDef(ReadSHTTask, ReadSHT_task, osPriorityLow, 0, 128);
  ReadSHTTaskHandle = osThreadCreate(osThread(ReadSHTTask), NULL);

  /* definition and creation of DisplayDataTask */
  osThreadDef(DisplayDataTask, DisplayData_task, osPriorityHigh, 0, 512);
  DisplayDataTaskHandle = osThreadCreate(osThread(DisplayDataTask), NULL);

  /* definition and creation of StoreSD */
  osThreadDef(StoreSD, StoreSD_task, osPriorityLow, 0, 1024);
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
		//init oled
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		ssd1306_Init();
		//init sht31
		if (!sht3x_init(&handle)) {
			myprintf("SHT3x access failed.\n\r");
			for (int var = 0; var < 50; ++var) {
				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
				osDelay(50);
				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
				osDelay(50);
			}
			//init sd card
			GetSdProperties();
			CreateSDCardLogFile();
		}
		osThreadTerminate(InitDevicesTaskHandle);
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
	bool pxl = 0;
	/* Infinite loop */
	for (;;) {
		snprintf(temp_text, sizeof(temp_text), "Temp: %.2f", temp);
		snprintf(hum_text, sizeof(hum_text), "Hum:  %.2f", hum);
		osSemaphoreWait(i2cSemHandle, osWaitForever);
		ssd1306_SetCursor(3, 10);
		ssd1306_WriteString(temp_text, Font_11x18, White);
		ssd1306_SetCursor(3, 35);
		ssd1306_WriteString(hum_text, Font_11x18, White);
		ssd1306_DrawRectangle(0, 1, 127, 63, White);
		ssd1306_DrawPixel(125, 25, pxl);
		ssd1306_UpdateScreen();
		osSemaphoreRelease(i2cSemHandle);
		pxl = !pxl;
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
	for (;;) {
		UpdateLogFile();
		osDelay(10000);
	}
  /* USER CODE END StoreSD_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
