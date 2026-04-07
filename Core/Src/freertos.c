/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "adc.h"

extern UART_HandleTypeDef huart1;
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
extern StepperMotor motors[MOTOR_COUNT];
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId motorTaskHandle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(motorTask, StartMotorTask, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
 // 初始化电机系统
  Motor_InitAll();
  
  // 发送任务启动消息
HAL_UART_Transmit(&huart1, (uint8_t*)"Motor Task Started\n", 19, 100);
  Motor_SetDirection(0, 1);
  Motor_Enable(0, 1);
  /* Infinite loop */
  for(;;)
  {
    /* 电机运动序列演示 */
    
    // 电机1：正向移动500步
//    Motor_StartMove(0, 500, 1);
//		float voltage = ADC_ReadVoltage(LIQUID_SENSE_ADC, LIQUID_SENSE_ADC_CHANNEL);
//		UART_SendFormatted("ADC init: raw=%d, voltage=%.2f V\r\n",
//                       (int)(voltage/3.3f*4095), voltage);
		osDelay(100);
//    osDelay(3000);  // 等待3秒完成移动
    
//    // 电机1：反向移动500步
//    Motor_StartMove(0, 500, 0);
//    osDelay(3000);
//    
//    // 电机2：正向移动300步
//    Motor_StartMove(1, 300, 1);
//    osDelay(2000);
//    
//    // 电机2：反向移动300步
//    Motor_StartMove(1, 300, 0);
//    osDelay(2000);
//    
//    // 电机3：正向移动200步
//    Motor_StartMove(2, 200, 1);
//    osDelay(1500);
//    
//    // 电机3：反向移动200步
//    Motor_StartMove(2, 200, 0);
//    osDelay(1500);
//    
//    // 所有电机同时正向移动100步（同步演示）
//    Motor_StartMove(0, 100, 1);
//    Motor_StartMove(1, 100, 1);
//    Motor_StartMove(2, 100, 1);
//    osDelay(2000);
//    
//    // 停止所有电机
//    Motor_Stop_All();
//    osDelay(1000);
//    
    // 发送状态消息
//    HAL_UART_Transmit(&huart1, (uint8_t*)"Motor Sequence Completed, Restarting...\n", 42, 100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */