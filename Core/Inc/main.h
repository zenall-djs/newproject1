/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// 步进电机相关常量定义
#define MOTOR_COUNT 3
#define STEP_DELAY_MS 20
#define UART_RX_BUFFER_SIZE 32

// 串口命令定义
#define CMD_START 0xAA
#define CMD_END 0x55
#define CMD_MOVE 0x01
#define CMD_STOP 0x02
#define CMD_SET_SPEED 0x03
#define CMD_SET_DIR 0x04 

// 响应状态码
#define STATUS_START 0x00
#define STATUS_COMPLETE 0x01
#define STATUS_STOP 0x02
#define STATUS_ERROR 0xFF

// 步进电机结构体
typedef struct {
    GPIO_TypeDef* step_port;
    uint16_t step_pin;
    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef* en_port;
    uint16_t en_pin;

    // 电机控制参数
    uint32_t target_steps;
    uint32_t current_steps;
    uint8_t is_moving;
    uint8_t direction;
    uint32_t step_delay;
} StepperMotor;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
// 步进电机控制函数
void Motor_InitAll(void);
void Motor_Init(uint8_t id);
void Motor_Enable(uint8_t motor_id, uint8_t enable);
void Motor_SetDirection(uint8_t motor_id, uint8_t direction);
void Motor_StartMove(uint8_t motor_id, uint32_t steps, uint8_t dir);
void Motor_Stop(uint8_t motor_id);
void Motor_Stop_All(void);
void Motor_ProcessStep(void);

// 串口通信函数
void UART_Receive_Start(void);
uint8_t UART_Command_Parser(uint8_t* data, uint8_t length);
void UART_Send_Response(uint8_t motor_id, uint8_t status);
void UART_ParseCommand(char* buffer); 

// 任务函数
void StartMotorTask(void const * argument);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI__NSS_Pin GPIO_PIN_4
#define SPI__NSS_GPIO_Port GPIOA
#define DRDY_Pin GPIO_PIN_4
#define DRDY_GPIO_Port GPIOC
#define START_Pin GPIO_PIN_5
#define START_GPIO_Port GPIOC
#define DC_Motors2_Pin GPIO_PIN_15
#define DC_Motors2_GPIO_Port GPIOE
#define DC_Motors3_Pin GPIO_PIN_11
#define DC_Motors3_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define DC_Motors1_Pin GPIO_PIN_11
#define DC_Motors1_GPIO_Port GPIOD
#define DC_Motors0_Pin GPIO_PIN_12
#define DC_Motors0_GPIO_Port GPIOD
#define MODEC0_Pin GPIO_PIN_13
#define MODEC0_GPIO_Port GPIOD
#define MODEB0_Pin GPIO_PIN_14
#define MODEB0_GPIO_Port GPIOD
#define MODEA0_Pin GPIO_PIN_15
#define MODEA0_GPIO_Port GPIOD
#define M_STEP0_Pin GPIO_PIN_6
#define M_STEP0_GPIO_Port GPIOC
#define M_EN0_Pin GPIO_PIN_7
#define M_EN0_GPIO_Port GPIOC
#define M_DIR0_Pin GPIO_PIN_8
#define M_DIR0_GPIO_Port GPIOC
#define MODEC1_Pin GPIO_PIN_9
#define MODEC1_GPIO_Port GPIOC
#define MODEB1_Pin GPIO_PIN_8
#define MODEB1_GPIO_Port GPIOA
#define MODEA1_Pin GPIO_PIN_11
#define MODEA1_GPIO_Port GPIOA
#define M_STEP1_Pin GPIO_PIN_12
#define M_STEP1_GPIO_Port GPIOA
#define M_EN1_Pin GPIO_PIN_15
#define M_EN1_GPIO_Port GPIOA
#define M_DIR1_Pin GPIO_PIN_10
#define M_DIR1_GPIO_Port GPIOC
#define MODEC2_Pin GPIO_PIN_11
#define MODEC2_GPIO_Port GPIOC
#define MODEB2_Pin GPIO_PIN_12
#define MODEB2_GPIO_Port GPIOC
#define MODEA2_Pin GPIO_PIN_0
#define MODEA2_GPIO_Port GPIOD
#define M_STEP2_Pin GPIO_PIN_1
#define M_STEP2_GPIO_Port GPIOD
#define M_EN2_Pin GPIO_PIN_2
#define M_EN2_GPIO_Port GPIOD
#define M_DIR2_Pin GPIO_PIN_3
#define M_DIR2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define MODEC0_Pin GPIO_PIN_13
#define MODEC0_GPIO_Port GPIOD
#define MODEB0_Pin GPIO_PIN_14
#define MODEB0_GPIO_Port GPIOD
#define MODEA0_Pin GPIO_PIN_15
#define MODEA0_GPIO_Port GPIOD
#define M_STEP0_Pin GPIO_PIN_6
#define M_STEP0_GPIO_Port GPIOC
#define M_EN0_Pin GPIO_PIN_7
#define M_EN0_GPIO_Port GPIOC
#define M_DIR0_Pin GPIO_PIN_8
#define M_DIR0_GPIO_Port GPIOC

#define MODEC1_Pin GPIO_PIN_9
#define MODEC1_GPIO_Port GPIOC
#define MODEB1_Pin GPIO_PIN_8
#define MODEB1_GPIO_Port GPIOA
#define MODEA1_Pin GPIO_PIN_11
#define MODEA1_GPIO_Port GPIOA
#define M_STEP1_Pin GPIO_PIN_12
#define M_STEP1_GPIO_Port GPIOA
#define M_EN1_Pin GPIO_PIN_15
#define M_EN1_GPIO_Port GPIOA
#define M_DIR1_Pin GPIO_PIN_10
#define M_DIR1_GPIO_Port GPIOC

#define MODEC2_Pin GPIO_PIN_11
#define MODEC2_GPIO_Port GPIOC
#define MODEB2_Pin GPIO_PIN_12
#define MODEB2_GPIO_Port GPIOC
#define MODEA2_Pin GPIO_PIN_0
#define MODEA2_GPIO_Port GPIOD
#define M_STEP2_Pin GPIO_PIN_1
#define M_STEP2_GPIO_Port GPIOD
#define M_EN2_Pin GPIO_PIN_2
#define M_EN2_GPIO_Port GPIOD
#define M_DIR2_Pin GPIO_PIN_3
#define M_DIR2_GPIO_Port GPIOD
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
