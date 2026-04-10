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

// 步进电机结构体
typedef struct {
    GPIO_TypeDef* step_port;
    uint16_t step_pin;
    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef* en_port;
    uint16_t en_pin;
    uint32_t target_steps;
    uint32_t current_steps;
    uint8_t is_moving;
    uint8_t direction;
    uint32_t step_delay;
} StepperMotor;

// 系统状态枚举
typedef enum {
    SYS_IDLE,
    SYS_WAITING_START,
    SYS_RUNNING
} SystemState;

// 操作类型枚举
typedef enum {
    OP_NONE,
    OP_CAL,
    OP_SAMPLE,
    OP_URINE,
    OP_CLEAN,
    OP_POWER_CLEAN,
    OP_MAINT,
    OP_PRIME_STDA,
    OP_PRIME_STDB,
    OP_PRIME_REF,
    OP_PRIME_DILUTED,
    OP_DISP_STDA,
    OP_DISP_STDB
} OperationType;

// 流程步骤结构体
typedef struct {
    uint32_t duration_ms;
    void (*action)(void);
    const char* response_fmt;
} Step;

// 流程控制结构体
typedef struct {
    SystemState state;
    OperationType current_op;
    uint32_t current_step;
    uint32_t step_start_time;
    uint32_t wait_start_time;
    const Step* steps;
    uint32_t step_count;
} ProcessCtrl;

// 离子校准参数
typedef struct {
    float slope;
    float intercept;
    uint8_t valid;
} CalibrationParam;

// 上次结果存储
typedef struct {
    char sample_result[100];
    char urine_result[100];
    char slope_result[100];
} LastResults;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define MOTOR_COUNT 3
#define STEP_DELAY_MS 20
#define TIMEOUT_MS 10000
#define MEASURE_STABLE_DELAY_MS 15000

#define STEPS_PER_REV 1600

// ADC通道定义
#define ADC_NA_CHANNEL     ADC_CHANNEL_10   // ADC1_IN10
#define ADC_K_CHANNEL      ADC_CHANNEL_11   // ADC1_IN11
#define ADC_CL_CHANNEL     ADC_CHANNEL_12   // ADC1_IN12
#define ADC_REF_CHANNEL    ADC_CHANNEL_13   // ADC1_IN13
#define ADC_TEMP_CHANNEL   ADC_CHANNEL_10   // ADC2_IN10

// 温度传感器系数
#define TEMP_SLOPE   (100.0f / 3.3f)   // °C/V
#define TEMP_OFFSET  0.0f

// 默认校准值
#define DEFAULT_SLOPE_NA   59.16f
#define DEFAULT_SLOPE_K    59.16f
#define DEFAULT_SLOPE_CL   -59.16f
#define DEFAULT_INTERCEPT  0.0f

// 标液已知浓度
#define STD_A_NA  140.0f
#define STD_A_K   4.0f
#define STD_A_CL  100.0f
#define STD_B_NA  110.0f
#define STD_B_K   8.0f
#define STD_B_CL  70.0f

// ---------- 光耦检测引脚定义 ----------
#define LIQUID_SENSE_PIN       GPIO_PIN_0
#define LIQUID_SENSE_PORT      GPIOA
#define LIQUID_PRESENT   GPIO_PIN_RESET
#define LIQUID_ABSENT    GPIO_PIN_SET

// ---------- 光耦检测ADC通道 ----------
#define LIQUID_SENSE_ADC         &hadc1          
#define LIQUID_SENSE_ADC_CHANNEL ADC_CHANNEL_12   
#define LIQUID_PRESENT_THRESHOLD_V 1.0f            // 电压阈值，大于此值认为有液

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

// 电机控制
void Motor_InitAll(void);
void Motor_Init(uint8_t id);
void Motor_Enable(uint8_t motor_id, uint8_t enable);
void Motor_SetDirection(uint8_t motor_id, uint8_t direction);
void Motor_StartMove(uint8_t motor_id, uint32_t steps, uint8_t dir);
void Motor_Stop(uint8_t motor_id);
void Motor_Stop_All(void);
void Motor_ProcessStep(void);

// 带光耦检测的电机移动（仅电机0，enable_check=1启用检测，=0禁用）
void Motor_MoveStepsWithLiquidCheck(uint8_t motor_id, uint32_t steps, uint8_t dir, uint8_t enable_check);

// 同步等待电机完成
void Motor_WaitCompletion(uint8_t motor_id);

// 流程控制
void Process_StartProcedure(OperationType op, const Step* steps, uint32_t count, uint8_t wait_start);
void Process_Abort(void);
void Process_CheckTimeout(void);
void Process_RunStep(void);

// ADC读取与计算
float ADC_ReadVoltage(ADC_HandleTypeDef* hadc, uint32_t channel);
float Read_Temperature(void);
void Read_ElectrodeMV(float* na, float* k, float* cl, float* ref);
float CalculateConcentration(float mv, float ref_mv, CalibrationParam* cal, float temp);

// 校准与结果存储
void PerformCalibration(float na_mv_stda, float k_mv_stda, float cl_mv_stda,
                        float na_mv_stdb, float k_mv_stdb, float cl_mv_stdb);
void UpdateLastResult(const char* type, float na, float k, float cl);

// 命令处理
void Cmd_CAL(void);
void Cmd_SAMPLE(void);
void Cmd_URINE(void);
void Cmd_CLEAN(void);
void Cmd_POWER_CLEAN(void);
void Cmd_MAINT(void);
void Cmd_PRIME_STDA(void);
void Cmd_PRIME_STDB(void);
void Cmd_PRIME_REF(void);
void Cmd_PRIME_DILUTED(void);
void Cmd_DISP_STDA(void);
void Cmd_DISP_STDB(void);
void Cmd_START(void);
void Cmd_LAST_RESULT(void);
void Cmd_LAST_SLOPE(void);
void Cmd_READ_MV(void);
void Cmd_VERSION(void);
void Cmd_ECAPE(void);
void Cmd_DEBUG_MODE(void);
void Cmd_DEBUG_OFF(void);

// 串口通信
void UART_SendString(const char *str);
void UART_SendResponse(const char *status, uint8_t motor_id, uint32_t steps);
void UART_SendFormatted(const char *fmt, ...);

// FreeRTOS任务
void StartMotorTask(void const * argument);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
// 步进电机引脚定义
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

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
