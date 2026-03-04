/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 64     
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 全局变量定义
StepperMotor motors[MOTOR_COUNT];
volatile uint32_t step_timer_count = 0;

// 串口接收相关变量
uint8_t rx_byte;                        // 当前接收的字节
char rx_buffer[RX_BUFFER_SIZE];          // 接收缓冲区
uint8_t rx_index = 0;                    // 缓冲区索引
uint8_t cmd_ready = 0;                   // 完整命令接收完成标志
char cmd_buffer[RX_BUFFER_SIZE];         // 待处理命令缓冲区
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
// 电机控制函数
void Motor_InitAll(void);
void Motor_Init(uint8_t id);
void Motor_SetStepMode_1_8(void);
void Motor_Enable(uint8_t motor_id, uint8_t enable);
void Motor_SetDirection(uint8_t motor_id, uint8_t direction);
void Motor_StartMove(uint8_t motor_id, uint32_t steps, uint8_t dir);
void Motor_Stop(uint8_t motor_id);
void Motor_Stop_All(void);
void Motor_ProcessStep(void);

// 串口通信函数
void UART_SendString(const char *str);
void UART_SendResponse(const char *status, uint8_t motor_id, uint32_t steps);
void Parse_Command(char *cmd);

// 任务函数
void StartMotorTask(void const * argument);

// 中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief 初始化所有电机
  */
void Motor_InitAll(void) {
    Motor_SetStepMode_1_8();
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_Init(i);
    }
}

/**
  * @brief 设置所有电机为1/8细分模式（011）
  */
void Motor_SetStepMode_1_8(void) {
    // 电机0的细分引脚设置：MODEA=1, MODEB=1, MODEC=0
    HAL_GPIO_WritePin(MODEA0_GPIO_Port, MODEA0_Pin, GPIO_PIN_SET);    
    HAL_GPIO_WritePin(MODEB0_GPIO_Port, MODEB0_Pin, GPIO_PIN_SET);    
    HAL_GPIO_WritePin(MODEC0_GPIO_Port, MODEC0_Pin, GPIO_PIN_RESET);  
    
    // 电机1的细分引脚设置
    HAL_GPIO_WritePin(MODEA1_GPIO_Port, MODEA1_Pin, GPIO_PIN_SET);    
    HAL_GPIO_WritePin(MODEB1_GPIO_Port, MODEB1_Pin, GPIO_PIN_SET);    
    HAL_GPIO_WritePin(MODEC1_GPIO_Port, MODEC1_Pin, GPIO_PIN_RESET);  
	
    // 电机2的细分引脚设置
    HAL_GPIO_WritePin(MODEA2_GPIO_Port, MODEA2_Pin, GPIO_PIN_SET);    
    HAL_GPIO_WritePin(MODEB2_GPIO_Port, MODEB2_Pin, GPIO_PIN_SET);    
    HAL_GPIO_WritePin(MODEC2_GPIO_Port, MODEC2_Pin, GPIO_PIN_RESET);  
    
    HAL_Delay(100);
}

/**
  * @brief 初始化单个电机
  */
void Motor_Init(uint8_t id) {
    if (id >= MOTOR_COUNT) return;
    
    StepperMotor* motor = &motors[id];
    
    // 引脚配置
    switch(id) {
        case 0:
            motor->step_port = M_STEP0_GPIO_Port;
            motor->step_pin = M_STEP0_Pin;
            motor->dir_port = M_DIR0_GPIO_Port;
            motor->dir_pin = M_DIR0_Pin;
            motor->en_port = M_EN0_GPIO_Port;
            motor->en_pin = M_EN0_Pin;
            break;
        case 1:
            motor->step_port = M_STEP1_GPIO_Port;
            motor->step_pin = M_STEP1_Pin;
            motor->dir_port = M_DIR1_GPIO_Port;
            motor->dir_pin = M_DIR1_Pin;
            motor->en_port = M_EN1_GPIO_Port;
            motor->en_pin = M_EN1_Pin;
            break;
        case 2:
            motor->step_port = M_STEP2_GPIO_Port;
            motor->step_pin = M_STEP2_Pin;
            motor->dir_port = M_DIR2_GPIO_Port;
            motor->dir_pin = M_DIR2_Pin;
            motor->en_port = M_EN2_GPIO_Port;
            motor->en_pin = M_EN2_Pin;
            break;
    }
    
    // 初始化参数调整
    motor->target_steps = 0;
    motor->current_steps = 0;
    motor->is_moving = 0;
    motor->direction = 1;
    motor->step_delay = STEP_DELAY_MS; 
    
    // 初始状态设置
    HAL_GPIO_WritePin(motor->step_port, motor->step_pin, GPIO_PIN_RESET);
    Motor_SetDirection(id, 1);
    Motor_Enable(id, 0);  // 初始禁用
    
    // 初始化延迟
    HAL_Delay(10);
}

/**
  * @brief 使能/禁用电机
  */
void Motor_Enable(uint8_t motor_id, uint8_t enable) {
    if (motor_id >= MOTOR_COUNT) return;
    
    StepperMotor* motor = &motors[motor_id];
    // DRV8825使能逻辑：低电平使能，高电平禁用
    HAL_GPIO_WritePin(motor->en_port, motor->en_pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
  * @brief 设置电机方向
  */
void Motor_SetDirection(uint8_t motor_id, uint8_t direction) {
    if (motor_id >= MOTOR_COUNT) return;
    
    StepperMotor* motor = &motors[motor_id];
    // 设置方向引脚
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief 启动电机移动
  */
void Motor_StartMove(uint8_t motor_id, uint32_t steps, uint8_t dir) {
    if (motor_id >= MOTOR_COUNT) return;
    
    StepperMotor* motor = &motors[motor_id];
    
    // 设置运动参数
    motor->target_steps = steps;
    motor->current_steps = 0;
    motor->direction = dir;
    motor->is_moving = 1;
    
	// 设置方向并使能电机
    Motor_SetDirection(motor_id, dir);
    Motor_Enable(motor_id, 1);
}

/**
  * @brief 停止单个电机
  */
void Motor_Stop(uint8_t motor_id) {
    if (motor_id >= MOTOR_COUNT) return;
    
    StepperMotor* motor = &motors[motor_id];
    motor->is_moving = 0;
    motor->target_steps = 0;
    motor->current_steps = 0;
    Motor_Enable(motor_id, 0);
    
    HAL_GPIO_WritePin(motor->step_port, motor->step_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->en_port, motor->en_pin, GPIO_PIN_SET);  // 高电平 = 禁用
    HAL_Delay(1);
}

/**
  * @brief 停止所有电机
  */
void Motor_Stop_All(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_Stop(i);
    }
}

/**
  * @brief 定时器中断中处理步进脉冲
  */
void Motor_ProcessStep(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        StepperMotor* motor = &motors[i];
        
        if (motor->is_moving && motor->current_steps < motor->target_steps) {
            // 产生步进脉冲（翻转STEP引脚）
            HAL_GPIO_TogglePin(motor->step_port, motor->step_pin);
            motor->current_steps++;
            
            // 检查是否完成移动
            if (motor->current_steps >= motor->target_steps) {
                motor->is_moving = 0;
                Motor_Enable(i, 0);  // 移动完成，禁用电机
                
                // 发送完成响应
                char resp[32];
                sprintf(resp, "M%d completed\r\n", i);
                UART_SendString(resp);
            }
        }
    }
}

/**
  * @brief 定时器中断回调函数
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        step_timer_count++;
        // 根据定时器频率处理步进（假设定时器周期为1ms）
        if (step_timer_count >= 1) {
            Motor_ProcessStep();
            step_timer_count = 0;
        }
    }
}

/**
  * @brief UART接收完成回调
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 将接收到的字符存入缓冲区
        if (rx_index < RX_BUFFER_SIZE - 1) {
            if (rx_byte == '\n' || rx_byte == '\r') {
                if (rx_index > 0) {
                    rx_buffer[rx_index] = '\0';
                    
                    // 回显
                    char echo[RX_BUFFER_SIZE + 3];
                    sprintf(echo, "rec：%s\r\n", rx_buffer);
                    UART_SendString(echo);
                    
                    // 复制到命令缓冲区并置标志
                    strcpy(cmd_buffer, rx_buffer);
                    cmd_ready = 1;
                    
                    // 清空索引，准备下一次接收
                    rx_index = 0;
                }
            } else {
                rx_buffer[rx_index++] = (char)rx_byte;
            }
        } else {
            // 缓冲区溢出，丢弃当前数据并重置
            rx_index = 0;
        }
        // 重新开启接收
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

/**
  * @brief 通过串口发送字符串
  */
void UART_SendString(const char *str) {
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
}

/**
  * @brief 发送响应
  */
void UART_SendResponse(const char *status, uint8_t motor_id, uint32_t steps) {
    char resp[64];
    if (steps > 0) {
        sprintf(resp, "M%d %s %lu\r\n", motor_id, status, steps);
    } else {
        sprintf(resp, "M%d %s\r\n", motor_id, status);
    }
    UART_SendString(resp);
}

/**
  * @brief 解析并执行ASCII命令
  * 命令格式：
  *   M0F500   - 电机0正转500步
  *   M0R500   - 电机0反转500步
  *   M0S      - 电机0停止
  * 电机ID: 0~2,
  */
void Parse_Command(char *cmd) {
    // 去除首尾空白和换行符
    uint8_t motor_id;
    char dir_char;
    uint32_t steps;
    
    // 解析格式: M<id><F/R><steps> 或 M<id>S
//  while (*cmd == ' ') cmd++;  
	if (cmd[0] != 'M' && cmd[0] != 'm') {
//        UART_SendString("Invalid command format. Use M0F500 / M0R500 / M0S\r\n");
			  printf("Invalid command format. Use M0F500 / M0R500 / M0S[%s]\r\n",cmd);
        return;
    }
    
    // 提取电机ID
    if (cmd[1] < '0' || cmd[1] > '2') {
        UART_SendString("Motor ID must be 0,1,2\r\n");
        return;
    }
    motor_id = cmd[1] - '0';
    
    // 检查命令类型
    dir_char = toupper(cmd[2]); 
    if (dir_char == 'S') {
        // 停止命令
        Motor_Stop(motor_id);
        UART_SendResponse("STOPPED", motor_id, 0);
        return;
    }
    else if (dir_char == 'F' || dir_char == 'R') {
        // 移动命令，需要提取步数
        char *steps_str = cmd + 3;  
        // 确保步数是数字
        for (char *p = steps_str; *p != '\0'; p++) {
            if (!isdigit((unsigned char)*p)) {
                UART_SendString("Steps must be a positive number\r\n");
                return;
            }
        }
        steps = atoi(steps_str);
        if (steps == 0) {
            UART_SendString("Steps must be > 0\r\n");
            return;
        }
        
        uint8_t dir = (dir_char == 'F') ? 1 : 0;
        Motor_StartMove(motor_id, steps, dir);
        
        char resp[64];
        sprintf(resp, "M%d %s %lu\r\n", motor_id, (dir ? "FORWARD" : "REVERSE"), steps);
        UART_SendString(resp);
    }
    else {
        UART_SendString("Invalid command. Use F/R for direction or S for stop.\r\n");
    }
}

/**
  * @brief 电机驱动任务（由FreeRTOS调度）
  */
void StartMotorTask(void const * argument) {
    // 初始化电机系统
    Motor_InitAll();
    
    for (;;) {
        if (cmd_ready) {
            cmd_ready = 0; 
					
            Parse_Command(cmd_buffer);      
        }
        osDelay(10);  
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();

    /* USER CODE BEGIN 2 */
    // 启动定时器中断（用于步进脉冲生成）
    HAL_TIM_Base_Start_IT(&htim2);

		// 设置中断优先级
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);   
    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);    

    // 启动UART接收中断
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    
    // 发送系统启动消息
    UART_SendString("Stepper Motor System Ready\n");
    UART_SendString("Commands: M0F500, M0R500, M0S\n");
    /* USER CODE END 2 */

    /* Call init function for freertos objects (in cmsis_os2.c) */
    MX_FREERTOS_Init();

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        osDelay(1000);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {    
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */