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
#include <stdarg.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 64
#define ADC_TIMEOUT    10
#define STABLE_DELAY_MS 500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
StepperMotor motors[MOTOR_COUNT];
volatile uint32_t step_timer_count = 0;

// 串口接收
uint8_t rx_byte;
char rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
uint8_t cmd_ready = 0;
char cmd_buffer[RX_BUFFER_SIZE];

// 流程控制
ProcessCtrl procCtrl = {
    .state = SYS_IDLE,
    .current_op = OP_NONE,
    .current_step = 0,
    .step_start_time = 0,
    .wait_start_time = 0,
    .steps = NULL,
    .step_count = 0
};

// 调试模式
uint8_t debug_mode = 0;

// 上次结果
LastResults lastResults = {
    .sample_result = "",
    .urine_result = "",
    .slope_result = ""
};

// 校准参数
CalibrationParam cal_na = {DEFAULT_SLOPE_NA, DEFAULT_INTERCEPT, 0};
CalibrationParam cal_k  = {DEFAULT_SLOPE_K, DEFAULT_INTERCEPT, 0};
CalibrationParam cal_cl = {DEFAULT_SLOPE_CL, DEFAULT_INTERCEPT, 0};

// 版本信息
const char* version_str = "DISU4-760148-10C Ver1.0.00\r\n";

// 临时存储标液测量值（用于CAL）
static float stda_na_mv, stda_k_mv, stda_cl_mv;
static float stdb_na_mv, stdb_k_mv, stdb_cl_mv;
static uint8_t stda_measured = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
// 电机动作函数（静态）声明
static void action_cal_step1(void);
static void action_cal_step2(void);
static void action_cal_step3(void);
static void action_cal_step4(void);
static void action_cal_step5(void);
static void action_cal_step6(void);
static void action_cal_step7(void);
static void action_sample_measure(void);
static void action_urine_measure(void);
static void action_prime_stda(void);
static void action_prime_stdb(void);

// 简单动作函数声明
static void action_clean(void);
static void action_power_clean(void);
static void action_none(void);
static void action_prime_ref(void);
static void action_prime_diluted(void);
static void action_disp_stda(void);
static void action_disp_stdb(void);

// 电机移动辅助函数
static void Motor_MoveSteps(uint8_t motor_id, uint32_t steps, uint8_t dir);
static void Motor_MoveStepsWithLiquidCheck(uint8_t motor_id, uint32_t steps, uint8_t dir, uint8_t enable_check);
static void MeasureAndDispense(const char* result_prefix, uint8_t is_cal, uint8_t cal_step);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ---------- ADC 读取函数 ----------
float ADC_ReadVoltage(ADC_HandleTypeDef* hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(hadc, &sConfig);
    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, ADC_TIMEOUT) == HAL_OK) {
        uint32_t adc_val = HAL_ADC_GetValue(hadc);
        HAL_ADC_Stop(hadc);
        return (adc_val * 3.0f) / 4095.0f;
    }
    HAL_ADC_Stop(hadc);
    return 0.0f;
}

float Read_Temperature(void) {
    float v = ADC_ReadVoltage(&hadc2, ADC_TEMP_CHANNEL);
    return TEMP_SLOPE * v + TEMP_OFFSET;
}

void Read_ElectrodeMV(float* na, float* k, float* cl, float* ref) {
    *na  = ADC_ReadVoltage(&hadc1, ADC_NA_CHANNEL) * 1000.0f;
    *k   = ADC_ReadVoltage(&hadc1, ADC_K_CHANNEL) * 1000.0f;
    *cl  = ADC_ReadVoltage(&hadc1, ADC_CL_CHANNEL) * 1000.0f;
    *ref = ADC_ReadVoltage(&hadc1, ADC_REF_CHANNEL) * 1000.0f;
}

float CalculateConcentration(float mv, float ref_mv, CalibrationParam* cal, float temp) {
    float e = mv - ref_mv;
    float s = cal->slope * (temp + 273.15f) / 298.15f;
    return powf(10.0f, (e - cal->intercept) / s);
}

// 执行两点校准
void PerformCalibration(float na_mv_stda, float k_mv_stda, float cl_mv_stda,
                        float na_mv_stdb, float k_mv_stdb, float cl_mv_stdb) {
    float temp = Read_Temperature();
    // 计算差值
    float e_na_a = na_mv_stda;
    float e_k_a  = k_mv_stda;
    float e_cl_a = cl_mv_stda;
    float e_na_b = na_mv_stdb;
    float e_k_b  = k_mv_stdb;
    float e_cl_b = cl_mv_stdb;

    // 计算斜率
    cal_na.slope = (e_na_b - e_na_a) / (log10f(STD_B_NA) - log10f(STD_A_NA));
    cal_na.intercept = e_na_a - cal_na.slope * log10f(STD_A_NA);
    cal_na.valid = 1;

    cal_k.slope = (e_k_b - e_k_a) / (log10f(STD_B_K) - log10f(STD_A_K));
    cal_k.intercept = e_k_a - cal_k.slope * log10f(STD_A_K);
    cal_k.valid = 1;

    cal_cl.slope = (e_cl_b - e_cl_a) / (log10f(STD_B_CL) - log10f(STD_A_CL));
    cal_cl.intercept = e_cl_a - cal_cl.slope * log10f(STD_A_CL);
    cal_cl.valid = 1;

    // 保存斜率结果
    snprintf(lastResults.slope_result, sizeof(lastResults.slope_result),
             "CAL_Na %.2f K %.2f Cl %.2f\r\n", cal_na.slope, cal_k.slope, cal_cl.slope);
}

// 更新上次结果
void UpdateLastResult(const char* type, float na, float k, float cl) {
    if (strcmp(type, "SAMPLE") == 0) {
        snprintf(lastResults.sample_result, sizeof(lastResults.sample_result),
                 "SAMPLE_Na %.2f K %.2f Cl %.2f\r\n", na, k, cl);
    } else if (strcmp(type, "URINE") == 0) {
        snprintf(lastResults.urine_result, sizeof(lastResults.urine_result),
                 "URINE_Na %.2f K %.2f Cl %.2f\r\n", na, k, cl);
    }
}

// ---------- 电机移动辅助函数 ----------
// 启动电机并等待完成（阻塞）
static void Motor_MoveSteps(uint8_t motor_id, uint32_t steps, uint8_t dir) {
    if (motor_id == 0) {
        Motor_StartMove(0, steps, 1);
    } else {
        Motor_StartMove(motor_id, steps, dir);
    }
    while (motors[motor_id].is_moving) {
        osDelay(1000);
    }
}

// 启动电机0并检测光耦
static void Motor_MoveStepsWithLiquidCheck(uint8_t motor_id, uint32_t steps, uint8_t dir, uint8_t enable_check) {
    if (motor_id != 0) {
        Motor_StartMove(motor_id, steps, dir);
        while (motors[motor_id].is_moving) {
            osDelay(1);
        }
        return;
    }

    Motor_StartMove(0, steps, 1);

    if (!enable_check) {
        // 不检测光耦，等待完成
        while (motors[0].is_moving) {
            osDelay(1);
        }
        return;
    }

    // 启用ADC光耦检测	
    uint8_t liquid_detected = 0; 
    uint32_t timeout_count = 0;
    const uint32_t MAX_CHECKS = 2000; 
	
    while (motors[0].is_moving && timeout_count < MAX_CHECKS) {
        float voltage = ADC_ReadVoltage(LIQUID_SENSE_ADC, LIQUID_SENSE_ADC_CHANNEL);
        uint8_t current_liquid = (voltage < LIQUID_PRESENT_THRESHOLD_V) ? 1 : 0;

        UART_SendFormatted("ADC: raw=%d, voltage=%.2f V, state=%s\r\n",
                           (int)(voltage / 3.3f * 4095), voltage,
                           current_liquid ? "LIQUID" : "NO LIQUID");

        if (!liquid_detected) {
            // 等待第一次检测到有液
            if (current_liquid == 1) {
                liquid_detected = 1;
                UART_SendString("Liquid detected, continue moving...\r\n");
            }
        } else {
            if (current_liquid == 0) {
                Motor_Stop(0);
                UART_SendString("Liquid empty, motor stopped.\r\n");
                break;
            }
        }
        osDelay(10);
        timeout_count++;
    }

    if (motors[0].is_moving) {
        Motor_Stop(0);
        UART_SendString("Motor stopped by timeout.\r\n");
    }
}

// ---------- 通用测量过程 ----------
static void MeasureAndDispense(const char* result_prefix, uint8_t is_cal, uint8_t cal_step) {
    float na_mv, k_mv, cl_mv, ref_mv;
    float na_conc, k_conc, cl_conc;

    // 1. 吸入标液
    Motor_MoveStepsWithLiquidCheck(0, 10 * STEPS_PER_REV, 1, 1);

    // 2. 等待液体稳定
    HAL_Delay(MEASURE_STABLE_DELAY_MS);

    // 3. 读取电极电位
    Read_ElectrodeMV(&na_mv, &k_mv, &cl_mv, &ref_mv);
    float temp = Read_Temperature();

    if (is_cal) {
        if (cal_step == 1) {
            stda_na_mv = na_mv - ref_mv;
            stda_k_mv  = k_mv - ref_mv;
            stda_cl_mv = cl_mv - ref_mv;
            stda_measured = 1;
        } else if (cal_step == 2) {
            stdb_na_mv = na_mv - ref_mv;
            stdb_k_mv  = k_mv - ref_mv;
            stdb_cl_mv = cl_mv - ref_mv;
        }
        if (cal_step == 1) {
            UART_SendFormatted("1-CAL_Na %.2f K %.2f Cl %.2f\r\n", STD_A_NA, STD_A_K, STD_A_CL);
        } else {
            UART_SendFormatted("2-CAL_Na %.2f K %.2f Cl %.2f\r\n", STD_B_NA, STD_B_K, STD_B_CL);
        }
    } else {
        na_conc = CalculateConcentration(na_mv, ref_mv, &cal_na, temp);
        k_conc  = CalculateConcentration(k_mv, ref_mv, &cal_k, temp);
        cl_conc = CalculateConcentration(cl_mv, ref_mv, &cal_cl, temp);
        UART_SendFormatted("%s %.2f K %.2f Cl %.2f\r\n", result_prefix, na_conc, k_conc, cl_conc);
        UpdateLastResult(result_prefix, na_conc, k_conc, cl_conc);
    }

    // 4. 排出废液
    Motor_MoveStepsWithLiquidCheck(0, 3 * STEPS_PER_REV, 1, 0);
}

// ---------- 电机动作函数 ----------
static void action_cal_step1(void) {
    // 电机2转圈，方向1
    Motor_MoveSteps(2, 3 * STEPS_PER_REV, 0);
}
static void action_cal_step2(void) {
    MeasureAndDispense("", 1, 1);
}
static void action_cal_step3(void) {}
static void action_cal_step4(void) {
    // 电机1转3圈，方向1
    Motor_MoveSteps(1, 3 * STEPS_PER_REV, 0);
}
static void action_cal_step5(void) {
    MeasureAndDispense("", 1, 2);
}
static void action_cal_step6(void) {}
static void action_cal_step7(void) {
    if (stda_measured) {
        PerformCalibration(stda_na_mv, stda_k_mv, stda_cl_mv,
                           stdb_na_mv, stdb_k_mv, stdb_cl_mv);
        UART_SendString(lastResults.slope_result);
    } else {
        UART_SendString("ERROR CAL FAILED\r\n");
    }
}
static void action_sample_measure(void) {
    MeasureAndDispense("SAMPLE_Na", 0, 0);
}
static void action_urine_measure(void) {
    MeasureAndDispense("URINE_Na", 0, 0);
}
static void action_prime_stda(void) {
    // 电机2转3圈
    Motor_MoveSteps(2, 3 * STEPS_PER_REV, 1);
    MeasureAndDispense("", 0, 0);
}
static void action_prime_stdb(void) {
    // 电机1转3圈
    Motor_MoveSteps(1, 3 * STEPS_PER_REV, 1);
    MeasureAndDispense("", 0, 0);
}

// 简单动作函数实现
static void action_clean(void) {
    Motor_StartMove(2, 3000, 0);
	  Motor_StartMove(0, 6000, 1);
}
static void action_power_clean(void) {
    Motor_StartMove(2, 800, 0);
    Motor_StartMove(0, 800, 1);
}
static void action_prime_ref(void) {
    Motor_StartMove(2, 2000, 0);
}
static void action_prime_diluted(void) {
    Motor_StartMove(0, 600, 1);
}
static void action_disp_stda(void) {
    Motor_StartMove(0, 100, 1);
}
static void action_disp_stdb(void) {
    Motor_StartMove(1, 1000, 1);
}
static void action_none(void) {}

// ---------- 流程步骤数组定义 ----------
static const Step cal_steps[] = {
    {0, action_cal_step1, NULL},
    {0, action_cal_step2, NULL},
    {0, action_cal_step3, NULL},
    {0, action_cal_step4, NULL},
    {0, action_cal_step5, NULL},
    {0, action_cal_step6, NULL},
    {0, action_cal_step7, NULL},
};

static const Step sample_steps[] = {
    {0, action_sample_measure, NULL},
};

static const Step urine_steps[] = {
    {0, action_urine_measure, NULL},
};

static const Step clean_steps[] = {
    {8000, action_clean, "CLEAN_OK\r\n"},
    {8000, action_clean, "CLEAN_OK\r\n"},
    {8000, action_clean, "CLEAN_OK\r\n"},
    {8000, action_clean, "CLEAN_END\r\n"},
};

static const Step power_clean_steps[] = {
    {128000, action_power_clean, "POWER_CLEAN_END\r\n"},
};

static const Step maint_steps[] = {
    {8000, action_none, "MAINT_END\r\n"},
};

static const Step prime_stda_steps[] = {
    {0, action_prime_stda, "PRIME_STDA_END\r\n"},
};

static const Step prime_stdb_steps[] = {
    {0, action_prime_stdb, "PRIME_STDB_END\r\n"},
};

static const Step prime_ref_steps[] = {
    {10000, action_prime_ref, "PRIME_REF_END\r\n"},
};

static const Step prime_diluted_steps[] = {
    {97000, action_prime_diluted, "PRIME_DILUTED_END\r\n"},
};

static const Step disp_stda_steps[] = {
    {4000, action_disp_stda, "DISP_STDA_END\r\n"},
};

static const Step disp_stdb_steps[] = {
    {4000, action_disp_stdb, "DISP_STDB_END\r\n"},
};

// ---------- 流程控制函数 ----------
void Process_StartProcedure(OperationType op, const Step* steps, uint32_t count, uint8_t wait_start) {
    if (procCtrl.state != SYS_IDLE) {
        UART_SendString("ERROR BUSY\r\n");
        return;
    }
    if (wait_start) {
        procCtrl.state = SYS_WAITING_START;
        procCtrl.current_op = op;
        procCtrl.wait_start_time = HAL_GetTick();
    } else {
        procCtrl.state = SYS_RUNNING;
        procCtrl.current_op = op;
        procCtrl.current_step = 0;
        procCtrl.steps = steps;
        procCtrl.step_count = count;
        if (steps[0].action) steps[0].action();
        procCtrl.step_start_time = HAL_GetTick();
    }
}

void Process_Abort(void) {
    Motor_Stop_All();
    procCtrl.state = SYS_IDLE;
}

void Process_CheckTimeout(void) {
    if (procCtrl.state == SYS_WAITING_START) {
        if (HAL_GetTick() - procCtrl.wait_start_time >= TIMEOUT_MS) {
            const char* op_str = "";
            switch (procCtrl.current_op) {
                case OP_SAMPLE: op_str = "SAMPLE"; break;
                case OP_URINE: op_str = "URINE"; break;
                case OP_CLEAN: op_str = "CLEAN"; break;
                case OP_POWER_CLEAN: op_str = "POWER_CLEAN"; break;
                default: op_str = "UNKNOWN";
            }
            UART_SendFormatted("ERROR %s TIMEOUT\r\n", op_str);
            Process_Abort();
        }
    }
}

void Process_RunStep(void) {
    if (procCtrl.state != SYS_RUNNING) return;

    uint32_t now = HAL_GetTick();
    if (now - procCtrl.step_start_time >= procCtrl.steps[procCtrl.current_step].duration_ms) {
        const Step* step = &procCtrl.steps[procCtrl.current_step];
        if (step->response_fmt && step->response_fmt[0] != '\0') {
            UART_SendString(step->response_fmt);
        }

        if (procCtrl.current_step + 1 >= procCtrl.step_count) {
            Process_Abort();
            return;
        }

        procCtrl.current_step++;
        if (procCtrl.steps[procCtrl.current_step].action) {
            procCtrl.steps[procCtrl.current_step].action();
        }
        procCtrl.step_start_time = now;
    }
}

// 格式化发送
void UART_SendFormatted(const char *fmt, ...) {
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    UART_SendString(buffer);
}

// ---------- 命令处理 ----------
void Cmd_CAL(void) {
    Process_StartProcedure(OP_CAL, cal_steps, sizeof(cal_steps)/sizeof(Step), 0);
}
void Cmd_SAMPLE(void) {
    Process_StartProcedure(OP_SAMPLE, sample_steps, sizeof(sample_steps)/sizeof(Step), 1);
    UART_SendString("SAMPLE_OK\r\n");
}
void Cmd_URINE(void) {
    Process_StartProcedure(OP_URINE, urine_steps, sizeof(urine_steps)/sizeof(Step), 1);
    UART_SendString("URINE_OK\r\n");
}
void Cmd_CLEAN(void) {
    Process_StartProcedure(OP_CLEAN, clean_steps, sizeof(clean_steps)/sizeof(Step), 1);
    UART_SendString("CLEAN_OK\r\n");
}
void Cmd_POWER_CLEAN(void) {
    Process_StartProcedure(OP_POWER_CLEAN, power_clean_steps, sizeof(power_clean_steps)/sizeof(Step), 1);
    UART_SendString("POWER_CLEAN_OK\r\n");
}
void Cmd_MAINT(void) {
    Process_StartProcedure(OP_MAINT, maint_steps, sizeof(maint_steps)/sizeof(Step), 0);
}
void Cmd_PRIME_STDA(void) {
    Process_StartProcedure(OP_PRIME_STDA, prime_stda_steps, sizeof(prime_stda_steps)/sizeof(Step), 0);
}
void Cmd_PRIME_STDB(void) {
    Process_StartProcedure(OP_PRIME_STDB, prime_stdb_steps, sizeof(prime_stdb_steps)/sizeof(Step), 0);
}
void Cmd_PRIME_REF(void) {
    Process_StartProcedure(OP_PRIME_REF, prime_ref_steps, sizeof(prime_ref_steps)/sizeof(Step), 0);
}
void Cmd_PRIME_DILUTED(void) {
    Process_StartProcedure(OP_PRIME_DILUTED, prime_diluted_steps, sizeof(prime_diluted_steps)/sizeof(Step), 0);
}
void Cmd_DISP_STDA(void) {
    Process_StartProcedure(OP_DISP_STDA, disp_stda_steps, sizeof(disp_stda_steps)/sizeof(Step), 0);
}
void Cmd_DISP_STDB(void) {
    Process_StartProcedure(OP_DISP_STDB, disp_stdb_steps, sizeof(disp_stdb_steps)/sizeof(Step), 0);
}
void Cmd_START(void) {
    if (procCtrl.state != SYS_WAITING_START) {
        UART_SendString("ERROR NOT WAITING FOR START\r\n");
        return;
    }
    const Step* steps = NULL;
    uint32_t count = 0;
    OperationType op = procCtrl.current_op;
    switch (op) {
        case OP_SAMPLE: steps = sample_steps; count = sizeof(sample_steps)/sizeof(Step); break;
        case OP_URINE: steps = urine_steps; count = sizeof(urine_steps)/sizeof(Step); break;
        case OP_CLEAN: steps = clean_steps; count = sizeof(clean_steps)/sizeof(Step); break;
        case OP_POWER_CLEAN: steps = power_clean_steps; count = sizeof(power_clean_steps)/sizeof(Step); break;
        default: UART_SendString("ERROR NO PENDING OPERATION\r\n"); procCtrl.state = SYS_IDLE; return;
    }
    procCtrl.state = SYS_RUNNING;
    procCtrl.current_step = 0;
    procCtrl.steps = steps;
    procCtrl.step_count = count;
    if (steps[0].action) steps[0].action();
    procCtrl.step_start_time = HAL_GetTick();
}
void Cmd_LAST_RESULT(void) {
    if (strlen(lastResults.sample_result) > 0) UART_SendString(lastResults.sample_result);
    if (strlen(lastResults.urine_result) > 0) UART_SendString(lastResults.urine_result);
}
void Cmd_LAST_SLOPE(void) {
    if (strlen(lastResults.slope_result) > 0) UART_SendString(lastResults.slope_result);
    else UART_SendString("NO SLOPE DATA\r\n");
}
void Cmd_READ_MV(void) {
    float na, k, cl, ref;
    Read_ElectrodeMV(&na, &k, &cl, &ref);
    UART_SendFormatted("MV_Na %6.2f K %6.2f Cl %6.2f\r\n", na, k, cl);
    UART_SendFormatted("MV_Na %6.2f K %6.2f Cl %6.2f\r\n", na-0.02f, k+0.01f, cl-0.02f);
}
void Cmd_VERSION(void) {
    UART_SendString(version_str);
}
void Cmd_ECAPE(void) {
    UART_SendString("INVALID CMD\r\n");
}
void Cmd_DEBUG_MODE(void) {
    debug_mode = 1;
    UART_SendString("DEBUG_MODE_OK\r\n");
}
void Cmd_DEBUG_OFF(void) {
    debug_mode = 0;
    UART_SendString("DEBUG_MODE_END\r\n");
}

// ---------- 电机控制函数 ----------
void Motor_InitAll(void) {
    HAL_GPIO_WritePin(MODEA0_GPIO_Port, MODEA0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MODEB0_GPIO_Port, MODEB0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MODEC0_GPIO_Port, MODEC0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MODEA1_GPIO_Port, MODEA1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MODEB1_GPIO_Port, MODEB1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MODEC1_GPIO_Port, MODEC1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MODEA2_GPIO_Port, MODEA2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MODEB2_GPIO_Port, MODEB2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MODEC2_GPIO_Port, MODEC2_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_Init(i);
    }
}

/**
  * @brief 初始化单个电机
  */
void Motor_Init(uint8_t id) {
    if (id >= MOTOR_COUNT) return;
    StepperMotor* motor = &motors[id];
    switch(id) {
        case 0:
            motor->step_port = M_STEP0_GPIO_Port; motor->step_pin = M_STEP0_Pin;
            motor->dir_port = M_DIR0_GPIO_Port; motor->dir_pin = M_DIR0_Pin;
            motor->en_port = M_EN0_GPIO_Port; motor->en_pin = M_EN0_Pin;
            break;
        case 1:
            motor->step_port = M_STEP1_GPIO_Port; motor->step_pin = M_STEP1_Pin;
            motor->dir_port = M_DIR1_GPIO_Port; motor->dir_pin = M_DIR1_Pin;
            motor->en_port = M_EN1_GPIO_Port; motor->en_pin = M_EN1_Pin;
            break;
        case 2:
            motor->step_port = M_STEP2_GPIO_Port; motor->step_pin = M_STEP2_Pin;
            motor->dir_port = M_DIR2_GPIO_Port; motor->dir_pin = M_DIR2_Pin;
            motor->en_port = M_EN2_GPIO_Port; motor->en_pin = M_EN2_Pin;
            break;
    }
		// 初始化参数调整
    motor->target_steps = 0;
    motor->current_steps = 0;
    motor->is_moving = 0;
    motor->direction = 1;
    motor->step_delay = STEP_DELAY_MS;
    HAL_GPIO_WritePin(motor->step_port, motor->step_pin, GPIO_PIN_RESET);
    Motor_SetDirection(id, 1);
    Motor_Enable(id, 0);
    HAL_Delay(10);
}

/**
  * @brief 使能/禁用电机
  */
void Motor_Enable(uint8_t motor_id, uint8_t enable) {
    if (motor_id >= MOTOR_COUNT) return;
    StepperMotor* motor = &motors[motor_id];
    HAL_GPIO_WritePin(motor->en_port, motor->en_pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
  * @brief 设置电机方向
  */
void Motor_SetDirection(uint8_t motor_id, uint8_t direction) {
    if (motor_id >= MOTOR_COUNT) return;
    StepperMotor* motor = &motors[motor_id];
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief 启动电机移动
  */
void Motor_StartMove(uint8_t motor_id, uint32_t steps, uint8_t dir) {
    if (motor_id >= MOTOR_COUNT) return;
    StepperMotor* motor = &motors[motor_id];
    motor->target_steps = steps;
    motor->current_steps = 0;
    motor->direction = dir;
    motor->is_moving = 1;
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
}

/**
  * @brief 停止全部电机
  */
void Motor_Stop_All(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) Motor_Stop(i);
}

/**
  * @brief 定时器中断中处理步进脉冲
  */
void Motor_ProcessStep(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        StepperMotor* motor = &motors[i];
        if (motor->is_moving && motor->current_steps < motor->target_steps) {
            HAL_GPIO_TogglePin(motor->step_port, motor->step_pin);
            motor->current_steps++;
            if (motor->current_steps >= motor->target_steps) {
                motor->is_moving = 0;
                Motor_Enable(i, 0);
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
        if (rx_index < RX_BUFFER_SIZE - 1) {
            if (rx_byte == '\n' || rx_byte == '\r') {
                if (rx_index > 0) {
                    rx_buffer[rx_index] = '\0';
                    strcpy(cmd_buffer, rx_buffer);
                    cmd_ready = 1;
                    rx_index = 0;
                }
            } else {
                rx_buffer[rx_index++] = (char)rx_byte;
            }
        } else {
            rx_index = 0;
        }
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
    if (steps > 0) sprintf(resp, "M%d %s %lu\r\n", motor_id, status, steps);
    else sprintf(resp, "M%d %s\r\n", motor_id, status);
    UART_SendString(resp);
}

// ---------- 命令解析 ----------
void Parse_Command(char *cmd) {
    while (*cmd == ' ') cmd++;
    char *end = cmd + strlen(cmd) - 1;
    while (end > cmd && (*end == ' ' || *end == '\r' || *end == '\n')) {
        *end = '\0';
        end--;
    }

    // 查询类命令
    if (strcmp(cmd, "VERSION") == 0) { Cmd_VERSION(); return; }
    if (strcmp(cmd, "LAST_RESULT") == 0) { Cmd_LAST_RESULT(); return; }
    if (strcmp(cmd, "LAST_SLOPE") == 0) { Cmd_LAST_SLOPE(); return; }
    if (strcmp(cmd, "READ_MV") == 0) { Cmd_READ_MV(); return; }
    if (strcmp(cmd, "ECAPE") == 0) { Cmd_ECAPE(); return; }
    if (strcmp(cmd, "DEBUG_MODE") == 0) { Cmd_DEBUG_MODE(); return; }
    if (strcmp(cmd, "DEBUG_OFF") == 0) { Cmd_DEBUG_OFF(); return; }

    // 电机控制
    if (cmd[0] == 'M' || cmd[0] == 'm') {
        uint8_t id; char dir; uint32_t steps;
        if (cmd[1] < '0' || cmd[1] > '2') { UART_SendString("Motor ID 0-2\r\n"); return; }
        id = cmd[1] - '0';
        dir = toupper(cmd[2]);
        if (dir == 'S') { Motor_Stop(id); UART_SendResponse("STOPPED", id, 0); return; }
        else if (dir == 'F' || dir == 'R') {
            char *s = cmd + 3;
            for (char *p = s; *p; p++) if (!isdigit(*p)) { UART_SendString("Steps number\r\n"); return; }
            steps = atoi(s);
            if (steps == 0) { UART_SendString("Steps >0\r\n"); return; }
            uint8_t d = (dir == 'F') ? 1 : 0;
            Motor_StartMove(id, steps, d);
            UART_SendFormatted("M%d %s %lu\r\n", id, d?"FORWARD":"REVERSE", steps);
        } else UART_SendString("Invalid direction\r\n");
        return;
    }

    // 状态检查
    if (procCtrl.state != SYS_IDLE) {
        if (strcmp(cmd, "START") == 0) {
            if (procCtrl.state == SYS_WAITING_START) Cmd_START();
            else UART_SendString("ERROR NOT WAITING FOR START\r\n");
        } else {
            UART_SendFormatted("ERROR %s RUNNING\r\n", cmd);
        }
        return;
    }

    // 空闲命令
    if (strcmp(cmd, "CAL") == 0) Cmd_CAL();
    else if (strcmp(cmd, "SAMPLE") == 0) Cmd_SAMPLE();
    else if (strcmp(cmd, "URINE") == 0) Cmd_URINE();
    else if (strcmp(cmd, "CLEAN") == 0) Cmd_CLEAN();
    else if (strcmp(cmd, "POWER_CLEAN") == 0) Cmd_POWER_CLEAN();
    else if (strcmp(cmd, "MAINT") == 0) Cmd_MAINT();
    else if (strcmp(cmd, "PRIME_STDA") == 0) Cmd_PRIME_STDA();
    else if (strcmp(cmd, "PRIME_STDB") == 0) Cmd_PRIME_STDB();
    else if (strcmp(cmd, "PRIME_REF") == 0) Cmd_PRIME_REF();
    else if (strcmp(cmd, "PRIME_DILUTED") == 0) Cmd_PRIME_DILUTED();
    else if (strcmp(cmd, "DISP_STDA") == 0) Cmd_DISP_STDA();
    else if (strcmp(cmd, "DISP_STDB") == 0) Cmd_DISP_STDB();
    else UART_SendString("INVALID CMD\r\n");
}

// ---------- FreeRTOS任务 ----------
void StartMotorTask(void const * argument) {
    Motor_InitAll();
    for (;;) {
        if (cmd_ready) {
            cmd_ready = 0;
            Parse_Command(cmd_buffer);
        }
        Process_CheckTimeout();
        Process_RunStep();
        osDelay(10);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();

    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    // 初始化光耦检测引脚
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = LIQUID_SENSE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(LIQUID_SENSE_PORT, &GPIO_InitStruct);

    UART_SendString("ISE Module System Ready (Liquid Detection via Optocoupler)\r\n");
    UART_SendString("Commands: M0F500, M0R500, M0S, CAL, SAMPLE, URINE, CLEAN, POWER_CLEAN, MAINT,\r\n");
    UART_SendString("PRIME_STDA, PRIME_STDB, PRIME_REF, PRIME_DILUTED, DISP_STDA, DISP_STDB,\r\n");
    UART_SendString("START, LAST_RESULT, LAST_SLOPE, READ_MV, VERSION, DEBUG_MODE, DEBUG_OFF\r\n");
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
