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
#include "oled.h"
#include "AS5600.h"
#include "FOC.h"
#include "CurrentLoop.h"
#include "vofa_telemetry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void MotorTask(void *argument);
void CommTask(void *argument);
void DisplayTask(void *argument);

TaskHandle_t MotorTask_Handler;   //������
TaskHandle_t CommTask_Handler;
TaskHandle_t DisplayTask_Handler;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float angle;
volatile float theta_e;
float Motor_MM;
/* 0=boot, 1=current calibration, 2=angle alignment, 3=closed loop, 4=fault. */
volatile uint32_t motor_control_state;
volatile uint32_t motor_sensor_fail_count;
volatile uint32_t motor_last_angle_adc_count;

volatile BaseType_t motor_create;
volatile BaseType_t comm_create;
volatile BaseType_t display_create;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

	taskENTER_CRITICAL();
	motor_create = xTaskCreate(
			MotorTask, "task1", 128, NULL, 30, &MotorTask_Handler
	);

	comm_create = xTaskCreate(
			CommTask, "task2", 256, NULL, 22, &CommTask_Handler
	);

	display_create = xTaskCreate(
			DisplayTask, "task3", 128, NULL, 18, &DisplayTask_Handler
	);
	taskEXIT_CRITICAL();

	vTaskDelete(NULL);
//	for(;;);
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void MotorTask(void *argument)
{
    uint8_t sensor_ok = 0;
    TickType_t calibration_start;    //启动时间
    TickType_t previous_wake_time;	 //上一次运行时间

    motor_control_state = 1U;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
    FOC_SetAlignVoltage(0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(100));

    CurrentSense_BeginCalibration();
    CurrentSense_Start();
    calibration_start = xTaskGetTickCount();
    while (!motor_current.offset_ready && (xTaskGetTickCount() - calibration_start) < pdMS_TO_TICKS(100))//等待电流校准
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (motor_current.offset_ready)//电流校准完毕后校准角度
    {
        sensor_ok = (as5600_read_byte(&angle, &Motor_MM, 0.002f) == HAL_OK);
        if (!sensor_ok) motor_sensor_fail_count++;
        if (sensor_ok)
        {
            motor_control_state = 2U;
            FOC_SetAlignVoltage(0.7f, 0.0f);
            vTaskDelay(pdMS_TO_TICKS(3000));
            sensor_ok = (as5600_read_byte(&angle, &Motor_MM, 0.5f) == HAL_OK);
            if (!sensor_ok) motor_sensor_fail_count++;
            if (sensor_ok)
            {
                FOC_CalibrateZero(angle);
                theta_e = FOC_GetElectricalAngle(angle);
                motor_last_angle_adc_count = motor_current.sample_count;
                FOC_SetAlignVoltage(0.0f, 0.0f);
                CurrentLoop_SetTarget(0.0f, 0.2f);
                CurrentLoop_SetEnabled(1);
                motor_control_state = 3U;
            }
        }
    }

    if (!sensor_ok)
    {
        motor_control_state = 4U;
        CurrentLoop_SetEnabled(0);
        CurrentLoop_SetTarget(0.0f, 0.0f);
        FOC_SetAlignVoltage(0.0f, 0.0f);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
    }

    previous_wake_time = xTaskGetTickCount();
    while (1)
    {
        if (sensor_ok && as5600_read_byte(&angle, &Motor_MM, 0.001f) == HAL_OK)  //执行控制
        {
            theta_e = FOC_GetElectricalAngle(angle);
            motor_last_angle_adc_count = motor_current.sample_count;
            xTaskNotifyGive(DisplayTask_Handler);
        }
        else if (sensor_ok)       //异常断开电机
        {
            sensor_ok = 0;
            motor_sensor_fail_count++;
            motor_control_state = 4U;
            CurrentLoop_SetEnabled(0);
            CurrentLoop_SetTarget(0.0f, 0.0f);
            FOC_SetAlignVoltage(0.0f, 0.0f);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
        }

        vTaskDelayUntil(&previous_wake_time, pdMS_TO_TICKS(1));
    }
}




void CommTask(void *argument)
{
    TickType_t previous_wake_time = xTaskGetTickCount();

    if (VOFA_TelemetryInit() != HAL_OK)
    {
        for (;;)
            vTaskDelay(pdMS_TO_TICKS(1000));
    }

    for (;;)
    {
        VOFA_TelemetrySend();
        vTaskDelayUntil(&previous_wake_time,
                        pdMS_TO_TICKS(VOFA_TELEMETRY_PERIOD_MS));
    }
}




void DisplayTask(void *argument){
	OLED_Print(0, 0, "FOC: RUNNING");
	OLED_Print(0, 1, "Angle:");
	OLED_Print(0, 2, "RPM:");
	OLED_Print(0, 3, "Theta:");
	OLED_Update();
	while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		OLED_Print(0, 1, "Angle:");
		OLED_PrintFixed(7, 1,(uint16_t)(angle*100/3.1415926f), 2);
		OLED_Print(0, 2, "RPM:");
		OLED_PrintFixed(5, 2,(uint16_t)(Motor_MM*100), 2);
		OLED_Print(0, 3, "Theta:");
		OLED_PrintFixed(7, 3,(uint16_t)(theta_e*100), 2);
		OLED_Update();
		vTaskDelay(100);
	}
}


/* USER CODE END Application */

