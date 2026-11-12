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
#include "FOC.h"
#include "vofa_telemetry.h"
#include "haptic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void MotorTask(void *argument);
void DisplayTask(void *argument);
void CommTask(void *argument);

TaskHandle_t MotorTask_Handler;   //������
TaskHandle_t DisplayTask_Handler;
TaskHandle_t CommTask_Handler;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile BaseType_t motor_create;
volatile BaseType_t display_create;
volatile BaseType_t comm_create;
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
        CommTask, "comm", 256, NULL, 22, &CommTask_Handler
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
    FOC_Feedback_t feedback;
    TickType_t previous_wake_time = xTaskGetTickCount();
    uint32_t last_ms = HAL_GetTick();

    FOC_Start(last_ms);
    FOC_SetControlMode(FOC_MODE_SPEED);
    FOC_SetTargetSpeed(0.0f);

    for (;;)
    {
        uint32_t now_ms = HAL_GetTick();
        float dt = (now_ms - last_ms) * 0.001f;
        last_ms = now_ms;

        if (FOC_Update(now_ms))
        {
            FOC_GetFeedback(&feedback);
//            Roller(&feedback, dt);
//							Song_Control(&feedback);
							Volume_Control(&feedback);
//						Snap_fastener(&feedback);
            if (DisplayTask_Handler != NULL)
                xTaskNotifyGive(DisplayTask_Handler);
        }

        vTaskDelayUntil(&previous_wake_time, pdMS_TO_TICKS(1));
    }
}


void DisplayTask(void *argument){
    FOC_Feedback_t feedback;
	OLED_Print(0, 0, "FOC: RUNNING");
	OLED_Print(0, 1, "Angle:");
	OLED_Print(0, 2, "RPM:");
	OLED_Print(0, 3, "Theta:");
	OLED_Update();
	while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        FOC_GetFeedback(&feedback);
		OLED_Print(0, 1, "Angle:");
		OLED_PrintFixed(7, 1,(uint16_t)(feedback.mechanical_angle*100/3.1415926f), 2);
		OLED_Print(0, 2, "RPM:");
		OLED_PrintFixed(5, 2,(uint16_t)(feedback.mechanical_speed*100), 2);
		OLED_Print(0, 3, "Theta:");
		OLED_PrintFixed(7, 3,(uint16_t)(feedback.electrical_angle*100), 2);
		OLED_Update();
		vTaskDelay(100);
	}
}



void CommTask(void *argument)
{
    TickType_t previous_wake_time = xTaskGetTickCount();
    if (VOFA_TelemetryInit() != HAL_OK)
    {
        vofa_errors++;
        vTaskDelete(NULL);
        return;
    }
    for (;;)
    {
        VOFA_TelemetrySend();
        vTaskDelayUntil(&previous_wake_time, pdMS_TO_TICKS(VOFA_TELEMETRY_PERIOD_MS));
    }
}

/* USER CODE END Application */

