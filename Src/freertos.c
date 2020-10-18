/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
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
extern MPU6050_int16_t accOffset, gyroOffset;
extern uint32_t microTick;
extern MPU6050_float_t curr_filtered_angle;
extern float accel_xz, accel_yz;
extern MPU6050_float_t tmp_angle;
extern MPU6050_float_t accel_angle;
extern float dt_calc;
extern uint32_t dt_proc, dt_proc_max, t_from, t_to;
extern int8_t print_flag;
extern uint32_t t_prev, t_now;
extern float boundary_inner;

MPU6050_int16_t acc, gyro;
MPU6050_int32_t diffacc = {0, 0, 0};
MPU6050_int32_t diffgyro = {0, 0, 0};
int16_t tmpr;
uint32_t sync_period = 20UL;
uint32_t DLPF_DELAY = 1000UL;

int8_t init_flag = 0;
int8_t isWake = 0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 1024
};
/* Definitions for motorSync */
osThreadId_t motorSyncHandle;
const osThreadAttr_t motorSync_attributes = {
		.name = "motorSync",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 1024
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
uint8_t myQueue01Buffer[ 1 * sizeof( int8_t ) ];
osStaticMessageQDef_t myQueue01ControlBlock;
const osMessageQueueAttr_t myQueue01_attributes = {
		.name = "myQueue01",
		.cb_mem = &myQueue01ControlBlock,
		.cb_size = sizeof(myQueue01ControlBlock),
		.mq_mem = &myQueue01Buffer,
		.mq_size = sizeof(myQueue01Buffer)
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMotorSync(void *argument);

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

	/* Create the queue(s) */
	/* creation of myQueue01 */
	myQueue01Handle = osMessageQueueNew (1, sizeof(int8_t), &myQueue01_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of motorSync */
	motorSyncHandle = osThreadNew(StartMotorSync, NULL, &motorSync_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

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
	GPIO_PinState LD4_state = GPIO_PIN_RESET;

	step_reset(0UL);
	MPU6050_Init(MPU6050_DLPF_BW_5);
	accOffset.x = 931;
	accOffset.y = 676;
	accOffset.z = -7461;
	gyroOffset.x = -27;
	gyroOffset.y = 20;
	gyroOffset.z = -12;
	osThreadFlagsSet(motorSyncHandle, 0x0001U);
	initDT();

	/* Infinite loop */
	for(;;)
	{
		if ((MY_GetTick() - t_prev) >= DLPF_DELAY)
		{
			dt_calc = (MY_GetTick() - t_prev) / 1000000.0F;
			MPU6050_GetData(&acc.x, &acc.y, &acc.z, &gyro.x, &gyro.y, &gyro.z, &tmpr);
			t_prev = MY_GetTick();

			diffacc.x = (int32_t)(acc.x - accOffset.x);
			diffacc.y = (int32_t)(acc.y - accOffset.y);
			diffacc.z  = (int32_t)(acc.z - accOffset.z);
			diffgyro.x = (int32_t)(gyro.x - gyroOffset.x);
			diffgyro.y = (int32_t)(gyro.y - gyroOffset.y);
			diffgyro.z = (int32_t)(gyro.z - gyroOffset.z);
			calcAccelYPR();
			calcGyroYPR();
			calcAngularAccelYPR();
			calcFilteredYPR();
			if (fabs(curr_filtered_angle.x) <= boundary_inner)
			{
				if (LD4_state == GPIO_PIN_RESET)
				{
					HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
					LD4_state = GPIO_PIN_SET;
				}
			}
			else
			{
				if (LD4_state == GPIO_PIN_SET)
				{
					HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
					LD4_state = GPIO_PIN_RESET;
				}
			}
		}
	}
	/* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMotorSync */
/**
 * @brief Function implementing the motorSync thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMotorSync */
void StartMotorSync(void *argument)
{
	/* USER CODE BEGIN StartMotorSync */
	osThreadFlagsWait(0x0001U, osFlagsWaitAll, osWaitForever);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	osThreadYield();

	/* Infinite loop */
	for(;;)
	{
		reactToAngleGyro(defaultTaskHandle);
	}
	/* USER CODE END StartMotorSync */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
