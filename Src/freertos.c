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
extern MPU6050_float_t filtered_angle;
extern float accel_xz, accel_yz;
extern MPU6050_float_t tmp_angle;
extern MPU6050_float_t accel_angle;
extern float dt;
extern int8_t print_flag;
extern uint32_t t_prev, t_now;

MPU6050_int16_t acc, gyro;
MPU6050_int32_t diffacc = {0, 0, 0};
MPU6050_int32_t diffgyro = {0, 0, 0};
int16_t tmpr;
uint32_t sync_period = 20UL;


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
//void calibAccelGyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *tmpr);
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
//	char msg[50];
//	osStatus_t status = osError;
//	uint32_t prev_tick;
//	int8_t _angle = 0;
	//	uint32_t count = 0;

	MPU6050_Init(MPU6050_DLPF_BW_42);
//	MPU6050_InitOffset(&acc.x, &acc.y, &acc.z, &gyro.x, &gyro.y, &gyro.z, &tmpr);
	accOffset.x = 1920;
	accOffset.y = 1369;
	accOffset.z = -14669;
	gyroOffset.x = 0;
	gyroOffset.y = 33;
	gyroOffset.z = -8;


	//	sprintf(msg, "%d,%d,%d,%d,%d,%d\r\n", accOffset.x, accOffset.y, accOffset.z, gyroOffset.x, gyroOffset.y, gyroOffset.z);
	//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 3000UL);
	osThreadFlagsSet(motorSyncHandle, 0x0001U);
//	prev_tick = HAL_GetTick();
	initDT();
	/* Infinite loop */
	for(;;)
	{
		//		sprintf(msg, "(%4d)%5ld,%+5ld,%+5ld\r\n", ++count, diffacc.X, diffacc.Y, diffacc.Z);
		//		printf(msg);
		//		sprintf(msg, "{\"C\":%d,\"X\":%ld,\"Y\":%ld,\"Z\":%ld}\r\n", ++count, diffacc.X, diffacc.Y, diffacc.Z);

		//		sprintf(msg, "dt=%8.8f\r\n", 10.0F);
		//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10UL);
		//		if ((MY_GetTick() - prev_tick2) >= 4800UL)
		if ((MY_GetTick() - t_prev) >= 4800UL)
		{
			MPU6050_GetData(&acc.x, &acc.y, &acc.z, &gyro.x, &gyro.y, &gyro.z, &tmpr);
			calcDT();
			diffacc.x = (int32_t)(acc.x - accOffset.x);
			diffacc.y = (int32_t)(acc.y - accOffset.y);
			diffacc.z  = (int32_t)(acc.z - accOffset.z);
			diffgyro.x = (int32_t)(gyro.x - gyroOffset.x);
			diffgyro.y = (int32_t)(gyro.y - gyroOffset.y);
			diffgyro.z = (int32_t)(gyro.z - gyroOffset.z);
			calcAccelYPR();
			calcGyroYPR();
			calcFilteredYPR();
		}
		//		count++;
		//		sprintf(msg, "status=%d\r\n", status);
		//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 3000UL);

		//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 3000UL);
		//		sprintf(msg, "{\"C\":%d,\"Y\":%ld}\r\n", ++count, diffacc.Y);
		//		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);

//		if ((HAL_GetTick() - prev_tick) >= sync_period)
//		{
			//			osThreadFlagsSet(motorSyncHandle, 0x0001U);
			//			osThreadFlagsWait(0x0001U, osFlagsWaitAll, osWaitForever);
//			status = osMessageQueuePut(myQueue01Handle, &_angle, 0U, 100UL);

			//			osThreadYield();
			//			osThreadFlagsSet(motorSyncHandle, 0x0001U);

			//			sprintf(msg, "Tick=%10lu\r\n", microTick);
			//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 3000UL);

			//			if (status == osOK)
			//				sprintf(msg, "C=%5d  D=%12f  T=%12f  A=%12f  R=%12u\r\n", count, filtered_angle.x, tmp_angle.x, accel_angle.x, qmsg);
			//			else
			//								sprintf(msg, "C=%5d  X=%12ld  Y=%12ld  Z=%12ld\r\n", count, diffacc.x, diffacc.y, diffacc.z);
			//				sprintf(msg, "C=%5d  X=%12d  Y=%12d  Z=%12d\r\n", count, acc.x, acc.y, acc.z);
			//				sprintf(msg, "C=%5d  X=%12d  Y=%12d  Z=%12d\r\n", count, gyro.x, gyro.y, gyro.z);
			//				sprintf(msg, "C=%5d  D=%+12f  T=%+12f  A=%+12f\r\n", count, filtered_angle.x, tmp_angle.x, accel_angle.x);
			//				sprintf(msg, "pow(-10,2)=%12lf\r\n", pow(-10, 2));
			//				sprintf(msg, "angle=%+5d\r\n", angle);
			//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 3000UL);

			//			sprintf(msg, "diffacc.x=%8ld  diffacc.y=%8ld  diffacc.z=%8ld\r\n", diffacc.x, diffacc.y, diffacc.z);
			//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 3000UL);
			//			sprintf(msg, "diffgyro.x=%8ld  diffgyro.y=%8ld  diffgyro.z=%8ld\r\n", diffgyro.x, diffgyro.y, diffgyro.z);
			//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 3000UL);

			//			sprintf(msg, "acc.x=%8d  acc.y=%8d  acc.z=%8d\r\n", acc.x, acc.y, acc.z);
			//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 3000UL);

			//			sprintf(msg, "gyro.x=%8d  gyro.y=%8d  gyro.z=%8d\r\n", gyro.x, gyro.y, gyro.z);
			//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 3000UL);

			//			sprintf(msg, "dt=%8.8f\r\n", dt);
			//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 3000UL);
			//			count++;
//			prev_tick = HAL_GetTick();
//		}

		//		if ((HAL_GetTick() - prev_tick2) >= 10000UL)
		//		{
		//			sprintf(msg, "%lu\r\n", count);
		//			HAL_UART_Transmit(&huart2, msg, strlen(msg), 3000UL);
		//			prev_tick2 = HAL_GetTick();
		//		}
		//		osThreadYield();
		//		if (HAL_OK == HAL_UART_Receive(&huart1, (uint8_t*)rxData, 1U, 3000UL))
		//		{
		//			HAL_UART_Transmit(&huart2, (uint8_t*)rxData, 1U, 0x0A);
		//		}

		//		osDelay(2);

		//		if (HAL_GetTick() - mystart > 10000 && myflag == 0)
		//		{
		//			printf("count1 = %d\n", count);
		//			myflag = 1;
		//		}

		//						sprintf(msg, "ax=%d | ay=%d | az=%d\n", ax, ay, az);
		//				printf(msg);
		//				sprintf(msg, "gx=%d | gy=%d | gz=%d\n", gx, gy, gz);
		//				printf(msg);
		//				sprintf(msg, "tmpr=%d\n", tmpr);
		//				printf(msg);
		//		calibAccelGyro(&ax, &ay, &az, &gx, &gy, &gz, &tmpr);

		//		sprintf(msg, "ax=%ld | ay=%ld | az=%ld | ", baseAcX, baseAcY, baseAcZ);
		//		printf(msg);
		//		sprintf(msg, "gx=%ld | gy=%ld | gz=%ld\n", baseGyX, baseGyY, baseGyZ);
		//		printf(msg);


		//		sprintf(msg, "tmpr=%d\n", tmpr);
		//		printf(msg);
		//		osDelay(1);
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
//	osStatus_t status = osError;
//	GPIO_PinState prev_state =GPIO_PIN_RESET;
//	uint32_t prev_tick;
	//	uint32_t prev_tick2;
	//	uint32_t count = 0;
	//	char msg[50];
	//	int8_t angle = 0;

	osThreadFlagsWait(0x0001U, osFlagsWaitAll, osWaitForever);
	osThreadYield();
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//	prev_tick = HAL_GetTick();
	//	prev_tick2 = HAL_GetTick();
	/* Infinite loop */
	for(;;)
	{
		//		count++;
//		if ((HAL_GetTick() - prev_tick) >= sync_period)
//		{
			//			osThreadFlagsSet(defaultTaskHandle, 0x0001U);
			//			osThreadFlagsWait(0x0001U, osFlagsWaitAll, osWaitForever);
//			status = osMessageQueueGet(myQueue01Handle, &angle, NULL, 100UL);
			//			osThreadYield();
			//			osThreadFlagsSet(defaultTaskHandle, 0x0001U);
//			if (status == osOK)
//			{
//				if (prev_state != GPIO_PIN_SET)
//				{
//					HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//					prev_state = GPIO_PIN_SET;
//				}
//			}
//			else
//			{
//				if (prev_state != GPIO_PIN_RESET)
//				{
//					HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//					prev_state = GPIO_PIN_RESET;
//				}
//
//			}
//			prev_tick = HAL_GetTick();
//		}
		reactToAngleGyro();

		//		if ((HAL_GetTick() - prev_tick2) >= 10000UL)
		//		{
		//			sprintf(msg, "%lu\r\n", count);
		//			HAL_UART_Transmit(&huart2, msg, strlen(msg), 3000UL);
		//			prev_tick2 = HAL_GetTick();
		//		}
	}
	/* USER CODE END StartMotorSync */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
