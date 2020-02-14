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
#include "stepperMotor.h"
#include "MPU6050.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define leftSignal 100L
#define rightSignal 200L
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern MPU6050_int16_t accOffset, gyroOffset;
MPU6050_int16_t acc, gyro;
MPU6050_int32_t diffacc = {0, 0, 0};
MPU6050_int32_t diffgyro = {0, 0, 0};
int16_t tmpr;


uint8_t mediumSeq = 0;
char leftMsg[3] = "L\r\n";
char rightMsg[3] = "R\r\n";
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128
};
/* Definitions for motorLeft */
osThreadId_t motorLeftHandle;
const osThreadAttr_t motorLeft_attributes = {
  .name = "motorLeft",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128
};
/* Definitions for motorRight */
osThreadId_t motorRightHandle;
const osThreadAttr_t motorRight_attributes = {
  .name = "motorRight",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128
};
/* Definitions for motorSync */
osThreadId_t motorSyncHandle;
const osThreadAttr_t motorSync_attributes = {
  .name = "motorSync",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void calibAccelGyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *tmpr);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMotorLeft(void *argument);
void StartMotorRight(void *argument);
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

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of motorLeft */
  motorLeftHandle = osThreadNew(StartMotorLeft, NULL, &motorLeft_attributes);

  /* creation of motorRight */
  motorRightHandle = osThreadNew(StartMotorRight, NULL, &motorRight_attributes);

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
//	uint16_t count = 0;
	//	uint32_t mystart = HAL_GetTick();
	//	int8_t myflag = 0;

	//	printf("Default()\n");

	MPU6050_Init(MPU6050_DLPF_BW_42);
	MPU6050_InitOffset(&acc.X, &acc.Y, &acc.Z, &gyro.X, &gyro.Y, &gyro.Z, &tmpr);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	//	sprintf(msg, "%d,%d,%d,%d,%d,%d\n", accOffset.X, accOffset.Y, accOffset.Z, gyroOffset.X, gyroOffset.Y, gyroOffset.Z);
	//	printf(msg);

	/* Infinite loop */
	for(;;)
	{
		//		printf("%5d//", ++count);
		MPU6050_GetData(&acc.X, &acc.Y, &acc.Z, &gyro.X, &gyro.Y, &gyro.Z, &tmpr);
		diffacc.X = acc.X - accOffset.X;
		diffacc.Y = acc.Y - accOffset.Y;
		diffacc.Z = acc.Z - accOffset.Z;
		diffgyro.X = gyro.X - gyroOffset.X;
		diffgyro.Y = gyro.Y - gyroOffset.Y;
		diffgyro.Z = gyro.Z - gyroOffset.Z;
		//		sprintf(msg, "(%4d)%5ld,%+5ld,%+5ld\r\n", ++count, diffacc.X, diffacc.Y, diffacc.Z);
		//		printf(msg);
//		sprintf(msg, "{\"C\":%d,\"X\":%ld,\"Y\":%ld,\"Z\":%ld}\r\n", ++count, diffacc.X, diffacc.Y, diffacc.Z);
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0x0A);
		osDelay(10);
		//				count++;
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

/* USER CODE BEGIN Header_StartMotorLeft */
/**
 * @brief Function implementing the motorLeft thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMotorLeft */
void StartMotorLeft(void *argument)
{
  /* USER CODE BEGIN StartMotorLeft */
	//	printf("Left()\n");
	//	uint8_t leftSeq = 0;
	//	uint16_t count = 0;
	//	char msg[10] = "On/Off";
	/* Infinite loop */
	for(;;)
	{
		//		HAL_UART_Transmit(&huart2, (uint8_t*)leftMsg, strlen(leftMsg), 0xFFFF);

		//		if (leftSeq == *argument)
		//		{
		//		bigStepper_reactToAccel(SM2A_GPIO_Port, SM2A_Pin, SM2A__GPIO_Port, SM2A__Pin, SM2B_GPIO_Port, SM2B_Pin, SM2B__GPIO_Port, SM2B__Pin);
		bigStepper_forward_sequence2(SM2A_GPIO_Port, SM2A_Pin, SM2A__GPIO_Port, SM2A__Pin, SM2B_GPIO_Port, SM2B_Pin, SM2B__GPIO_Port, SM2B__Pin);
		//		if (++count == 500)
		//		{
		//			HAL_GPIO_WritePin(SM1EN_GPIO_Port, SM1EN_Pin, GPIO_PIN_RESET);
		//			HAL_GPIO_WritePin(SM2EN_GPIO_Port, SM2EN_Pin, GPIO_PIN_RESET);
		//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
		//		}
		//		if (count == 1000)
		//		{
		//			HAL_GPIO_WritePin(SM1EN_GPIO_Port, SM1EN_Pin, GPIO_PIN_SET);
		//			HAL_GPIO_WritePin(SM2EN_GPIO_Port, SM2EN_Pin, GPIO_PIN_RESET);
		//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
		//		}
		//		if (count == 1500)
		//		{
		//			HAL_GPIO_WritePin(SM1EN_GPIO_Port, SM1EN_Pin, GPIO_PIN_SET);
		//			HAL_GPIO_WritePin(SM2EN_GPIO_Port, SM2EN_Pin, GPIO_PIN_SET);
		//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
		//		}
		//		if (count == 2000)
		//		{
		//			HAL_GPIO_WritePin(SM1EN_GPIO_Port, SM1EN_Pin, GPIO_PIN_RESET);
		//			HAL_GPIO_WritePin(SM2EN_GPIO_Port, SM2EN_Pin, GPIO_PIN_SET);
		//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
		//		}
		//			leftSeq++;
		//			osSignalSet(motorSyncHandle, (int32_t)100);
		//		}
		//		while(1);
	}
  /* USER CODE END StartMotorLeft */
}

/* USER CODE BEGIN Header_StartMotorRight */
/**
 * @brief Function implementing the motorRight thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMotorRight */
void StartMotorRight(void *argument)
{
  /* USER CODE BEGIN StartMotorRight */
	//	uint8_t rightSeq = 0;
	//	printf("Right()\n");
	/* Infinite loop */
	for(;;)
	{
		//		HAL_UART_Transmit(&huart2, (uint8_t*)rightMsg, strlen(rightMsg), 0xFFFF);

		//		if (rightSeq == *argument)
		//		{
		bigStepper_reactToAccel(SM1A_GPIO_Port, SM1A_Pin, SM1A__GPIO_Port, SM1A__Pin, SM1B_GPIO_Port, SM1B_Pin, SM1B__GPIO_Port, SM1B__Pin);
		//			rightSeq++;
		//			osSignalSet(motorSyncHandle, (int32_t)200);
		//		}
		//		while(1);
	}
  /* USER CODE END StartMotorRight */
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
	//	uint32_t mystart = HAL_GetTick();
	//	int8_t myflag = 0;
	//	uint16_t count = 0;
	//	char msg[20];
	/* Infinite loop */
	for(;;)
		//	  printf("HelloWorld!\n");
	{
		//	  osSignalWait(300, 0);
		bigStepper_reactToAccel_parallel();
		//		count++;
		//		if (HAL_GetTick() - mystart > 10000 && myflag == 0)
		//		{
		//			sprintf(msg, "count2 = %d\n", count);
		//			//					printf(msg);
		//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
		//			myflag = 1;
		//		}
	}
  /* USER CODE END StartMotorSync */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//void calibAccelGyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *tmpr)
//{
//	int32_t sumAcX = 0;
//	int32_t sumAcY = 0;
//	int32_t sumAcZ = 0;
//	int32_t sumGyX = 0;
//	int32_t sumGyY = 0;
//	int32_t sumGyZ = 0;
//
//	int16_t max = 1000;
//	//초기 보정값은 10번의 가속도 자이로 센서의 값을 받아 해당 평균값을 가진다.//
//	for(int i=0; i<max; i++){
//		MPU6050_GetData(ax, ay, az, gx, gy, gz, tmpr);
//		sumAcX += *ax, sumAcY += *ay, sumAcZ += *az;
//		sumGyX += *gx, sumGyY += *gy, sumGyZ += *gz;
//		osDelay(1);
//	}
//
//	baseAcX = sumAcX / max;
//	baseAcY = sumAcY / max;
//	baseAcZ = sumAcZ / max;
//
//	baseGyX = sumGyX / max;
//	baseGyY = sumGyY / max;
//	baseGyZ = sumGyZ / max;
//}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
