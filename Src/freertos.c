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
#include "complementary_filter.h"
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
extern float_t accel_xz, accel_yz;
extern MPU6050_float_t tmp_angle;
extern MPU6050_float_t accel_angle;
extern float_t dt;

MPU6050_int16_t acc, gyro;
MPU6050_int32_t diffacc = {0, 0, 0};
MPU6050_int32_t diffgyro = {0, 0, 0};
int16_t tmpr;


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024
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
  .stack_size = 1024
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
uint8_t myQueue01Buffer[ 16 * sizeof( int8_t ) ];
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

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(int8_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of motorLeft */
//  motorLeftHandle = osThreadNew(StartMotorLeft, NULL, &motorLeft_attributes);

  /* creation of motorRight */
//  motorRightHandle = osThreadNew(StartMotorRight, NULL, &motorRight_attributes);

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
	//	char msg[100];
	osStatus_t status = osError;
	uint32_t prev_tick;
	int8_t angle;
	//	uint16_t count = 0;

	MPU6050_Init(MPU6050_DLPF_BW_42);
	MPU6050_InitOffset(&acc.x, &acc.y, &acc.z, &gyro.x, &gyro.y, &gyro.z, &tmpr);

	//	sprintf(msg, "%d,%d,%d,%d,%d,%d\r\n", accOffset.x, accOffset.y, accOffset.z, gyroOffset.x, gyroOffset.y, gyroOffset.z);
	//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 3000UL);
	prev_tick = HAL_GetTick();
	initDT();
	/* Infinite loop */
	for(;;)
	{
		//		sprintf(msg, "(%4d)%5ld,%+5ld,%+5ld\r\n", ++count, diffacc.X, diffacc.Y, diffacc.Z);
		//		printf(msg);
		//		sprintf(msg, "{\"C\":%d,\"X\":%ld,\"Y\":%ld,\"Z\":%ld}\r\n", ++count, diffacc.X, diffacc.Y, diffacc.Z);

		//		sprintf(msg, "dt=%8.8f\r\n", 10.0F);
		//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10UL);

		MPU6050_GetData(&acc.x, &acc.y, &acc.z, &gyro.x, &gyro.y, &gyro.z, &tmpr);
		diffacc.x = (int32_t)(acc.x - accOffset.x);
		diffacc.y = (int32_t)(acc.y - accOffset.y);
		diffacc.z  = (int32_t)(acc.z - accOffset.z);
		diffgyro.x = (int32_t)(gyro.x - gyroOffset.x);
		diffgyro.y = (int32_t)(gyro.y - gyroOffset.y);
		diffgyro.z = (int32_t)(gyro.z - gyroOffset.z);
		calcDT();
		calcAccelYPR();
		calcGyroYPR();
		calcFilteredYPR(&angle);
		//		sprintf(msg, "status=%d\r\n", status);
		//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 3000UL);

		//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 3000UL);
		//		sprintf(msg, "{\"C\":%d,\"Y\":%ld}\r\n", ++count, diffacc.Y);
		//		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);


		if ((HAL_GetTick() - prev_tick) >= 250UL)
		{
			osThreadFlagsSet(motorSyncHandle, 0x0001U);
			osThreadFlagsWait(0x0001U, osFlagsWaitAll, osWaitForever);
			status = osMessageQueuePut(myQueue01Handle, &angle, 0U, 100U);
			osThreadYield();
			osThreadFlagsSet(motorSyncHandle, 0x0001U);

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
			prev_tick = HAL_GetTick();
		}
		osThreadYield();
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
	osThreadFlagsClear(0x0002U);
	/* Infinite loop */
	for(;;)
	{
		osThreadFlagsWait(0x0001U, osFlagsWaitAll, osWaitForever);
		//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		osThreadFlagsSet(defaultTaskHandle, 0x0001U);
		osThreadFlagsSet(motorSyncHandle, 0x0001U);
		//		HAL_UART_Transmit(&huart2, (uint8_t*)leftMsg, strlen(leftMsg), 0xFFFF);

		//		if (leftSeq == *argument)
		//		{
		//		bigStepper_reactToAccel(SM2A_GPIO_Port, SM2A_Pin, SM2A__GPIO_Port, SM2A__Pin, SM2B_GPIO_Port, SM2B_Pin, SM2B__GPIO_Port, SM2B__Pin);
		//		bigStepper_forward_sequence2(SM2A_GPIO_Port, SM2A_Pin, SM2A__GPIO_Port, SM2A__Pin, SM2B_GPIO_Port, SM2B_Pin, SM2B__GPIO_Port, SM2B__Pin);
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
	uint8_t count = 0U;
	/* Infinite loop */
	for(;;)
	{
		//		HAL_UART_Transmit(&huart2, (uint8_t*)rightMsg, strlen(rightMsg), 0xFFFF);

		//		if (rightSeq == *argument)
		//		{
		//		reactToAccel(SM1A_GPIO_Port, SM1A_Pin, SM1A__GPIO_Port, SM1A__Pin, SM1B_GPIO_Port, SM1B_Pin, SM1B__GPIO_Port, SM1B__Pin);
		//			rightSeq++;
		//			osSignalSet(motorSyncHandle, (int32_t)200);
		//		}
		//		while(1);
		count++;
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
	osStatus_t status = osError;
	uint32_t prev_tick;
	int8_t qrcv = 0;
	Robot_Direction direction_flag = FORWARD;

	osDelay(5000);
	prev_tick = HAL_GetTick();
	/* Infinite loop */
	for(;;)
	{
		reactToAccel_parallel(direction_flag);
		if ((HAL_GetTick() - prev_tick) >= 250UL)
		{
			osThreadFlagsSet(defaultTaskHandle, 0x0001U);
			osThreadFlagsWait(0x0001U, osFlagsWaitAll, osWaitForever);
			status = osMessageQueueGet(myQueue01Handle, &qrcv, NULL, 100U);
			osThreadYield();
			osThreadFlagsSet(defaultTaskHandle, 0x0001U);
//			if (status == osOK)
			if (qrcv > 0)
				direction_flag = FORWARD;
			else
				direction_flag = BACKWARD;
			prev_tick = HAL_GetTick();
		}
		osThreadYield();

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
