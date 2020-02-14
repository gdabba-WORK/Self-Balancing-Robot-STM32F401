/*
 * stepperMotor.c
 *
 *  Created on: 2020. 1. 31.
 *      Author: gdabba
 */

#include "stepperMotor.h"
#define MYDELAY 1

extern MPU6050_int32_t diffacc;

//void HAL_Delay(uint32_t Delay)
//{
//	uint32_t tickstart = HAL_GetTick();
//	uint32_t wait = Delay;
//	//	uint8_t count = 0;
//	//	char msg[20];
//	/* Add a freq to guarantee minimum wait */
//	if (wait < HAL_MAX_DELAY)
//	{
////		wait += (uint32_t)(uwTickFreq);
//	}
//	//	sprintf(msg, "start = %ld\nwait = %ld\n", tickstart, wait);
//	//	sprintf(msg, "%ld\n%ld\n", tickstart, wait);
//	//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
//	while((HAL_GetTick() - tickstart) < wait)
//	{
//		//		printf("while() %d\n", ++count);
//	}
//	//	sprintf(msg, "end = %ld\n", HAL_GetTick());
//	//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
//}

void bigStepper_forward_sequence(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_)
{
	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_RESET);
	HAL_Delay(MYDELAY);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_RESET);
	HAL_Delay(MYDELAY);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_SET);
	HAL_Delay(MYDELAY);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_SET);
	HAL_Delay(MYDELAY);
}

void bigStepper_forward_sequence2(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_)
{
	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_RESET);
	HAL_Delay(MYDELAY);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_RESET);
	HAL_Delay(MYDELAY);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_SET);
	HAL_Delay(MYDELAY);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_SET);
	HAL_Delay(MYDELAY);
}





void bigStepper_backward_sequence(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_)
{
	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_SET);
	HAL_Delay(MYDELAY);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_SET);
	HAL_Delay(MYDELAY);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_RESET);
	HAL_Delay(MYDELAY);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_RESET);
	HAL_Delay(MYDELAY);
}

void bigStepper_slower(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_)
{
	for (int i=2; i<=10; i++)
	{
		for (int j=0; j<50; j++)
		{
			HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_SET);
			HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_SET);
			HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_RESET);
			HAL_Delay(i);

			HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_SET);
			HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_SET);
			HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_RESET);
			HAL_Delay(i);

			HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_SET);
			HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_SET);
			HAL_Delay(i);

			HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_SET);
			HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_SET);
			HAL_Delay(i);
		}
	}
}

void bigStepper_forward_sequence_parallel(void)
{
	// AB상
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	HAL_Delay(MYDELAY);

	// `AB상
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	HAL_Delay(MYDELAY);

	// `A`B상
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_SET);
	HAL_Delay(MYDELAY);

	// A`B상
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_SET);
	HAL_Delay(MYDELAY);

	// Default
	//	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	//	HAL_Delay(MYDELAY);
}

void bigStepper_forward_sequence_parallel2(void)
{
	// A상
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	HAL_Delay(MYDELAY);

	// B상
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	HAL_Delay(MYDELAY);

	// `A상
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	HAL_Delay(MYDELAY);

	// `B상
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_SET);
	HAL_Delay(MYDELAY);

	// Default
	//	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	//	HAL_Delay(MYDELAY);
}

void bigStepper_backward_sequence_parallel(void)
{
	// A`B상
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_SET);
	HAL_Delay(MYDELAY);
	//	bigStepper_all_reset();

	// `A`B상
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_SET);
	HAL_Delay(MYDELAY);
	//	bigStepper_all_reset();

	// `AB상
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	HAL_Delay(MYDELAY);
	//	bigStepper_all_reset();

	// AB상
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	HAL_Delay(MYDELAY);



	// Default
	//	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
	//	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	//	HAL_Delay(MYDELAY);
}

void bigStepper_reactToAccel(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_)
{
	if (diffacc.Y > 0)
		bigStepper_forward_sequence(gpioA, pinA, gpioA_, pinA_, gpioB, pinB, gpioB_, pinB_);
	else
		bigStepper_backward_sequence(gpioA, pinA, gpioA_, pinA_, gpioB, pinB, gpioB_, pinB_);
}

void bigStepper_reactToAccel_parallel(void)
{
	//	if (diffacc.Y > 4000L)
	for (uint8_t i=0; i<200; i++)
	{
		bigStepper_forward_sequence_parallel();
		//		HAL_Delay(500);
	}
	//	bigStepper_all_reset();

	//	else if(diffacc.Y < -4000L)
	//		for (uint8_t i=0; i<12; i++)
	//			bigStepper_forward_sequence_parallel();
}

void bigStepper_all_reset(void)
{
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
}
