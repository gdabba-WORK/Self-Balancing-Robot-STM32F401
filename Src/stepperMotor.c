/*
 * stepperMotor.c
 *
 *  Created on: 2020. 1. 31.
 *      Author: gdabba
 */

#include "stepperMotor.h"

extern MPU6050_int32_t diffacc;

uint8_t step = 0U;
uint8_t step_delay_dynamic = 10U;
uint8_t step_delay_static = 0U;

Robot_Direction direction_flag = FORWARD;
Motor_Mode mode_flag = ONETWO_PHASE;
Motor_State state_flag = BOTH_MOTOR;


void HAL_Delay(uint32_t Delay)
{
	uint32_t tickstart = HAL_GetTick();
	uint32_t wait = Delay;
	//	uint8_t count = 0;
	//	char msg[20];
	/* Add a freq to guarantee minimum wait */
	if (wait < HAL_MAX_DELAY)
	{
		wait += (uint32_t)(uwTickFreq);
	}
	//	sprintf(msg, "start = %ld\nwait = %ld\n", tickstart, wait);
	//	sprintf(msg, "%ld\n%ld\n", tickstart, wait);
	//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	while((HAL_GetTick() - tickstart) < wait)
	{
		//		printf("while() %d\n", ++count);
	}
	//	sprintf(msg, "end = %ld\n", HAL_GetTick());
	//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
}

void bigStepper_forward_sequence(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_)
{
	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_RESET);
	HAL_Delay(step_delay_dynamic);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_RESET);
	HAL_Delay(step_delay_dynamic);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_SET);
	HAL_Delay(step_delay_dynamic);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_SET);
	HAL_Delay(step_delay_dynamic);
}

void bigStepper_forward_sequence2(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_)
{
	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_RESET);
	HAL_Delay(step_delay_dynamic);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_RESET);
	HAL_Delay(step_delay_dynamic);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_SET);
	HAL_Delay(step_delay_dynamic);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_SET);
	HAL_Delay(step_delay_dynamic);
}





void bigStepper_backward_sequence(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_)
{
	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_SET);
	HAL_Delay(step_delay_dynamic);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_SET);
	HAL_Delay(step_delay_dynamic);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_RESET);
	HAL_Delay(step_delay_dynamic);

	HAL_GPIO_WritePin(gpioA, pinA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioA_, pinA_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(gpioB, pinB, GPIO_PIN_SET);
	HAL_GPIO_WritePin(gpioB_, pinB_, GPIO_PIN_RESET);
	HAL_Delay(step_delay_dynamic);
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

void reactToAccel(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_)
{
	if (diffacc.y > 0)
		bigStepper_forward_sequence(gpioA, pinA, gpioA_, pinA_, gpioB, pinB, gpioB_, pinB_);
	else
		bigStepper_backward_sequence(gpioA, pinA, gpioA_, pinA_, gpioB, pinB, gpioB_, pinB_);
}

// A상
void step_A(uint8_t step_delay_dynamic)
{
	if ((state_flag & LEFT_MOTOR) == LEFT_MOTOR)
	{
		HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	}
	if ((state_flag & RIGHT_MOTOR) == RIGHT_MOTOR)
	{
		HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(step_delay_dynamic);
}

// B상
void step_B(uint8_t step_delay_dynamic)
{
	if ((state_flag & LEFT_MOTOR) == LEFT_MOTOR)
	{
		HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	}
	if ((state_flag & RIGHT_MOTOR) == RIGHT_MOTOR)
	{
		HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(step_delay_dynamic);
}

// `A상
void step_a(uint8_t step_delay_dynamic)
{
	if ((state_flag & LEFT_MOTOR) == LEFT_MOTOR)
	{
		HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	}
	if ((state_flag & RIGHT_MOTOR) == RIGHT_MOTOR)
	{
		HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(step_delay_dynamic);
}

// `B상
void step_b(uint8_t step_delay_dynamic)
{
	if ((state_flag & LEFT_MOTOR) == LEFT_MOTOR)
	{
		HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_SET);
	}
	if ((state_flag & RIGHT_MOTOR) == RIGHT_MOTOR)
	{
		HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_SET);
	}
	HAL_Delay(step_delay_dynamic);
}

// AB상
void step_AB(uint8_t step_delay_dynamic)
{
	if ((state_flag & LEFT_MOTOR) == LEFT_MOTOR)
	{
		HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	}
	if ((state_flag & RIGHT_MOTOR) == RIGHT_MOTOR)
	{
		HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(step_delay_dynamic);
}

// `AB상
void step_aB(uint8_t step_delay_dynamic)
{
	if ((state_flag & LEFT_MOTOR) == LEFT_MOTOR)
	{
		HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	}
	if ((state_flag & RIGHT_MOTOR) == RIGHT_MOTOR)
	{
		HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(step_delay_dynamic);
}

// `A`B상
void step_ab(uint8_t step_delay_dynamic)
{
	if ((state_flag & LEFT_MOTOR) == LEFT_MOTOR)
	{
		HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_SET);
	}
	if ((state_flag & RIGHT_MOTOR) == RIGHT_MOTOR)
	{
		HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_SET);
	}
	HAL_Delay(step_delay_dynamic);
}

// A`B상
void step_Ab(uint8_t step_delay_dynamic)
{
	if ((state_flag & LEFT_MOTOR) == LEFT_MOTOR)
	{
		HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_SET);
	}
	if ((state_flag & RIGHT_MOTOR) == RIGHT_MOTOR)
	{
		HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_SET);
	}
	HAL_Delay(step_delay_dynamic);
}

void step_reset(void)
{
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
}

void unipolar_parallel_sequence_onePhase(uint8_t step_delay_dynamic)
{
	switch (step%4U) {
	case 0U :
		step_A(step_delay_dynamic); break;
	case 1U :
		step_B(step_delay_dynamic); break;
	case 2U :
		step_a(step_delay_dynamic); break;
	case 3U :
		step_b(step_delay_dynamic); break;
	}
}

void unipolar_parallel_sequence_twoPhase(uint8_t step_delay_dynamic)
{
	switch (step%4) {
	case 0U :
		step_AB(step_delay_dynamic); break;
	case 1U :
		step_aB(step_delay_dynamic); break;
	case 2U :
		step_ab(step_delay_dynamic); break;
	case 3U :
		step_Ab(step_delay_dynamic); break;
	}
}

void unipolar_parallel_sequence_onetwoPhase(uint8_t step_delay_dynamic)
{
	switch (step%8) {
	case 0U :
		step_A(step_delay_dynamic); break;
	case 1U :
		step_AB(step_delay_dynamic); break;
	case 2U :
		step_B(step_delay_dynamic); break;
	case 3U :
		step_aB(step_delay_dynamic); break;
	case 4U :
		step_a(step_delay_dynamic); break;
	case 5U :
		step_ab(step_delay_dynamic); break;
	case 6U :
		step_b(step_delay_dynamic); break;
	case 7U :
		step_Ab(step_delay_dynamic); break;
	}
}

void reactToAccel_parallel(Robot_Direction* direction_flag, int8_t* angle)
{
//	static uint8_t step_delay_dynamic = 10U;
	static Robot_Direction prev_direction_flag = FORWARD;
	static int8_t prev_angle = 0;
	//	static uint16_t step_max = 400U;

	if (prev_angle != (*angle))
	{
		prev_angle = (*angle);
		if ((*angle) >= -10 && (*angle) <= 10)
			step_delay_static = (uint8_t)(5 - abs(*angle)/2);
		else
			step_delay_static = 0U;
		step_delay_dynamic = step_delay_static;
	}

	if (prev_direction_flag != *direction_flag)
	{
		prev_direction_flag = *direction_flag;
		step_delay_dynamic = 5U;
	}

	step_reset();
	switch (mode_flag) {
	case ONE_PHASE :
		unipolar_parallel_sequence_onePhase(step_delay_dynamic);
		break;
	case TWO_PHASE :
		unipolar_parallel_sequence_twoPhase(step_delay_dynamic);
		break;
	case ONETWO_PHASE :
		unipolar_parallel_sequence_onetwoPhase(step_delay_dynamic);
		break;
	}

	if (*direction_flag == FORWARD)
		step++;
	else
		step--;

	if (step_delay_dynamic > step_delay_static)
		step_delay_dynamic--;

	//	if (step_max > 0U)
		//	{
		//		step_reset();
	//		switch (mode_flag) {
	//		case ONE_PHASE :
	//			unipolar_parallel_sequence_onePhase(step_delay_dynamic);
	//			break;
	//		case TWO_PHASE :
	//			unipolar_parallel_sequence_twoPhase(step_delay_dynamic);
	//			break;
	//		case ONETWO_PHASE :
	//			unipolar_parallel_sequence_onetwoPhase(step_delay_dynamic);
	//			break;
	//		}
	//
	//		if (direction_flag == FORWARD)
	//			step++;
	//		else
	//			step--;
	//
	//		if (step_delay_dynamic > step_delay_static)
	//			step_delay_dynamic--;
	//
	//		step_max--;
	//	}
}
