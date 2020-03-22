/*
 * stepperMotor.c
 *
 *  Created on: 2020. 1. 31.
 *      Author: gdabba
 */

#include "stepperMotor.h"

extern MPU6050_int32_t diffacc;
extern uint32_t microTick2;

int8_t boundary = 2;
uint8_t step = 0U;
uint16_t step_max = 40U;
uint32_t step_delay_static = 1000UL;
uint32_t step_delay_dynamic = 5UL;
//uint32_t step_delay = 2020UL;

uint32_t step_delay_high = 3000UL;
uint32_t step_delay_low = 1000UL;
float alpha = 0.0F;

Robot_Direction direction_flag = STOP;
Motor_Mode mode_flag = ONETWO_PHASE;
Motor_State state_flag = BOTH_MOTOR;

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
void step_A(uint32_t step_delay)
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
	//	HAL_Delay(step_delay_dynamic);
	MY_Delay(step_delay);
}

// B상
void step_B(uint32_t step_delay)
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
	//	HAL_Delay(step_delay_dynamic);
	MY_Delay(step_delay);
}

// `A상
void step_a(uint32_t step_delay)
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
	//	HAL_Delay(step_delay_dynamic);
	MY_Delay(step_delay);
}

// `B상
void step_b(uint32_t step_delay)
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
	//	HAL_Delay(step_delay_dynamic);
	MY_Delay(step_delay);
}

// AB상
void step_AB(uint32_t step_delay)
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
	//	HAL_Delay(step_delay_dynamic);
	MY_Delay(step_delay);
}

// `AB상
void step_aB(uint32_t step_delay)
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
	//	HAL_Delay(step_delay_dynamic);
	MY_Delay(step_delay);
}

// `A`B상
void step_ab(uint32_t step_delay)
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
	//	HAL_Delay(step_delay_dynamic);
	MY_Delay(step_delay);
}

// A`B상
void step_Ab(uint32_t step_delay)
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
	//	HAL_Delay(step_delay_dynamic);
	MY_Delay(step_delay);
}
void step_ABa(uint32_t step_delay)
{
	if ((state_flag & LEFT_MOTOR) == LEFT_MOTOR)
	{
		HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);
	}
	if ((state_flag & RIGHT_MOTOR) == RIGHT_MOTOR)
	{
		HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	}
	//	HAL_Delay(step_delay_dynamic);
	MY_Delay(step_delay);
}
void step_Bab(uint32_t step_delay)
{
	if ((state_flag & LEFT_MOTOR) == LEFT_MOTOR)
	{
		HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_SET);
	}
	if ((state_flag & RIGHT_MOTOR) == RIGHT_MOTOR)
	{
		HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_SET);
	}
	//	HAL_Delay(step_delay_dynamic);
	MY_Delay(step_delay);
}
void step_abA(uint32_t step_delay)
{
	if ((state_flag & LEFT_MOTOR) == LEFT_MOTOR)
	{
		HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_SET);
	}
	if ((state_flag & RIGHT_MOTOR) == RIGHT_MOTOR)
	{
		HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_SET);
	}
	//	HAL_Delay(step_delay_dynamic);
	MY_Delay(step_delay);
}
void step_bAB(uint32_t step_delay)
{
	if ((state_flag & LEFT_MOTOR) == LEFT_MOTOR)
	{
		HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_SET);
	}
	if ((state_flag & RIGHT_MOTOR) == RIGHT_MOTOR)
	{
		HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_SET);
	}
	//	HAL_Delay(step_delay_dynamic);
	MY_Delay(step_delay);
}
void step_reset(uint32_t step_delay)
{
	HAL_GPIO_WritePin(SM1A_GPIO_Port, SM1A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1A__GPIO_Port, SM1A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B_GPIO_Port, SM1B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM1B__GPIO_Port, SM1B__Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(SM2A_GPIO_Port, SM2A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2A__GPIO_Port, SM2A__Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B_GPIO_Port, SM2B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SM2B__GPIO_Port, SM2B__Pin, GPIO_PIN_RESET);
	//	HAL_Delay(step_delay_dynamic);
	MY_Delay(step_delay);
}

void unipolar_parallel_sequence_onePhase(uint32_t step_delay)
{
	switch (step%4U) {
	case 0U :
		step_A(step_delay); break;
	case 1U :
		step_B(step_delay); break;
	case 2U :
		step_a(step_delay); break;
	case 3U :
		step_b(step_delay); break;
	}
}

void unipolar_parallel_sequence_twoPhase(uint32_t step_delay)
{
	switch (step%4) {
	case 0U :
		step_AB(step_delay); break;
	case 1U :
		step_aB(step_delay); break;
	case 2U :
		step_ab(step_delay); break;
	case 3U :
		step_Ab(step_delay); break;
	}
}

void unipolar_parallel_sequence_onetwoPhase(uint32_t step_delay)
{
	switch (step%8) {
	case 0U :
		step_A(step_delay); break;
	case 1U :
		step_AB(step_delay); break;
	case 2U :
		step_B(step_delay); break;
	case 3U :
		step_aB(step_delay); break;
	case 4U :
		step_a(step_delay); break;
	case 5U :
		step_ab(step_delay); break;
	case 6U :
		step_b(step_delay); break;
	case 7U :
		step_Ab(step_delay); break;
	}
}

void unipolar_parallel_sequence_threePhase(uint32_t step_delay)
{
	switch (step%4) {
	case 0U :
		step_ABa(step_delay); break;
	case 1U :
		step_Bab(step_delay); break;
	case 2U :
		step_abA(step_delay); break;
	case 3U :
		step_bAB(step_delay); break;
	}
}

void reactToAccel_parallel(int8_t* angle)
{
	//	static uint8_t step_delay_dynamic = 10U;
	static Robot_Direction prev_direction_flag = STOP;
	static int8_t prev_angle = 0;
	static uint16_t step_remain = 0U;
	static uint32_t step_delay = 1000UL;

	if (prev_angle != (*angle))
	{
		prev_angle = (*angle);
		//			step_delay_static = 0U;

		if ((*angle) >= -(boundary) && (*angle) <= (boundary))
			direction_flag = STOP;
		else if ((*angle) > (boundary))
		{
			direction_flag = FORWARD;
			//			if ((*angle) <= 10)
			//				step_delay_static = 1U;
			//				step_delay_static = (uint8_t)(4 - abs(*angle)/2);
		}
		else if((*angle) < -(boundary))
		{
			direction_flag = BACKWARD;
			//			if ((*angle) >= -10)
			//				step_delay_static = 1U;
			//				step_delay_static = (uint8_t)(4 - abs(*angle)/2);
		}
		//			step_delay_dynamic = step_delay_static;
		if (prev_direction_flag != direction_flag)
		{
			prev_direction_flag = direction_flag;
			//			step_delay_dynamic = 5U;
		}
		step_remain = step_max;
		step_delay = step_delay_static;
	}


	if (step_remain > 0U)
	{
		if (direction_flag == STOP)
			step_reset(step_delay);
		else
		{
			switch (mode_flag) {
			case ONE_PHASE :
				unipolar_parallel_sequence_onePhase(step_delay);
				break;
			case TWO_PHASE :
				unipolar_parallel_sequence_twoPhase(step_delay);
				break;
			case ONETWO_PHASE :
				unipolar_parallel_sequence_onetwoPhase(step_delay);
				break;
			case THREE_PHASE :
				unipolar_parallel_sequence_threePhase(step_delay);
				break;
			}

			if (direction_flag == FORWARD)
				step++;
			else if (direction_flag == BACKWARD)
				step--;

			//			if (step_delay_dynamic > step_delay_static)
			//				step_delay_dynamic--;
		}
		step_remain--;
		step_delay+=step_delay_dynamic;
	}
}

float getAlpha()
{
	return ((float)(step_delay_high - step_delay_low) * 4.0F / (float)(step_max * step_max));
}

void reactToAngle(int8_t* angle)
{
	static Robot_Direction prev_direction_flag = STOP;
	static int8_t prev_angle = 0;
	static uint16_t step_remain = 0U;
	static uint32_t step_delay = 0UL;
	static float alpha = 0.0F;
	if (prev_angle != (*angle))
	{
		prev_angle = (*angle);
		//			step_delay_static = 0U;

		if ((*angle) >= -(boundary) && (*angle) <= (boundary))
			direction_flag = STOP;
		else if ((*angle) > (boundary))
		{
			direction_flag = FORWARD;
			//			if ((*angle) <= 10)
			//				step_delay_static = 1U;
			//				step_delay_static = (uint8_t)(4 - abs(*angle)/2);
		}
		else if((*angle) < -(boundary))
		{
			direction_flag = BACKWARD;
			//			if ((*angle) >= -10)
			//				step_delay_static = 1U;
			//				step_delay_static = (uint8_t)(4 - abs(*angle)/2);
		}
		//			step_delay_dynamic = step_delay_static;
		if (prev_direction_flag != direction_flag)
		{
			prev_direction_flag = direction_flag;
			//			step_delay_dynamic = 5U;
			// 내용 추가하기
		}
		step_remain = step_max;
		step_delay = step_delay_high;
		alpha = getAlpha();
	}


	if (step_remain > 0U)
	{
		if (direction_flag == STOP)
			step_reset(step_delay);
		else
		{
			switch (mode_flag) {
			case ONE_PHASE :
				unipolar_parallel_sequence_onePhase(step_delay);
				break;
			case TWO_PHASE :
				unipolar_parallel_sequence_twoPhase(step_delay);
				break;
			case ONETWO_PHASE :
				unipolar_parallel_sequence_onetwoPhase(step_delay);
				break;
			case THREE_PHASE :
				unipolar_parallel_sequence_threePhase(step_delay);
				break;
			}

			if (direction_flag == FORWARD)
				step++;
			else if (direction_flag == BACKWARD)
				step--;

			//			if (step_delay_dynamic > step_delay_static)
			//				step_delay_dynamic--;
		}
		step_delay = (uint32_t)(alpha * (float)(step_remain - (step_max/2))*(float)(step_remain - (step_max/2))) + step_delay_low;
		step_remain--;
	}
}
