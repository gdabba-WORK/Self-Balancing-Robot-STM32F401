/*
 * stepperMotor.c
 *
 *  Created on: 2020. 1. 31.
 *      Author: gdabba
 */

#include "stepperMotor.h"

extern MPU6050_int32_t diffacc;
extern uint32_t microTick2;
extern MPU6050_float_t accel_angle;
extern MPU6050_float_t gyro_angle;
extern MPU6050_float_t curr_filtered_angle;
extern MPU6050_float_t accel;
extern float prev_filtered_angle_x;
extern int8_t print_flag;
extern int8_t angle;
extern const float RADIANS_TO_DEGREES;
extern float REAL_DEGREE_COEFFICIENT;
extern uint32_t microTick;
extern float dt_calc;

float boundary_inner = 0.50F;
float boundary_outer = 1.00F;
uint8_t step = 0U;
int16_t step_max = 50;
uint32_t step_total = 0UL;
uint32_t step_delay_static = 1000UL;
uint32_t step_delay_dynamic = 5UL;

uint32_t step_delay_vertical = 10000UL;
uint32_t step_delay_horizontal = 2000UL;
uint32_t step_delay_low = 10000UL;
uint32_t step_delay_high = 10000UL;
uint32_t step_delay = 5000UL;
uint32_t step_delay_temp = 0UL;
uint32_t step_delay_total = 0UL;

uint32_t prev_step_delay = 0UL;

uint32_t dt_proc = 0UL;
uint32_t t_from = 0UL;
uint32_t t_to = 0UL;


float alpha_former = 0.30F;
float alpha_latter = 0.030F;
float LIMIT_BETA = 	0.50F;
float coefficient = 0.10F;

float cos_val = 0.0F;

Robot_Direction direction_flag = FRONT;
Robot_Direction prev_direction_flag = FRONT;
float prev_gyro_angle = 0.0F;

Motor_Mode mode_flag = ONETWO_PHASE;
Motor_State state_flag = BOTH_MOTOR;
Motor_Rotation rotation_flag = FORWARD;

Robot_Drive drive_flag = HALT;
Robot_Drive prev_drive_flag = HALT;


float VELOCITY_CONSTANT = 0.0F;
const float ACCELERATION_OF_GRAVITY = 9.806650F;
float ACCELERATION_OF_RISING = 0.20F;
const float STEP_RADIAN = 0.020F;
const float WHEEL_RADIUS = 0.0350F;
const float WHEEL_CONSTANT = 0.00070F;
float DEGREE_COEFFICIENT = 3.140F;
const float AXIS_TO_SENSOR = 0.180F;

uint32_t step_delay_to = 0UL;
float max_angular_acceleration = 0.0F;
float max_acceleration = 0.0F;
extern float angular_acceleration;
float prev_angle = 0.0F;
float curr_angle = 0.0F;
float dt_temp = 0.0F;
float starting_angle = 0.0F;
float inertia_moment_G = 0.0F;
float inertia_moment_A = 0.0F;
int8_t FIND = 0;

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

void new_delay(uint32_t step_delay)
{
	uint32_t start_tick = MY_GetTick();

	while ((MY_GetTick() - start_tick) < step_delay)
	{
		if ((MY_GetTick() - start_tick) >= 1000UL)
		{
			if (angular_acceleration > max_angular_acceleration)
			{
				max_angular_acceleration = angular_acceleration;
				prev_angle = fabs(prev_filtered_angle_x);
				curr_angle = fabs(curr_filtered_angle.x);
				dt_temp = dt_calc;
			}

			if (accel.y > max_acceleration)
			{
				max_acceleration = accel.y;
			}
		}
		osThreadYield();
	}
}
// A상
void new_step_A(uint32_t step_delay)
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
	new_delay(step_delay);
}

// B상
void new_step_B(uint32_t step_delay)
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
	new_delay(step_delay);
}

// `A상
void new_step_a(uint32_t step_delay)
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
	new_delay(step_delay);
}

// `B상
void new_step_b(uint32_t step_delay)
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
	new_delay(step_delay);
}

// AB상
void new_step_AB(uint32_t step_delay)
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
	new_delay(step_delay);
}

// `AB상
void new_step_aB(uint32_t step_delay)
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
	new_delay(step_delay);
}

// `A`B상
void new_step_ab(uint32_t step_delay)
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
	new_delay(step_delay);
}

// A`B상
void new_step_Ab(uint32_t step_delay)
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
	new_delay(step_delay);
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

void new_unipolar_parallel_sequence_onetwoPhase(uint32_t step_delay)
{
	switch (step%8) {
	case 0U :
		new_step_A(step_delay); break;
	case 1U :
		new_step_AB(step_delay); break;
	case 2U :
		new_step_B(step_delay); break;
	case 3U :
		new_step_aB(step_delay); break;
	case 4U :
		new_step_a(step_delay); break;
	case 5U :
		new_step_ab(step_delay); break;
	case 6U :
		new_step_b(step_delay); break;
	case 7U :
		new_step_Ab(step_delay); break;
	}
}

//void reactToAccel_parallel(int8_t* angle)
//{
//	//	static uint8_t step_delay_dynamic = 10U;
//	static Robot_Direction prev_direction_flag = STOP;
//	static int8_t prev_angle = 0;
//	static int16_t step_remain = 0U;
//	static uint32_t step_delay = 1000UL;
//
//	if (prev_angle != (*angle))
//	{
//		prev_angle = (*angle);
//		//			step_delay_static = 0U;
//
//		if ((*angle) >= -(boundary) && (*angle) <= (boundary))
//			direction_flag = STOP;
//		else if ((*angle) > (boundary))
//		{
//			direction_flag = FORWARD;
//			//			if ((*angle) <= 10)
//			//				step_delay_static = 1U;
//			//				step_delay_static = (uint8_t)(4 - abs(*angle)/2);
//		}
//		else if((*angle) < -(boundary))
//		{
//			direction_flag = BACKWARD;
//			//			if ((*angle) >= -10)
//			//				step_delay_static = 1U;
//			//				step_delay_static = (uint8_t)(4 - abs(*angle)/2);
//		}
//		//			step_delay_dynamic = step_delay_static;
//		if (prev_direction_flag != direction_flag)
//		{
//			prev_direction_flag = direction_flag;
//			//			step_delay_dynamic = 5U;
//		}
//		step_remain = step_max;
//		step_delay = step_delay_static;
//	}
//
//
//	if (step_remain > 0)
//	{
//		if (direction_flag == STOP)
//			step_reset(step_delay);
//		else
//		{
//			switch (mode_flag) {
//			case ONE_PHASE :
//				unipolar_parallel_sequence_onePhase(step_delay);
//				break;
//			case TWO_PHASE :
//				unipolar_parallel_sequence_twoPhase(step_delay);
//				break;
//			case ONETWO_PHASE :
//				unipolar_parallel_sequence_onetwoPhase(step_delay);
//				break;
//			case THREE_PHASE :
//				unipolar_parallel_sequence_threePhase(step_delay);
//				break;
//			}
//
//			if (direction_flag == FORWARD)
//				step++;
//			else if (direction_flag == BACKWARD)
//				step--;
//
//			//			if (step_delay_dynamic > step_delay_static)
//			//				step_delay_dynamic--;
//		}
//		step_remain--;
//		step_delay+=step_delay_dynamic;
//	}
//}

float getAlpha(void)
{
	return ((float)(step_delay_high - step_delay_low) * 16.0F / (float)(step_max * step_max));
}

void reactToAngle(void)
{
	//	static Robot_Direction prev_direction_flag = STOP;
	//	static int8_t prev_angle = 0;
	static int16_t step_remain = -1;
	static uint32_t step_delay = 0UL;
	static uint32_t step_delay_total = 0UL;
	char msg[150];
	//	static float alpha = 0.0F;

	if (step_remain < 0)
	{
		//		if (prev_angle != (*angle))
		//		{
		//			prev_angle = (*angle);
		//			step_delay_static = 0U;

		if ((curr_filtered_angle.x) >= -(boundary_outer) && (curr_filtered_angle.x) <= (boundary_outer))
			direction_flag = STOP;
		else if ((curr_filtered_angle.x) > (boundary_outer))
		{
			direction_flag = FORWARD;
			//			if ((*angle) <= 10)
			//				step_delay_static = 1U;
			//				step_delay_static = (uint8_t)(4 - abs(*angle)/2);
		}
		else if((curr_filtered_angle.x) < -(boundary_outer))
		{
			direction_flag = BACKWARD;
			//			if ((*angle) >= -10)
			//				step_delay_static = 1U;
			//				step_delay_static = (uint8_t)(4 - abs(*angle)/2);
		}
		//			step_delay_dynamic = step_delay_static;

		//			if (prev_direction_flag != direction_flag)
		//			{
		//				prev_direction_flag = direction_flag;
		//				//			step_delay_dynamic = 5U;
		//				// 내용 추가하기
		//			}
		if (print_flag)
		{
			if (direction_flag != STOP)
			{
				//			sprintf(msg, "%-10d%s\r\n", *angle, (direction_flag == FORWARD) ? "FORWARD" : "BACKWARD");
				//			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
				sprintf(msg, "%-10s%10.6f\tf=%7.3f\ta=%7.3f\tg=%7.3f\r\n",
						(direction_flag == FORWARD) ? "FORWARD" : "BACKWARD", (float)step_delay_total/1000000.0F,
								curr_filtered_angle.x, accel_angle.x, gyro_angle.x);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			}
		}
		step_remain = step_max;
		step_delay = step_delay_high;
		alpha_former = getAlpha();
		alpha_latter = alpha_former * coefficient;
		step_delay_total = 0UL;
		//		}
	}

	if (step_remain >= 0)
	{
		if (direction_flag == STOP)
		{
			switch (mode_flag) {
			case ONE_PHASE :
				unipolar_parallel_sequence_onePhase(0UL);
				break;
			case TWO_PHASE :
				unipolar_parallel_sequence_twoPhase(0UL);
				break;
			case ONETWO_PHASE :
				unipolar_parallel_sequence_onetwoPhase(0UL);
				break;
			case THREE_PHASE :
				unipolar_parallel_sequence_threePhase(0UL);
				break;
			}
			step_remain = -1;
		}
		else
		{
			if (direction_flag == FORWARD)
				step++;
			else if (direction_flag == BACKWARD)
				step--;

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


			//			if (step_delay_dynamic > step_delay_static)
			//				step_delay_dynamic--;
			step_remain--;
			if (step_remain > (step_max * 3 / 4))
				step_delay = (uint32_t)(alpha_former * (float)(step_remain - (step_max * 3 / 4))*(float)(step_remain - (step_max * 3 / 4))) + step_delay_low;
			else
				step_delay = (uint32_t)(alpha_latter * (float)(step_remain - (step_max * 3 / 4))*(float)(step_remain - (step_max * 3 / 4))) + step_delay_low;
			step_delay_total = step_delay_total + step_delay;
		}
	}
}
/*
uint32_t getStepDelay(void)
{
	uint32_t new_delay;

	alpha_latter = alpha_former * coefficient;

	if (drive_flag == HALT)
	{
		new_delay = step_delay_high;
	}
	else if (drive_flag == READY)
	{
		//		if (prev_drive_flag == HALT)
		//			new_delay = step_delay_high;
		//		else
		new_delay = (uint32_t)(alpha_latter * pow((sqrt(((float)(step_delay - step_delay_low)/alpha_latter) + 1.0F)), 2));
		//			new_delay = (uint32_t)(alpha_latter * pow((sqrt(((float)step_delay)/alpha_latter) + 1.0F), 2));
	}
	else if (drive_flag == RUN)
	{
		new_delay = (uint32_t)(alpha_former * pow((-sqrt(((float)(step_delay - step_delay_low)/alpha_former) + 1.0F)), 2));
		//		new_delay = (uint32_t)(alpha_former * pow((-sqrt(((float)step_delay)/alpha_former) + 1.0F), 2));
	}

	if (new_delay < step_delay_low)
		return step_delay_low;
	else if (new_delay > step_delay_high)
		return step_delay_high;

	return new_delay;
}
 */


/*
void reactToAngleGyro(void)
{
	char msg[150];

	prev_direction_flag = direction_flag;
	if (filtered_angle.x >= 0.0F)
		direction_flag = FORWARD;
	else
		direction_flag = BACKWARD;

	prev_drive_flag = drive_flag;
	if (((filtered_angle.x) >= -(boundary_inner)) && ((filtered_angle.x) <= (boundary_inner)))
		drive_flag = HALT;
	else if ((filtered_angle.x >= -(boundary_outer)) && ((filtered_angle.x) <= (boundary_outer)))
		drive_flag = READY;
	else
		drive_flag = RUN;

	adjustVelocityLimit();
	step_delay = getStepDelay();

	if ((prev_drive_flag == HALT) && (drive_flag != HALT))
	{
		step_delay_total = 0UL;
		step_total = 0UL;
		step_delay_total = step_delay_total + step_delay;
		step_total = step_total + 1UL;
	}

	if ((prev_drive_flag != HALT) && (drive_flag != HALT))
	{
		step_delay_total = step_delay_total + step_delay;
		step_total = step_total + 1UL;
	}
	if (prev_drive_flag != HALT && drive_flag == HALT)
	{
		if (print_flag)
		{
			//			sprintf(msg, "step_delay_total=%10.6f\r\n", (float)step_delay_total/1000000.0F);
			//			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "step_total=%10ld\r\n", step_total);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
		}
	}

	if (drive_flag == HALT)
	{
		unipolar_parallel_sequence_onetwoPhase(0UL);
	}
	else
	{
		if (direction_flag == FORWARD)
			step++;
		else if (direction_flag == BACKWARD)
			step--;
		unipolar_parallel_sequence_onetwoPhase(step_delay);
		//		if (print_flag)
		//		{
		//			sprintf(msg, "%-10s\tf=%7.3f\ta=%7.3f\tg=%7.3f\ts=%7.3f\tstep_delay=%ld\r\n",
		//					(direction_flag == FORWARD) ? "FORWARD" : "BACKWARD", filtered_angle.x, accel_angle.x, gyro_angle.x, cos_val, step_delay);
		//			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
		//		}
	}
}
 */
uint32_t getStepDelay(void)
{
	uint32_t new_delay = 0UL;
	float step_delay_of_float = (float)step_delay / 1000000.0F;
	char msg[50];

	if (drive_flag == HALT)
	{
		VELOCITY_CONSTANT = 0.0F;
	}

	else if (drive_flag == RUN)
	{
		VELOCITY_CONSTANT = 0.0F;
		new_delay = step_delay;
	}
	else if (drive_flag == READY)
	{
		VELOCITY_CONSTANT = 0.0F;
	}
	else if (drive_flag == ACCEL)
	{
		if (VELOCITY_CONSTANT == 0.0F)
		{
			//			VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)new_delay / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * cos(filtered_angle.x / RADIANS_TO_DEGREES));
			new_delay = step_delay_low;
			if (step_delay_temp != 0UL)
				step_delay_temp = 0UL;
			//			VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * DEGREE_COEFFICIENT)) / DEGREE_COEFFICIENT));
			//			VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * REAL_DEGREE_COEFFICIENT)) / REAL_DEGREE_COEFFICIENT));
			VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * (logf(cos(curr_filtered_angle.x / RADIANS_TO_DEGREES)) / REAL_DEGREE_COEFFICIENT));
		}
		else
		{
			if (fabs(prev_filtered_angle_x) >= fabs(curr_filtered_angle.x))
			{
				//				new_delay = (STEP_RADIAN * WHEEL_RADIUS) / ((ACCELERATION_OF_GRAVITY * cos(filtered_angle.x  / RADIANS_TO_DEGREES)) + VELOCITY_CONSTANT) * 1000000.0F;
				//				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)new_delay / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * cos(filtered_angle.x / RADIANS_TO_DEGREES));
				//				new_delay = (WHEEL_CONSTANT / ((ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * DEGREE_COEFFICIENT)) / DEGREE_COEFFICIENT)) + VELOCITY_CONSTANT)) * 1000000.0F;
				//				new_delay = (WHEEL_CONSTANT / ((ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * REAL_DEGREE_COEFFICIENT)) / REAL_DEGREE_COEFFICIENT)) + VELOCITY_CONSTANT)) * 1000000.0F;
				new_delay = (WHEEL_CONSTANT / ((ACCELERATION_OF_GRAVITY * (logf(cos(curr_filtered_angle.x / RADIANS_TO_DEGREES)) / REAL_DEGREE_COEFFICIENT)) + VELOCITY_CONSTANT)) * 1000000.0F;
				if (step_delay_temp != 0UL)
					step_delay_temp = 0UL;
				//				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)new_delay / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * DEGREE_COEFFICIENT)) / DEGREE_COEFFICIENT));
				//				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)new_delay / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * REAL_DEGREE_COEFFICIENT)) / REAL_DEGREE_COEFFICIENT));
				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)new_delay / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * (logf(cos(curr_filtered_angle.x / RADIANS_TO_DEGREES)) / REAL_DEGREE_COEFFICIENT));
			}
			else
			{
				//				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * cos(filtered_angle.x / RADIANS_TO_DEGREES));
				//				new_delay = step_delay_low;
				new_delay = step_delay_low;
				if (step_delay_temp == 0UL)
					step_delay_temp = step_delay;
				//				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay_temp / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * DEGREE_COEFFICIENT)) / DEGREE_COEFFICIENT));
				//				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay_temp / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * REAL_DEGREE_COEFFICIENT)) / REAL_DEGREE_COEFFICIENT));
				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay_temp / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * (logf(cos(curr_filtered_angle.x / RADIANS_TO_DEGREES)) / REAL_DEGREE_COEFFICIENT));
			}
		}
		//		new_delay = (uint32_t)(WHEEL_CONSTANT / (step_delay_of_double * (ACCELERATION_OF_GRAVITY * step_delay_of_double * sin(fabs(filtered_angle.x / RADIANS_TO_DEGREES)) / cos(filtered_angle.x / RADIANS_TO_DEGREES) + ACCELERATION_OF_RISING) + (WHEEL_CONSTANT / step_delay_of_double)) * 1000000.0);
		//		if (print_flag)
		//		{
		//			sprintf(msg, "new_delay=%10lu\r\n", new_delay);
		//			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
		//		}
		if (new_delay < step_delay_low)
			return step_delay_low;
	}

	else if (drive_flag == DECEL)
	{
		//		new_delay = (uint32_t)(WHEEL_CONSTANT / (-(step_delay_of_double * ACCELERATION_OF_GRAVITY * sin(fabs(filtered_angle.x / RADIANS_TO_DEGREES)) / cos(filtered_angle.x / RADIANS_TO_DEGREES)) + (WHEEL_CONSTANT / step_delay_of_double)) * 1000000.0);
		//		VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay / 1000000.0F)) + (ACCELERATION_OF_GRAVITY * log10(cos(filtered_angle.x / RADIANS_TO_DEGREES)));
		//		new_delay = step_delay_low;
		if (VELOCITY_CONSTANT == 0.0F)
		{
			//			VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)new_delay / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * cos(filtered_angle.x / RADIANS_TO_DEGREES));
			new_delay = step_delay_high-1;
			if (step_delay_temp != 0UL)
				step_delay_temp = 0UL;
			//			VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay / 1000000.0F)) + (ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * DEGREE_COEFFICIENT)) / DEGREE_COEFFICIENT));
			//			VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay / 1000000.0F)) + (ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * REAL_DEGREE_COEFFICIENT)) / REAL_DEGREE_COEFFICIENT));
			VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay / 1000000.0F)) + (ACCELERATION_OF_GRAVITY * (logf(cos(curr_filtered_angle.x / RADIANS_TO_DEGREES)) / REAL_DEGREE_COEFFICIENT));
		}
		else
		{
			if (fabs(prev_filtered_angle_x) >= fabs(curr_filtered_angle.x))
			{
				//				new_delay = (STEP_RADIAN * WHEEL_RADIUS) / ((ACCELERATION_OF_GRAVITY * cos(filtered_angle.x  / RADIANS_TO_DEGREES)) + VELOCITY_CONSTANT) * 1000000.0F;
				//				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)new_delay / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * cos(filtered_angle.x / RADIANS_TO_DEGREES));
				//				new_delay = (WHEEL_CONSTANT / (-(ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * DEGREE_COEFFICIENT)) / DEGREE_COEFFICIENT)) + VELOCITY_CONSTANT)) * 1000000.0F;
				//				new_delay = (WHEEL_CONSTANT / (-(ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * REAL_DEGREE_COEFFICIENT)) / REAL_DEGREE_COEFFICIENT)) + VELOCITY_CONSTANT)) * 1000000.0F;
				new_delay = (WHEEL_CONSTANT / (-(ACCELERATION_OF_GRAVITY * (logf(cos(curr_filtered_angle.x / RADIANS_TO_DEGREES)) / REAL_DEGREE_COEFFICIENT)) + VELOCITY_CONSTANT)) * 1000000.0F;
				if (step_delay_temp != 0UL)
					step_delay_temp = 0UL;
				//				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)new_delay / 1000000.0F)) + (ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * DEGREE_COEFFICIENT)) / DEGREE_COEFFICIENT));
				//				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)new_delay / 1000000.0F)) + (ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * REAL_DEGREE_COEFFICIENT)) / REAL_DEGREE_COEFFICIENT));
				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)new_delay / 1000000.0F)) + (ACCELERATION_OF_GRAVITY * (logf(cos(curr_filtered_angle.x / RADIANS_TO_DEGREES)) / REAL_DEGREE_COEFFICIENT));
			}
			else
			{
				//				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay / 1000000.0F)) - (ACCELERATION_OF_GRAVITY * cos(filtered_angle.x / RADIANS_TO_DEGREES));
				//				new_delay = step_delay_low;
				new_delay = step_delay_high-1;
				if (step_delay_temp == 0UL)
					step_delay_temp = step_delay;
				//				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay_temp / 1000000.0F)) + (ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * DEGREE_COEFFICIENT)) / DEGREE_COEFFICIENT));
				//				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay_temp / 1000000.0F)) + (ACCELERATION_OF_GRAVITY * (logf(cos((filtered_angle.x / RADIANS_TO_DEGREES) * REAL_DEGREE_COEFFICIENT)) / REAL_DEGREE_COEFFICIENT));
				VELOCITY_CONSTANT = (WHEEL_CONSTANT / ((float)step_delay_temp / 1000000.0F)) + (ACCELERATION_OF_GRAVITY * (logf(cos(curr_filtered_angle.x / RADIANS_TO_DEGREES)) / REAL_DEGREE_COEFFICIENT));
			}
		}
		if (new_delay >= step_delay_high)
		{
			drive_flag = READY;
			return step_delay_high;
		}
	}

	return new_delay;
}

void adjustVelocityLimit(void)
{
	uint32_t step_delay_incline;

	cos_val = cos(curr_filtered_angle.x / RADIANS_TO_DEGREES);
	step_delay_incline = (uint32_t)((float)step_delay_high * fabs(cos_val) * LIMIT_BETA);

	if (step_delay_incline < step_delay_horizontal)
		step_delay_low = step_delay_horizontal;
	else
		step_delay_low = step_delay_incline;
}

void reactToAngleGyro(void)
{
	char msg[150];

	//	t_from = microTick;

	if (curr_filtered_angle.x > (boundary_inner))
		direction_flag = FRONT;
	else if (curr_filtered_angle.x < (-boundary_inner))
		direction_flag = REAR;


	if (fabs(curr_filtered_angle.x) <= (boundary_inner))
	{
		if (REAL_DEGREE_COEFFICIENT <= 0.1F)
		{
			drive_flag = HALT;
		}
		else
		{
			drive_flag = RUN;
		}
	}
	else
	{
		if ((drive_flag == READY) || (drive_flag == HALT))
		{
			drive_flag = ACCEL;
			if (direction_flag == FRONT)
				rotation_flag = FORWARD;
			else if (direction_flag == REAR)
				rotation_flag = BACKWARD;
			prev_direction_flag = direction_flag;
		}
		else if (prev_direction_flag != direction_flag)
		{
			drive_flag = DECEL;
			prev_direction_flag = direction_flag;
		}
	}
	//	else if ((drive_flag != ACCEL) || (prev_direction_flag != direction_flag))
	//	{
	//		prev_drive_flag = drive_flag;
	//		drive_flag = ACCEL;
	//		if (direction_flag == FRONT)
	//			rotation_flag = FORWARD;
	//		else if (direction_flag == REAR)
	//			rotation_flag = BACKWARD;
	//		prev_direction_flag = direction_flag;
	//	}

	adjustVelocityLimit();
	step_delay = getStepDelay();

	//	if ((prev_drive_flag == HALT) && (drive_flag != HALT))
	//	{
	//		step_delay_total = 0UL;
	//		step_total = 0UL;
	//		step_delay_total = step_delay_total + step_delay;
	//		step_total = step_total + 1UL;
	//	}
	//
	//	if ((prev_drive_flag != HALT) && (drive_flag != HALT))
	//	{
	//		step_delay_total = step_delay_total + step_delay;
	//		step_total = step_total + 1UL;
	//	}
	//	if (prev_drive_flag != HALT && drive_flag == HALT)
	//	{
	//		if (print_flag)
	//		{
	//			//			sprintf(msg, "step_delay_total=%10.6f\r\n", (float)step_delay_total/1000000.0F);
	//			//			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
	//			sprintf(msg, "step_total=%10ld\r\n", step_total);
	//			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
	//		}
	//	}

	//	t_to = microTick;
	//	dt_proc = t_to - t_from;

	if (drive_flag == HALT)
	{
		unipolar_parallel_sequence_onetwoPhase(0UL);
	}
	else
	{
		if (rotation_flag == FORWARD)
			step++;
		else if (rotation_flag == BACKWARD)
			step--;
		unipolar_parallel_sequence_onetwoPhase(step_delay);
		//		if (print_flag)
		//		{
		//			sprintf(msg, "%-10s\tf=%7.3f\ta=%7.3f\tg=%7.3f\ts=%7.3f\tstep_delay=%ld\r\n",
		//					(direction_flag == FORWARD) ? "FORWARD" : "BACKWARD", filtered_angle.x, accel_angle.x, gyro_angle.x, cos_val, step_delay);
		//			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
		//		}
	}
}

void momentFinder(void)
{
	char msg[150];
	uint16_t count = 1;

	if (FIND == 1)
	{
		prev_direction_flag = direction_flag;
		prev_drive_flag = drive_flag;

		if (step_max == 0)
		{
			drive_flag = HALT;
			step_total = 0;
			step_max = 80;
			inertia_moment_G = ((cos(starting_angle / RADIANS_TO_DEGREES) * ((WHEEL_CONSTANT / (step_delay / 1000000.0F)) / 0.001F)) - (sin(starting_angle / RADIANS_TO_DEGREES) * ACCELERATION_OF_GRAVITY)) / (AXIS_TO_SENSOR * max_angular_acceleration);
			inertia_moment_A = ((cos(starting_angle / RADIANS_TO_DEGREES) * ((WHEEL_CONSTANT / (step_delay / 1000000.0F)) / 0.001F)) + (sin(starting_angle / RADIANS_TO_DEGREES) * ACCELERATION_OF_GRAVITY)) / (AXIS_TO_SENSOR * max_acceleration);

			sprintf(msg, "COUNT STARTING_ANGLE STEP_DELAY ANGULAR_ACC PREV_ANGLE CURR_ANGLE DT INERTIA_MOMENT_G INERTIA_MOMENT_A\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "{\"CNT\":%03d,\"SD\":%05ld,\"SA\":%09.6f,\"AA\":%09.6f,\"PA\":%09.6f,\"CA\":%09.6f,\"DT\":%.6f,\"IMG\":%09.6f,\"IMA\":%09.6f}\r\n", count, step_delay, starting_angle, max_angular_acceleration, prev_angle, curr_angle, dt_temp, inertia_moment_G, inertia_moment_A);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);

			count++;
			max_angular_acceleration = 0.0F;
			max_acceleration = 0.0F;
			prev_angle = 0.0F;
			curr_angle = 0.0F;
			dt_temp = 0.0F;
			inertia_moment_G = 0.0F;
			inertia_moment_A = 0.0F;
			starting_angle = 0.0F;
		}
		else
			drive_flag = ACCEL;

		if (curr_filtered_angle.x > (boundary_inner))
		{
			direction_flag = FRONT;
			rotation_flag = FORWARD;
		}
		else if (curr_filtered_angle.x < (-boundary_inner))
		{
			direction_flag = REAR;
			rotation_flag = FORWARD;
		}
		else
		{
			drive_flag = HALT;
			step_total = 0;
			step_max = 80;
			inertia_moment_G = ((cos(starting_angle / RADIANS_TO_DEGREES) * ((WHEEL_CONSTANT / (step_delay / 1000000.0F)) / 0.001F)) - (sin(starting_angle / RADIANS_TO_DEGREES) * ACCELERATION_OF_GRAVITY)) / (AXIS_TO_SENSOR * max_angular_acceleration);
			inertia_moment_A = ((cos(starting_angle / RADIANS_TO_DEGREES) * ((WHEEL_CONSTANT / (step_delay / 1000000.0F)) / 0.001F)) + (sin(starting_angle / RADIANS_TO_DEGREES) * ACCELERATION_OF_GRAVITY)) / (AXIS_TO_SENSOR * max_acceleration);

			sprintf(msg, "COUNT STARTING_ANGLE STEP_DELAY ANGULAR_ACC PREV_ANGLE CURR_ANGLE DT INERTIA_MOMENT_G INERTIA_MOMENT_A\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "{\"CNT\":%03d,\"SD\":%05ld,\"SA\":%09.6f,\"AA\":%09.6f,\"PA\":%09.6f,\"CA\":%09.6f,\"DT\":%.6f,\"IMG\":%09.6f,\"IMA\":%09.6f}\r\n", count, step_delay, starting_angle, max_angular_acceleration, prev_angle, curr_angle, dt_temp, inertia_moment_G, inertia_moment_A);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);

			count++;
			max_angular_acceleration = 0.0F;
			max_acceleration = 0.0F;
			prev_angle = 0.0F;
			curr_angle = 0.0F;
			dt_temp = 0.0F;
			inertia_moment_G = 0.0F;
			inertia_moment_A = 0.0F;
			starting_angle = 0.0F;
		}

		//		if ((prev_drive_flag != HALT) && (drive_flag == HALT))
		//		{
		//			inertia_moment_G = ((cos(starting_angle / RADIANS_TO_DEGREES) * ((WHEEL_CONSTANT / (step_delay / 1000000.0F)) / 0.001F)) - (sin(starting_angle / RADIANS_TO_DEGREES) * ACCELERATION_OF_GRAVITY)) / (AXIS_TO_SENSOR * max_angular_acceleration);
		//			inertia_moment_A = ((cos(starting_angle / RADIANS_TO_DEGREES) * ((WHEEL_CONSTANT / (step_delay / 1000000.0F)) / 0.001F)) + (sin(starting_angle / RADIANS_TO_DEGREES) * ACCELERATION_OF_GRAVITY)) / (AXIS_TO_SENSOR * max_acceleration);
		//
		//			sprintf(msg, "COUNT STARTING_ANGLE STEP_DELAY ANGULAR_ACC PREV_ANGLE CURR_ANGLE DT INERTIA_MOMENT_G INERTIA_MOMENT_A\r\n");
		//			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
		//			sprintf(msg, "{\"CNT\":%03d,\"SD\":%05ld,\"SA\":%09.6f,\"AA\":%09.6f,\"PA\":%09.6f,\"CA\":%09.6f,\"DT\":%.6f,\"IMG\":%09.6f,\"IMA\":%09.6f}\r\n", count, starting_angle, step_delay, max_angular_acceleration, prev_angle, curr_angle, dt_temp, inertia_moment_G, inertia_moment_A);
		//			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
		//
		//			count++;
		//			max_angular_acceleration = 0.0F;
		//			max_acceleration = 0.0F;
		//			prev_angle = 0.0F;
		//			curr_angle = 0.0F;
		//			dt_temp = 0.0F;
		//			inertia_moment_G = 0.0F;
		//			inertia_moment_A = 0.0F;
		//			starting_angle = 0.0F;
		//		}

		if (drive_flag == HALT)
		{
			step_reset(5000000UL);
		}
		else
		{
			if (prev_drive_flag == HALT)
			{
				unipolar_parallel_sequence_onetwoPhase(1000000UL);
				starting_angle = fabs(curr_filtered_angle.x);
			}
			else
			{
				if (rotation_flag == FORWARD)
					step++;
				else if (rotation_flag == BACKWARD)
					step--;
				new_unipolar_parallel_sequence_onetwoPhase(step_delay);

				step_max--;
				step_total++;
				step_delay = step_delay + 100UL;
			}
		}

		if (count > 100U)
		{
			FIND = 0;
			max_angular_acceleration = 0.0F;
			max_acceleration = 0.0F;
			prev_angle = 0.0F;
			curr_angle = 0.0F;
			dt_temp = 0.0F;
			inertia_moment_G = 0.0F;
			inertia_moment_A = 0.0F;
			starting_angle = 0.0F;
		}
	}
}
