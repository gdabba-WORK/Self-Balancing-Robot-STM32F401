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
extern MPU6050_float_t accel_f;
extern float prev_filtered_angle_x;
extern int8_t print_flag;
extern int8_t angle;
extern const float RADIANS_TO_DEGREES;
extern float REAL_DEGREE_COEFFICIENT;
extern uint32_t microTick;
extern float dt_calc;
extern MPU6050_float_t gyro_f;
extern float COMPLEMENTARY_ALPHA;

float boundary_inner = 1.00F;
float boundary_outer = 1.00F;
uint8_t step = 0U;
uint8_t step_remain = 0U;
uint16_t step_max = 50U;
uint32_t step_total = 0UL;
uint32_t step_delay_static = 1000UL;
uint32_t step_delay_dynamic = 5UL;

uint32_t step_delay_vertical = 5000UL;
uint32_t step_delay_horizontal = 2000UL;
uint32_t step_delay_low = 5000UL;
uint32_t step_delay_high = 8500UL;
uint32_t step_delay = 8500UL;
uint32_t step_delay_temp = 0UL;
uint32_t step_delay_total = 0UL;
extern uint32_t DLPF_DELAY;

uint32_t prev_step_delay = 0UL;

uint32_t dt_proc = 0UL;
uint32_t dt_proc_max = 0UL;
uint32_t t_from = 0UL;
uint32_t t_to = 0UL;


float alpha_former = 0.30F;
float alpha_latter = 0.030F;
float LIMIT_BETA = 	0.460F;
float coefficient = 0.10F;

float cos_val = 0.0F;

Robot_Direction direction_flag = FRONT;
Robot_Direction prev_direction_flag = FRONT;

Motor_Mode mode_flag = ONETWO_PHASE;
Motor_State state_flag = BOTH_MOTOR;
Motor_Rotation rotation_flag = FORWARD;

Robot_Drive drive_flag = HALT;
Robot_Drive prev_drive_flag = HALT;


float VELOCITY_CONSTANT = 0.0F;
const float ACCELERATION_OF_GRAVITY = 9.806650F;
float ACCELERATION_OF_RISING = 0.000F;
float ACCELERATION_OF_STOPPING = 0.0F;
float ACCELERATION_OF_HALTING = 0.0F;
const float STEP_RADIAN = 0.020F;
const float WHEEL_RADIUS = 0.0350F;
const float WHEEL_CONSTANT = 0.00070F;
float DEGREE_COEFFICIENT = 3.140F;
const float AXIS_TO_SENSOR = 0.180F;
const float FLOOR_TO_SENSOR = 0.2150F;
float TIME_CONSTANT = 0.0100F;
float INERTIA_MOMENT = 0.030F;

uint32_t step_delay_to = 0UL;
float max_angular_acceleration = 0.0F;
float max_accelero_acceleration = 0.0F;
extern float angular_acceleration;
extern float accelero_acceleration;
float prev_angle_G = 0.0F;
float curr_angle_G = 0.0F;
float dt_temp_G = 0.0F;
float prev_angle_A = 0.0F;
float curr_angle_A = 0.0F;
float dt_temp_A = 0.0F;
float starting_angle = 0.0F;
int8_t find_flag = 0;

int8_t excite_flag = 0;
float accel = 0.0F;

// A상
void step_A(void)
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
}

// B상
void step_B(void)
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
}

// `A상
void step_a(void)
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
}

// `B상
void step_b(void)
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
}

// AB상
void step_AB(void)
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
}

// `AB상
void step_aB(void)
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
}

// `A`B상
void step_ab(void)
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
}

// A`B상
void step_Ab(void)
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
}
void step_ABa(void)
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
}
void step_Bab(void)
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
}
void step_abA(void)
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
}
void step_bAB(void)
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
}


void unipolar_parallel_sequence_onePhase(uint32_t step_delay, osThreadId_t handle)
{
	switch (step%4U) {
	case 0U :
		step_A(); break;
	case 1U :
		step_B(); break;
	case 2U :
		step_a(); break;
	case 3U :
		step_b(); break;
	}
	MY_Delay(step_delay, handle);
}

void unipolar_parallel_sequence_twoPhase(uint32_t step_delay, osThreadId_t handle)
{
	switch (step%4) {
	case 0U :
		step_AB(); break;
	case 1U :
		step_aB(); break;
	case 2U :
		step_ab(); break;
	case 3U :
		step_Ab(); break;
	}
	MY_Delay(step_delay, handle);
}

void unipolar_parallel_sequence_onetwoPhase(uint32_t step_delay, osThreadId_t handle)
{
	switch (step%8) {
	case 0U :
		step_A(); break;
	case 1U :
		step_AB(); break;
	case 2U :
		step_B(); break;
	case 3U :
		step_aB(); break;
	case 4U :
		step_a(); break;
	case 5U :
		step_ab(); break;
	case 6U :
		step_b(); break;
	case 7U :
		step_Ab(); break;
	}
	MY_Delay(step_delay, handle);
}

void unipolar_parallel_sequence_threePhase(uint32_t step_delay, osThreadId_t handle)
{
	switch (step%4) {
	case 0U :
		step_ABa(); break;
	case 1U :
		step_Bab(); break;
	case 2U :
		step_abA(); break;
	case 3U :
		step_bAB(); break;
	}
	MY_Delay(step_delay, handle);
}

float getAlpha(void)
{
	return ((float)(step_delay_high - step_delay_low) * 16.0F / (float)(step_max * step_max));
}

uint32_t getStepDelay(void)
{
	uint32_t new_delay = 0UL;
	float step_delay_float = (float)step_delay / 1000000.0F;
	float temp_float = 0.000000F;
	TIME_CONSTANT = step_delay_float;

	if (drive_flag == HALT)
	{
		new_delay = step_delay_high;
	}

	else if (drive_flag == RUN)
	{
		if (step_delay < step_delay_low)
			new_delay = step_delay_low;
		else
			new_delay = step_delay;
	}

	else if (drive_flag == READY)
	{
		new_delay = step_delay_high;
	}

	else if (drive_flag == ACCEL)
	{
		if (step%2 == 0U)
		{
			temp_float = ((INERTIA_MOMENT * (((fabsf(gyro_f.x) / RADIANS_TO_DEGREES) / TIME_CONSTANT) - 0.250000F)) +
					(ACCELERATION_OF_GRAVITY * sinf(fabsf(curr_filtered_angle.x / RADIANS_TO_DEGREES))) + ACCELERATION_OF_RISING);

			if (temp_float > 0.000000F)
			{
				if (prev_drive_flag == HALT)
					new_delay = (WHEEL_CONSTANT / (TIME_CONSTANT * (temp_float / cosf(curr_filtered_angle.x / RADIANS_TO_DEGREES)))) * 1000000.0F;
				else
					new_delay = (WHEEL_CONSTANT / (TIME_CONSTANT * (temp_float / cosf(curr_filtered_angle.x / RADIANS_TO_DEGREES)) + (WHEEL_CONSTANT / step_delay_float))) * 1000000.0F;
			}
			else
				new_delay = step_delay;
		}
		else if (step%2 == 1U)
		{
			temp_float = ((INERTIA_MOMENT * (((fabsf(gyro_f.x) / RADIANS_TO_DEGREES) / TIME_CONSTANT) - 0.200000F)) +
					(ACCELERATION_OF_GRAVITY * sinf(fabsf(curr_filtered_angle.x / RADIANS_TO_DEGREES))) + ACCELERATION_OF_RISING);

			if (temp_float > 0.000000F)
			{
				if (prev_drive_flag == HALT)
					new_delay = (WHEEL_CONSTANT / (TIME_CONSTANT * (temp_float / cosf(curr_filtered_angle.x / RADIANS_TO_DEGREES)))) * 1000000.0F;
				else
					new_delay = (WHEEL_CONSTANT / (TIME_CONSTANT * (temp_float / cosf(curr_filtered_angle.x / RADIANS_TO_DEGREES)) + (WHEEL_CONSTANT / step_delay_float))) * 1000000.0F;
			}
			else
				new_delay = step_delay;
		}

		if (new_delay < step_delay_low)
			new_delay = step_delay_low;
		else if (new_delay > step_delay_high)
			new_delay = step_delay_high;
	}

	else if (drive_flag == DECEL_EXACT_FALL)
	{
		if (step%2 == 0U)
		{
			temp_float = ((-(TIME_CONSTANT * ((INERTIA_MOMENT * (0.250000F)) -	(ACCELERATION_OF_GRAVITY * sinf(fabsf(curr_filtered_angle.x / RADIANS_TO_DEGREES))) + ACCELERATION_OF_STOPPING)) /
					cosf(curr_filtered_angle.x / RADIANS_TO_DEGREES)) + (WHEEL_CONSTANT / step_delay_float));
			if (temp_float > 0.000000F)
				new_delay = (WHEEL_CONSTANT / temp_float) * 1000000.0F;
		}
		else if (step%2 == 1U)
		{
			temp_float = ((-(TIME_CONSTANT * ((INERTIA_MOMENT * (0.200000F)) -	(ACCELERATION_OF_GRAVITY * sinf(fabsf(curr_filtered_angle.x / RADIANS_TO_DEGREES))) + ACCELERATION_OF_STOPPING)) /
					cosf(curr_filtered_angle.x / RADIANS_TO_DEGREES)) + (WHEEL_CONSTANT / step_delay_float));
			if (temp_float > 0.000000F)
				new_delay = (WHEEL_CONSTANT / temp_float) * 1000000.0F;
		}
		if ((new_delay > step_delay_high) || (new_delay == 0UL))
		{
			drive_flag = HALT;
			new_delay = step_delay_high;
		}
	}
	else if (drive_flag == DECEL_EXACT_LIE)
	{
		if (step%2 == 0U)
		{
			temp_float = ((-(TIME_CONSTANT * ((INERTIA_MOMENT * (0.250000F)) +	(ACCELERATION_OF_GRAVITY * sinf(fabsf(curr_filtered_angle.x / RADIANS_TO_DEGREES))) + ACCELERATION_OF_STOPPING)) /
					cosf(curr_filtered_angle.x / RADIANS_TO_DEGREES)) + (WHEEL_CONSTANT / step_delay_float));
			if (temp_float > 0.000000F)
				new_delay = (WHEEL_CONSTANT / temp_float) * 1000000.0F;
		}
		else if (step%2 == 1U)
		{
			temp_float = ((-(TIME_CONSTANT * ((INERTIA_MOMENT * (0.200000F)) +	(ACCELERATION_OF_GRAVITY * sinf(fabsf(curr_filtered_angle.x / RADIANS_TO_DEGREES))) + ACCELERATION_OF_STOPPING)) /
					cosf(curr_filtered_angle.x / RADIANS_TO_DEGREES)) + (WHEEL_CONSTANT / step_delay_float));
			if (temp_float > 0.000000F)
				new_delay = (WHEEL_CONSTANT / temp_float) * 1000000.0F;
		}
		if ((new_delay > step_delay_high) || (new_delay == 0UL))
		{
			drive_flag = HALT;
			new_delay = step_delay_high;
		}
	}
	else if (drive_flag == DECEL_APPROX)
	{
		if (step%2 == 0U)
		{
			temp_float = (-(TIME_CONSTANT * ((INERTIA_MOMENT * (0.250000F)) + ACCELERATION_OF_HALTING)) + (WHEEL_CONSTANT / step_delay_float));
			if (temp_float > 0.000000F)
				new_delay = (WHEEL_CONSTANT / temp_float) * 1000000.0F;
		}
		else if (step%2 == 1U)
		{
			temp_float = (-(TIME_CONSTANT * ((INERTIA_MOMENT * (0.200000F)) + ACCELERATION_OF_HALTING)) + (WHEEL_CONSTANT / step_delay_float));
			if (temp_float > 0.000000F)
				new_delay = (WHEEL_CONSTANT / temp_float) * 1000000.0F;
		}
		if ((new_delay > step_delay_high) || (new_delay == 0UL))
		{
			drive_flag = HALT;
			new_delay = step_delay_high;
		}
	}
	else if (drive_flag == SUDDEN_ACCEL)
	{
		new_delay = step_delay_low;
	}

	else if (drive_flag == SUDDEN_DECEL)
	{
		if (step%2 == 0U)
		{
			temp_float = ((-(TIME_CONSTANT * ((INERTIA_MOMENT * (((fabsf(gyro_f.x) / RADIANS_TO_DEGREES) / TIME_CONSTANT) + 0.250000F)) +
					(ACCELERATION_OF_GRAVITY * sinf(fabsf(curr_filtered_angle.x / RADIANS_TO_DEGREES))) + ACCELERATION_OF_RISING)) / cosf(curr_filtered_angle.x / RADIANS_TO_DEGREES)) + (WHEEL_CONSTANT / step_delay_float));

			if (temp_float > 0.000000F)
				new_delay = (WHEEL_CONSTANT / temp_float) * 1000000.0F;
		}
		else if (step%2 == 1U)
		{
			temp_float = ((-(TIME_CONSTANT * ((INERTIA_MOMENT * (((fabsf(gyro_f.x) / RADIANS_TO_DEGREES) / TIME_CONSTANT) + 0.200000F)) +
					(ACCELERATION_OF_GRAVITY * sinf(fabsf(curr_filtered_angle.x / RADIANS_TO_DEGREES))) + ACCELERATION_OF_RISING)) / cosf(curr_filtered_angle.x / RADIANS_TO_DEGREES)) + (WHEEL_CONSTANT / step_delay_float));

			if (temp_float > 0.000000F)
				new_delay = (WHEEL_CONSTANT / temp_float) * 1000000.0F;
		}
		if ((new_delay > step_delay_high) || (new_delay == 0UL))
		{
			drive_flag = HALT;
			new_delay = step_delay_high;
		}
	}

	return new_delay;
}

void adjustVelocityLimit(void)
{
	uint32_t step_delay_incline;

	cos_val = cosf(curr_filtered_angle.x / RADIANS_TO_DEGREES);
	step_delay_incline = (uint32_t)((float)step_delay_vertical * fabsf(cos_val) * LIMIT_BETA);

	if (step_delay_incline < (DLPF_DELAY + 500UL))
		step_delay_low = (DLPF_DELAY + 500UL);
	else
		step_delay_low = step_delay_incline;
}

void setFlag(void)
{
	prev_drive_flag = drive_flag;
	prev_direction_flag = direction_flag;

	if (fabsf(curr_filtered_angle.x) <= boundary_inner)
	{
		if (prev_drive_flag == HALT)
		{
			drive_flag = HALT;
		}
		else
		{
			if (step_delay == step_delay_high)
			{
				drive_flag = HALT;
			}
			else
			{
				drive_flag = DECEL_APPROX;
			}
		}
	}
	else
	{
		if (curr_filtered_angle.x > 0.000000F)
		{
			direction_flag = FRONT;
			if (step_delay == step_delay_high)
				rotation_flag = FORWARD;
		}
		else
		{
			direction_flag = REAR;
			if (step_delay == step_delay_high)
				rotation_flag = BACKWARD;
		}

		if (((direction_flag == FRONT) && (rotation_flag == FORWARD)) || ((direction_flag == REAR) && (rotation_flag == BACKWARD)))
		{
			if (((direction_flag == FRONT) && ((-gyro_f.x) >= 0.000000F)) || ((direction_flag == REAR) && ((-gyro_f.x) < 0.000000F)))
			{
				drive_flag = ACCEL;
			}
			else
			{
				if (step%2 == 0U)
				{
					if (ACCELERATION_OF_GRAVITY * fabsf(sinf(curr_filtered_angle.x / RADIANS_TO_DEGREES)) > (INERTIA_MOMENT * 0.250000F))
					{
						drive_flag = RUN;
					}
					else
					{
						drive_flag = DECEL_EXACT_FALL;
					}
				}

				else if (step%2 == 1U)
				{
					if (ACCELERATION_OF_GRAVITY * fabsf(sinf(curr_filtered_angle.x / RADIANS_TO_DEGREES)) > (INERTIA_MOMENT * 0.200000F))
					{
						drive_flag = RUN;
					}
					else
					{
						drive_flag = DECEL_EXACT_FALL;
					}
				}
			}
		}
		else
		{
			if (((direction_flag == FRONT) && ((-gyro_f.x) >= 0.000000F)) || ((direction_flag == REAR) && ((-gyro_f.x) < 0.000000F)))
			{
				drive_flag = SUDDEN_DECEL;
			}
			else
			{
				drive_flag = DECEL_EXACT_LIE;
			}
		}
	}
}
void reactToAngleGyro(osThreadId_t handle)
{
	setFlag();
	adjustVelocityLimit();
	prev_step_delay = step_delay;
	step_delay = getStepDelay();
	accel = ((WHEEL_CONSTANT / ((float)step_delay / 1000000.0F)) - (WHEEL_CONSTANT / ((float)prev_step_delay / 1000000.0F))) / ((float)prev_step_delay / 1000000.0F);

	if (drive_flag == HALT)
	{
		unipolar_parallel_sequence_onetwoPhase((DLPF_DELAY+500UL), handle);
	}
	else
	{
		if (rotation_flag == FORWARD)
			step++;
		else if (rotation_flag == BACKWARD)
			step--;
		unipolar_parallel_sequence_onetwoPhase(step_delay, handle);
	}
}
/*
void momentFinder_with_accel_and_torque(void)
{
	char msg[150];
	static uint16_t count = 1U;

	if (find_flag == 1)
	{
		prev_direction_flag = direction_flag;
		prev_drive_flag = drive_flag;

		if (step_max == 0U)
		{
			drive_flag = HALT;
			step_total = 0UL;
			step_max = 50U;

			//			sprintf(msg, "COUNT STEP_DELAY STARTING_ANGLE ACC_ACCEL PREV_ANGLE_A CURR_ANGLE_A DT_A ACC_GYRO PREV_ANGLE_G CURR_ANGLE_G DT_G\r\n");
			//			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			//			sprintf(msg, "{\"CNT\":%03u,\"SD\":%05lu,\"SA\":%09.6f,"
			//					"\"AA\":%09.6f,\"PAA\":%09.6f,\"CAA\":%09.6f,\"DTA\":%.6f,"
			//					"\"AG\":%09.6f,\"PAG\":%09.6f,\"CAG\":%09.6f,\"DTG\":%.6f}\r\n",
			//					count, step_delay, starting_angle,
			//					max_accelero_acceleration, prev_angle_A, curr_angle_A, dt_temp_A,
			//					max_angular_acceleration, prev_angle_G, curr_angle_G, dt_temp_G);
			//			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 5000UL);
			sprintf(msg, "\"%u\":{\"SD\":%lu,\"AG\":%.6f,\"AA\":%.6f,"
					"\"SA\":%.6f,\"PAG\":%.6f,\"CAG\":%.6f,\"DTG\":%.6f},\r\n",
					count, step_delay, max_angular_acceleration, max_accelero_acceleration,
					starting_angle, prev_angle_G, curr_angle_G, dt_temp_G);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 5000UL);

			count++;
			//			step_delay = step_delay + 100UL;
			max_angular_acceleration = 0.000000F;
			prev_angle_G = 0.0F;
			curr_angle_G = 0.0F;
			dt_temp_G = 0.0F;
			max_accelero_acceleration = 0.0F;
			prev_angle_A = 0.0F;
			curr_angle_A = 0.0F;
			dt_temp_A = 0.0F;
			inertia_moment = 0.0F;
			starting_angle = 0.0F;
		}
		else
		{
			drive_flag = ACCEL;

			if (curr_filtered_angle.x > (boundary_inner))
			{
				direction_flag = FRONT;
				rotation_flag = FORWARD;
			}
			else if (curr_filtered_angle.x < (-boundary_inner))
			{
				direction_flag = REAR;
				rotation_flag = BACKWARD;
			}
			else
			{
				drive_flag = HALT;
				step_total = 0UL;
				step_max = 50U;

				sprintf(msg, "\"%u\":{\"SD\":%lu,\"AG\":%.6f,\"AA\":%.6f,"
						"\"SA\":%.6f,\"PAG\":%.6f,\"CAG\":%.6f,\"DTG\":%.6f},\r\n",
						count, step_delay, max_angular_acceleration, max_accelero_acceleration,
						starting_angle, prev_angle_G, curr_angle_G, dt_temp_G);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 5000UL);

				count++;
				//				step_delay = step_delay + 100UL;
				max_angular_acceleration = 0.0F;
				prev_angle_G = 0.0F;
				curr_angle_G = 0.0F;
				dt_temp_G = 0.0F;
				max_accelero_acceleration = 0.0F;
				prev_angle_A = 0.0F;
				curr_angle_A = 0.0F;
				dt_temp_A = 0.0F;
				inertia_moment = 0.0F;
				starting_angle = 0.0F;
			}

			//		if ((prev_drive_flag != HALT) && (drive_flag == HALT))
			//		{
			//			inertia_moment_G = ((cosf(starting_angle / RADIANS_TO_DEGREES) * ((WHEEL_CONSTANT / (step_delay / 1000000.0F)) / 0.001F)) - (sinf(starting_angle / RADIANS_TO_DEGREES) * ACCELERATION_OF_GRAVITY)) / (AXIS_TO_SENSOR * max_angular_acceleration);
			//			inertia_moment_A = ((cosf(starting_angle / RADIANS_TO_DEGREES) * ((WHEEL_CONSTANT / (step_delay / 1000000.0F)) / 0.001F)) + (sinf(starting_angle / RADIANS_TO_DEGREES) * ACCELERATION_OF_GRAVITY)) / (AXIS_TO_SENSOR * max_acceleration);
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
		}
		if (drive_flag == HALT)
		{
			step_reset(1000000UL);
		}
		else
		{
			if (prev_drive_flag == HALT)
			{
				unipolar_parallel_sequence_onetwoPhase(1000000UL);
				starting_angle = fabsf(curr_filtered_angle.x);
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
			}
		}

		if (count > 100U)
		{
			find_flag = 0;
			max_angular_acceleration = 0.0F;
			max_accelero_acceleration = 0.0F;
			prev_angle_G = 0.0F;
			curr_angle_G = 0.0F;
			dt_temp_G = 0.0F;
			inertia_moment = 0.0F;
			starting_angle = 0.0F;
			count = 1U;
		}
	}
}

void momentFinder_only_torque(void)
{
	char msg[150];
	static uint16_t count = 1U;

	if (excite_flag == 1)
	{
		unipolar_parallel_sequence_onetwoPhase(1000000UL);
		starting_angle = fabsf(curr_filtered_angle.x);

		if (step_remain != 0U)
		{
			if (rotation_flag == FORWARD)
				step++;
			else if (rotation_flag == BACKWARD)
				step--;
			new_unipolar_parallel_sequence_onetwoPhase(step_delay);

			step_remain--;
			if (rotation_flag == BACKWARD)
			{
				sprintf(msg, "\"%u\":{\"SD\":%lu,\"AG\":%.6f,\"AA\":%.6f,"
						"\"SA\":%.6f,\"PAG\":%.6f,\"CAG\":%.6f,\"DTG\":%.6f},\r\n",
						count, step_delay, max_angular_acceleration, max_accelero_acceleration,
						starting_angle, prev_angle_G, curr_angle_G, dt_temp_G);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 5000UL);
				count++;
			}

			max_angular_acceleration = 0.0F;
			prev_angle_G = 0.0F;
			curr_angle_G = 0.0F;
			dt_temp_G = 0.0F;
			max_accelero_acceleration = 0.0F;
			prev_angle_A = 0.0F;
			curr_angle_A = 0.0F;
			dt_temp_A = 0.0F;
			starting_angle = 0.0F;
		}
	}
	else if (excite_flag == 0)
	{
		step_reset(1000000UL);
	}
}
 */
