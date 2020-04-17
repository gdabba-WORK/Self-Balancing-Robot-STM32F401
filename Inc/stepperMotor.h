/*
 * stepperMotor.h
 *
 *  Created on: 2020. 1. 31.
 *      Author: gdabba
 */

#ifndef STEPPERMOTOR_H_
#define STEPPERMOTOR_H_
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

typedef enum
{
	ONE_PHASE		= 0U,
	TWO_PHASE		= 1U,
	ONETWO_PHASE	= 2U,
	THREE_PHASE	= 3U
} Motor_Mode;

typedef enum
{
	NO_MOTOR		= 0U,
	LEFT_MOTOR		= 1U,
	RIGHT_MOTOR	= 2U,
	BOTH_MOTOR		= 3U
} Motor_State;

typedef enum
{
	FORWARD	= 0U,
	BACKWARD	= 1U
} Motor_Rotation;

typedef enum
{
	FRONT		= 0U,
	REAR		= 1U,
	STOP		= 2U
} Robot_Direction;

typedef enum
{
	HALT			= 0U,
	READY			= 1U,
	RUN				= 2U,
	ACCEL			= 3U,
	DECEL_APPROX	= 4U,
	DECEL_EXACT_FALL	= 5U,
	DECEL_EXACT_LIE	= 6U,
	SUDDEN_ACCEL	= 7U,
	SUDDEN_DECEL	= 8U
} Robot_Drive;
void HAL_Delay(uint32_t Delay);

void bigStepper_forward_sequence(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_);
void bigStepper_forward_sequence2(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_);
void bigStepper_backward_sequence(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_);
void bigStepper_slower(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_);
void reactToAccel(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_);

void step_A(uint32_t step_delay);
void step_B(uint32_t step_delay);
void step_a(uint32_t step_delay);
void step_b(uint32_t step_delay);
void step_AB(uint32_t step_delay);
void step_aB(uint32_t step_delay);
void step_ab(uint32_t step_delay);
void step_Ab(uint32_t step_delay);
void new_step_A(uint32_t step_delay);
void new_step_B(uint32_t step_delay);
void new_step_a(uint32_t step_delay);
void new_step_b(uint32_t step_delay);
void new_step_AB(uint32_t step_delay);
void new_step_aB(uint32_t step_delay);
void new_step_ab(uint32_t step_delay);
void new_step_Ab(uint32_t step_delay);
void step_ABa(uint32_t step_delay);
void step_Bab(uint32_t step_delay);
void step_abA(uint32_t step_delay);
void step_bAB(uint32_t step_delay);
void step_reset(uint32_t step_delay);

void unipolar_parallel_sequence_onePhase(uint32_t step_delay);
void unipolar_parallel_sequence_twoPhase(uint32_t step_delay);
void unipolar_parallel_sequence_onetwoPhase(uint32_t step_delay);
void unipolar_parallel_sequence_threePhase(uint32_t step_delay);
void new_unipolar_parallel_sequence_onetwoPhase(uint32_t step_delay);

//void reactToAccel_parallel(int8_t* angle);
float getAlpha(void);
void reactToAngle(void);

uint32_t getStepDelay(void);
void adjustVelocityLimit(void);
void setFlag(void);
void reactToAngleGyro(void);

void momentFinder_with_accel_and_torque(void);
void momentFinder_only_torque(void);
#endif /* STEPPERMOTOR_H_ */
