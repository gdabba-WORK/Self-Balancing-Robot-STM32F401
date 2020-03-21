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
#include "MPU6050.h"
#include "usart.h"

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
	FORWARD		= 0U,
	BACKWARD		= 1U,
	STOP			= 2U
} Robot_Direction;

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

void step_A(uint8_t step_delay_dynamic);
void step_B(uint8_t step_delay_dynamic);
void step_a(uint8_t step_delay_dynamic);
void step_b(uint8_t step_delay_dynamic);
void step_AB(uint8_t step_delay_dynamic);
void step_aB(uint8_t step_delay_dynamic);
void step_ab(uint8_t step_delay_dynamic);
void step_Ab(uint8_t step_delay_dynamic);
void step_ABa(uint8_t step_delay_dynamic);
void step_Bab(uint8_t step_delay_dynamic);
void step_abA(uint8_t step_delay_dynamic);
void step_bAB(uint8_t step_delay_dynamic);

void step_reset(void);

void unipolar_parallel_sequence_onePhase(uint8_t step_delay_dynamic);
void unipolar_parallel_sequence_twoPhase(uint8_t step_delay_dynamic);
void unipolar_parallel_sequence_onetwoPhase(uint8_t step_delay_dynamic);
void unipolar_parallel_sequence_threePhase(uint8_t step_delay_dynamic);

void reactToAccel_parallel(int8_t*);

#endif /* STEPPERMOTOR_H_ */
