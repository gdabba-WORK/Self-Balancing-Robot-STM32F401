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
#include "cmsis_os2.h"

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

void step_A(void);
void step_B(void);
void step_a(void);
void step_b(void);
void step_AB(void);
void step_aB(void);
void step_ab(void);
void step_Ab(void);
void step_ABa(void);
void step_Bab(void);
void step_abA(void);
void step_bAB(void);
void step_reset(uint32_t step_delay);

void unipolar_parallel_sequence_onePhase(uint32_t step_delay, osThreadId_t handle);
void unipolar_parallel_sequence_twoPhase(uint32_t step_delay, osThreadId_t handle);
void unipolar_parallel_sequence_onetwoPhase(uint32_t step_delay, osThreadId_t handle);
void unipolar_parallel_sequence_threePhase(uint32_t step_delay, osThreadId_t handle);

float getAlpha(void);
void reactToAngle(void);

uint32_t getStepDelay(void);
void adjustVelocityLimit(void);
void setFlag(void);
void reactToAngleGyro(osThreadId_t handle);

void momentFinder_with_accel_and_torque(void);
void momentFinder_only_torque(void);
#endif /* STEPPERMOTOR_H_ */
