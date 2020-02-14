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


void bigStepper_forward_sequence(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_);

void bigStepper_forward_sequence2(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_);

void bigStepper_backward_sequence(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_);

void bigStepper_slower(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_);

void bigStepper_forward_sequence_parallel(void);
void bigStepper_forward_sequence_parallel2(void);

void bigStepper_backward_sequence_parallel(void);

void bigStepper_reactToAccel(GPIO_TypeDef * gpioA, uint16_t pinA, GPIO_TypeDef * gpioA_, uint16_t pinA_,
		GPIO_TypeDef * gpioB, uint16_t pinB, GPIO_TypeDef * gpioB_, uint16_t pinB_);

void bigStepper_reactToAccel_parallel(void);

void bigStepper_all_reset(void);

#endif /* STEPPERMOTOR_H_ */
