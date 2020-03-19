/*
 * complementary_filter.h
 *
 *  Created on: 2020. 2. 20.
 *      Author: gdabba
 */

#ifndef INC_COMPLEMENTARY_FILTER_H_
#define INC_COMPLEMENTARY_FILTER_H_

#include "main.h"
#include <math.h>

typedef struct {
	float_t x;
	float_t y;
	float_t z;
} MPU6050_float_t;
#endif /* INC_COMPLEMENTARY_FILTER_H_ */

void initDT(void);

void calcDT(void);

void calcAccelYPR(void);

void calcGyroYPR(void);

void calcFilteredYPR(int8_t*);
