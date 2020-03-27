/*
 * complementary_filter.h
 *
 *  Created on: 2020. 2. 20.
 *      Author: gdabba
 */

#ifndef INC_COMPLEMENTARY_FILTER_H_
#define INC_COMPLEMENTARY_FILTER_H_

#include "main.h"

typedef struct {
	float x;
	float y;
	float z;
} MPU6050_float_t;


void initDT(void);

void calcDT(void);

void calcAccelYPR(void);

void calcGyroYPR(void);

void calcFilteredYPR();

#endif /* INC_COMPLEMENTARY_FILTER_H_ */


