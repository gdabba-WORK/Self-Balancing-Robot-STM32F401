/*
 * complementary_filter.c
 *
 *  Created on: 2020. 2. 20.
 *      Author: gdabba
 */

#include "complementary_filter.h"

extern MPU6050_int32_t diffacc;
extern MPU6050_int32_t diffgyro;

uint32_t t_prev = 0;
uint32_t t_now = 0;
uint32_t dt = 0;

MPU6050_float_t accel;
MPU6050_float_t accel_angle;

MPU6050_float_t gyro;
MPU6050_float_t gyro_angle;

MPU6050_float_t filtered_angle;

void initDT(void)
{
	t_prev = microTick;
}

void calcDT(void)
{
	t_now = microTick;
	dt = (t_now - t_prev) / 1000
}

void calcAccelYPR()
{
	float accel_xz, accel_yz;
	const float RADIANS_TO_DEGREES = 180 / 3.14159;

	accel.x = (float)diffacc.x;
	accel.y = (float)diffacc.y;
	accel.z = (float)(diffacc.z + 16384);

	accel_yz = sqrt(pow(accel.y, 2) + pow(accel.z, 2));
	accel_angle.y = atan(-accel.x / accel.yz) * RADIANS_TO_DEGREES;

	accel_xz = sqrt(pow(accel.x, 2) + pow(accel.z, 2));
	accel_angle.x = atan(accel_y / accel_xz) * RADIANS_TO_DEGREES;

	accel_angle.z = 0;
}
