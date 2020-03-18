/*
 * complementary_filter.c
 *
 *  Created on: 2020. 2. 20.
 *      Author: gdabba
 */

#include "complementary_filter.h"

extern MPU6050_int32_t diffacc;
extern MPU6050_int32_t diffgyro;
extern uint32_t microTick;

uint32_t t_prev = 0;
uint32_t t_now = 0;
float_t dt = 0.0F;

MPU6050_float_t accel = {0, 0, 0};
MPU6050_float_t accel_angle = {0, 0, 0};

MPU6050_float_t gyro = {0, 0, 0};
MPU6050_float_t gyro_angle = {0, 0, 0};

float_t accel_xz, accel_yz;
MPU6050_float_t tmp_angle = {0, 0, 0};
MPU6050_float_t filtered_angle = {0, 0, 0};

int8_t angle = 0;

const float_t RADIANS_TO_DEGREES = 180 / 3.14159F;
const float_t GYROXYZ_TO_DEGREES_PER_SEC = 131.0F;

void initDT(void)
{
	t_prev = microTick;
}

void calcDT(void)
{
	t_now = microTick;
	dt = (float_t)(t_now - t_prev) / 1000000.0F;
	t_prev = t_now;
}

void calcAccelYPR(void)
{
	accel.x = (float_t)diffacc.x / 16384.0F;
	accel.y = (float_t)diffacc.y / 16384.0F;

	// 센서가 뒤집어져서 accelerometer Z축이 음수의 값을 가짐
	accel.z = (float_t)(diffacc.z - 16384L) / 16384.0F;

	accel_yz = (float)sqrt(pow(accel.y, 2) + pow(accel.z, 2));
	accel_angle.y = (float)atan(-accel.x / accel_yz) * RADIANS_TO_DEGREES;

	accel_xz = (float)sqrt(pow(accel.x, 2) + pow(accel.z, 2));
	accel_angle.x = (float)atan(accel.y / accel_xz) * RADIANS_TO_DEGREES;

	accel_angle.z = 0;
}

void calcGyroYPR(void)
{
	gyro.x = (float_t)diffgyro.x / GYROXYZ_TO_DEGREES_PER_SEC;
	gyro.y = (float_t)diffgyro.y / GYROXYZ_TO_DEGREES_PER_SEC;
	gyro.z = (float_t)diffgyro.z / GYROXYZ_TO_DEGREES_PER_SEC;

	gyro_angle.x = gyro.x * dt;
	gyro_angle.y = gyro.y * dt;
	gyro_angle.z = gyro.z * dt;
}

void calcFilteredYPR(void)
{
	const float_t ALPHA = 0.96;

	tmp_angle.x = filtered_angle.x + gyro_angle.x;
	tmp_angle.y = filtered_angle.y + gyro_angle.y;
	tmp_angle.z = filtered_angle.z + gyro_angle.z;

	filtered_angle.x = (ALPHA * tmp_angle.x) + ((1.0F-ALPHA) * accel_angle.x);
	filtered_angle.y = (ALPHA * tmp_angle.y) + ((1.0F-ALPHA) * accel_angle.y);
	filtered_angle.z = tmp_angle.z;

	angle = (int8_t)filtered_angle.x;
}
