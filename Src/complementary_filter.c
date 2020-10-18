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
extern int8_t print_flag;
extern const float AXIS_TO_SENSOR;
extern const float FLOOR_TO_SENSOR;
extern float ACCELERATION_OF_GRAVITY;
extern uint32_t DLPF_DELAY;
extern Motor_Rotation rotation_flag;
extern float accel;

const float RADIANS_TO_DEGREES = 180.0F / 3.141590F;
const float GYROXYZ_TO_DEGREES_PER_SEC = 65.5360F;

uint32_t t_prev = 0;
uint32_t t_now = 0;
float dt_calc = 0.0F;
int8_t angle = 0;
float prev_filtered_angle_x = 0.0F;
float angular_accel_angle = 0.0F;

MPU6050_float_t accel_f = {0.0F, 0.0F, 0.0F};
MPU6050_float_t accel_angle = {0.0F, 0.0F, 0.0F};

MPU6050_float_t gyro_f = {0.0F, 0.0F, 0.0F};
MPU6050_float_t gyro_angle = {0.0F, 0.0F, 0.0F};

float accel_xz, accel_yz;
MPU6050_float_t tmp_angle = {0.0F, 0.0F, 0.0F};
MPU6050_float_t curr_filtered_angle = {0.0F, 0.0F, 0.0F};

float COMPLEMENTARY_ALPHA = 0.9960F;
float REAL_DEGREE_COEFFICIENT = 0.0F;

int8_t zero_flag = 0;

float prev_gyro_x = 0.0F;
float angular_acceleration = 0.0F;
float accelero_acceleration = 0.0F;

float prev_accel_y = 0.0F;
float prev_accel_z = 0.0F;

// 시간 변화량 측정을 위한 초기화 작업
void initDT(void)
{
	t_prev = MY_GetTick();
}

// 현재 측정 시간과 이전 측정 시간의 차로 구하는 시간 변화량 측정
void calcDT(void)
{
	t_now = MY_GetTick();
	dt_calc = (float)(t_now - t_prev) / 1000000.0F;
	t_prev = MY_GetTick();
}

// Accelerometer에 의한 각도 측정
void calcAccelYPR(void)
{
	accel_f.x = (float)diffacc.x / 8192.0F;
	prev_accel_y = accel_f.y;
	accel_f.y = (float)diffacc.y / 8192.0F;
	// 센서가 뒤집어져서 accelerometer Z축이 음수의 값을 가짐
	prev_accel_z = accel_f.z;
	accel_f.z = (float)(-(diffacc.z - 8192L)) / 8192.0F;

	accel_xz = (float)sqrtf(powf(accel_f.x, 2.0F) + powf(accel_f.z, 2.0F));
	accel_angle.x = (float)atanf(accel_f.y / accel_xz) * RADIANS_TO_DEGREES;

	accelero_acceleration = accel_f.y;
}

// Gyroscope에 의한 각도 측정
void calcGyroYPR(void)
{
	prev_gyro_x = gyro_f.x;
	gyro_f.x = (float)(diffgyro.x) / GYROXYZ_TO_DEGREES_PER_SEC;

	gyro_angle.x = -(gyro_f.x * dt_calc);

	angular_acceleration = ((gyro_f.x - prev_gyro_x) / RADIANS_TO_DEGREES) / dt_calc;
}

// 상보필터 원리를 적용하여, Accelerometer와 Gyroscope로 얻어낸
// 각각의 각도들을 연산을 통해 최종 각도 예측
void calcFilteredYPR()
{
	tmp_angle.x = curr_filtered_angle.x + gyro_angle.x;
	prev_filtered_angle_x = curr_filtered_angle.x;
	curr_filtered_angle.x = (COMPLEMENTARY_ALPHA * tmp_angle.x) + ((1.0000F-COMPLEMENTARY_ALPHA) * angular_accel_angle);
}

// MPU-6050 센서와 바닥으로 부터의 거리를 실측하고,
// 중력 가속도와 로봇의 현재 속도를 바탕으로 각가속도를 예측
void calcAngularAccelYPR()
{
	float asin_parameter = 0.0F;

	if (rotation_flag == FORWARD)
	{
		if (curr_filtered_angle.x >= 0.0F)
		{
			asin_parameter = ((ACCELERATION_OF_GRAVITY * prev_accel_y) - (-accel * prev_accel_z) - (-accel * powf((gyro_f.x / RADIANS_TO_DEGREES), 2.0F) * FLOOR_TO_SENSOR / ACCELERATION_OF_GRAVITY)
					- (angular_acceleration * FLOOR_TO_SENSOR)) / (ACCELERATION_OF_GRAVITY - (-powf(accel, 2.0F) / ACCELERATION_OF_GRAVITY));
		}
		else if (curr_filtered_angle.x < 0.0F)
		{
			asin_parameter = ((ACCELERATION_OF_GRAVITY * prev_accel_y) - (-accel * prev_accel_z) - (-accel * powf((gyro_f.x / RADIANS_TO_DEGREES), 2.0F) * FLOOR_TO_SENSOR / ACCELERATION_OF_GRAVITY)
					- (angular_acceleration * FLOOR_TO_SENSOR)) / (ACCELERATION_OF_GRAVITY + (-powf(accel, 2.0F) / ACCELERATION_OF_GRAVITY));
		}
	}
	else if (rotation_flag == BACKWARD)
	{
		if (curr_filtered_angle.x >= 0.0F)
		{
			asin_parameter = ((ACCELERATION_OF_GRAVITY * prev_accel_y) - (accel * prev_accel_z) - (accel * powf((gyro_f.x / RADIANS_TO_DEGREES), 2.0F) * FLOOR_TO_SENSOR / ACCELERATION_OF_GRAVITY)
					- (angular_acceleration * FLOOR_TO_SENSOR)) / (ACCELERATION_OF_GRAVITY - (powf(accel, 2.0F) / ACCELERATION_OF_GRAVITY));
		}
		else if (curr_filtered_angle.x < 0.0F)
		{
			asin_parameter = ((ACCELERATION_OF_GRAVITY * prev_accel_y) - (accel * prev_accel_z) - (accel * powf((gyro_f.x / RADIANS_TO_DEGREES), 2.0F) * FLOOR_TO_SENSOR / ACCELERATION_OF_GRAVITY)
					- (angular_acceleration * FLOOR_TO_SENSOR)) / (ACCELERATION_OF_GRAVITY + (powf(accel, 2.0F) / ACCELERATION_OF_GRAVITY));
		}
	}

	if (asin_parameter > 1.0F)
		asin_parameter = 1.0F;
	else if (asin_parameter < -1.0F)
		asin_parameter = -1.0F;

	angular_accel_angle = asinf(asin_parameter) * RADIANS_TO_DEGREES;
}
