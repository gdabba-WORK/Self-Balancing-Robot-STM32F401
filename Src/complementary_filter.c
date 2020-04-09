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
extern float ACCELERATION_OF_GRAVITY;
const float RADIANS_TO_DEGREES = 180.0F / 3.141590F;
const float GYROXYZ_TO_DEGREES_PER_SEC = 131.0F;

uint32_t t_prev = 0;
uint32_t t_now = 0;
float dt_calc = 0.0F;
int8_t angle = 0;
float prev_filtered_angle_x = 0.0F;

MPU6050_float_t accel_f = {0.0F, 0.0F, 0.0F};
MPU6050_float_t accel_angle = {0.0F, 0.0F, 0.0F};

MPU6050_float_t gyro_f = {0.0F, 0.0F, 0.0F};
MPU6050_float_t gyro_angle = {0.0F, 0.0F, 0.0F};

float accel_xz, accel_yz;
MPU6050_float_t tmp_angle = {0.0F, 0.0F, 0.0F};
MPU6050_float_t curr_filtered_angle = {0.0F, 0.0F, 0.0F};

float COMPLEMENTARY_ALPHA = 0.990F;
float REAL_DEGREE_COEFFICIENT = 0.0F;

int8_t zero_flag = 0;

float prev_gyro_x = 0.0F;
float angular_acceleration = 0.0F;
float accelero_acceleration = 0.0F;

void initDT(void)
{
	t_prev = MY_GetTick();
}

void calcDT(void)
{
	//	char msg[50];
	t_now = MY_GetTick();
	dt_calc = (float)(t_now - t_prev) / 1000000.0F;
	//	if (print_flag)
	//	{
	//		sprintf(msg, "t_now= %lu\r\n", t_now);
	//		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 0xFFFF);
	//		sprintf(msg, "t_prev=%lu\r\n", t_prev);
	//		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 0xFFFF);
	//		sprintf(msg, "dt=%.6f\r\n", dt);
	//		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 0xFFFF);
	//	}
	t_prev = MY_GetTick();
}

void calcAccelYPR(void)
{
	accel_f.x = (float)diffacc.x / 16384.0F;
	accel_f.y = (float)diffacc.y / 16384.0F;

	// 센서가 뒤집어져서 accelerometer Z축이 음수의 값을 가짐
	//	accel.z = (float_t)(diffacc.z - 16384L) / 16384.0F;
	accel_f.z = (float)fabsf((diffacc.z - 16384L)) / 16384.0F;

	//	accel_yz = (float)sqrt(pow(accel.y, 2) + pow(accel.z, 2));
	//	accel_angle.y = (float)atan(-accel.x / accel_yz) * RADIANS_TO_DEGREES;

	accel_xz = (float)sqrt(pow(accel_f.x, 2) + pow(accel_f.z, 2));
	accel_angle.x = (float)atan(accel_f.y / accel_xz) * RADIANS_TO_DEGREES;

	//	accel_angle.z = 0;

	accelero_acceleration = accel_f.y * ACCELERATION_OF_GRAVITY;
}

void calcGyroYPR(void)
{
	prev_gyro_x = gyro_f.x;
	gyro_f.x = (float)(diffgyro.x) / GYROXYZ_TO_DEGREES_PER_SEC;
	//	gyro.y = (float_t)diffgyro.y / GYROXYZ_TO_DEGREES_PER_SEC;
	//	gyro.z = (float_t)diffgyro.z / GYROXYZ_TO_DEGREES_PER_SEC;

	gyro_angle.x = -(gyro_f.x * dt_calc);
	//	gyro_angle.y = gyro.y * dt;
	//	gyro_angle.z = gyro.z * dt;

	angular_acceleration = ((gyro_f.x - prev_gyro_x) / RADIANS_TO_DEGREES) / dt_calc;
}

void calcFilteredYPR()
{
	tmp_angle.x = curr_filtered_angle.x + gyro_angle.x;
	//	tmp_angle.y = filtered_angle.y + gyro_angle.y;
	//	tmp_angle.z = filtered_angle.z + gyro_angle.z;


	prev_filtered_angle_x = curr_filtered_angle.x;
	curr_filtered_angle.x = (COMPLEMENTARY_ALPHA * tmp_angle.x) + ((1.0F-COMPLEMENTARY_ALPHA) * accel_angle.x);
	//	filtered_angle.y = (ALPHA * tmp_angle.y) + ((1.0F-ALPHA) * accel_angle.y);
	//	filtered_angle.z = tmp_angle.z;

	//	angle = (int8_t)filtered_angle.x;
	//	REAL_DEGREE_COEFFICIENT = fabsf((prev_filtered_angle_x - curr_filtered_angle.x) / RADIANS_TO_DEGREES) / dt_calc;
	//	if (REAL_DEGREE_COEFFICIENT == 0.0F)
	//		zero_flag = 1;


}
