/**
 ******************************************************************************
 * @file    MPU6050.c
 * @author  Waveshare Team
 * @version V1.0
 * @date    29-August-2014
 * @brief   This file includes the MPU6050 driver functions

 ******************************************************************************
 * @attention
 *
 *Waveshare의 소스를 HAL라이브러리에서 동작되도록 포팅함
 ******************************************************************************
 */


#include "MPU6050.h"


MPU6050_int16_t gyroOffset, accOffset;
HAL_StatusTypeDef status;

//센서로 부터 1바이트 읽기
// 파라메터1 : 센서 어드레스, 파라메터 2 : 센서내 레지스터 어드레스
// 리턴값 : 센서응답값

uint8_t MPU6050_ReadOneByte(uint8_t RegAddr)
{
	//	char msg[50];

	uint8_t Data = 0;
	status = HAL_I2C_Mem_Read(&MPU6050_I2C_PORT,MPU6050_DEVICE_ADDR,RegAddr,1,&Data,1,0x0A);
	//	sprintf(msg, "MPU6050_ReadOneByte() : %u\r\n", status);
	//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	//	printf("MPU6050_ReadOneByte() : %u\n", status);
	return Data;
}

//센서에  1바이트 쓰기
// 파라메터1 : 센서 어드레스, 파라메터 2 : 센서내 레지스터 어드레스
// 리턴값 : 센서응답값
void MPU6050_WriteOneByte(uint8_t RegAddr, uint8_t Data)
{
	char msg[50];

	status = HAL_I2C_Mem_Write(&MPU6050_I2C_PORT,MPU6050_DEVICE_ADDR,RegAddr,I2C_MEMADD_SIZE_8BIT,&Data,1,1000);
	sprintf(msg, "MPU6050_WriteOneByte() : %u\r\n", status);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	return;
}

bool MPU6050_WriteBits(uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data)
{

	uint8_t Dat, Mask;

	Dat = MPU6050_ReadOneByte(RegAddr);
	Mask = (0xFF << (BitStart + 1)) | 0xFF >> ((8 - BitStart) + Length - 1);
	Data <<= (8 - Length);
	Data >>= (7 - BitStart);
	Dat &= Mask;
	Dat |= Data;
	MPU6050_WriteOneByte(RegAddr, Dat);

	return true;
}

bool MPU6050_WriteOneBit(uint8_t RegAddr, uint8_t BitNum, uint8_t Data)
{
	uint8_t Dat;

	Dat = MPU6050_ReadOneByte(RegAddr);
	Dat = (Data != 0) ? (Dat | (1 << BitNum)) : (Dat & ~(1 << BitNum));
	MPU6050_WriteOneByte(RegAddr, Dat);

	return true;
}

//버퍼읽기 (디바이스 어드레스, 레지스터 어드레스, 데이터 크기, 버퍼 포인터)
bool MPU6050_ReadBuff(uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	// 메모리 읽기(디바이스 어드레스, 8비트 어드레스 메모리 크기, 버퍼 포인터, 버퍼숫자, 시도횟수)
	status = HAL_I2C_Mem_Read(&MPU6050_I2C_PORT,MPU6050_DEVICE_ADDR,RegAddr,I2C_MEMADD_SIZE_8BIT,pBuff,Num,0x0A);
	//	printf("MPU6050_ReadBuff() : %u\n", status);
	return status;
}

bool MPU6050_Check(void)
{
	if(MPU6050_ADDRESS_AD0_LOW == MPU6050_ReadOneByte(MPU6050_RA_WHO_AM_I))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void MPU6050_CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{
	uint8_t i;

	*(pAvgBuffer + ((*pIndex) ++)) = InVal;
	*pIndex &= 0x07;

	*pOutVal = 0;
	for(i = 0; i < 8; i ++)
	{
		*pOutVal += *(pAvgBuffer + i);
	}
	*pOutVal >>= 3;
}

void MPU6050_SetClockSource(uint8_t source)
{
	MPU6050_WriteBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void MPU6050_SetFullScaleGyroRange(uint8_t range)
{
	MPU6050_WriteBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void MPU6050_SetLPF(uint8_t	LowPassFilter)
{
	MPU6050_WriteOneByte(MPU6050_RA_CONFIG, LowPassFilter);
}

void MPU6050_SetFullScaleAccelRange(uint8_t range)
{
	MPU6050_WriteBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void MPU6050_SetSleepEnabled(uint8_t enabled)
{
	MPU6050_WriteOneBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

void MPU6050_SetDeviceResetEnabled(uint8_t enabled)
{
	MPU6050_WriteOneBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, enabled);
}

void MPU6050_SetI2CMasterModeEnabled(uint8_t enabled)
{
	MPU6050_WriteOneBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void MPU6050_SetI2CBypassEnabled(uint8_t enabled)
{
	MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}



void MPU6050_Init(uint8_t	lpf)
{
	// added by gdabba
	//	MPU6050_WriteOneByte(MPU6050_RA_PWR_MGMT_1, 0x00);
	//	uint8_t buffer[1] = {0};
	//	char msg[10];

	MPU6050_SetDeviceResetEnabled(1);
	MPU6050_SetSleepEnabled(0);

	// MPU6050 Sample Rate = 1kHz
	MPU6050_WriteOneByte(MPU6050_RA_SMPLRT_DIV, 7);

	//	MPU6050_ReadBuff(MPU6050_RA_SMPLRT_DIV, 1, buffer);
	//	sprintf(msg, "SMPLRT=%u", buffer[0]);
	//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	//	osDelay(3000);

	//	MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
	MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_1000);
	MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);

	/*
	 	MPU6050_DLPF_BW_256         0x00
		MPU6050_DLPF_BW_188         0x01
		MPU6050_DLPF_BW_98          0x02
		MPU6050_DLPF_BW_42          0x03
		MPU6050_DLPF_BW_20          0x04
		MPU6050_DLPF_BW_10          0x05
		MPU6050_DLPF_BW_5           0x06
	 */
	//	MPU6050_SetLPF(lpf);

	//	MPU6050_SetI2CMasterModeEnabled(0);
	//	MPU6050_SetI2CBypassEnabled(1);

	//	MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, 0);
	//	MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, 0);
	//	MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, 1);
	//	MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, 1);
	//	MPU6050_WriteOneBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1);
	osDelay(100);  // 자이로 안정화 대기
}

void MPU6050_InitOffset(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* tmpr)
{
	uint16_t i;
	int32_t	 TempAx = 0, TempAy = 0, TempAz = 0, TempGx = 0, TempGy = 0, TempGz = 0;

	for(i = 0; i < 2048; i++)
	{
		MPU6050_GetData(ax,ay,az,gx,gy,gz,tmpr);

		TempAx += *ax;
		TempAy += *ay;
		TempAz += *az;
		TempGx += *gx;
		TempGy += *gy;
		TempGz += *gz;
		osDelay(1);
	}

	accOffset.X = TempAx >> 11;
	accOffset.Y = TempAy >> 11;
	accOffset.Z = TempAz >> 11;

	gyroOffset.X = TempGx >> 11;
	gyroOffset.Y = TempGy >> 11;
	gyroOffset.Z = TempGz >> 11;
}

void MPU6050_GetData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* tmpr)
{
	uint8_t Buffer[14] = {0};
	int16_t InBuffer[7] = {0};
	int16_t temp;

	MPU6050_ReadBuff(MPU6050_RA_ACCEL_XOUT_H, 14, Buffer);

	InBuffer[0] = (((int16_t)Buffer[0]) << 8) | Buffer[1];
	InBuffer[1] = (((int16_t)Buffer[2]) << 8) | Buffer[3];
	InBuffer[2] = (((int16_t)Buffer[4]) << 8) | Buffer[5];

	InBuffer[3] = (((int16_t)Buffer[8]) << 8) | Buffer[9];
	InBuffer[4] = (((int16_t)Buffer[10]) << 8) | Buffer[11];
	InBuffer[5] = (((int16_t)Buffer[12]) << 8) | Buffer[13];

	temp = (((int16_t)Buffer[6]) << 8) | Buffer[7];
	InBuffer[6] = (int16_t)(temp * 10L / 34) + 3653;

	*ax = *(InBuffer + 0);
	//	*ax = *(InBuffer + 0) / 16384;
	*ay = *(InBuffer + 1);
	//	*ay = *(InBuffer + 1) / 16384;
	*az = *(InBuffer + 2);
	//	*az = *(InBuffer + 2) / 16384;
	*gx = *(InBuffer + 3);
	//	*gx = *(InBuffer + 3) / 32.8f;
	*gy = *(InBuffer + 4);
	//	*gy = *(InBuffer + 4) / 32.8f;
	*gz = *(InBuffer + 5);
	//	*gz = *(InBuffer + 5) / 32.8f;
	*tmpr = *(InBuffer + 6);
}
