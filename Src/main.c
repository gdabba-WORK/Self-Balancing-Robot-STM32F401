/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_data = 0;
extern uint32_t step_delay_static, step_delay_dynamic;
extern uint32_t step_delay_low, step_delay_high, step_delay_vertical, step_delay_horizontal;
extern int16_t step_max;
extern Robot_Direction lean_direction_flag;
extern Motor_Mode mode_flag;
extern Motor_State state_flag;
extern uint32_t sync_period;
extern float boundary_outer;
extern float boundary_inner;
uint32_t microTick = 0UL;
extern float alpha_former;
extern float alpha_latter;
extern float beta;
extern float coefficient;
extern int8_t angle;
extern int8_t _angle;
extern float dt;
extern HAL_StatusTypeDef status;
int8_t print_flag = 0;
extern float ALPHA;
extern MPU6050_float_t filtered_angle;
extern float cos_val;
extern MPU6050_int16_t accOffset, gyroOffset;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */


	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */


	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C3_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_TIM10_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	//	char msg[20] = "Hello World!\n\r";
	HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	HAL_TIM_Base_Start_IT(&htim10);

	/* USER CODE END 2 */
	/* Init scheduler */
	osKernelInitialize();

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void)
{
	/* USART1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	char msg[50];

	if (huart->Instance == USART1)
	{
		switch (rx_data) {
		case '0' :
			if ((sync_period - 5UL) > 0UL)
				sync_period = sync_period - 5UL;
			//			state_flag = NO_MOTOR;
			break;
		case '1' :
			if ((step_max - 4U) > 0U)
				step_max = step_max - 4U;
			//			state_flag = LEFT_MOTOR;
			break;
		case '2' :
			sprintf(msg, "step_max=%10d\r\n", step_max);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			//			state_flag = BOTH_MOTOR;
			break;
		case '3' :
			step_max = step_max + 4U;
			//			state_flag = RIGHT_MOTOR;
			break;
		case '4' :
			step_delay_vertical = step_delay_vertical - 10UL;
			//			step_delay_static = 0U;
			//			step_delay_dynamic = 4U;
			break;
		case '5' :
			sprintf(msg, "step_delay_vertical=%10lu\r\n", step_delay_vertical);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			//			step_delay_static = 1U;
			//			step_delay_dynamic = 4U;
			break;
		case '6' :
			if ((step_delay_vertical + 10UL) < step_delay_high)
				step_delay_vertical = step_delay_vertical + 10UL;
			//			step_delay_static = 2U;
			//			step_delay_dynamic = 4U;
			break;
		case 'q' :
			if ((step_delay_horizontal - 10UL) > 1020)
				step_delay_horizontal = step_delay_horizontal - 10UL;
			//			step_delay_static = 0U;
			//			step_delay_dynamic = 4U;
			break;
		case 'w' :
			sprintf(msg, "step_delay_horizontal=%10lu\r\n", step_delay_horizontal);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			//			step_delay_static = 1U;
			//			step_delay_dynamic = 4U;
			break;
		case 'e' :
			if ((step_delay_horizontal + 10UL) < step_delay_vertical)
				step_delay_horizontal = step_delay_horizontal + 10UL;
			//			step_delay_static = 2U;
			//			step_delay_dynamic = 4U;
			break;
		case '7' :
			if ((step_delay_high - 10UL) > step_delay_low)
				step_delay_high = step_delay_high - 10UL;
			//			mode_flag = ONE_PHASE;
			break;
		case '8' :
			sprintf(msg, "step_delay_high=%10lu\r\n", step_delay_high);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			//			mode_flag = TWO_PHASE;
			break;
		case '9' :
			step_delay_high = step_delay_high + 10UL;
			//			mode_flag = ONETWO_PHASE;
			break;
		case '.' :
			sync_period = sync_period + 5UL;
			//			if (direction_flag == FORWARD)
			//				direction_flag = BACKWARD;
			//			else
			//				direction_flag = FORWARD;
			break;
		case '+' :
			sprintf(msg, "sync_period=%10lu\r\n", sync_period);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			break;
		case '/' :
			if ((boundary_outer - 0.1F) > boundary_inner)
				boundary_outer = boundary_outer - 0.1F;
			break;
		case '*' :
			sprintf(msg, "boundary_outer=%10.2f\r\n", boundary_outer);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			break;
		case '-' :
			boundary_outer = boundary_outer + 0.1F;
			break;
		case 'j' :
			sprintf(msg, "coefficient=%.2f\r\n", coefficient);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			break;
		case 'k' :
			sprintf(msg, "alpha_former=%.2f\r\n", alpha_former);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			break;
		case 'l' :
			sprintf(msg, "alpha_latter=%.2f\r\n", alpha_latter);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			break;
		case 'n' :
			if ((coefficient - 0.01F) > 0.0F)
				coefficient = coefficient - 0.01F;
			break;
		case 'm' :
			if ((coefficient + 0.01F) <= 1.0F)
				coefficient = coefficient + 0.01F;
			break;
		case 'u' :
			if ((alpha_former - 0.1F) > 0.0F)
				alpha_former = alpha_former - 0.1F;
			break;
		case 'i' :
			if ((alpha_former + 0.1F) <= 200.0F)
				alpha_former = alpha_former + 0.1F;
			break;
		case '[' :
			if ((beta - 0.01F) > 0.0F)
				beta = beta - 0.01F;
			break;
		case ']' :
			if ((beta + 0.01F) <= 1.0F)
				beta = beta + 0.01F;
			break;
		case 'o' :
			sprintf(msg, "angle=%9.6f\r\n", filtered_angle.x);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			break;
		case 't' :
			sprintf(msg, "dt=%.6f\r\n", dt);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			break;
		case 'r' :
			sprintf(msg, "status=%s\r\n", (status == HAL_OK) ? "HAL_OK" :
					(status == HAL_ERROR) ? "HAL_ERROR" :
							(status == HAL_BUSY) ? "HAL_BUSY" : "HAL_TIMEOUT");
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			break;
		case 'p' :
			print_flag = !(print_flag);
			break;
		case 'P' :
			//			sprintf(msg, "step_max=%10d\r\n", step_max);
			//			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "step_delay_vertical=%10lu\r\n", step_delay_vertical);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "step_delay_horizontal=%10lu\r\n", step_delay_horizontal);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "step_delay_high=%10lu\r\n", step_delay_high);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "sync_period=%10lu\r\n", sync_period);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "boundary_inner=%10.2f\r\n", boundary_inner);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "boundary_outer=%10.2f\r\n", boundary_outer);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "coefficient=%.2f\r\n", coefficient);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "alpha_former=%.2f\r\n", alpha_former);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "alpha_latter=%.2f\r\n", alpha_latter);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "beta=%.2f\r\n", beta);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "dt=%.6f\r\n", dt);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "ALPHA=%.2f\r\n", ALPHA);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "gyroOffset=%10d\t%10d\t%10d\t\r\n", gyroOffset.x, gyroOffset.y, gyroOffset.z);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "accOffset=%10d\t%10d\t%10d\t\r\n", accOffset.x, accOffset.y, accOffset.z);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			sprintf(msg, "status=%s\r\n", (status == HAL_OK) ? "HAL_OK" :
					(status == HAL_ERROR) ? "HAL_ERROR" :
							(status == HAL_BUSY) ? "HAL_BUSY" : "HAL_TIMEOUT");
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			break;
		case 'v' :
			sprintf(msg, "ALPHA=%.2f\r\n", ALPHA);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			break;
		case 'c' :
			if ((ALPHA - 0.01F) >= 0.00F)
				ALPHA = ALPHA - 0.01F;
			break;
		case 'b' :
			if ((ALPHA + 0.01F) <= 1.00F)
				ALPHA = ALPHA + 0.01F;
			break;
		case 'a' :
			if ((boundary_inner - 0.1F) > 0.0F)
				boundary_inner = boundary_inner - 0.1F;
			//			state_flag = RIGHT_MOTOR;
			break;
		case 'd' :
			if ((boundary_inner + 0.1F) < boundary_outer)
				boundary_inner = boundary_inner + 0.1F;
			//			state_flag = LEFT_MOTOR;
			break;
			//		case 'w' :
			//			state_flag = BOTH_MOTOR;
			//			if (direction_flag == BACKWARD)
			//			{
			//				direction_flag = FORWARD;
			//				step_delay_dynamic = 4U;
			//			}
			//			break;
		case 's' :
			sprintf(msg, "boundary_inner=%10.2f\r\n", boundary_inner);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			//			state_flag = BOTH_MOTOR;
			//			if (direction_flag == FORWARD)
			//			{
			//				direction_flag = BACKWARD;
			//				step_delay_dynamic = 4U;
			//			}
			break;
		case 'z' :
			sprintf(msg, "cos_val=%7.3f\r\n", cos_val);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 3000UL);
			break;
		}
		//		HAL_UART_Transmit(&huart2, &rx_data, 1, 10);
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
}
void HAL_Delay(uint32_t Delay)
{
	uint32_t tickstart = HAL_GetTick();
	uint32_t wait = Delay;
	/* Add a freq to guarantee minimum wait */
	if (wait < HAL_MAX_DELAY)
	{
		wait += (uint32_t)(uwTickFreq);
	}
	while((HAL_GetTick() - tickstart) < wait)
	{
	}
}

uint32_t MY_GetTick()
{
	return microTick;
}

void MY_Delay(uint32_t step_delay)
{
	uint32_t start_tick = MY_GetTick();

	while ((MY_GetTick() - start_tick) < step_delay)
	{
		osThreadYield();
	}
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM10) {
		microTick+=100UL;
	}
	else
		/* USER CODE END Callback 0 */
		if (htim->Instance == TIM1) {
			HAL_IncTick();
		}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
