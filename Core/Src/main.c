/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>

#include "bno08x.h"
#include "bno08x_i2c_f446.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define COMPARE_DIV (512 / 498)
#define COMPRE_MAX (498)

#define WHEEL_R (1)
#define FR (45)
#define FL (-45)
#define RR (135)
#define RL (-135)

#define MOTOR_OUTPUT_PERIOD 5
#define ADC_SMPL_PERIOD 100

#define Kp 1200
#define Ti 10000
#define Td 0.05

#define LINE_SENSOR_NUM 16
#define LINE_SENSOR_OFFSET 0
#define LINE_SENSOR_VEC_MUL 1000
#define LINE_SENSOR_ONE_ANGLE (M_PI * 2 / LINE_SENSOR_NUM)

// all rad
float imu_x = 0;
float imu_y = 0;
float imu_roll = 0; // roll

float target_angle = 0;
float motor_angle_u = 0;

typedef struct {
	uint32_t x;
	uint32_t y;
} int32_vector2_t;

typedef struct {
	int32_t x;
	int32_t y;
} uint32_vector2_t;

typedef struct {
	float x;
	float y;
} float_vector2_t;

uint32_vector2_t line_vec[LINE_SENSOR_NUM] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int max_int(int val, int min, int max) {
	if (val > max) {
		return max;
	} else if (val < min) {
		return min;
	} else {
		return val;
	}
}

void set_fr(int p) {
	int ch_a = 0;
	int ch_b = 0;
	if (p > 0) {
		ch_a = (int) ((float) p / COMPARE_DIV);
		ch_b = 0;
	} else if (p < 0) {
		ch_a = 0;
		ch_b = (int) (-((float) p) / COMPARE_DIV);
	} else {
		ch_a = 0;
		ch_b = 0;
	}

	ch_a = max_int(ch_a, 0, COMPRE_MAX);
	ch_b = max_int(ch_b, 0, COMPRE_MAX);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ch_a);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ch_b);
}

// Motor

void set_fl(int p) {
	int ch_a = 0;
	int ch_b = 0;
	if (p > 0) {
		ch_a = (int) ((float) p / COMPARE_DIV);
		ch_b = 0;
	} else if (p < 0) {
		ch_a = 0;
		ch_b = (int) (-((float) p) / COMPARE_DIV);
	} else {
		ch_a = 0;
		ch_b = 0;
	}

	ch_a = max_int(ch_a, 0, COMPRE_MAX);
	ch_b = max_int(ch_b, 0, COMPRE_MAX);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ch_a);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ch_b);
}

void set_rr(int p) {
	int ch_a = 0;
	int ch_b = 0;
	if (p > 0) {
		ch_a = (int) ((float) p / COMPARE_DIV);
		ch_b = 0;
	} else if (p < 0) {
		ch_a = 0;
		ch_b = (int) (-((float) p) / COMPARE_DIV);
	} else {
		ch_a = 0;
		ch_b = 0;
	}

	ch_a = max_int(ch_a, 0, COMPRE_MAX);
	ch_b = max_int(ch_b, 0, COMPRE_MAX);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ch_b); // Reversed
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ch_a); // Reversed
}

void set_rl(int p) {
	int ch_a = 0;
	int ch_b = 0;
	if (p > 0) {
		ch_a = (int) ((float) p / COMPARE_DIV);
		ch_b = 0;
	} else if (p < 0) {
		ch_a = 0;
		ch_b = (int) (-((float) p) / COMPARE_DIV);
	} else {
		ch_a = 0;
		ch_b = 0;
	}

	ch_a = max_int(ch_a, 0, COMPRE_MAX);
	ch_b = max_int(ch_b, 0, COMPRE_MAX);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ch_a);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ch_b);
}

// Thank you Shirokuma
float calcMotorSpeed(float angle, float speed, float motorAngle) {
	return sin((angle - motorAngle) * DEG2RAD) * speed;
}

float calcWorldMotorSpeed(float motorAngle, float wheel_r, float angle,
		float speed_x, float speed_y, float speed_angle) {
	return (-sin(angle + motorAngle) * cos(angle) * speed_x
			+ cos(angle + motorAngle) * cos(angle) * speed_y + speed_angle)
			/ wheel_r;
}

// -512 ~ 512
void move(float angle, float speed) {
	set_fr((int) calcMotorSpeed(angle, speed, 45));
	set_fl((int) calcMotorSpeed(angle, speed, -45));
	set_rr((int) calcMotorSpeed(angle, speed, 135));
	set_rl((int) calcMotorSpeed(angle, speed, -135));
}

void moveWorld(float x, float y, float world_angle, float theta) {
	set_fr(calcWorldMotorSpeed(FR, WHEEL_R, world_angle, x, y, theta));
	set_fl(calcWorldMotorSpeed(FL, WHEEL_R, world_angle, x, y, theta));
	set_rr(calcWorldMotorSpeed(RR, WHEEL_R, world_angle, x, y, theta));
	set_rl(calcWorldMotorSpeed(RL, WHEEL_R, world_angle, x, y, theta));
}

void rotate(float theta) {
	set_fr(theta);
	set_fl(theta);
	set_rr(theta);
	set_rl(theta);
}

int read_line_sensor(int id) {
	HAL_GPIO_WritePin(Line_A_GPIO_Port, Line_A_Pin, id & 0b0001);
	HAL_GPIO_WritePin(Line_B_GPIO_Port, Line_B_Pin, (id & 0b0010) >> 1);
	HAL_GPIO_WritePin(Line_C_GPIO_Port, Line_C_Pin, (id & 0b0100) >> 2);

	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

	sConfig.Channel = (id & 0b1000) >> 3 ? ADC_CHANNEL_10 : ADC_CHANNEL_11;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	int val = -1;

	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
		val = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);

	return val;
}

void init_line_sensor_vec() {
	for (int i = 0; i < LINE_SENSOR_NUM; i++) {
		line_vec[i].x = cos(
		LINE_SENSOR_OFFSET + LINE_SENSOR_ONE_ANGLE * i) * LINE_SENSOR_VEC_MUL;
		line_vec[i].y = sin(
		LINE_SENSOR_OFFSET + LINE_SENSOR_ONE_ANGLE * i) * LINE_SENSOR_VEC_MUL;
	}
}

void line_sensor_angle(float_vector2_t *v, int *max) {
	uint32_vector2_t sum = { x: 0, y: 0 };
	v->x = 0;
	v->y = 0;
	*max = 0;

	for (int i = 0; i < LINE_SENSOR_NUM; i++) {
		int val = read_line_sensor(i);
		if (val > *max) {
			*max = val;
		}
		sum.x += line_vec[i].x * val;
		sum.y += line_vec[i].y * val;
	}

	printf("%ld, %ld\n", sum.x, sum.y);

	float_vector2_t f_sum = { x: (sum.x / LINE_SENSOR_VEC_MUL), y: (sum.y
			/ LINE_SENSOR_VEC_MUL) };

	float div = sqrt(f_sum.x * f_sum.x + f_sum.y * f_sum.y);

	v->x = f_sum.x / div;
	v->y = f_sum.y / div;

//	atan2(sum.x, sum.y);
}

//float PID(float c, float pos) {
//	static float e = 0, ei = 0, ed = 0, e_pre = 0;
//
//	e = c - pos;
//	ei += e * (PID_SMPL_PERIOD * 1e-3);
//	ed = (e - e_pre) / (PID_SMPL_PERIOD * 1e-3);
//	e_pre = e;
//
//	return Kp * (e + 1 / Ti * ei + Td * ed);
//}

//void testmove(int angle, int speed) {
//	for (int i = 0; i < speed; i += 2) {
//		move(angle, i);
//		HAL_Delay(1);
//	}
//	HAL_Delay(100);
//	for (int i = speed; i > 0; i -= 2) {
//		move(angle, i);
//		HAL_Delay(1);
//	}
//}

void quaternion_to_euler_angle(float x, float y, float z, float w,
		float *eular_x, float *eular_y, float *eular_z) {
	float ysqr = y * y;

	float t0 = 2.0 * (w * x + y * z);
	float t1 = 1.0 - 2.0 * (x * x + ysqr);
	float X = atan2(t0, t1);

	float t2 = 2.0 * (w * y - z * x);
	t2 = (t2 > 1.0) ? 1.0 : t2;
	t2 = (t2 < -1.0) ? -1.0 : t2;
	float Y = asin(t2);

	float t3 = 2.0 * (w * z + x * y);
	float t4 = 1.0 - 2.0 * (ysqr + z * z);

	float Z = atan2(t3, t4);

	*eular_x = X;
	*eular_y = Y;
	*eular_z = Z;

	return;
}

void bno08x_callback(bno08x_data_t *data) {
//	printf("%f, %f, %f, %f\n", data->gameRotationVector.i,
//			data->gameRotationVector.j, data->gameRotationVector.k,
//			data->gameRotationVector.real);
	quaternion_to_euler_angle(data->gameRotationVector.i,
			data->gameRotationVector.j, data->gameRotationVector.k,
			data->gameRotationVector.real, &imu_x, &imu_y, &imu_roll);
//	printf("%f, %f, %f\n", imu_x, imu_y, imu_roll);

	static int t_pre = 0;
	int t = HAL_GetTick();

	int t_w = t - t_pre;

	static float e = 0, ei = 0, ed = 0, e_pre = 0;

	e = target_angle - imu_roll;
	ei += e * (t_w * 1e-3);
	ed = (e - e_pre) / (t_w * 1e-3);
	e_pre = e;

	motor_angle_u = Kp * (e + 1 / Ti * ei + Td * ed);

	t_pre = t;
}
//void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *pI2c) {
//	printf("rx\n");
//}
//
//void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *pI2c) {
//	printf("tx\n");
//}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	setbuf(stdout, NULL);
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
//	nvic_init();
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_I2C1_Init();
	MX_TIM5_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	printf("start\n");
	i2c_hal_t *hal = bno08x_init_i2c();
	if (hal == NULL) {
		printf("error while init!\n");
	}
	printf("main: bno08x_init_i2c\n");
	bno08x_open(hal, bno08x_callback);
	printf("main: opened\n");

// M0
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

// M1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

// M3
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

// M4
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	init_line_sensor_vec();

	printf("i'm entering loop\n");

	float_vector2_t v = { x:0, y:0 };
	int line_max = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
//		for (int i = 90; i < 450; i++) {
//			move(i, 512);
//			HAL_Delay(3);
//		}
		bno08x_update();

		uint32_t tick = HAL_GetTick();
		if (tick % MOTOR_OUTPUT_PERIOD == 0) {
//			rotate(-motor_u);
		}

		if (tick % ADC_SMPL_PERIOD == 0) {
//			for (int i = 0; i < 16; i++) {
//				int val = read_line_sensor(i);
//				printf("%d,", val);
//			}
//			printf("\n");

			line_sensor_angle(&v, &line_max);
			printf("%f, %f, %d\n", v.x, v.y, line_max);
		}

		if (line_max < 1000)
		{
			v.x = 0;
			v.y = 0;
		}

		moveWorld(v.x * 1000, v.y * 1000, 0, -motor_angle_u);
//		printf("%f\n", motor_angle_u);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */
	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */
	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */
	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
	HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 449;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 449;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_OC_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | Line_A_Pin | Line_B_Pin | Line_C_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, BNO_BOOT_Pin | BNO_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin Line_A_Pin Line_B_Pin Line_C_Pin */
	GPIO_InitStruct.Pin = LD2_Pin | Line_A_Pin | Line_B_Pin | Line_C_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : BNO_BOOT_Pin BNO_RST_Pin */
	GPIO_InitStruct.Pin = BNO_BOOT_Pin | BNO_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 10);
	return len;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
		printf("Error_Handler\n");
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
