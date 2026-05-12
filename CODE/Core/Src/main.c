/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "VL53L0X.h"
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

// ── MPU6050 (existing) ──
MPU6050 mpu;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;

// ── VL53L0X (new) ──
// ── VL53L0X (new) ──
statInfo_t_VL53L0X tofData;       // ← correct type for Squieler lib
uint16_t distance_mm = 0;
uint32_t lastTofTick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define DEBUG 1

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_8) {

		mpuInterrupt = true;
	}
}

#if DEBUG
#define DBG(...) printf(__VA_ARGS__)
#else
  #define DBG(...)   // expands to nothing in production
#endif
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	setvbuf(stdout, NULL, _IONBF, 0);
	printf("\033[2J\033[H");
	printf("=== Anti-Drone Turret Boot ===\r\n");

	// ── MPU6050 Init ──
	printf("[MPU] Initializing...\r\n");
	mpu.initialize();
	if (!mpu.testConnection()) {
		printf("[MPU] ⚠️  NOT CONNECTED\r\n");
	} else {
		devStatus = mpu.dmpInitialize();
		if (devStatus == 0) {
			mpu.CalibrateAccel(6);
			mpu.CalibrateGyro(6);
			mpu.setDMPEnabled(true);
			mpu.resetFIFO();
			mpu.getIntStatus();
			packetSize = mpu.dmpGetFIFOPacketSize();
			printf("[MPU] ✅ DMP Ready\r\n");
		} else {
			printf("[MPU] ❌ DMP Failed (code %d)\r\n", devStatus);
		}
	}

	// ── VL53L0X Init ──
	printf("[TOF] Initializing VL53L0X...\r\n");
	if (initVL53L0X(1, &hi2c1) != 1) {
		printf("[TOF] ⚠️  Init FAILED\r\n");
	} else {
		// High accuracy mode
		setMeasurementTimingBudget(33 * 1000UL);
		printf("[TOF] ✅ Ready\r\n");
	}

	// ── Servo PWM Start ──
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   // Tilt servo
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   // Pan servo
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500); // center
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500); // center
	printf("[SERVO] ✅ PWM Started\r\n");

	printf("=== Running ===\r\n");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		// ── MPU6050: highest priority, runs every interrupt ──
		if (mpuInterrupt) {
			mpuInterrupt = false;

			// ── Step 1: Reset FIFO every time to prevent stale data ──
			mpu.resetFIFO();

			// ── Step 2: Wait for exactly ONE fresh packet ──
			uint16_t fifoCount = 0;
			uint32_t timeout = HAL_GetTick();
			while (fifoCount < packetSize) {
				fifoCount = mpu.getFIFOCount();
				if ((HAL_GetTick() - timeout) > 50) {
					// timeout — skip this cycle
					mpu.resetFIFO();
					break;
				}
			}

			// ── Step 3: Read exactly one packet ──
			if (fifoCount >= packetSize) {
				mpu.getFIFOBytes(fifoBuffer, packetSize);
				mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetGravity(&gravity, &q);
				mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
				DBG("[IMU] Yaw=%.2f Pitch=%.2f Roll=%.2f\r\n",
						ypr[0] * 180.0f / M_PI, ypr[1] * 180.0f / M_PI,
						ypr[2] * 180.0f / M_PI);
			}
		}
		// ── VL53L0X: only read every 100ms, non-blocking ──
		if ((HAL_GetTick() - lastTofTick) >= 100) {
			lastTofTick = HAL_GetTick();
			distance_mm = readRangeSingleMillimeters(&tofData);
			if (distance_mm < 8190) {
				DBG("[TOF] Distance: %d mm\r\n", distance_mm);
			}
		}
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

/* USER CODE BEGIN 4 */

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
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
