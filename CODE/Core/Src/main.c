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
#include <stdbool.h>
#include <stdlib.h>
#include "VL53L0X.h"
#include "Servo_Driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	STATE_SEARCHING = 0,
	STATE_TRACKING,
	STATE_RANGING,
	STATE_ENGAGING,
	STATE_COOLDOWN
} TurretState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEBUG 1

// ── Ranging / engagement (placeholders — need physical calibration) ──
#define ENGAGE_RANGE_MM           1500
#define TOF_POLL_INTERVAL_MS      100

// ── Search sweep (placeholders — need physical calibration) ──
#define SEARCH_SWEEP_SPEED        90
#define SEARCH_SWEEP_DURATION_MS  100

// ── Tracking (placeholders — not yet tuned, logic not yet implemented) ──
#define TRACK_DEADZONE_PX         20
#define TRACK_PAN_SPEED_MAX       60
#define CAM_LOST_TIMEOUT_MS       500

// ── Firing (placeholders) ──
#define FIRE_DUTY_PERCENT         100
#define ENGAGE_DURATION_MS        500
#define COOLDOWN_DURATION_MS      2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// ── VL53L0X ──
statInfo_t_VL53L0X tofData;
uint16_t distance_mm = 0;
uint32_t lastTofTick = 0;

// ── FSM ──
TurretState_t current_state = STATE_SEARCHING;
uint32_t state_enter_tick = 0;

// ── Search sweep (positional pan, was continuous-rotation) ──
#define SEARCH_PAN_STEP_DEG          2
#define SEARCH_PAN_STEP_INTERVAL_MS  5
// ── Search sweep ──
int8_t search_direction = 1;
uint32_t search_last_step_tick = 0;
int16_t current_pan_angle = PAN_ANGLE_CENTER_DEG;
// ── CAM lock data (stub until ESP32-CAM UART link is wired up) ──
volatile bool cam_lock = false;
volatile int16_t cam_x_offset = 0;
volatile int16_t cam_y_offset = 0;
uint32_t last_cam_update_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void FSM_Enter(TurretState_t new_state);
void TOF_Update(void);
bool Cam_GetLock(int16_t *x_offset, int16_t *y_offset);
void State_Searching(void);
void State_Tracking(void);
void State_Ranging(void);
void State_Engaging(void);
void State_Cooldown(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

	PUTCHAR_PROTOTYPE {
		HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
		return ch;
	}

#ifdef __cplusplus
}
#endif

#if DEBUG
#define DBG(...) printf(__VA_ARGS__)
#else
#define DBG(...)
#endif

	/* ── VL53L0X: non-blocking poll, runs regardless of state ── */
	void TOF_Update(void) {
		if ((HAL_GetTick() - lastTofTick) >= TOF_POLL_INTERVAL_MS) {
			lastTofTick = HAL_GetTick();
			distance_mm = readRangeSingleMillimeters(&tofData);
		}
	}

	/* ── ESP32-CAM lock read — STUB.
	 * TODO: replace with real UART parse of "<lock>,<x_offset>,<y_offset>\n"
	 * once the second UART peripheral is selected and configured in CubeMX. */
	bool Cam_GetLock(int16_t *x_offset, int16_t *y_offset) {
		(void) x_offset;
		(void) y_offset;
		return false;
	}

	/* ── FSM transition handler — runs entry actions once per state change ── */
	void FSM_Enter(TurretState_t new_state) {
		current_state = new_state;
		state_enter_tick = HAL_GetTick();

		switch (new_state) {
		case STATE_SEARCHING:
			DBG("[FSM] -> SEARCHING\r\n");
			search_direction = 1;
			search_last_step_tick = HAL_GetTick();
			break;

		case STATE_TRACKING:
			DBG("[FSM] -> TRACKING\r\n");
			last_cam_update_tick = HAL_GetTick();
			break;

		case STATE_RANGING:
			DBG("[FSM] -> RANGING\r\n");
			// Positional pan holds its angle on its own, nothing to stop
			break;
		case STATE_ENGAGING:
			DBG("[FSM] -> ENGAGING\r\n");
			Fire_setDuty(FIRE_DUTY_PERCENT);
			break;

		case STATE_COOLDOWN:
			DBG("[FSM] -> COOLDOWN\r\n");
			Fire_stop();
			break;
		}
	}

	/* ── SEARCHING: sweep pan back and forth, watch for a CAM lock ── */
	void State_Searching(void) {
		if ((HAL_GetTick() - search_last_step_tick)
				>= SEARCH_PAN_STEP_INTERVAL_MS) {
			search_last_step_tick = HAL_GetTick();

			current_pan_angle += SEARCH_PAN_STEP_DEG * search_direction;

			if (current_pan_angle >= PAN_ANGLE_MAX_DEG) {
				current_pan_angle = PAN_ANGLE_MAX_DEG;
				search_direction = -1;
			} else if (current_pan_angle <= PAN_ANGLE_MIN_DEG) {
				current_pan_angle = PAN_ANGLE_MIN_DEG;
				search_direction = 1;
			}

			Servo_pan_setAngle((uint8_t) current_pan_angle);
		}

		int16_t x_off, y_off;
		if (Cam_GetLock(&x_off, &y_off)) {
			cam_lock = true;
			cam_x_offset = x_off;
			cam_y_offset = y_off;
			FSM_Enter(STATE_TRACKING);
		}
	}
	/* ── TRACKING: TODO — pan/tilt correction toward target not yet implemented ── */
	void State_Tracking(void) {
		int16_t x_off, y_off;
		if (Cam_GetLock(&x_off, &y_off)) {
			cam_x_offset = x_off;
			cam_y_offset = y_off;
			last_cam_update_tick = HAL_GetTick();
		}

		if ((HAL_GetTick() - last_cam_update_tick) >= CAM_LOST_TIMEOUT_MS) {
			DBG("[TRACK] Lock lost\r\n");
			FSM_Enter(STATE_SEARCHING);
			return;
		}

		// TODO: proportional pan correction using cam_x_offset, TRACK_DEADZONE_PX, TRACK_PAN_SPEED_MAX
		// TODO: tilt Y-axis correction using cam_y_offset

		if (abs(cam_x_offset) < TRACK_DEADZONE_PX) {
			FSM_Enter(STATE_RANGING);
		}
	}

	/* ── RANGING: check distance against engage threshold ── */
	void State_Ranging(void) {
		if (distance_mm < ENGAGE_RANGE_MM) {
			FSM_Enter(STATE_ENGAGING);
		} else {
			FSM_Enter(STATE_TRACKING);
		}
	}

	/* ── ENGAGING: fire for a fixed duration, then cool down ── */
	void State_Engaging(void) {
		if ((HAL_GetTick() - state_enter_tick) >= ENGAGE_DURATION_MS) {
			FSM_Enter(STATE_COOLDOWN);
		}
	}

	/* ── COOLDOWN: pause before resuming search ── */
	void State_Cooldown(void) {
		if ((HAL_GetTick() - state_enter_tick) >= COOLDOWN_DURATION_MS) {
			FSM_Enter(STATE_SEARCHING);
		}
	}
	/* USER CODE END 0 */

	/**
	 *
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
		MX_USART3_Init();
		/* USER CODE BEGIN 2 */
		setvbuf(stdout, NULL, _IONBF, 0);
		printf("\033[2J\033[H");
		printf("=== Anti-Drone Turret Boot ===\r\n");

		// ── VL53L0X Init ──
//		printf("[TOF] Initializing VL53L0X...\r\n");
//		setTimeout(200); // ms — bail out of any driver-internal poll loop instead of spinning forever
//		if (initVL53L0X(1, &hi2c1) != 1) {
//			printf("[TOF] Init FAILED\r\n");
//		} else {
//			setMeasurementTimingBudget(33 * 1000UL);
//			printf("[TOF] Ready\r\n");
//		}

		// ── Servo PWM Start ──
		Servo_init();
//		// ── Servo PWM Start ──
//		Servo_init();

		// ── Startup self-test: sweep tilt through full range ──
		DBG("[INIT] Tilt self-test sweep\r\n");
		DBG("0 degree\r\n");
		Servo_pan_setAngle(PAN_ANGLE_MIN_DEG);
		HAL_Delay(3000);
		DBG("90 degree\r\n");
		Servo_pan_setAngle(90);
		HAL_Delay(3000);
		DBG("180 degree\r\n");
		Servo_pan_setAngle(PAN_ANGLE_MAX_DEG);
		HAL_Delay(10000);

		// ── FSM entry ──
//		FSM_Enter(STATE_SEARCHING);

//		printf("=== Running ===\r\n");
		/* USER CODE END 2 */

		/* Infinite loop */
		/* USER CODE BEGIN WHILE */
		while (1) {
			//TODO Add limit switch handling
			//TODO Add second UART + real Cam_GetLock() implementation
			//TODO Calibrate ENGAGE_RANGE_MM, SEARCH_SWEEP_DURATION_MS, TRACK_PAN_SPEED_MAX, TRACK_DEADZONE_PX

//			TOF_Update();
//			State_Searching();
//
//			switch (current_state) {
//			case STATE_SEARCHING:
//				break;
//			case STATE_TRACKING:
//				State_Tracking();
//				break;
//			case STATE_RANGING:
//				State_Ranging();
//				break;
//			case STATE_ENGAGING:
//				State_Engaging();
//				break;
//			case STATE_COOLDOWN:
//				State_Cooldown();
//				break;
//			}

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

		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)
				!= HAL_OK) {
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
