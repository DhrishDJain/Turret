#include "Servo_Driver.h"

void Servo_init() {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   // Tilt servo
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   // Pan servo
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500); // center
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1500); // center
	printf("[SERVO] ✅ PWM Started\r\n");
}
void Servo_cal_routine(void) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1900);
	HAL_Delay(300);                            // ← was delay() — wrong on STM32
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1100);
	HAL_Delay(300);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500);      // center tilt

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1900);
	HAL_Delay(200);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1100); // ← was 1900 again (bug!)
	HAL_Delay(200);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1500);      // center pan
}
