#include "Servo_Driver.h"

/* ── Internal state ── */
static uint8_t tilt_angle_min = TILT_ANGLE_MIN_DEG;
static uint8_t tilt_angle_max = TILT_ANGLE_MAX_DEG;

static uint8_t pan_angle_min = PAN_ANGLE_MIN_DEG;
static uint8_t pan_angle_max = PAN_ANGLE_MAX_DEG;

/* ── Init: start PWM on all three channels ── */
void Servo_init(void) {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   // Tilt
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   // Pan
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);   // Fire

	// Safe startup state
	Servo_tilt_setPulse(SERVO_MID_PULSE_US);
	Servo_pan_center();
	Fire_stop();
}

/* ── Tilt (positional, PA8 / CH1) ── */
void Servo_tilt_setPulse(uint16_t pulse_us) {
	if (pulse_us < SERVO_MIN_PULSE_US)
		pulse_us = SERVO_MIN_PULSE_US;
	if (pulse_us > SERVO_MAX_PULSE_US)
		pulse_us = SERVO_MAX_PULSE_US;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
}

void Servo_tilt_setAngle(uint8_t angle_deg) {
	if (angle_deg < tilt_angle_min)
		angle_deg = tilt_angle_min;
	if (angle_deg > tilt_angle_max)
		angle_deg = tilt_angle_max;

	uint32_t pulse = SERVO_MIN_PULSE_US
			+ ((uint32_t) (angle_deg)
					* (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180U;

	Servo_tilt_setPulse((uint16_t) pulse);
}

/* ── Pan (positional, PA9 / CH2) ── */
void Servo_pan_setAngle(uint8_t angle_deg) {
	if (angle_deg < pan_angle_min)
		angle_deg = pan_angle_min;
	if (angle_deg > pan_angle_max)
		angle_deg = pan_angle_max;

	uint32_t pulse = SERVO_MIN_PULSE_US
			+ ((uint32_t) (angle_deg)
					* (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180U;

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
}

void Servo_pan_center(void) {
	Servo_pan_setAngle(PAN_ANGLE_CENTER_DEG);
}

/* ── Fire (TB6612FNG PWM, direction hardwired, PA10 / CH3) ── */
void Fire_setDuty(uint8_t percent) {
	if (percent > 100)
		percent = 100;

	uint32_t pulse = ((uint32_t) percent * (htim1.Init.Period + 1)) / 100U;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
}

void Fire_stop(void) {
	Fire_setDuty(0);
}
