#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H
#include "stm32f4xx_hal.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim1;

/* ── Pulse width range (µs) — placeholders, verify against your servo's datasheet ── */
#define SERVO_MIN_PULSE_US   500
#define SERVO_MID_PULSE_US   1500
#define SERVO_MAX_PULSE_US   2500

/* ── Tilt angle limits (deg) — placeholders pending physical calibration ── */
#define TILT_ANGLE_MIN_DEG   0
#define TILT_ANGLE_MAX_DEG   180

/* ── Pan angle limits (deg) — now positional, was continuous-rotation ── */
#define PAN_ANGLE_MIN_DEG     0
#define PAN_ANGLE_MAX_DEG     180
#define PAN_ANGLE_CENTER_DEG  90

#ifdef __cplusplus
extern "C" {
#endif

void Servo_cal_routine(void);
void Servo_init(void);

void Servo_tilt_setPulse(uint16_t pulse_us);
void Servo_tilt_setAngle(uint8_t angle_deg);

void Servo_pan_setAngle(uint8_t angle_deg);
void Servo_pan_center(void);

void Fire_setDuty(uint8_t percent);
void Fire_stop(void);

#ifdef __cplusplus
}
#endif
#endif
