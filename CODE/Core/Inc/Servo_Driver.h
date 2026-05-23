#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H
#include "stm32f4xx_hal.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim1;

#ifdef __cplusplus
extern "C" {
#endif

void Servo_cal_routine(void);
void Servo_init(void);
void Servo_pan(int US);

#ifdef __cplusplus
}
#endif
#endif
