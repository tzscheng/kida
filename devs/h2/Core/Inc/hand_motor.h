#ifndef __HAND_MOTOR_H
#define __HAND_MOTOR_H

#include "hand_config.h"

// main.c의 타이머 핸들 참조
extern TIM_HandleTypeDef htim1, htim3, htim4, htim15;

void Hand_Motor_Init(void);
void Hand_Motor_PID_Compute(void);
void Hand_Motor_Drive(void);
#endif
