#ifndef __HAND_SENSOR_H
#define __HAND_SENSOR_H

#include "main.h"

// [NEW] 새로운 STM32G4 환경에 맞춘 DMA 버퍼
extern volatile uint16_t adc1_buf[8]; // M1 ~ M8 각도
extern volatile uint16_t adc2_buf[5]; // M9 ~ M13 각도
extern volatile uint16_t adc3_buf[5]; // M9 ~ M13 전류

void Hand_Sensor_Update(void);

#endif
