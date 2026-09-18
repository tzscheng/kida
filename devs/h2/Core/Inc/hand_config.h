#ifndef __HAND_CONFIG_H
#define __HAND_CONFIG_H

#include "main.h"

// 모터별 초기 설정값 구조체 (ROM 저장용)
typedef struct {
    int32_t offset;
    float kp, ki, kd;
    uint16_t pwm_limit;
    uint16_t dead_zone;
    uint8_t inv;
    uint16_t adc_min;
    uint16_t adc_max;
    float init_angle;
} Motor_Config_Table;

// 실시간 제어 및 상태 저장 구조체 (RAM 관리용)
typedef struct {
    uint8_t control_mode;   // 0: Lock/Open, 1: 위치제어, 2: 궤적제어, 3: 상수힘
    int16_t turn_count;     // 다회전 카운트
    uint16_t prev_raw_adc;  // 이전 사이클의 ADC 값

    int32_t adc_offset;
    float kp, ki, kd;
    uint16_t pwm_limit;
    uint16_t dead_zone;
    uint8_t dir_invert;
    uint16_t adc_min;
    uint16_t adc_max;
    float init_angle;

    uint16_t raw_adc;          // 각도 센서 RAW
    uint16_t raw_current_adc;  // [NEW] 전류 센서 RAW (M9~M13)

    float current_angle;
    float current_target;
    float target_angle;

    float error;
    float error_sum;
    float error_prev;

    uint16_t final_pwm;
    uint8_t final_dir;
} Motor_Control;

extern Motor_Control motors[14];
extern const Motor_Config_Table motor_init_data[14];

#endif
