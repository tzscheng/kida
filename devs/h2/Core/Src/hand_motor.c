#include "hand_motor.h"
#include "tim.h"
#include <math.h>

Motor_Control motors[14];

// 모터별 고유 설정 테이블 (ROM - 기존 값 유지)
const Motor_Config_Table motor_init_data[14] = {
    {0},
    { 1610, 50.0f,  0.0f, 5.0f, 4000, 150, 1, 1550, 2700, 90 }, // M1
    { 2090, 200.0f, 0.0f, 3.0f, 4000, 150, 0, 2090, 3140, 90 }, // M2
    { 2000, 200.0f, 0.0f, 3.0f, 4000, 150, 0, 2200, 2900, 50 }, // M3
    { 1450, 200.0f, 0.0f, 3.0f, 4000, 150, 0, 1600, 2830, 30 }, // M4
    { 1100, 200.0f, 0.0f, 3.0f, 4000, 150, 0, 1100, 1680, 25 }, // M5
    { 1950, 200.0f, 0.0f, 3.0f, 4000, 150, 0, 2100, 3320, 30 }, // M6
    { 1780, 200.0f, 0.0f, 3.0f, 4000, 150, 0, 1780, 2450, 30 }, // M7
    { 1650, 200.0f, 0.0f, 3.0f, 5000, 500, 0, 1550, 2800, 75 }, // M8
    { 1300, 100.0f, 0.0f, 0.0f, 4000, 150, 1,    0, 4095, 100}, // M9
    { 1300, 100.0f, 0.0f, 0.0f, 4000, 150, 0,    0, 4095, -50}, // M10
    { 1300, 100.0f, 0.0f, 0.0f, 4000, 150, 0,    0, 4095, 160}, // M11
    { 1300, 80.0f, 0.0f, 0.0f, 4000, 150, 1,    0, 4095, 180}, // M12
    {    0, 100.0f, 0.0f, 0.0f, 5655, 150, 0,   50, 4000, 180}  // M13
};

void Hand_Motor_Init(void) {
    for(int i = 1; i <= 13; i++) {
        motors[i].adc_offset = motor_init_data[i].offset;
        motors[i].kp         = motor_init_data[i].kp;
        motors[i].ki         = motor_init_data[i].ki;
        motors[i].kd         = motor_init_data[i].kd;
        motors[i].pwm_limit  = motor_init_data[i].pwm_limit;
        motors[i].dead_zone  = motor_init_data[i].dead_zone;
        motors[i].dir_invert = motor_init_data[i].inv;
        motors[i].adc_min    = motor_init_data[i].adc_min;
        motors[i].adc_max    = motor_init_data[i].adc_max;
        motors[i].init_angle = motor_init_data[i].init_angle;

        motors[i].control_mode = 1;
        motors[i].target_angle = motors[i].init_angle;
        motors[i].prev_raw_adc = motors[i].raw_adc;
        motors[i].turn_count   = 0;

        motors[i].error_sum = 0.0f;
        motors[i].error_prev = 0.0f;
    }
}

void Hand_Motor_PID_Compute(void) {
    for(int i = 1; i <= 13; i++) {

        // --- [STEP 0] 트래젝토리(램핑) 연산: 모드 2 전용 ---
        // target_angle(최종 목표)까지 일정한 속도로 current_target을 이동시킵니다.
        if (motors[i].control_mode == 2) {
            // step_size: 10ms마다 이동할 각도 (예: 0.5f는 초당 50도 이동)
            // 연구원님의 기계적 특성에 맞춰 이 값을 조절하여 속도를 제어하세요.
            float step_size = 0.8f;

            if (motors[i].current_target < motors[i].target_angle) {
                motors[i].current_target += step_size;
                if (motors[i].current_target > motors[i].target_angle)
                    motors[i].current_target = motors[i].target_angle;
            }
            else if (motors[i].current_target > motors[i].target_angle) {
                motors[i].current_target -= step_size;
                if (motors[i].current_target < motors[i].target_angle)
                    motors[i].current_target = motors[i].target_angle;
            }
            // 트래젝토리 모드일 때는 서서히 변하는 current_target을 기준으로 에러 계산
            motors[i].error = motors[i].current_target - motors[i].current_angle;
        }
        else {
            // 모드 2가 아닐 때는 즉시 target_angle로 에러를 계산합니다.
            motors[i].error = motors[i].target_angle - motors[i].current_angle;

            // 모드 전환 시 튀는 것을 방지하기 위해 current_target을 현재 각도로 동기화
            motors[i].current_target = motors[i].current_angle;
        }

        // --- [STEP 1] 모드 3 & 모드 4: 상수 힘 제어 (M9~M12 전용) ---
                // 모드 4: 초기 파지용 고출력 (PWM 4000) -> 모터 타는 것을 방지하기 위해 짧게만 사용 권장
                // 모드 3: 파지 유지용 저출력 (PWM 2500) -> 모터 발열 감소 및 보호용
		if ((motors[i].control_mode == 3 || motors[i].control_mode == 4) && i >= 9 && i <= 12) {
			uint16_t target_pwm = (motors[i].control_mode == 4) ? 4000 : 2500;

			switch(i) {
				case 9:  motors[i].final_dir = 1; motors[i].final_pwm = target_pwm; break;
				case 10: motors[i].final_dir = 0; motors[i].final_pwm = target_pwm; break;
				case 11: motors[i].final_dir = 1; motors[i].final_pwm = target_pwm; break;
				case 12: motors[i].final_dir = 0; motors[i].final_pwm = target_pwm; break;
			}
			// 하드웨어 방향 반전 반영
			if (motors[i].dir_invert) motors[i].final_dir = !motors[i].final_dir;

			// PID 연산을 건너뛰고 다음 모터로 이동
			continue;
		}

        // --- [STEP 2] 안전장치 (단일 회전 모터 M1~M8, M13) ---
        // 물리적 한계 범위를 벗어나면 즉시 정지시킵니다.
        if (i <= 8 || i == 13) {
            if (motors[i].raw_adc <= motors[i].adc_min || motors[i].raw_adc >= motors[i].adc_max) {
                motors[i].final_pwm = 0;
                continue;
            }
        }

        // --- [STEP 3] PID 연산 ---
        // P(비례) 항
        float p_term = motors[i].kp * motors[i].error;

        // I(적분) 항: 누적 오차 계산 및 안티 윈드업(Anti-windup) 적용
        motors[i].error_sum += motors[i].error;
        if (motors[i].error_sum > 1000.0f)  motors[i].error_sum = 1000.0f;
        if (motors[i].error_sum < -1000.0f) motors[i].error_sum = -1000.0f;
        float i_term = motors[i].ki * motors[i].error_sum;

        // D(미분) 항: 오차의 변화율 계산
        float d_term = motors[i].kd * (motors[i].error - motors[i].error_prev);
        motors[i].error_prev = motors[i].error;

        // 최종 제어값 합산
        float control_val = p_term + i_term + d_term;

        // --- [STEP 4] 출력 변환 및 오버플로우 방지 ---
        // 방향(Direction) 결정
        uint8_t dir = (control_val >= 0) ? 0 : 1;
        if(motors[i].dir_invert) dir = !dir;
        motors[i].final_dir = dir;

        // 출력(PWM) 계산 및 리밋 적용
        float abs_val = fabsf(control_val);

        // [중요] uint16_t 변환 전 float 단계에서 오버플로우 방지 (1313 문제 해결)
        if (abs_val > (float)motors[i].pwm_limit) abs_val = (float)motors[i].pwm_limit;

        uint16_t out_pwm = (uint16_t)abs_val;

        // 정지 마찰력을 뚫기 위한 데드존 보상
        if(out_pwm > 0) out_pwm += motors[i].dead_zone;

        // 최종 PWM 출력값 리밋 (데드존 합산 후 다시 체크)
        if(out_pwm > motors[i].pwm_limit) out_pwm = motors[i].pwm_limit;

        // 불감대 설정: 에러가 매우 작으면 미세 진동 방지를 위해 정지
        if(motors[i].error < 0.5f && motors[i].error > -0.5f) {
            out_pwm = 0;
            motors[i].error_sum = 0; // 정지 시 누적 오차 리셋 (선택 사항)
        }

        motors[i].final_pwm = out_pwm;
    }
}

/**
  * @brief  [NEW] 새로운 하드웨어 핀맵 및 타이머 출력
  */
void Hand_Motor_Drive(void) {
    // 1. 새로운 방향 제어 GPIO 매핑
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, motors[1].final_dir);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, motors[2].final_dir);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, motors[3].final_dir);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,  motors[4].final_dir);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  motors[5].final_dir);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,  motors[6].final_dir);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, motors[7].final_dir);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, motors[8].final_dir);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, motors[9].final_dir);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, motors[10].final_dir);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, motors[11].final_dir);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, motors[12].final_dir);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, motors[13].final_dir);

    // 2. 새로운 속도(PWM) 타이머 채널 매핑
    __HAL_TIM_SET_COMPARE(&htim3,  TIM_CHANNEL_1, motors[1].final_pwm);
    __HAL_TIM_SET_COMPARE(&htim3,  TIM_CHANNEL_2, motors[2].final_pwm);
    __HAL_TIM_SET_COMPARE(&htim3,  TIM_CHANNEL_3, motors[3].final_pwm);
    __HAL_TIM_SET_COMPARE(&htim3,  TIM_CHANNEL_4, motors[4].final_pwm);
    __HAL_TIM_SET_COMPARE(&htim4,  TIM_CHANNEL_1, motors[5].final_pwm);
    __HAL_TIM_SET_COMPARE(&htim4,  TIM_CHANNEL_2, motors[6].final_pwm);
    __HAL_TIM_SET_COMPARE(&htim4,  TIM_CHANNEL_3, motors[7].final_pwm);
    __HAL_TIM_SET_COMPARE(&htim4,  TIM_CHANNEL_4, motors[8].final_pwm);
    __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_1, motors[9].final_pwm);
    __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_2, motors[10].final_pwm);
    __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_3, motors[11].final_pwm);
    __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_4, motors[12].final_pwm);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, motors[13].final_pwm);
}
