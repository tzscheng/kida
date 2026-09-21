#include "hand_sensor.h"
#include "hand_config.h"

static uint8_t is_first_run = 1; // 초기 Phantom Turn 방지용 플래그

void Hand_Sensor_Update(void) {
    // [STEP 1] 새로운 DMA 버퍼 데이터 매핑
    for(int i = 1; i <= 8; i++) {
        motors[i].raw_adc = adc1_buf[i - 1];
    }
    for(int i = 9; i <= 13; i++) {
        motors[i].raw_adc = adc2_buf[i - 9];
    }

    // [NEW] 전류 센서 데이터 수집
    for(int i = 9; i <= 13; i++) {
        motors[i].raw_current_adc = adc3_buf[i - 9];
    }

    // [STEP 2] 첫 실행 시 이전 ADC 값을 현재 값으로 동기화
    if (is_first_run) {
        for (int i = 1; i <= 13; i++) motors[i].prev_raw_adc = motors[i].raw_adc;
        is_first_run = 0;
    }

    // [STEP 3] 13개 모터 각도 계산 (기존 로직 100% 유지)
    for (int i = 1; i <= 13; i++) {
        if (i >= 9 && i <= 12) {
            // --- 다회전 처리 (M9~M12) ---
            int32_t diff = (int32_t)motors[i].raw_adc - (int32_t)motors[i].prev_raw_adc;
            if (diff < -2048)      motors[i].turn_count++;
            else if (diff > 2048)  motors[i].turn_count--;

            motors[i].prev_raw_adc = motors[i].raw_adc;

            float single_deg = (float)((int32_t)motors[i].raw_adc - motors[i].adc_offset) * (360.0f / 4096.0f);
            motors[i].current_angle = (float)(motors[i].turn_count * 360) + single_deg;
        }
        else if (i == 13) {
            // --- [특수] M13 전용 계산 (180도 / 3600 pulse) ---
            int32_t diff = (int32_t)motors[i].raw_adc - motors[i].adc_offset;
            motors[i].current_angle = (float)diff * (180.0f / 3600.0f);
        }
        else {
            // --- 단일 회전 처리 (M1~M8) ---
            int32_t diff = (int32_t)motors[i].raw_adc - motors[i].adc_offset;
            motors[i].current_angle = (float)diff * (360.0f / 4096.0f);
        }
    }
}
