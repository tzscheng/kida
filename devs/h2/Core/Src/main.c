#include "main.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hand_config.h"
#include "hand_sensor.h"
#include "hand_motor.h"  // <-- 이 헤더파일이 정상적으로 들어있어야 합니다!
/* USER CODE END Includes */

// [NEW] 새로운 DMA 버퍼 크기 매핑
volatile uint16_t adc1_buf[8], adc2_buf[5], adc3_buf[5];

uint8_t system_state = 1;
uint8_t grip_step = 0;
uint32_t step_timer = 0;
static uint8_t state4_step = 0;
static uint32_t state4_timer = 0;
static uint8_t state5_step = 0;

uint8_t bit_2 = 0;

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();

  // [NEW] 1. ADC DMA 시작 (크기 변경)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, 8);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, 5);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_buf, 5);

  HAL_Delay(50);

  // [NEW] 2. 모터 드라이버 Sleep 해제 (Wake up)
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);

  // 3. 센서 업데이트 및 모터 구조체 초기화
  Hand_Sensor_Update();
  Hand_Motor_Init();

  // [NEW] 4. 새로운 타이머 채널 PWM 시작
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);

  while (1) {
      Hand_Sensor_Update();

      // =========================================================
      // 여기에 기존에 작성하신 if (system_state == 0) ~ else if (system_state == 5)
      // 시퀀스 코드를 100% 동일하게 그대로 복사해서 넣으시면 됩니다.
      // (기존 모션 로직 완벽 호환됨)
      // =========================================================

      // --- [시스템 상태 제어] ---
      if (system_state == 0) {
          // 자유 모드: Live Watch 조작 대기
          grip_step = 0;
      }
      else if (system_state == 1) {
          // 홈 모드: 초기 위치 고정
          for (int i = 1; i <= 13; i++) {
              motors[i].target_angle = motors[i].init_angle;
              motors[i].control_mode = 1;
          }
          grip_step = 0;
      }

      else if (system_state == 4) {
                // =========================================================
                // [REVISED] 손 벌려 잡은 후, 2초 뒤 한 번 더 쥐어짜는 시나리오
                // =========================================================

                // [Step 0] 초기화 및 제어 모드 설정
    	  if (state4_step == 0) {
    	                // 스텝 1, 3에서 움직일 관절들을 궤적 모드(2)로 세팅하여 부드럽게 기동
    	                int target_joints[] = {1, 2, 3, 4, 5, 6, 7, 8};
    	                for (int j = 0; j < 8; j++) {
    	                    int m_idx = target_joints[j];   // [수정] 빠졌던 변수 선언 추가
    	                    motors[m_idx].control_mode = 2; // 부드러운 궤적 제어
    	                }
    	                for (int j = 9; j <= 12; j++) {
    	                    motors[j].control_mode = 1; // 와이어는 일반 위치 제어 시작
    	                }

    	                state4_timer = HAL_GetTick();
    	                state4_step = 1;
    	            }

                // [Step 1] 손가락 쫙 벌려 물체 감싸기
                else if (state4_step == 1) {
                    motors[1].target_angle = 10.0f;  // 90 -> 10
                    motors[3].target_angle = 20.0f;  // 50 -> 20
                    motors[5].target_angle = 25.0f;  // 25 유지
                    motors[7].target_angle = 50.0f;  // 30 -> 50

                    if (fabsf(motors[1].error) < 4.0f && fabsf(motors[3].error) < 4.0f &&
                        fabsf(motors[7].error) < 4.0f)
                    {
                        state4_timer = HAL_GetTick();
                        state4_step = 2; // 스텝 2로 이동
                    }
                }

                // [Step 2] 와이어 모터 상수 힘 제어(모드 3) 전환 후 2초 대기
    	  // [Step 2] 와이어 모터 상수 힘 제어(모드 3) 전환 (M10은 1초 늦게 시작) 후 2초 대기
    	            else if (state4_step == 2) {
    	                // 1. 9, 11, 12번 와이어는 진입 즉시 모드 3으로 전환
    	                motors[9].control_mode = 3;
    	                motors[11].control_mode = 3;
    	                motors[12].control_mode = 3;

    	                // 2. 10번 와이어는 스텝 2 시작 후 1초(1000ms)가 지난 후에 모드 3으로 전환
    	                if (HAL_GetTick() - state4_timer >= 500) {
    	                    motors[10].control_mode = 3;
    	                }
    	                else {
    	                    // 1초가 되기 전까지는 안전하게 기존 위치 제어 모드(1)를 유지
    	                    motors[10].control_mode = 1;
    	                }

    	                // 3. 스텝 2 전체 대기 시간은 여전히 2초(2000ms) 유지 후 스텝 3으로 이동
    	                if (HAL_GetTick() - state4_timer > 1000) {
    	                    state4_timer = HAL_GetTick();
    	                    state4_step = 3; // 스텝 3으로 이동
    	                }
    	            }

                // [Step 3] 1차 움켜쥐기
                else if (state4_step == 3) {
                    motors[2].target_angle = 70.0f;  // 90 -> 70
                    motors[4].target_angle = 70.0f;  // 30 -> 70
                    motors[6].target_angle = 80.0f;  // 30 -> 80
                    motors[8].target_angle = 40.0f;  // 85 -> 40

                    // 1차 움켜쥐기 관절들이 모두 목적지에 도달했는지 확인
                    if (fabsf(motors[2].error) < 4.0f && fabsf(motors[4].error) < 4.0f &&
                        fabsf(motors[6].error) < 4.0f && fabsf(motors[8].error) < 4.0f)
                    {
                        state4_timer = HAL_GetTick(); // 도달한 순간부터 스톱워치 시작
                        state4_step = 4;              // [NEW] 스텝 4(2초 대기 구간)로 이동
                    }
                }

                // [Step 4] 1차 움켜쥐기 완료 후 2초간 대기 플래그
                else if (state4_step == 4) {
                    // 스텝 3 도달 후 딱 2초(2000ms)가 지나면 다음 스텝 실행
                    if (HAL_GetTick() - state4_timer > 2000) {
                        state4_timer = HAL_GetTick();
                        state4_step = 5; // [NEW] 스텝 5(최종 쥐어짜기)로 이동
                    }
                }

                // [Step 5] 2차 최종 쥐어짜기 (더 강력한 압박)
    	  // [Step 5] 2차 최종 쥐어짜기 (더 강력한 압박)
    	            else if (state4_step == 5) {
    	                // 물체가 손에 걸려서 에러가 크게 남더라도, 일단 모터는 계속 힘을 주며 압박합니다.
    	                motors[2].target_angle = 30.0f;
    	                motors[4].target_angle = 90.0f;
    	                motors[6].target_angle = 90.0f;
    	                motors[8].target_angle = 30.0f;

    	                // ---------------------------------------------------------
    	                // [수정 포인트] 도달 조건문 바깥으로 비트 감지 로직을 꺼냈습니다.
    	                // 이제 손가락 오차(에러)와 무관하게, 스텝 5 상태라면
    	                // 라이브 워치에서 비트를 켜는 순간 '즉시' 반응합니다.
    	                // ---------------------------------------------------------
    	                if (bit_2 != 0) {
    	                    state4_timer = HAL_GetTick(); // 스텝 6용 타이머 시작
    	                    state4_step = 6;              // 즉시 스텝 6으로 점프!
    	                }
    	            }

    	            // [Step 6] M13 구조 재조정 (160도 -> 50도 이동)
    	            else if (state4_step == 6) {
    	                // M13 모터를 빠른 위치제어 모드(1)로 설정하고 목표각도 50도 부여
    	                motors[13].control_mode = 1;
    	                motors[13].target_angle = 50.0f;
    	                motors[9].control_mode = 4;
    	                motors[10].control_mode = 4;
    	                motors[11].control_mode = 4;
    	                motors[12].control_mode = 4;

    	                // 오차 계산
    	                float m13_error = fabsf(motors[13].target_angle - motors[13].current_angle);

    	                // 50도 근처에 도달했거나, 기계적 걸림을 대비해 2초(2000ms) 타임아웃 시 종료
    	                if (m13_error < 4.0f || (HAL_GetTick() - state4_timer > 2000)) {
    	                    // [선택] 완전히 동작을 끝내고 손을 풀려면 아래 주석을 해제하여 홈 모드로 보냅니다.
    	                    // system_state = 1;


    	                }
    	            }
            }


      // 안전장치: 상태 5가 아닐 때는 항상 단계를 0으로 초기화
      if (system_state != 5) state5_step = 0;

      Hand_Motor_PID_Compute();
      Hand_Motor_Drive();

      HAL_Delay(10); // 기존 제어 주기 유지
  }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_ADC345;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

