/**
  ******************************************************************************
  * @file    motor_output.h
  * @brief   duty 지령(per-mille) -> TIM1 4채널 CCR 값. 순수 계산.
  * @note    HAL/RTOS 비의존. 레지스터를 만지지 않고 값만 만든다. 레지스터 기록은
  *          platform_stm32의 얇은 adapter가 전담한다.
  ******************************************************************************
  */
#ifndef MOTOR_OUTPUT_H
#define MOTOR_OUTPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"

/* CCR 인덱스. PA8=CH1=M1A, PA9=CH2=M1B, PA10=CH3=M2A, PA11=CH4=M2B. */
#define MOTOR_CCR_LEFT_A   0U
#define MOTOR_CCR_LEFT_B   1U
#define MOTOR_CCR_RIGHT_A  2U
#define MOTOR_CCR_RIGHT_B  3U
#define MOTOR_CCR_COUNT    4U

typedef struct
{
  uint32_t ccr[MOTOR_CCR_COUNT];
} motor_ccr_t;

/**
  * @brief  duty를 ±MOTOR_DUTY_MAX_PM으로 자른다.
  */
int32_t motor_clamp_pm(int32_t duty_pm);

/**
  * @brief  클램프된 duty의 크기를 CCR 펄스 값으로 환산한다.
  * @note   |duty| x (ARR+1) / 1000. duty=1000이면 CCR=4667 > ARR이라 100 % 온이다.
  */
uint32_t motor_pulse_from_pm(int32_t duty_pm);

/**
  * @brief  좌우 duty 지령에서 네 CCR을 만든다.
  * @note   clamp -> MOTOR_SIGN_* -> sign-magnitude 순서다. 한 채널의 두 값 중
  *         적어도 하나는 항상 0이므로 "두 입력이 동시에 0이 아닌" 출력은 이
  *         함수로 만들어질 수 없다.
  */
motor_ccr_t motor_ccr_from_duty(int32_t left_pm, int32_t right_pm);

/**
  * @brief  한 채널의 두 CCR이 동시에 0이 아닌지 검사한다(불변식 확인용).
  */
bool motor_ccr_is_exclusive(const motor_ccr_t *out);

#endif /* MOTOR_OUTPUT_H */
