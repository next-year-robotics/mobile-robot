/**
  ******************************************************************************
  * @file    platform_adc.h
  * @brief   ADC1 배터리 전압(PA4 = IN4)과 VREFINT(IN17) 변환. HAL 타입은 나오지 않는다.
  * @note    HealthTask가 ADC1의 유일한 소유자다. 채널은 변환마다 코드가 바꾼다.
  *          CubeMX에는 IN4 한 채널, 소프트웨어 트리거, 단일 변환만 둔다.
  ******************************************************************************
  */
#ifndef PLATFORM_ADC_H
#define PLATFORM_ADC_H

#include <stdbool.h>
#include <stdint.h>

/**
  * @brief  VREFINT를 켜고 안정화용 변환을 한 번 버린다. 소유 task 첫머리에서 한 번 부른다.
  * @retval 변환 실패면 false. 실패해도 read가 다시 시도하므로 치명은 아니다.
  */
bool platform_adc_start(void);

/**
  * @brief  VREFINT → PA4 순서로 한 번씩 변환한다. 블로킹, 합계 약 50 µs.
  * @retval 둘 중 하나라도 실패하면 false. 출력값은 의미 없다.
  */
bool platform_adc_read_battery(uint16_t *raw_battery, uint16_t *raw_vrefint);

/** @brief OTP의 VREFINT_CAL (VDDA = 3.3 V, 30 °C에서 잰 12-bit 값). */
uint16_t platform_adc_vrefint_cal(void);

#endif /* PLATFORM_ADC_H */
