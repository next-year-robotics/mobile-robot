/**
  ******************************************************************************
  * @file    encoder_accumulator.h
  * @brief   16-bit 타이머 카운터의 wrap-safe 델타를 int64로 누적한다.
  * @note    HAL/RTOS 비의존. 타이머를 읽는 일은 adapter가 하고 여기서는 값만 받는다.
  ******************************************************************************
  */
#ifndef ENCODER_ACCUMULATOR_H
#define ENCODER_ACCUMULATOR_H

#include <stdint.h>

typedef struct
{
  int64_t  ticks;  /* 부호 보정이 끝난 누적 틱 */
  uint16_t prev;   /* 직전 샘플의 원시 카운터 */
  int8_t   sign;   /* ENCODER_SIGN_* (+1 / -1) */
} encoder_accum_t;

/**
  * @brief  누적기를 초기화한다. 첫 델타가 0이 되도록 현재 카운터를 기준으로 잡는다.
  */
void encoder_accum_init(encoder_accum_t *acc, int8_t sign, uint16_t initial);

/**
  * @brief  새 카운터 값을 넣어 델타를 누적한다.
  * @note   16-bit 랩어라운드는 부호 없는 뺄셈 뒤 ±32768 접기로 처리한다.
  *         100 Hz에서 최대 틱레이트를 넣어도 델타는 ±32767 근처에 가지 않는다.
  */
void encoder_accum_update(encoder_accum_t *acc, uint16_t now);

/**
  * @brief  직전 update에서 반영된 부호 보정 델타.
  */
int32_t encoder_accum_delta(const encoder_accum_t *acc, uint16_t now);

#endif /* ENCODER_ACCUMULATOR_H */
