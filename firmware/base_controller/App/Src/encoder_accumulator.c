#include "encoder_accumulator.h"

/**
  * @brief  now - prev를 16-bit에서 감은 뒤 부호 있는 값으로 편다.
  * @note   (int16_t) 캐스팅은 구현정의 동작이라 쓰지 않는다. 0x8000 이상이면
  *         65536을 빼서 음수 델타로 만든다 — 전 구간에서 정의된 계산이다.
  */
static int32_t fold_delta(uint16_t now, uint16_t prev)
{
  uint16_t raw = (uint16_t)(now - prev);

  return (raw < 0x8000U) ? (int32_t)raw : ((int32_t)raw - 65536);
}

void encoder_accum_init(encoder_accum_t *acc, int8_t sign, uint16_t initial)
{
  acc->ticks = 0;
  acc->prev = initial;
  acc->sign = sign;
}

int32_t encoder_accum_delta(const encoder_accum_t *acc, uint16_t now)
{
  return (int32_t)acc->sign * fold_delta(now, acc->prev);
}

void encoder_accum_update(encoder_accum_t *acc, uint16_t now)
{
  acc->ticks += (int64_t)encoder_accum_delta(acc, now);
  acc->prev = now;
}
