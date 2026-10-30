#include "battery_monitor.h"

#include "app_config.h"

#include <math.h>

/* 12-bit 정렬 오른쪽. 최댓값은 VDDA 이상을 뜻한다. */
#define ADC_FULL_SCALE  4095U

void battery_monitor_init(battery_monitor_t *m)
{
  m->volts = 0.0f;
  m->last_volts = 0.0f;
  m->vdda_mv = 0U;
  m->valid = false;
  m->sample_count = 0U;
  m->reject_count = 0U;
  m->reject_streak = 0U;
  m->last_reject = (uint32_t)BAT_REJECT_NONE;
}

battery_reject_t battery_convert(uint16_t raw_battery, uint16_t raw_vrefint,
                                 uint16_t vrefint_cal, float *out_volts,
                                 uint32_t *out_vdda_mv)
{
  uint32_t vdda_mv;
  float pin_volts;

  if (raw_vrefint == 0U)
  {
    return BAT_REJECT_VDDA;
  }

  /* VDDA = 3.3 V × VREFINT_CAL / VREFINT_DATA. 정수로 계산한다. 3300 × 4095는
     uint32 안에 넉넉히 든다. */
  vdda_mv = ((uint32_t)BAT_VREFINT_CAL_MV * (uint32_t)vrefint_cal) /
            (uint32_t)raw_vrefint;
  if ((vdda_mv < BAT_VDDA_MIN_MV) || (vdda_mv > BAT_VDDA_MAX_MV))
  {
    return BAT_REJECT_VDDA;
  }

  /* 포화 값은 "VDDA 이상"이라는 뜻뿐이다. 그대로 환산하면 실제보다 낮은 값을 확신에
     차서 내보내게 된다. 지금 배터리로는 닿을 수 없는 값(VB ≈ 19.9 V)이라 여기 오면
     분압기가 끊긴 것이다. */
  if (raw_battery >= ADC_FULL_SCALE)
  {
    return BAT_REJECT_SATURATED;
  }

  pin_volts = ((float)vdda_mv * 0.001f) * (float)raw_battery / (float)ADC_FULL_SCALE;
  *out_volts = pin_volts * BAT_DIVIDER_GAIN;
  *out_vdda_mv = vdda_mv;
  return BAT_REJECT_NONE;
}

void battery_monitor_update(battery_monitor_t *m, bool adc_ok,
                            uint16_t raw_battery, uint16_t raw_vrefint,
                            uint16_t vrefint_cal)
{
  battery_reject_t reject = BAT_REJECT_ADC;
  float volts = 0.0f;
  uint32_t vdda_mv = 0U;

  if (adc_ok)
  {
    reject = battery_convert(raw_battery, raw_vrefint, vrefint_cal, &volts,
                             &vdda_mv);
  }

  if (reject != BAT_REJECT_NONE)
  {
    m->reject_count++;
    m->last_reject = (uint32_t)reject;
    if (m->reject_streak < UINT32_MAX)
    {
      m->reject_streak++;
    }
    /* 한계에 닿으면 무효로 내린다. 다음 유효 표본은 필터를 처음부터 다시 잡는다.
       죽어 있던 사이의 값과 섞지 않는다. */
    if (m->reject_streak >= BAT_REJECT_LIMIT)
    {
      m->valid = false;
    }
    return;
  }

  m->reject_streak = 0U;
  m->last_volts = volts;
  m->vdda_mv = vdda_mv;
  m->sample_count++;

  if (!m->valid)
  {
    /* 첫 표본으로 필터를 채운다. 0에서 출발하면 시정수 동안 전압이 0에서 기어
       올라가는 것처럼 보인다. */
    m->volts = volts;
    m->valid = true;
    return;
  }

  m->volts += BAT_FILTER_ALPHA * (volts - m->volts);
}

float battery_monitor_volts_or_nan(const battery_monitor_t *m)
{
  return m->valid ? m->volts : NAN;
}
