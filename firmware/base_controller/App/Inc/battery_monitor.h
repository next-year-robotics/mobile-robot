/**
  ******************************************************************************
  * @file    battery_monitor.h
  * @brief   배터리 전압 환산과 필터. 순수 모듈.
  * @note    HAL/RTOS 비의존. ADC 원시값 두 개(PA4, VREFINT)와 VREFINT 공장 보정값을
  *          받아 VB_LOGIC 전압을 낸다. 변환은 HealthTask가 platform_adc로 한다.
  *
  *          VREFINT를 같이 읽는 이유: ADC의 기준은 VDDA(= NUCLEO 3V3)다. 실측 3.23 V로
  *          공칭 3.3 V보다 2 % 낮고 부하와 온도로 움직인다. VREFINT는 공장에서
  *          VDDA = 3.3 V로 잰 값이 OTP에 있으므로 그 비로 VDDA를 역산하면 분압비 하나만
  *          남는다. 멀티미터로 맞출 대상이 하나로 줄어든다.
  ******************************************************************************
  */
#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

/** @brief 표본을 버린 이유. SWD로 마지막 값을 본다. */
typedef enum
{
  BAT_REJECT_NONE = 0,
  BAT_REJECT_ADC,         /* 변환 실패 (HAL 오류 / 타임아웃) */
  BAT_REJECT_VDDA,        /* VREFINT로 역산한 VDDA가 허용 범위 밖 */
  BAT_REJECT_SATURATED    /* PA4가 VDDA 이상. 실제 전압을 알 수 없다 */
} battery_reject_t;

typedef struct
{
  float    volts;           /* 필터 출력 [V]. valid가 아니면 의미 없다 */
  float    last_volts;      /* 필터 전 마지막 유효 표본 [V] */
  uint32_t vdda_mv;         /* 마지막 유효 표본의 VDDA */
  bool     valid;
  uint32_t sample_count;    /* 받아들인 표본 */
  uint32_t reject_count;    /* 버린 표본 (누적) */
  uint32_t reject_streak;   /* 연속으로 버린 표본 */
  uint32_t last_reject;     /* battery_reject_t */
} battery_monitor_t;

void battery_monitor_init(battery_monitor_t *m);

/**
  * @brief  ADC 원시값 한 쌍을 VB_LOGIC 전압으로 바꾼다.
  * @param  raw_battery   PA4(ADC1_IN4) 12-bit 변환값
  * @param  raw_vrefint   VREFINT(ADC1_IN17) 12-bit 변환값
  * @param  vrefint_cal   OTP의 VREFINT_CAL (VDDA = 3.3 V, 30 °C에서 잰 값)
  * @param  out_volts     VB_LOGIC [V]
  * @param  out_vdda_mv   역산한 VDDA [mV]
  * @retval 버릴 표본이면 그 이유, 아니면 BAT_REJECT_NONE.
  */
battery_reject_t battery_convert(uint16_t raw_battery, uint16_t raw_vrefint,
                                 uint16_t vrefint_cal, float *out_volts,
                                 uint32_t *out_vdda_mv);

/**
  * @brief  주기 표본 하나를 넣는다.
  * @param  adc_ok  두 변환이 모두 끝났는가. false면 나머지 인자는 보지 않는다.
  */
void battery_monitor_update(battery_monitor_t *m, bool adc_ok,
                            uint16_t raw_battery, uint16_t raw_vrefint,
                            uint16_t vrefint_cal);

/**
  * @brief  발행할 값. 유효하지 않으면 NaN이다.
  * @note   0을 내보내지 않는다. 0 V는 누군가 "배터리가 비었다"로 읽는다.
  */
float battery_monitor_volts_or_nan(const battery_monitor_t *m);

#endif /* BATTERY_MONITOR_H */
