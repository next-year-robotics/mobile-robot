/**
  ******************************************************************************
  * @file    platform_adc.c
  * @brief   ADC1 단일 변환. HAL 호출은 이 파일 안에만 있다.
  ******************************************************************************
  */
#include "platform_adc.h"

#include "app_config.h"

#include "adc.h"
#include "main.h"

/* F446 VREFINT_CAL 주소. stm32f4xx_ll_adc.h의 VREFINT_CAL_ADDR과 같다. LL 헤더는
   CubeMX가 이 프로젝트에 복사하지 않으므로 이름을 바꿔 여기 둔다. */
#define PLATFORM_VREFINT_CAL_ADDR  ((const volatile uint16_t *)0x1FFF7A2AUL)

/**
  * @brief  rank 1의 채널을 바꾸고 한 번 변환한다.
  * @note   480 cycle 표본화다. VREFINT는 최소 10 µs가 필요하고(F446 데이터시트
  *         TS_vrefint), PA4는 100 k ∥ 22 k 분압기 뒤라 소스 임피던스가 18 kΩ이다.
  *         C1 100 nF가 표본화 순간의 전하를 대 주지만, 표본화 시간을 짧게 둘
  *         이유가 없다. 21 MHz에서 22.9 µs다.
  */
static bool convert(uint32_t channel, uint16_t *out)
{
  ADC_ChannelConfTypeDef config = {0};
  bool ok;

  config.Channel = channel;
  config.Rank = 1U;
  config.SamplingTime = ADC_SAMPLETIME_480CYCLES;

  if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK)
  {
    return false;
  }
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return false;
  }

  ok = (HAL_ADC_PollForConversion(&hadc1, BAT_ADC_TIMEOUT_MS) == HAL_OK);
  if (ok)
  {
    *out = (uint16_t)HAL_ADC_GetValue(&hadc1);
  }

  /* 실패해도 멈춰 둔다. 다음 Start가 BUSY로 막히지 않게 한다. */
  if (HAL_ADC_Stop(&hadc1) != HAL_OK)
  {
    ok = false;
  }
  return ok;
}

bool platform_adc_start(void)
{
  uint16_t discard;

  /* HAL_ADC_ConfigChannel이 VREFINT를 처음 고를 때 TSVREFE를 세운다. 기준이 서기까지
     최대 10 µs(t_START)라 그 직후의 변환은 낮게 나올 수 있다. 첫 표본이 필터를
     채우므로 여기서 한 번 버린다. 이후로 비트는 계속 서 있다. */
  return convert(ADC_CHANNEL_VREFINT, &discard);
}

bool platform_adc_read_battery(uint16_t *raw_battery, uint16_t *raw_vrefint)
{
  if (!convert(ADC_CHANNEL_VREFINT, raw_vrefint))
  {
    return false;
  }
  return convert(ADC_CHANNEL_4, raw_battery);
}

uint16_t platform_adc_vrefint_cal(void)
{
  return *PLATFORM_VREFINT_CAL_ADDR;
}
