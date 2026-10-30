#include "imu_bias.h"

#include "app_config.h"

static void window_reset(imu_bias_t *b)
{
  unsigned i;

  for (i = 0U; i < 3U; i++)
  {
    b->sum[i] = 0.0f;
    b->min[i] = 0.0f;
    b->max[i] = 0.0f;
  }
  b->count = 0U;
}

void imu_bias_init(imu_bias_t *b)
{
  unsigned i;

  window_reset(b);
  for (i = 0U; i < 3U; i++)
  {
    b->bias[i] = 0.0f;
  }
  b->valid = false;
  b->update_count = 0U;
  b->spread_reject_count = 0U;
}

bool imu_bias_update(imu_bias_t *b, const float gyro_radps[3], bool wheels_still)
{
  unsigned i;

  if (!wheels_still)
  {
    /* 창을 이어 붙이지 않는다. 멈춤-움직임-멈춤을 한 창으로 묶으면 그 사이의 회전이
       평균에 섞인다. */
    window_reset(b);
    return false;
  }

  for (i = 0U; i < 3U; i++)
  {
    float g = gyro_radps[i];

    if ((b->count == 0U) || (g < b->min[i]))
    {
      b->min[i] = g;
    }
    if ((b->count == 0U) || (g > b->max[i]))
    {
      b->max[i] = g;
    }
    b->sum[i] += g;
  }
  b->count++;

  if (b->count < IMU_BIAS_WINDOW_SAMPLES)
  {
    return false;
  }

  for (i = 0U; i < 3U; i++)
  {
    if ((b->max[i] - b->min[i]) > IMU_BIAS_MAX_SPREAD_RADPS)
    {
      b->spread_reject_count++;
      window_reset(b);
      return false;
    }
  }

  /* 창 평균으로 바로 바꾼다. 2 s 평균의 잡음은 0.001 °/s 수준이라 이전 값과 섞어
     누를 이유가 없다. 바뀐 만큼이 곧 그동안의 드리프트다. */
  for (i = 0U; i < 3U; i++)
  {
    b->bias[i] = b->sum[i] / (float)b->count;
  }
  b->valid = true;
  b->update_count++;
  window_reset(b);
  return true;
}

void imu_bias_apply(const imu_bias_t *b, const float in_radps[3],
                    float out_radps[3])
{
  unsigned i;

  for (i = 0U; i < 3U; i++)
  {
    out_radps[i] = b->valid ? (in_radps[i] - b->bias[i]) : in_radps[i];
  }
}
