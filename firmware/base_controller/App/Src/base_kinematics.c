/**
  ******************************************************************************
  * @file    base_kinematics.c
  * @brief   차동구동 역기구학과 바퀴 단위 변환.
  ******************************************************************************
  */
#include "base_kinematics.h"

#include "app_config.h"
#include "base_config.h"

#include <math.h>
#include <stddef.h>

#define BASE_TWO_PI  6.283185307179586

const base_geometry_t g_base_geometry = {
  BASE_WHEEL_RADIUS_M,
  BASE_WHEEL_SEPARATION_M,
  BASE_TICKS_PER_REV,
  VEL_TARGET_MAX_TPS
};

static bool geometry_sane(const base_geometry_t *g)
{
  return (g != NULL) && (g->wheel_radius_m > 0.0f) &&
         (g->wheel_separation_m > 0.0f) && (g->ticks_per_rev > 0.0f) &&
         (g->max_tps > 0);
}

static int32_t round_to_i32(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

bool base_kin_twist_to_tps(const base_geometry_t *g,
                           float linear_x_mps, float angular_z_radps,
                           int32_t *tps_left, int32_t *tps_right)
{
  float half_track;
  float mps_to_tps;
  float left;
  float right;
  float peak;
  float limit;

  *tps_left = 0;
  *tps_right = 0;

  if (!geometry_sane(g))
  {
    return false;
  }

  /* isfinite는 NaN과 +-Inf를 한 번에 막는다. `/cmd_vel`은 MCU 밖에서 온 값이므로
     여기가 신뢰 경계다. */
  if (!isfinite(linear_x_mps) || !isfinite(angular_z_radps))
  {
    return false;
  }

  half_track = g->wheel_separation_m * 0.5f;

  /* 바퀴 선속도 [m/s] -> 틱레이트 [tick/s].
     tps = v / (2 pi r) * ticks_per_rev  — 바퀴 1회전이 원주만큼 굴러가고 그때
     엔코더가 ticks_per_rev를 센다는 사실 그대로다. */
  mps_to_tps = g->ticks_per_rev / ((float)BASE_TWO_PI * g->wheel_radius_m);

  /* REP-103: + angular_z = 반시계 = 좌회전 = 왼쪽이 느리다. */
  left = (linear_x_mps - (angular_z_radps * half_track)) * mps_to_tps;
  right = (linear_x_mps + (angular_z_radps * half_track)) * mps_to_tps;

  /* 큰 twist가 들어오면 곱셈 결과가 int32 범위를 넘을 수 있다. 아래 비례 축소가
     그것도 함께 잡지만, Inf가 아닌 거대한 유한값에서 peak/limit 나눗셈이
     의미를 갖도록 순서를 이렇게 둔다. */
  peak = fabsf(left);
  if (fabsf(right) > peak)
  {
    peak = fabsf(right);
  }

  limit = (float)g->max_tps;
  if (peak > limit)
  {
    /* **좌우를 같은 비율로 줄인다.** 바퀴마다 따로 자르면 회전 반경이 바뀐다. */
    float scale = limit / peak;

    left *= scale;
    right *= scale;
  }

  *tps_left = round_to_i32(left);
  *tps_right = round_to_i32(right);

  /* 반올림이 상한을 한 틱 넘길 수 있다. 계약은 정수 상한이므로 여기서 마무리한다. */
  if (*tps_left > g->max_tps)
  {
    *tps_left = g->max_tps;
  }
  else if (*tps_left < -g->max_tps)
  {
    *tps_left = -g->max_tps;
  }
  if (*tps_right > g->max_tps)
  {
    *tps_right = g->max_tps;
  }
  else if (*tps_right < -g->max_tps)
  {
    *tps_right = -g->max_tps;
  }

  return true;
}

double base_kin_ticks_to_rad(const base_geometry_t *g, int64_t ticks)
{
  if (!geometry_sane(g))
  {
    return 0.0;
  }
  return ((double)ticks * BASE_TWO_PI) / (double)g->ticks_per_rev;
}

float base_kin_tps_to_radps(const base_geometry_t *g, int32_t tps)
{
  if (!geometry_sane(g))
  {
    return 0.0f;
  }
  return ((float)tps * (float)BASE_TWO_PI) / g->ticks_per_rev;
}
