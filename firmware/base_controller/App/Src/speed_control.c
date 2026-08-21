/**
  ******************************************************************************
  * @file    speed_control.c
  * @brief   바퀴 하나의 FF + PI 구현.
  * @note    상태는 적분기 하나뿐이다. 나머지는 매 호출 새로 계산한다 — 지난
  *          출력이 어딘가에 남아 있다가 되살아나는 경로를 만들지 않는다는
  *          control_core.c와 같은 원칙이다.
  ******************************************************************************
  */
#include "speed_control.h"

static float clampf(float v, float limit)
{
  if (v > limit)
  {
    return limit;
  }
  if (v < -limit)
  {
    return -limit;
  }
  return v;
}

/**
  * @brief  float duty를 정수 per-mille로 자른다.
  * @note   반올림 뒤 클램프한다. 클램프를 먼저 하면 경계에서 1 ‰이 새어 나간다.
  */
static int32_t to_pm(float duty, int32_t limit)
{
  float rounded = (duty >= 0.0f) ? (duty + 0.5f) : (duty - 0.5f);
  int32_t value = (int32_t)rounded;

  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }
  return value;
}

void speed_control_init(speed_control_t *s)
{
  s->integrator_pm = 0.0f;
  s->target_tps = 0;
  s->duty_pm = 0;
  s->saturated = false;
}

void speed_control_reset(speed_control_t *s)
{
  s->integrator_pm = 0.0f;
  s->duty_pm = 0;
  s->saturated = false;
}

void speed_control_set_target(speed_control_t *s, int32_t target_tps)
{
  /* 부호 반전은 두 값이 모두 0이 아니고 곱이 음수일 때다. 0을 거쳐 가는 경우는
     목표 0에서 이미 적분기가 지워지므로 여기서 또 볼 필요가 없다. */
  bool flipped = ((s->target_tps > 0) && (target_tps < 0)) ||
                 ((s->target_tps < 0) && (target_tps > 0));

  if (flipped)
  {
    speed_control_reset(s);
  }
  s->target_tps = target_tps;
}

float speed_control_feedforward(const speed_gains_t *g, int32_t target_tps)
{
  float target = (float)target_tps;

  if (target_tps == 0)
  {
    return 0.0f;
  }
  if (target_tps > 0)
  {
    return g->run_intercept + (g->kff * target);
  }
  return -g->run_intercept + (g->kff * target);
}

int32_t speed_control_update(speed_control_t *s, const speed_gains_t *g,
                             float measured_tps, bool advance_integrator)
{
  float duty_ff;
  float error;
  float proportional;
  float candidate;
  float duty_raw;

  /* 목표 0은 코스팅이다. PI를 돌리면 error = -measured가 되어 역방향 duty가
     나오고, 그건 전류 측정 수단이 없는 상태에서 거는 능동 제동이다. */
  if (s->target_tps == 0)
  {
    speed_control_reset(s);
    return 0;
  }

  duty_ff = speed_control_feedforward(g, s->target_tps);
  error = (float)s->target_tps - measured_tps;
  proportional = g->kp * error;

  if (advance_integrator)
  {
    float i_next = s->integrator_pm + (g->ki_cycle * error);

    candidate = duty_ff + proportional + i_next;

    /* 조건부 적분: 후보가 포화를 **더 밀어붙일 때만** 보류한다. error가 포화에서
       빠져나오는 방향이면 적분을 허용해야 복귀가 늦어지지 않는다. */
    if ((candidate > (float)g->duty_max_pm) && (error > 0.0f))
    {
      /* 적분기를 갱신하지 않는다 */
    }
    else if ((candidate < -(float)g->duty_max_pm) && (error < 0.0f))
    {
      /* 적분기를 갱신하지 않는다 */
    }
    else
    {
      s->integrator_pm = clampf(i_next, g->i_max_pm);
    }
  }

  duty_raw = duty_ff + proportional + s->integrator_pm;

  /* 포화 판정은 최종 duty_raw 기준이다. 후보값이 아니라 실제로 내보내려던 값이
     범위를 벗어났는지가 호스트에 알릴 사실이다. */
  s->saturated = (duty_raw > (float)g->duty_max_pm) ||
                 (duty_raw < -(float)g->duty_max_pm);

  s->duty_pm = to_pm(duty_raw, g->duty_max_pm);
  return s->duty_pm;
}
