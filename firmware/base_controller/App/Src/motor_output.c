#include "motor_output.h"

int32_t motor_clamp_pm(int32_t duty_pm)
{
  if (duty_pm > MOTOR_DUTY_MAX_PM)
  {
    return MOTOR_DUTY_MAX_PM;
  }
  if (duty_pm < -MOTOR_DUTY_MAX_PM)
  {
    return -MOTOR_DUTY_MAX_PM;
  }
  return duty_pm;
}

uint32_t motor_pulse_from_pm(int32_t duty_pm)
{
  int32_t clamped = motor_clamp_pm(duty_pm);
  int32_t magnitude = (clamped < 0) ? -clamped : clamped;

  /* 1000 x 4667 = 4 667 000. int32에 여유롭게 들어간다. */
  return (uint32_t)((magnitude * (int32_t)(MOTOR_PWM_ARR + 1U))
                    / MOTOR_DUTY_MAX_PM);
}

/**
  * @brief  한 채널의 두 입력에 sign-magnitude를 배치한다.
  * @note   MDD3A는 한쪽에 PWM, 다른 쪽에 0을 넣어 방향을 정한다. 둘 다 High는
  *         브레이크지만 이번 범위에서 쓰지 않는다.
  */
static void fill_pair(uint32_t *a, uint32_t *b, int32_t duty_pm)
{
  uint32_t pulse = motor_pulse_from_pm(duty_pm);

  if (duty_pm >= 0)
  {
    *a = pulse;
    *b = 0U;
  }
  else
  {
    *a = 0U;
    *b = pulse;
  }
}

motor_ccr_t motor_ccr_from_duty(int32_t left_pm, int32_t right_pm)
{
  motor_ccr_t out;

  /* 부호를 곱하기 전에 자른다. INT32_MIN에 -1을 곱하는 경로가 생기지 않는다. */
  int32_t left = MOTOR_SIGN_LEFT * motor_clamp_pm(left_pm);
  int32_t right = MOTOR_SIGN_RIGHT * motor_clamp_pm(right_pm);

  fill_pair(&out.ccr[MOTOR_CCR_LEFT_A], &out.ccr[MOTOR_CCR_LEFT_B], left);
  fill_pair(&out.ccr[MOTOR_CCR_RIGHT_A], &out.ccr[MOTOR_CCR_RIGHT_B], right);

  return out;
}

bool motor_ccr_is_exclusive(const motor_ccr_t *out)
{
  bool left_ok = (out->ccr[MOTOR_CCR_LEFT_A] == 0U)
                 || (out->ccr[MOTOR_CCR_LEFT_B] == 0U);
  bool right_ok = (out->ccr[MOTOR_CCR_RIGHT_A] == 0U)
                  || (out->ccr[MOTOR_CCR_RIGHT_B] == 0U);

  return left_ok && right_ok;
}
