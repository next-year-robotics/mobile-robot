#include "motor_safety.h"

void motor_safety_init(motor_safety_t *s, uint32_t now_ms, uint32_t timeout_ms)
{
  s->timeout_ms = timeout_ms;
  s->last_cmd_ms = now_ms - timeout_ms;
  s->fault_flags = MOTOR_FAULT_NONE;
  s->timed_out = true;
  s->timeout_notify = false;
}

void motor_safety_feed(motor_safety_t *s, uint32_t now_ms)
{
  if (s->fault_flags != MOTOR_FAULT_NONE)
  {
    /* 래치된 fault는 정상 명령으로 풀리지 않는다. 리셋만이 유일한 복구다. */
    return;
  }
  s->last_cmd_ms = now_ms;
}

void motor_safety_latch_fault(motor_safety_t *s, uint32_t flag)
{
  s->fault_flags |= flag;
  s->timed_out = true;
}

bool motor_safety_faulted(const motor_safety_t *s)
{
  return s->fault_flags != MOTOR_FAULT_NONE;
}

bool motor_safety_output_allowed(motor_safety_t *s, uint32_t now_ms)
{
  if (s->fault_flags != MOTOR_FAULT_NONE)
  {
    s->timed_out = true;
    return false;
  }

  if ((uint32_t)(now_ms - s->last_cmd_ms) >= s->timeout_ms)
  {
    if (!s->timed_out)
    {
      s->timed_out = true;
      s->timeout_notify = true;
    }
    return false;
  }

  s->timed_out = false;
  return true;
}

bool motor_safety_take_notify(motor_safety_t *s)
{
  if (!s->timeout_notify)
  {
    return false;
  }
  s->timeout_notify = false;
  return true;
}
