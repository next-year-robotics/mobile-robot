/**
  ******************************************************************************
  * @file    control_core.c
  * @brief   ControlTask cycle의 순수 구현.
  * @note    출력 결정은 언제나 "지령을 기억하고 매 cycle 새로 계산한다"이다.
  *          지난 duty가 어딘가에 남아 있다가 되살아나는 경로 자체를 만들지 않는다.
  *
  *          두 모드가 나란히 있다.
  *            개루프(duty) — 호스트가 준 per-mille를 클램프해 그대로 쓴다 (M3).
  *            폐루프(vel)  — 목표 tps를 받아 FF+PI가 duty를 만든다 (M4).
  *          모드는 마지막 duty/vel 지령이 정하고, 모드가 바뀌면 적분기를 리셋한다.
  ******************************************************************************
  */
#include "control_core.h"

#include "app_config.h"
#include "motor_output.h"

/* app_config.h의 확정값으로 초기화한다. const가 아닌 것은 M4-B 튜닝 중 SWD로
   덮어쓰기 위해서다 — 확정된 값은 반드시 app_config.h로 되돌려 커밋한다. */
speed_gains_t g_speed_gains_left = {
  KFF_LEFT, RUN_INTERCEPT_LEFT, SPEED_KP_LEFT, SPEED_KI_CYCLE_LEFT,
  SPEED_I_MAX_PM, MOTOR_DUTY_MAX_PM
};

speed_gains_t g_speed_gains_right = {
  KFF_RIGHT, RUN_INTERCEPT_RIGHT, SPEED_KP_RIGHT, SPEED_KI_CYCLE_RIGHT,
  SPEED_I_MAX_PM, MOTOR_DUTY_MAX_PM
};

static int32_t clamp_i32(int32_t value, int32_t limit)
{
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

static int32_t round_to_i32(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

/**
  * @brief  코스팅으로 되돌린다. 목표·적분기·지령 duty를 전부 지운다.
  * @note   워치독 만료, stop, urgent stop이 전부 여기로 온다.
  *         **"정지 = 자유 회전"이 이 단계의 유일한 정지 의미다.** 목표 0에서 PI를
  *         돌리면 error = -measured가 되어 역방향 duty가 나오고, 그건 전류 측정
  *         수단이 없는 상태에서 기어박스에 거는 역토크다 (M4.md 3.5).
  */
static void control_coast(control_core_t *c)
{
  c->cmd_left_pm = 0;
  c->cmd_right_pm = 0;
  speed_control_set_target(&c->spd_left, 0);
  speed_control_set_target(&c->spd_right, 0);
  speed_control_reset(&c->spd_left);
  speed_control_reset(&c->spd_right);
}

/**
  * @brief  폐루프 출력을 다시 계산한다.
  * @param  advance_integrator  정상 cycle(tick_delta == 1)에서만 true.
  */
static void control_run_closed_loop(control_core_t *c, bool advance_integrator)
{
  c->cmd_left_pm = speed_control_update(&c->spd_left, &g_speed_gains_left,
                                        c->measured_tps_left, advance_integrator);
  c->cmd_right_pm = speed_control_update(&c->spd_right, &g_speed_gains_right,
                                         c->measured_tps_right, advance_integrator);
}

/**
  * @brief  워치독/fault를 반영해 이번 cycle의 출력을 정한다.
  * @note   만료 edge는 여기서 한 번만 wd_seq로 바뀐다. 관측자는 wd_seq 변화를 보고
  *         `wd` 한 줄을 만든다 — ControlTask가 0을 반영한 뒤에 일어난다.
  */
static void control_commit(control_core_t *c, uint32_t now_ms, control_out_t *out)
{
  if (motor_safety_output_allowed(&c->safety, now_ms))
  {
    c->output_allowed = true;
    c->out_left_pm = c->cmd_left_pm;
    c->out_right_pm = c->cmd_right_pm;
    out->kind = CONTROL_OUT_WRITE;
    out->ccr  = motor_ccr_from_duty(c->cmd_left_pm, c->cmd_right_pm);
    return;
  }

  c->output_allowed = false;
  c->out_left_pm = 0;
  c->out_right_pm = 0;

  if (motor_safety_take_notify(&c->safety))
  {
    c->wd_seq++;
    c->wd_ms = now_ms;
  }

  /* 만료 상태에서는 목표도 적분기도 남기지 않는다. 출력이 다시 허용되는 유일한
     경로는 새 유효 명령이며, 그때 목표가 새로 실린다. */
  control_coast(c);

  if (motor_safety_faulted(&c->safety))
  {
    /* 래치된 fault에서는 CCR 0으로 그치지 않고 MOE까지 내린다. */
    out->kind = CONTROL_OUT_KILL;
    return;
  }

  out->kind = CONTROL_OUT_WRITE;
  out->ccr  = motor_ccr_from_duty(0, 0);
}

void control_core_init(control_core_t *c, uint32_t now_ms,
                       uint16_t enc_left_raw, uint16_t enc_right_raw)
{
  motor_safety_init(&c->safety, now_ms, CMD_WATCHDOG_MS);
  encoder_accum_init(&c->enc_left, ENCODER_SIGN_LEFT, enc_left_raw);
  encoder_accum_init(&c->enc_right, ENCODER_SIGN_RIGHT, enc_right_raw);
  speed_control_init(&c->spd_left);
  speed_control_init(&c->spd_right);
  c->measured_tps_left = 0.0f;
  c->measured_tps_right = 0.0f;

  c->cmd_left_pm = 0;
  c->cmd_right_pm = 0;
  c->out_left_pm = 0;
  c->out_right_pm = 0;
  c->closed_loop = false;
  c->applied_seq = 0U;
  c->tick_seq_seen = 0U;
  c->loop_count = 0U;
  c->loop_overruns = 0U;
  c->urgent_stop_count = 0U;
  c->stale_command_count = 0U;
  c->heartbeat = 0U;
  c->wd_seq = 0U;
  c->wd_ms = 0U;
  c->output_allowed = false;
}

void control_core_on_tick(control_core_t *c, uint32_t now_ms, uint32_t tick_seq,
                          uint16_t enc_left_raw, uint16_t enc_right_raw,
                          control_out_t *out)
{
  /* 부호 없는 뺄셈이라 tick_seq가 감겨도 성립한다. */
  uint32_t delta = tick_seq - c->tick_seq_seen;
  int32_t  d_left = encoder_accum_delta(&c->enc_left, enc_left_raw);
  int32_t  d_right = encoder_accum_delta(&c->enc_right, enc_right_raw);

  if (delta > 1U)
  {
    /* ISR이 여러 번 울리는 동안 한 번도 깨어나지 못했다. 병합된 만큼을 센다. */
    c->loop_overruns += (delta - 1U);
  }

  if (delta != 0U)
  {
    c->tick_seq_seen = tick_seq;
    c->loop_count++;
    c->heartbeat++;
  }

  /* 병합된 cycle에서도 델타 누적은 그대로 성립한다. 샘플 간격만 길어진다. */
  encoder_accum_update(&c->enc_left, enc_left_raw);
  encoder_accum_update(&c->enc_right, enc_right_raw);

  if (delta != 0U)
  {
    /* 병합된 표본은 여러 주기분의 델타다. tick_delta로 나누지 않으면 속도가
       그 배수만큼 튄다. 정상 경로에서는 delta == 1이므로 기존 식과 같다. */
    float scale = (float)CONTROL_RATE_HZ / (float)delta;

    c->measured_tps_left = (float)d_left * scale;
    c->measured_tps_right = (float)d_right * scale;

    if (c->closed_loop)
    {
      control_run_closed_loop(c, delta == 1U);
    }
  }

  control_commit(c, now_ms, out);
}

void control_core_on_command(control_core_t *c, uint32_t now_ms,
                             const control_command_t *cmd,
                             control_apply_result_t *result, control_out_t *out)
{
  result->seq = cmd->seq;
  result->applied_ms = now_ms;
  result->left = 0;
  result->right = 0;

  if (motor_safety_faulted(&c->safety))
  {
    /* 래치된 fault 뒤에는 적용되지 않은 duty를 ok로 거짓 보고하지 않는다. */
    result->kind = CONTROL_APPLY_REJECTED;
    control_commit(c, now_ms, out);
    return;
  }

  /* 이미 기한을 넘겨 도착한 명령은 적용하지 않는다. 부호 있는 뺄셈으로 wrap-safe하게
     비교한다 — 200 ms 지평에서 uint32 tick의 절반을 넘길 일이 없다. */
  if ((int32_t)(now_ms - cmd->deadline_ms) >= 0)
  {
    c->stale_command_count++;
    result->kind = CONTROL_APPLY_REJECTED;
    control_commit(c, now_ms, out);
    return;
  }

  switch (cmd->kind)
  {
    case CONTROL_CMD_DUTY:
      /* 개루프로 전환한다. 폐루프에서 쌓인 적분기를 남기지 않는다. */
      c->closed_loop = false;
      control_coast(c);
      /* 실제 출력은 어차피 클램프된다. 호스트가 무엇이 적용됐는지 알 수 있도록
         결과에는 클램프 뒤의 값을 싣는다. */
      c->cmd_left_pm = motor_clamp_pm(cmd->left);
      c->cmd_right_pm = motor_clamp_pm(cmd->right);
      break;

    case CONTROL_CMD_VEL:
    {
      int32_t target_left = clamp_i32(cmd->left, VEL_TARGET_MAX_TPS);
      int32_t target_right = clamp_i32(cmd->right, VEL_TARGET_MAX_TPS);

      if (!c->closed_loop)
      {
        /* 모드 전환이다. 개루프 duty와 남은 적분기를 지우고 시작한다. */
        c->closed_loop = true;
        control_coast(c);
      }

      /* 부호가 뒤집힌 바퀴만 자기 적분기를 지운다. 같은 부호의 목표 갱신과
         주기 refresh에서는 유지한다. */
      speed_control_set_target(&c->spd_left, target_left);
      speed_control_set_target(&c->spd_right, target_right);

      /* 새 FF + 새 P + **기존 I**로 즉시 다시 계산한다. 적분은 진행시키지 않는다 —
         ok는 "이 seq의 결과가 CCR에 실렸다"는 뜻을 유지해야 하고, 진행되는 적분의
         dT는 정확히 한 주기여야 한다. */
      control_run_closed_loop(c, false);
      break;
    }

    case CONTROL_CMD_STOP:
    default:
      /* 모드는 유지하고 목표만 0으로 만든다. */
      control_coast(c);
      break;
  }

  motor_safety_feed(&c->safety, cmd->accepted_ms);
  c->applied_seq = cmd->seq;

  control_commit(c, now_ms, out);

  result->kind = c->output_allowed ? CONTROL_APPLY_OK : CONTROL_APPLY_REJECTED;

  /* ACK에 실릴 값의 단위는 명령 종류를 따른다. `ok <ms> <a> <b>` 형식 자체는
     바뀌지 않으므로 호스트의 "보낸 값과 다르면 즉시 정지" 검사가 그대로 돈다. */
  if (cmd->kind == CONTROL_CMD_VEL)
  {
    result->left = c->spd_left.target_tps;
    result->right = c->spd_right.target_tps;
  }
  else
  {
    result->left = c->cmd_left_pm;
    result->right = c->cmd_right_pm;
  }
}

void control_core_on_stop(control_core_t *c, uint32_t now_ms, uint32_t fault_flags,
                          control_out_t *out)
{
  c->urgent_stop_count++;
  control_coast(c);

  if (fault_flags != MOTOR_FAULT_NONE)
  {
    motor_safety_latch_fault(&c->safety, fault_flags);
  }

  control_commit(c, now_ms, out);
}

void control_core_status(const control_core_t *c, uint32_t now_ms,
                         control_status_t *out)
{
  out->timestamp_ms = now_ms;
  out->ticks_left = c->enc_left.ticks;
  out->ticks_right = c->enc_right.ticks;
  out->applied_left_pm = c->cmd_left_pm;
  out->applied_right_pm = c->cmd_right_pm;
  out->applied_seq = c->applied_seq;
  out->fault_flags = c->safety.fault_flags;
  out->wd_seq = c->wd_seq;
  out->wd_ms = c->wd_ms;
  out->heartbeat = c->heartbeat;
  out->loop_count = c->loop_count;
  out->loop_overruns = c->loop_overruns;
  out->output_allowed = c->output_allowed;

  out->last_feed_ms = c->safety.last_cmd_ms;
  out->target_tps_left = c->spd_left.target_tps;
  out->target_tps_right = c->spd_right.target_tps;
  out->measured_tps_left = round_to_i32(c->measured_tps_left);
  out->measured_tps_right = round_to_i32(c->measured_tps_right);
  out->duty_left_pm = c->out_left_pm;
  out->duty_right_pm = c->out_right_pm;
  out->integrator_left_mpm = round_to_i32(c->spd_left.integrator_pm * 1000.0f);
  out->integrator_right_mpm = round_to_i32(c->spd_right.integrator_pm * 1000.0f);
  out->saturated_left = c->spd_left.saturated;
  out->saturated_right = c->spd_right.saturated;
  out->closed_loop = c->closed_loop;
}
