/**
  ******************************************************************************
  * @file    control_core.c
  * @brief   ControlTask cycle의 순수 구현.
  * @note    출력 결정은 언제나 "지령을 기억하고 매 cycle 새로 계산한다"이다.
  *          지난 duty가 어딘가에 남아 있다가 되살아나는 경로 자체를 만들지 않는다.
  ******************************************************************************
  */
#include "control_core.h"

#include "app_config.h"
#include "motor_output.h"

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
    out->kind = CONTROL_OUT_WRITE;
    out->ccr  = motor_ccr_from_duty(c->cmd_left_pm, c->cmd_right_pm);
    return;
  }

  c->output_allowed = false;

  if (motor_safety_take_notify(&c->safety))
  {
    c->wd_seq++;
    c->wd_ms = now_ms;
  }

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

  c->cmd_left_pm = 0;
  c->cmd_right_pm = 0;
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

  control_commit(c, now_ms, out);
}

void control_core_on_command(control_core_t *c, uint32_t now_ms,
                             const control_command_t *cmd,
                             control_apply_result_t *result, control_out_t *out)
{
  result->seq = cmd->seq;
  result->applied_ms = now_ms;
  result->left_pm = 0;
  result->right_pm = 0;

  if (motor_safety_faulted(&c->safety))
  {
    /* 래치된 fault 뒤에는 적용되지 않은 duty를 ok로 거짓 보고하지 않는다. */
    result->kind = CONTROL_APPLY_REJECTED;
    control_commit(c, now_ms, out);
    return;
  }

  /* 이미 기한을 넘겨 도착한 명령은 적용하지 않는다. 부호 있는 뺄셈으로 wrap-safe하게
     비교한다 — 500 ms 지평에서 uint32 tick의 절반을 넘길 일이 없다. */
  if ((int32_t)(now_ms - cmd->deadline_ms) >= 0)
  {
    c->stale_command_count++;
    result->kind = CONTROL_APPLY_REJECTED;
    control_commit(c, now_ms, out);
    return;
  }

  if (cmd->is_stop)
  {
    c->cmd_left_pm = 0;
    c->cmd_right_pm = 0;
  }
  else
  {
    /* 실제 출력은 어차피 클램프된다. 호스트가 무엇이 적용됐는지 알 수 있도록
       결과에는 클램프 뒤의 값을 싣는다. */
    c->cmd_left_pm = motor_clamp_pm(cmd->left_pm);
    c->cmd_right_pm = motor_clamp_pm(cmd->right_pm);
  }

  motor_safety_feed(&c->safety, cmd->accepted_ms);
  c->applied_seq = cmd->seq;

  control_commit(c, now_ms, out);

  result->kind = c->output_allowed ? CONTROL_APPLY_OK : CONTROL_APPLY_REJECTED;
  result->left_pm = c->cmd_left_pm;
  result->right_pm = c->cmd_right_pm;
}

void control_core_on_stop(control_core_t *c, uint32_t now_ms, uint32_t fault_flags,
                          control_out_t *out)
{
  c->urgent_stop_count++;
  c->cmd_left_pm = 0;
  c->cmd_right_pm = 0;

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
}
