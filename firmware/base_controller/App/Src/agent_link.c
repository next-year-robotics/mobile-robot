/**
  ******************************************************************************
  * @file    agent_link.c
  * @brief   agent 연결 상태기계와 명령 나이 판정.
  ******************************************************************************
  */
#include "agent_link.h"

#include "app_config.h"

#include <stddef.h>

static void enter_state(agent_link_t *l, uint32_t now_ms, agent_state_t next)
{
  if ((l->state == AGENT_STATE_CONNECTED) && (next != AGENT_STATE_CONNECTED))
  {
    /* 연결이 끊긴 순간을 놓치지 않게 edge로 남긴다. 소비는 호출자가 한다. */
    l->stop_pending = true;
  }

  l->state = next;
  l->state_since_ms = now_ms;
}

void agent_link_init(agent_link_t *l, uint32_t now_ms)
{
  l->state = AGENT_STATE_WAITING;
  l->state_since_ms = now_ms;
  l->next_ping_ms = now_ms;      /* 첫 ping은 기다리지 않는다 */
  l->ping_fail_streak = 0U;
  l->backoff_ms = AGENT_BACKOFF_MIN_MS;
  l->stop_pending = false;
  l->connect_count = 0U;
  l->disconnect_count = 0U;
  l->ping_fail_count = 0U;
  l->create_fail_count = 0U;
  l->spin_fail_count = 0U;
}

/* 부호 있는 뺄셈이라 uint32 tick이 감겨도 성립한다. */
static bool due(uint32_t now_ms, uint32_t deadline_ms)
{
  return (int32_t)(now_ms - deadline_ms) >= 0;
}

agent_action_t agent_link_next(const agent_link_t *l, uint32_t now_ms)
{
  switch (l->state)
  {
    case AGENT_STATE_WAITING:
      return due(now_ms, l->next_ping_ms) ? AGENT_ACTION_PING : AGENT_ACTION_WAIT;

    case AGENT_STATE_AVAILABLE:
      return AGENT_ACTION_CREATE;

    case AGENT_STATE_CONNECTED:
      /* **ping이 spin보다 앞선다.** 링크가 죽은 채로 executor만 돌리면 끊김을
         영영 알아차리지 못하고 엔티티도 다시 만들지 못한다. */
      return due(now_ms, l->next_ping_ms) ? AGENT_ACTION_PING : AGENT_ACTION_SPIN;

    case AGENT_STATE_DISCONNECTED:
    default:
      return AGENT_ACTION_DESTROY;
  }
}

static void grow_backoff(agent_link_t *l)
{
  uint32_t next = l->backoff_ms * 2U;

  l->backoff_ms = (next > AGENT_BACKOFF_MAX_MS) ? AGENT_BACKOFF_MAX_MS : next;
}

static void report_ping(agent_link_t *l, uint32_t now_ms, bool ok)
{
  if (l->state == AGENT_STATE_WAITING)
  {
    if (ok)
    {
      l->backoff_ms = AGENT_BACKOFF_MIN_MS;
      l->ping_fail_streak = 0U;
      enter_state(l, now_ms, AGENT_STATE_AVAILABLE);
    }
    else
    {
      l->ping_fail_count++;
      l->next_ping_ms = now_ms + l->backoff_ms;
      grow_backoff(l);
    }
    return;
  }

  if (l->state != AGENT_STATE_CONNECTED)
  {
    return;
  }

  if (ok)
  {
    l->ping_fail_streak = 0U;
    l->next_ping_ms = now_ms + AGENT_ALIVE_PING_MS;
    return;
  }

  l->ping_fail_count++;
  l->ping_fail_streak++;
  if (l->ping_fail_streak >= AGENT_PING_FAIL_LIMIT)
  {
    /* 정지 자체는 MCU 내부 200 ms 워치독이 이미 책임진다. 여기서 하는 일은
       "엔티티를 부수고 다시 만들어야 한다"는 판정이다. */
    l->disconnect_count++;
    enter_state(l, now_ms, AGENT_STATE_DISCONNECTED);
  }
  else
  {
    /* 한 번 놓친 것으로는 끊지 않는다. 짧게 한 번 더 확인한다. */
    l->next_ping_ms = now_ms + AGENT_PING_RETRY_MS;
  }
}

void agent_link_report(agent_link_t *l, uint32_t now_ms, agent_action_t action,
                       bool ok)
{
  switch (action)
  {
    case AGENT_ACTION_PING:
      report_ping(l, now_ms, ok);
      break;

    case AGENT_ACTION_CREATE:
      if (l->state != AGENT_STATE_AVAILABLE)
      {
        break;
      }
      if (ok)
      {
        l->connect_count++;
        l->ping_fail_streak = 0U;
        l->next_ping_ms = now_ms + AGENT_ALIVE_PING_MS;
        enter_state(l, now_ms, AGENT_STATE_CONNECTED);
      }
      else
      {
        /* 절반만 만들어진 엔티티가 남아 있을 수 있다. WAITING으로 바로 가지 않고
           DISCONNECTED를 거쳐 반드시 정리 단계를 밟는다. */
        l->create_fail_count++;
        l->disconnect_count++;
        enter_state(l, now_ms, AGENT_STATE_DISCONNECTED);
      }
      break;

    case AGENT_ACTION_SPIN:
      if (l->state != AGENT_STATE_CONNECTED)
      {
        break;
      }
      if (!ok)
      {
        l->spin_fail_count++;
        l->disconnect_count++;
        enter_state(l, now_ms, AGENT_STATE_DISCONNECTED);
      }
      break;

    case AGENT_ACTION_DESTROY:
      if (l->state != AGENT_STATE_DISCONNECTED)
      {
        break;
      }
      /* 정리가 실패해도 WAITING으로 간다. 부수지 못한 엔티티를 붙들고 있어 봐야
         다시 연결할 방법이 없고, 다음 create가 같은 자리를 덮는다. */
      l->ping_fail_streak = 0U;
      l->backoff_ms = AGENT_BACKOFF_MIN_MS;
      l->next_ping_ms = now_ms + AGENT_BACKOFF_MIN_MS;
      enter_state(l, now_ms, AGENT_STATE_WAITING);
      break;

    case AGENT_ACTION_WAIT:
    default:
      break;
  }
}

bool agent_link_motors_allowed(const agent_link_t *l)
{
  return l->state == AGENT_STATE_CONNECTED;
}

bool agent_link_take_stop_request(agent_link_t *l)
{
  bool pending = l->stop_pending;

  l->stop_pending = false;
  return pending;
}

cmd_stamp_verdict_t agent_link_check_stamp(bool epoch_synced, int64_t stamp_ns,
                                           int64_t now_ns, uint32_t max_age_ms,
                                           uint32_t *age_ms_out)
{
  int64_t age_ns;
  int64_t max_age_ns = (int64_t)max_age_ms * 1000000;

  if (age_ms_out != NULL)
  {
    *age_ms_out = 0U;
  }

  if (!epoch_synced)
  {
    return CMD_STAMP_NO_SYNC;
  }
  if (stamp_ns <= 0)
  {
    return CMD_STAMP_ZERO;
  }

  age_ns = now_ns - stamp_ns;

  if (age_ns < 0)
  {
    /* 미래다. 동기 오차 수준이면 나이 0으로 받아들이고, 그보다 멀면 시계가
       어긋난 것으로 본다 — 받아들이면 워치독이 영영 만료되지 않는다. */
    return (-age_ns <= max_age_ns) ? CMD_STAMP_OK : CMD_STAMP_FUTURE;
  }

  if (age_ms_out != NULL)
  {
    int64_t age_ms = age_ns / 1000000;

    *age_ms_out = (age_ms > (int64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)age_ms;
  }

  return (age_ns > max_age_ns) ? CMD_STAMP_STALE : CMD_STAMP_OK;
}
