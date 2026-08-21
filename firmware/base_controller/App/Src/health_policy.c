#include "health_policy.h"

void health_monitor_init(health_monitor_t *h, uint32_t heartbeat)
{
  h->last_heartbeat = heartbeat;
  h->stall_ms = 0U;
  h->max_stall_ms = 0U;
  h->stall_events = 0U;
  h->stalled = false;
}

bool health_monitor_update(health_monitor_t *h, uint32_t heartbeat,
                           uint32_t elapsed_ms, uint32_t stall_limit_ms)
{
  if (heartbeat != h->last_heartbeat)
  {
    h->last_heartbeat = heartbeat;
    h->stall_ms = 0U;
    h->stalled = false;
    return true;
  }

  h->stall_ms += elapsed_ms;
  if (h->stall_ms > h->max_stall_ms)
  {
    h->max_stall_ms = h->stall_ms;
  }

  if (h->stall_ms < stall_limit_ms)
  {
    return true;
  }

  /* 한계를 넘긴 구간마다 한 번만 센다. 정체가 이어지는 동안 카운터가 폭주하지 않는다. */
  if (!h->stalled)
  {
    h->stalled = true;
    h->stall_events++;
  }
  return false;
}
