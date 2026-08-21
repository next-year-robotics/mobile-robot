/**
  ******************************************************************************
  * @file    health_policy.h
  * @brief   control heartbeat 정체 관측. 순수 상태기계.
  * @note    HAL/RTOS 비의존. 이 단계에서는 **관측과 판단만** 한다. IWDG peripheral을
  *          켜거나 refresh하지 않는다 — 후속 단계에서 HealthTask가 이 판정을 그대로
  *          refresh 조건으로 쓰게 하는 것이 목적이다.
  ******************************************************************************
  */
#ifndef HEALTH_POLICY_H
#define HEALTH_POLICY_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint32_t last_heartbeat;
  uint32_t stall_ms;       /* heartbeat가 멈춰 있는 현재 누적 시간 */
  uint32_t max_stall_ms;   /* 지금까지 관측된 최댓값 */
  uint32_t stall_events;   /* stall 한계를 넘긴 횟수 */
  bool     stalled;
} health_monitor_t;

void health_monitor_init(health_monitor_t *h, uint32_t heartbeat);

/**
  * @brief  주기 관측 한 번.
  * @param  heartbeat        ControlTask status의 heartbeat
  * @param  elapsed_ms       직전 관측으로부터 흐른 시간
  * @param  stall_limit_ms   이만큼 heartbeat가 멈추면 stall로 본다
  * @retval 향후 IWDG refresh를 허용해도 되는가 (지금은 기록만 하고 쓰지 않는다).
  */
bool health_monitor_update(health_monitor_t *h, uint32_t heartbeat,
                           uint32_t elapsed_ms, uint32_t stall_limit_ms);

#endif /* HEALTH_POLICY_H */
