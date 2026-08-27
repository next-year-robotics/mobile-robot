/**
  ******************************************************************************
  * @file    agent_link.h
  * @brief   micro-ROS agent 연결 상태기계와 `/cmd_vel` stamp 판정. 순수 모듈이다.
  * @note    rcl/rclc를 부르지 않는다. 지금 무엇을 해야 하는가만 정하고 실제 호출은
  *          MicroRosTask가 한다. 그래야 재접속 규칙을 보드 없이 host에서 검증할 수 있다.
  *
  *          상태 전이는 이렇다.
  *            WAITING -> AVAILABLE -> CONNECTED -> DISCONNECTED -> WAITING
  ******************************************************************************
  */
#ifndef AGENT_LINK_H
#define AGENT_LINK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  AGENT_STATE_WAITING = 0,    /* agent를 찾는 중. 엔티티 없음 */
  AGENT_STATE_AVAILABLE,      /* ping 응답 있음. 엔티티는 아직 없음 */
  AGENT_STATE_CONNECTED,      /* 엔티티 생성 완료. executor를 돌린다 */
  AGENT_STATE_DISCONNECTED    /* 끊김 판정. 엔티티를 부숴야 한다 */
} agent_state_t;

/** @note `/base/status.agent_state`의 enum과 값이 같다. */
typedef enum
{
  AGENT_ACTION_WAIT = 0,      /* 쉬어라. 아직 다음 ping 시각이 아니다 */
  AGENT_ACTION_PING,
  AGENT_ACTION_CREATE,
  AGENT_ACTION_SPIN,
  AGENT_ACTION_DESTROY
} agent_action_t;

typedef enum
{
  CMD_STAMP_OK = 0,
  CMD_STAMP_NO_SYNC,   /* 아직 호스트 시계에 동기되지 않았다 */
  CMD_STAMP_ZERO,      /* stamp가 0이다 — 판정할 수 없는 명령 */
  CMD_STAMP_STALE,     /* 만들어진 지 너무 오래됐다 */
  CMD_STAMP_FUTURE     /* 시계가 크게 어긋났다 */
} cmd_stamp_verdict_t;

typedef struct
{
  agent_state_t state;
  uint32_t state_since_ms;
  uint32_t next_ping_ms;
  uint32_t ping_fail_streak;   /* CONNECTED에서 연속 실패한 ping 수 */
  uint32_t backoff_ms;         /* 지금 쓰는 재시도 간격 */
  bool     stop_pending;       /* CONNECTED를 벗어난 사건이 아직 소비되지 않았다 */

  /* SWD로 보는 누적값 */
  uint32_t connect_count;
  uint32_t disconnect_count;
  uint32_t ping_fail_count;
  uint32_t create_fail_count;
  uint32_t spin_fail_count;
} agent_link_t;

void agent_link_init(agent_link_t *l, uint32_t now_ms);

/** @brief 지금 해야 할 일 하나. 상태를 바꾸지 않는다 — 순수 질의다. */
agent_action_t agent_link_next(const agent_link_t *l, uint32_t now_ms);

/**
  * @brief  방금 한 일의 결과를 넣는다. 상태 전이는 전부 여기서 일어난다.
  * @note   `action`은 `agent_link_next()`가 돌려준 값을 그대로 넣는다. 다른 값을
  *         넣으면 무시한다. 호출자가 하지 않은 일의 결과를 보고할 수는 없다.
  */
void agent_link_report(agent_link_t *l, uint32_t now_ms, agent_action_t action,
                       bool ok);

/** @brief 지금 모터 출력이 허용되는가. CONNECTED에서만 참이다. */
bool agent_link_motors_allowed(const agent_link_t *l);

/**
  * @brief  CONNECTED를 벗어난 사건을 한 번만 돌려준다.
  * @note   호출자는 이걸 받은 즉시 urgent STOP을 건다. 매 주기 STOP을 반복해
  *         `urgent_stop_count`를 의미 없이 부풀리지 않으려고 edge로 만든다.
  */
bool agent_link_take_stop_request(agent_link_t *l);

/**
  * @brief  `/cmd_vel`의 `header.stamp`를 받아들일지 판정한다.
  * @param  age_ms_out  판정과 무관하게 계산된 명령 나이. 음수 나이는 0으로 싣는다.
  *
  * @note   수신 시각이 아니라 생성 시각으로 잰다. 링크가 200 ms 막혔다 한꺼번에
  *         뚫리면 수신 시각 기준 워치독은 방금 받았다고 판단해 낡은 명령을 그대로
  *         실행한다.
  *
  *         미래 stamp를 무조건 받아들이지는 않는다. 호스트 시계가 크게 앞서 있으면
  *         나이가 영원히 음수로 계산돼 워치독이 만료되지 않기 때문이다. 동기 오차
  *         수준(max_age 이내)의 미래는 나이 0으로 보고 받아들이고 그보다 멀면
  *         거절한다.
  */
cmd_stamp_verdict_t agent_link_check_stamp(bool epoch_synced, int64_t stamp_ns,
                                           int64_t now_ns, uint32_t max_age_ms,
                                           uint32_t *age_ms_out);

#endif /* AGENT_LINK_H */
