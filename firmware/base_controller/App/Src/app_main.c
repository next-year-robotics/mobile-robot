/**
  ******************************************************************************
  * @file    app_main.c
  * @brief   LegacyIoTask orchestration. ASCII 수신/파싱/ACK/telemetry만 맡는다.
  * @note    정책·파싱 계산은 전부 순수 모듈에 있다. 이 파일은 순서만 정한다.
  *          레지스터는 platform_stm32 adapter를 통해서만 만지고, **TIM1/CCR/MOE는
  *          여기서 건드리지 않는다.** 모터 출력의 정상 경로 소유자는 ControlTask다.
  *
  *          한 바퀴의 순서는 안전이 먼저다.
  *            1. RX 손상 동기화 + ControlTask에 urgent STOP.
  *            2. 수신이 풀려 있으면 다시 건다.
  *            3. **상한이 있는** 명령 처리. ACK는 ControlTask의 적용 결과를 받은 뒤에만.
  *            4. ControlTask가 게시한 워치독 만료 edge 통지.
  *            5. 50 Hz 텔레메트리.
  ******************************************************************************
  */
#include "app_main.h"

#include "app_config.h"
#include "app_link.h"
#include "control_ack.h"
#include "control_types.h"
#include "line_builder.h"
#include "motor_output.h"
#include "motor_safety.h"
#include "platform_stm32.h"
#include "protocol_ascii.h"
#include "rx_ring.h"

static rx_ring_t  g_rx;
static protocol_t g_proto;

/* 송신 버퍼. 송신은 전부 이 task에서만 하므로 하나를 돌려 쓴다. */
static char g_tx_buffer[TX_LINE_CAP];

static uint32_t g_last_report_ms;

/* ControlTask status에서 관측한 워치독 만료 edge. 변화가 보일 때만 `wd` 한 줄이 나간다. */
static uint32_t g_wd_seq_seen;

/* 발행한 command sequence. 0은 "없음"이라 1부터 쓴다. */
static uint32_t g_cmd_seq;

/* ISR이 올리고 task가 관측하는 계측값. */
static volatile uint32_t g_uart_error_count;
static volatile uint32_t g_uart_error_last;
static volatile uint32_t g_rx_rearm_fail_isr;
static volatile uint32_t g_rx_overflow_count;
/* RX overflow/UART error마다 ISR이 증가시킨다. ISR은 동시에 ControlTask에 urgent
   STOP도 건다. task는 epoch 변화를 보면 링을 통째로 비우고 다음 개행까지 parser를
   poison한다. */
static volatile uint32_t g_rx_fault_epoch;
static uint32_t g_rx_fault_seen;
static uint32_t g_rx_rearm_fail_run;
static uint32_t g_tx_fail_count;
static uint32_t g_tx_build_fail_count;
static uint32_t g_cmd_accepted_count;
static uint32_t g_cmd_timeout_count;
static uint32_t g_cmd_seq_mismatch_count;
static uint32_t g_cmd_rejected_count;

/* ------------------------------------------------------------------------- */
/* ISR 진입점                                                                 */
/* ------------------------------------------------------------------------- */

void app_on_rx_byte(uint8_t byte)
{
  if (!rx_ring_push(&g_rx, byte))
  {
    g_rx_overflow_count++;
    g_rx_fault_epoch++;
    /* CCR을 여기서 직접 쓰지 않는다. 최고 우선순위 ControlTask가 ISR 복귀 즉시
       선점해 출력을 0으로 만든다. */
    app_link_urgent_stop_from_isr(MOTOR_FAULT_NONE);
    /* 링 폐기와 parser poison은 IO task의 일이다. poll timeout을 기다리지 않고
       바로 깨워 재동기화를 앞당긴다. */
    app_link_notify_io_from_isr();
    return;
  }

  /* 프레임 경계에서만 깨운다. 230400 8N1에서 바이트마다 깨우면 notification이
     초당 2만 번을 넘고, 그만큼은 전부 ISR/문맥전환 오버헤드가 된다. */
  if (byte == (uint8_t)'\n')
  {
    app_link_notify_io_from_isr();
  }
}

void app_on_uart_error(uint32_t error_code)
{
  /* HAL의 ORE 복구/재무장 경계에서는 ErrorCode가 이미 clear된 callback이 뒤따를 수
     있다. 0으로 마지막 유효 FE/NE/ORE 진단값을 덮어쓰지 않는다. */
  if (error_code != 0U)
  {
    g_uart_error_last = error_code;
  }
  g_uart_error_count++;
  g_rx_fault_epoch++;
  app_link_urgent_stop_from_isr(MOTOR_FAULT_NONE);
  app_link_notify_io_from_isr();
}

void app_on_rx_rearm_failed(void)
{
  g_rx_rearm_fail_isr++;
}

/* ------------------------------------------------------------------------- */
/* 송신                                                                       */
/* ------------------------------------------------------------------------- */

/**
  * @brief  한 줄을 조립해 내보낸다. 실패는 계측하고 안전 상태로 넘어간다.
  * @note   빌더가 capacity를 실제로 검사하므로 넘치는 줄은 송신되지 않는다.
  *         송신 실패는 치명 경로다. platform_motor_kill()은 RTOS와 무관하게 즉시
  *         부를 수 있는 유일한 예외이며, 그 뒤 ControlTask에도 fault를 래치시켜
  *         이후 명령이 `ok`가 아니라 `err`가 되게 한다.
  */
static void send_line(const char *tag, const int64_t *values, size_t count)
{
  size_t offset;
  size_t length;

  if (!line_build(g_tx_buffer, sizeof(g_tx_buffer), tag, values, count,
                  &offset, &length))
  {
    g_tx_build_fail_count++;
    platform_motor_kill();
    app_link_urgent_stop(MOTOR_FAULT_TX_BUILD);
    return;
  }

  if (!platform_uart_send(&g_tx_buffer[offset], length))
  {
    g_tx_fail_count++;
    platform_motor_kill();
    app_link_urgent_stop(MOTOR_FAULT_TX);
  }
}

static void send_err(uint32_t now_ms)
{
  int64_t value = (int64_t)now_ms;

  send_line("err", &value, 1U);
}

/* ------------------------------------------------------------------------- */
/* 수신                                                                       */
/* ------------------------------------------------------------------------- */

/** @brief ISR이 새 RX 손상을 기록했는가 (아직 동기화하지 않은 epoch가 있는가). */
static bool rx_fault_pending(void)
{
  return g_rx_fault_epoch != g_rx_fault_seen;
}

/**
  * @brief  ISR이 남긴 손상 신호를 프로토콜 상태기계에 반영한다.
  * @note   기존 큐에는 오류 위치보다 앞선 newline이 있을 수 있다. 큐 전체를 버린 뒤
  *         fault 이후 새로 도착한 newline에서만 parser가 재동기화되게 한다.
  */
static bool sync_rx_faults(void)
{
  uint32_t epoch;
  uint32_t critical_state = platform_critical_enter();

  epoch = g_rx_fault_epoch;
  if (epoch == g_rx_fault_seen)
  {
    platform_critical_exit(critical_state);
    return false;
  }

  /* 오류 바이트보다 앞에 있던 newline에서 너무 일찍 재동기화되는 일을 막기 위해
     현재 큐를 전부 버린다. critical section 안이라 ISR 생산자와 head/tail이 섞이지 않는다. */
  rx_ring_discard_all(&g_rx);
  g_rx_fault_seen = epoch;
  platform_critical_exit(critical_state);

  protocol_poison(&g_proto);

  /* ISR이 이미 걸었지만 여기서 한 번 더 확정한다. 멱등이며, task 문맥에서 뒤늦게
     발견한 epoch에도 정지가 반드시 따라붙게 한다. */
  app_link_urgent_stop(MOTOR_FAULT_NONE);
  return true;
}

/**
  * @brief  수신이 풀려 있으면 다시 건다. ErrorCallback이 놓친 경우의 그물이다.
  */
static void rx_rearm_if_needed(void)
{
  if (platform_uart_rx_is_armed())
  {
    g_rx_rearm_fail_run = 0U;
    return;
  }

  if (platform_uart_rx_arm())
  {
    g_rx_rearm_fail_run = 0U;
    return;
  }

  g_rx_rearm_fail_run++;
  if (g_rx_rearm_fail_run >= CMD_RX_REARM_FAIL_LIMIT)
  {
    platform_motor_kill();
    app_link_urgent_stop(MOTOR_FAULT_RX_REARM);
  }
}

/**
  * @brief  수락된 명령을 ControlTask에 넘기고 **적용 결과를 받은 뒤에만** ACK한다.
  * @note   `ok`는 "queue에 넣었다"가 아니라 "이 sequence의 duty가 CCR에 실렸다"는
  *         뜻이다. 그래서 stop도 출력이 0이 된 뒤에 ACK된다.
  */
static void command_accept(const proto_command_t *cmd, uint32_t now_ms)
{
  control_command_t out;
  control_apply_result_t result;
  control_ack_t ack;
  bool received;
  int64_t values[3];
  uint32_t critical_state;

  out.seq = ++g_cmd_seq;
  out.left_pm = cmd->left_pm;
  out.right_pm = cmd->right_pm;
  out.is_stop = (cmd->kind != PROTO_CMD_DUTY);
  out.accepted_ms = now_ms;
  out.deadline_ms = now_ms + CMD_WATCHDOG_MS;

  /* RX fault ISR이 판정과 게시 사이에 끼어 손상된 프레임의 명령이 나가지 못하게
     짧게 막는다. 여기서 걸리면 이 줄은 실행하지 않는다. */
  critical_state = platform_critical_enter();
  if (rx_fault_pending())
  {
    platform_critical_exit(critical_state);
    (void)sync_rx_faults();
    send_err(now_ms);
    return;
  }
  platform_critical_exit(critical_state);

  if (!app_link_post_command(&out))
  {
    send_err(now_ms);
    return;
  }
  g_cmd_accepted_count++;

  received = app_link_wait_applied(out.seq, CMD_APPLY_TIMEOUT_MS, &result);

  /* 적용된 뒤에 손상이 들어왔다면 출력은 이미 다시 0이다. 그 duty를 ok로 보고하지 않는다. */
  ack = control_ack_decide(received, out.seq, &result, rx_fault_pending());

  switch (ack)
  {
    case CONTROL_ACK_OK:
      values[0] = (int64_t)result.applied_ms;
      values[1] = (int64_t)result.left_pm;
      values[2] = (int64_t)result.right_pm;
      send_line("ok", values, 3U);
      return;

    case CONTROL_ACK_ERR_TIMEOUT:
      g_cmd_timeout_count++;
      break;

    case CONTROL_ACK_ERR_SEQ:
      g_cmd_seq_mismatch_count++;
      break;

    case CONTROL_ACK_ERR_REJECTED:
    case CONTROL_ACK_ERR_RX_FAULT:
    default:
      g_cmd_rejected_count++;
      break;
  }

  send_err(now_ms);
}

/**
  * @brief  수신 링을 소비하며 줄이 완성될 때마다 해석한다.
  * @note   바이트 수와 줄 수에 **명시적 상한**이 있다. 호스트가 짧은 오류 줄을
  *         최고속으로 퍼부어도 이 함수가 telemetry나 워치독 통지를 굶길 수 없고,
  *         낮은 우선순위라 100 Hz ControlTask를 지연시킬 수도 없다.
  */
static void command_poll(uint32_t now_ms)
{
  uint32_t bytes = 0U;
  uint32_t lines = 0U;

  while ((bytes < CMD_POLL_MAX_BYTES) && (lines < CMD_POLL_MAX_LINES))
  {
    proto_command_t cmd;
    proto_event_t event;
    uint8_t byte;

    if (sync_rx_faults())
    {
      /* 링 전체를 비웠다. 다음 loop에서 fault 이후 새 바이트를 기다린다. */
      break;
    }

    if (!rx_ring_pop(&g_rx, &byte))
    {
      break;
    }
    bytes++;

    event = protocol_push(&g_proto, (char)byte, &cmd);
    if (event == PROTO_EVENT_NONE)
    {
      continue;
    }

    lines++;
    if (event == PROTO_EVENT_COMMAND)
    {
      command_accept(&cmd, now_ms);
    }
    else
    {
      /* 거절한 줄은 워치독을 갱신하지 않는다. */
      send_err(now_ms);
    }
  }
}

/* ------------------------------------------------------------------------- */
/* 진입점                                                                     */
/* ------------------------------------------------------------------------- */

bool app_init(void)
{
  /* 수신 인터럽트가 살아나기 전에 소비 측 상태를 확정한다. */
  rx_ring_init(&g_rx);
  protocol_init(&g_proto);
  g_uart_error_count = 0U;
  g_uart_error_last = 0U;
  g_rx_rearm_fail_isr = 0U;
  g_rx_overflow_count = 0U;
  g_rx_fault_epoch = 0U;
  g_rx_fault_seen = 0U;
  g_rx_rearm_fail_run = 0U;
  g_tx_fail_count = 0U;
  g_tx_build_fail_count = 0U;
  g_cmd_accepted_count = 0U;
  g_cmd_timeout_count = 0U;
  g_cmd_seq_mismatch_count = 0U;
  g_cmd_rejected_count = 0U;
  g_cmd_seq = 0U;
  g_wd_seq_seen = 0U;

  if (!platform_init())
  {
    return false;
  }

  g_last_report_ms = platform_now_ms();
  return true;
}

void app_step(void)
{
  uint32_t now_ms = platform_now_ms();
  control_status_t status;

  /* 1. 안전이 먼저다. RX 손상이 있으면 이전 큐를 폐기하고 출력 0을 확정시킨다. */
  (void)sync_rx_faults();

  /* 2. 링크 유지. */
  if (g_rx_rearm_fail_isr != 0U)
  {
    g_rx_rearm_fail_isr = 0U;
    g_rx_rearm_fail_run++;
  }
  rx_rearm_if_needed();

  /* 3. 상한이 있는 명령 처리. */
  command_poll(now_ms);

  if (!app_link_get_status(&status))
  {
    return;
  }

  /* 4. 워치독 만료 통지. ControlTask가 출력 0을 반영한 뒤에만 edge가 올라간다. */
  if (status.wd_seq != g_wd_seq_seen)
  {
    int64_t value = (int64_t)status.wd_ms;

    g_wd_seq_seen = status.wd_seq;
    send_line("wd", &value, 1U);
  }

  /* 5. 50 Hz 텔레메트리. 누적 int64 의미를 그대로 유지한다. */
  if ((uint32_t)(now_ms - g_last_report_ms) >= TICKS_REPORT_PERIOD_MS)
  {
    int64_t values[3];

    g_last_report_ms += TICKS_REPORT_PERIOD_MS;
    values[0] = (int64_t)now_ms;
    values[1] = status.ticks_left;
    values[2] = status.ticks_right;
    send_line("ticks", values, 3U);
  }
}

void app_get_io_metrics(app_io_metrics_t *out)
{
  out->rx_overflow_count = g_rx_overflow_count;
  out->uart_error_count = g_uart_error_count;
  out->uart_error_last = g_uart_error_last;
  out->rx_rearm_fail_count = g_rx_rearm_fail_run;
  out->tx_fail_count = g_tx_fail_count;
  out->tx_build_fail_count = g_tx_build_fail_count;
  out->cmd_accepted_count = g_cmd_accepted_count;
  out->cmd_timeout_count = g_cmd_timeout_count;
  out->cmd_seq_mismatch_count = g_cmd_seq_mismatch_count;
  out->cmd_rejected_count = g_cmd_rejected_count;
}
