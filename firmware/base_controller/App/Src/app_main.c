/**
  ******************************************************************************
  * @file    app_main.c
  * @brief   M3 개루프 super-loop orchestration.
  * @note    정책·파싱 계산은 전부 순수 모듈에 있다. 이 파일은 순서만 정한다.
  *          레지스터는 platform_stm32 adapter를 통해서만 만진다.
  *
  *          한 바퀴의 순서는 안전이 먼저다.
  *            1. RX 손상 동기화 + 워치독/fault를 출력에 반영한다.
  *            2. 수신이 풀려 있으면 다시 건다.
  *            3. **상한이 있는** 명령 처리. 수락된 명령은 ACK보다 출력이 먼저다.
  *            4. 워치독 만료 통지.
  *            5. LED / 엔코더 샘플 / 50 Hz 텔레메트리.
  ******************************************************************************
  */
#include "app_main.h"

#include "app_config.h"
#include "encoder_accumulator.h"
#include "line_builder.h"
#include "motor_output.h"
#include "motor_safety.h"
#include "platform_stm32.h"
#include "protocol_ascii.h"
#include "rx_ring.h"

/* 호스트가 지령한 duty(per-mille, 클램프 후). 워치독이 만료돼도 이 값은 남고
   출력만 0이 된다. */
static int32_t g_cmd_duty_left;
static int32_t g_cmd_duty_right;

static rx_ring_t      g_rx;
static protocol_t     g_proto;
static motor_safety_t g_safety;
static encoder_accum_t g_enc_left;
static encoder_accum_t g_enc_right;

/* 송신 버퍼. 송신은 전부 super-loop에서만 하므로 하나를 돌려 쓴다. */
static char g_tx_buffer[TX_LINE_CAP];

static uint32_t g_last_led_ms;
static uint32_t g_last_sample_ms;
static uint32_t g_last_report_ms;

/* ISR이 올리고 super-loop가 관측하는 계측값. */
static volatile uint32_t g_uart_error_count;
static volatile uint32_t g_uart_error_last;
static volatile uint32_t g_rx_rearm_fail_isr;
/* RX overflow/UART error마다 ISR이 증가시킨다. ISR은 동시에 CCR도 즉시 0으로 쓴다.
   main은 epoch 변화를 보면 링을 통째로 비우고 newline까지 parser를 poison한다. */
static volatile uint32_t g_rx_fault_epoch;
static uint32_t g_rx_fault_seen;
static uint32_t g_rx_rearm_fail_run;
static uint32_t g_tx_fail_count;
static uint32_t g_tx_build_fail_count;

/* ------------------------------------------------------------------------- */
/* ISR 진입점                                                                 */
/* ------------------------------------------------------------------------- */

void app_on_rx_byte(uint8_t byte)
{
  if (!rx_ring_push(&g_rx, byte))
  {
    g_rx_fault_epoch++;
    platform_motor_zero();
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
  platform_motor_zero();
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
  */
static void send_line(const char *tag, const int64_t *values, size_t count)
{
  size_t offset;
  size_t length;

  if (!line_build(g_tx_buffer, sizeof(g_tx_buffer), tag, values, count,
                  &offset, &length))
  {
    g_tx_build_fail_count++;
    motor_safety_latch_fault(&g_safety, MOTOR_FAULT_TX_BUILD);
    platform_motor_kill();
    return;
  }

  if (!platform_uart_send(&g_tx_buffer[offset], length))
  {
    g_tx_fail_count++;
    motor_safety_latch_fault(&g_safety, MOTOR_FAULT_TX);
    platform_motor_kill();
  }
}

/* ------------------------------------------------------------------------- */
/* 모터                                                                       */
/* ------------------------------------------------------------------------- */

/**
  * @brief  워치독/fault를 반영해 실제 CCR을 다시 쓴다.
  * @note   지령은 기억해 두고 출력은 매번 새로 계산한다. 이러면 "지난 duty가
  *         어딘가에 남아 있는" 경로 자체가 존재할 수 없다.
  */
static void motor_commit(uint32_t now_ms)
{
  motor_ccr_t out;

  if (motor_safety_output_allowed(&g_safety, now_ms))
  {
    out = motor_ccr_from_duty(g_cmd_duty_left, g_cmd_duty_right);
    platform_motor_write(&out);
    return;
  }

  if (motor_safety_faulted(&g_safety))
  {
    /* 래치된 fault에서는 CCR 0으로 그치지 않고 MOE까지 내린다. */
    platform_motor_kill();
    return;
  }

  out = motor_ccr_from_duty(0, 0);
  platform_motor_write(&out);
}

/* ------------------------------------------------------------------------- */
/* 수신                                                                       */
/* ------------------------------------------------------------------------- */

/**
  * @brief  ISR이 남긴 손상 신호를 프로토콜 상태기계에 반영한다.
  * @note   기존 큐에는 오류 위치보다 앞선 newline이 있을 수 있다. 큐 전체를 버린 뒤
  *         fault 이후 새로 도착한 newline에서만 parser가 재동기화되게 한다.
  */
static bool sync_rx_faults(uint32_t now_ms)
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
  g_cmd_duty_left = 0;
  g_cmd_duty_right = 0;
  motor_commit(now_ms);
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
    motor_safety_latch_fault(&g_safety, MOTOR_FAULT_RX_REARM);
    platform_motor_kill();
  }
}

/**
  * @brief  수락된 명령을 반영한다. **출력이 ACK보다 먼저다.**
  */
static void command_accept(const proto_command_t *cmd, uint32_t now_ms)
{
  int64_t values[3];
  int32_t left = 0;
  int32_t right = 0;
  uint32_t critical_state;

  if (motor_safety_faulted(&g_safety))
  {
    /* fault latch 뒤에는 적용되지 않은 duty를 ok로 거짓 보고하지 않는다. */
    values[0] = (int64_t)now_ms;
    send_line("err", values, 1U);
    return;
  }

  if (cmd->kind == PROTO_CMD_DUTY)
  {
    /* 실제 출력은 어차피 클램프된다. 호스트가 무엇이 적용됐는지 알 수 있도록
       ack에는 클램프 뒤의 값을 싣는다. */
    left = motor_clamp_pm(cmd->left_pm);
    right = motor_clamp_pm(cmd->right_pm);
  }

  /* RX fault ISR이 이 판정과 CCR 반영 사이에 끼어 nonzero 출력을 되살리지 못하게
     짧게 막는다. platform_motor_write의 critical section은 중첩되어도 PRIMASK를 보존한다. */
  critical_state = platform_critical_enter();
  if (g_rx_fault_epoch != g_rx_fault_seen)
  {
    platform_critical_exit(critical_state);
    (void)sync_rx_faults(now_ms);
    values[0] = (int64_t)now_ms;
    send_line("err", values, 1U);
    return;
  }

  g_cmd_duty_left = left;
  g_cmd_duty_right = right;
  motor_safety_feed(&g_safety, now_ms);
  motor_commit(now_ms);
  platform_critical_exit(critical_state);

  /* stop이 UART 완료를 기다리지 않게 한다. duty도 같은 이유로 출력부터 반영한다. */
  values[0] = (int64_t)now_ms;
  values[1] = (int64_t)left;
  values[2] = (int64_t)right;
  send_line("ok", values, 3U);
}

/**
  * @brief  수신 링을 소비하며 줄이 완성될 때마다 해석한다.
  * @note   바이트 수와 줄 수에 **명시적 상한**이 있다. 호스트가 짧은 오류 줄을
  *         최고속으로 퍼부어도 이 함수가 워치독 평가를 굶길 수 없다.
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

    if (sync_rx_faults(now_ms))
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
      int64_t value = (int64_t)now_ms;

      /* 거절한 줄은 워치독을 갱신하지 않는다. */
      send_line("err", &value, 1U);
    }
  }
}

/* ------------------------------------------------------------------------- */
/* 진입점                                                                     */
/* ------------------------------------------------------------------------- */

bool app_init(void)
{
  uint32_t now_ms;

  /* 수신 인터럽트가 살아나기 전에 소비 측 상태를 확정한다. */
  rx_ring_init(&g_rx);
  protocol_init(&g_proto);
  g_cmd_duty_left = 0;
  g_cmd_duty_right = 0;
  g_uart_error_count = 0U;
  g_uart_error_last = 0U;
  g_rx_rearm_fail_isr = 0U;
  g_rx_fault_epoch = 0U;
  g_rx_fault_seen = 0U;
  g_rx_rearm_fail_run = 0U;
  g_tx_fail_count = 0U;
  g_tx_build_fail_count = 0U;

  now_ms = platform_now_ms();
  motor_safety_init(&g_safety, now_ms, CMD_WATCHDOG_MS);

  if (!platform_init())
  {
    return false;
  }

  /* 시작 시점의 카운터를 기준으로 잡아 첫 델타가 0이 되게 한다. */
  encoder_accum_init(&g_enc_left, ENCODER_SIGN_LEFT,
                     platform_encoder_left_count());
  encoder_accum_init(&g_enc_right, ENCODER_SIGN_RIGHT,
                     platform_encoder_right_count());

  now_ms = platform_now_ms();
  g_last_led_ms = now_ms;
  g_last_sample_ms = now_ms;
  g_last_report_ms = now_ms;

  return true;
}

void app_step(void)
{
  uint32_t now_ms = platform_now_ms();

  /* 1. 안전이 먼저다. RX 손상이 있으면 이전 큐를 폐기하고 출력을 0으로 묶는다. */
  (void)sync_rx_faults(now_ms);

  /* 2. 시각을 방금 읽은 값으로 watchdog/fault를 바로 판정한다. */
  motor_commit(now_ms);

  /* 3. 링크 유지. */
  if (g_rx_rearm_fail_isr != 0U)
  {
    g_rx_rearm_fail_isr = 0U;
    g_rx_rearm_fail_run++;
  }
  rx_rearm_if_needed();

  /* 4. 상한이 있는 명령 처리. */
  command_poll(now_ms);

  /* 5. 워치독 만료 통지. */
  if (motor_safety_take_notify(&g_safety))
  {
    int64_t value = (int64_t)now_ms;

    send_line("wd", &value, 1U);
  }

  /* 6. 주기 작업. */
  if ((uint32_t)(now_ms - g_last_led_ms) >= LD2_TOGGLE_PERIOD_MS)
  {
    g_last_led_ms += LD2_TOGGLE_PERIOD_MS;
    platform_led_toggle();
  }

  if ((uint32_t)(now_ms - g_last_sample_ms) >= ENCODER_SAMPLE_PERIOD_MS)
  {
    g_last_sample_ms += ENCODER_SAMPLE_PERIOD_MS;
    encoder_accum_update(&g_enc_left, platform_encoder_left_count());
    encoder_accum_update(&g_enc_right, platform_encoder_right_count());
  }

  if ((uint32_t)(now_ms - g_last_report_ms) >= TICKS_REPORT_PERIOD_MS)
  {
    int64_t values[3];

    g_last_report_ms += TICKS_REPORT_PERIOD_MS;
    values[0] = (int64_t)now_ms;
    values[1] = g_enc_left.ticks;
    values[2] = g_enc_right.ticks;
    send_line("ticks", values, 3U);
  }
}
