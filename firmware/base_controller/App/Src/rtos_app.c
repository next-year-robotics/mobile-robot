/**
  ******************************************************************************
  * @file    rtos_app.c
  * @brief   static-allocation FreeRTOS 실행 뼈대.
  * @note    **application이 소유하는 RTOS 객체는 전부 여기서 native FreeRTOS API로
  *          만든다.** CMSIS-RTOS v2는 생성 코드가 부르는 osKernelInitialize/
  *          osKernelStart 접점에만 남는다. 같은 객체를 두 API로 번갈아 다루지 않는다.
  *
  *          동적 할당은 없다. configSUPPORT_DYNAMIC_ALLOCATION=0이며 heap_4.c는
  *          링크 대상에서 빠져 있다. idle/timer task 메모리도 여기서 준다.
  ******************************************************************************
  */
#include "rtos_app.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

#include "app_config.h"
#include "app_link.h"
#include "app_main.h"
#include "control_core.h"
#include "control_types.h"
#include "health_policy.h"
#include "main.h"
#include "motor_safety.h"
#include "platform_stm32.h"

#if U3_SMOKE_TEST
#include "line_builder.h"
#include "platform_uart1.h"
#include "u3_smoke.h"
#endif

#if MICRO_ROS
#include "micro_ros_app.h"
#endif

/* ControlTask direct notification bit. TIM6/명령/fault가 같은 task를 깨운다. */
#define CTRL_NOTIFY_TICK     (1UL << 0)
#define CTRL_NOTIFY_COMMAND  (1UL << 1)
#define CTRL_NOTIFY_FAULT    (1UL << 2)
/* micro-ROS `/cmd_vel`. 레거시 명령과 bit를 나눠 어느 mailbox를 비울지 구분한다. */
#define CTRL_NOTIFY_CMDVEL   (1UL << 3)

#define IO_NOTIFY_RX         (1UL << 0)

rtos_metrics_t g_rtos_metrics;
volatile uint32_t g_control_tick_seq;
volatile uint32_t g_control_tick_cycles;
volatile uint32_t g_iwdg_test_stall_control;

/* ------------------------------------------------------------------------- */
/* static RTOS 객체                                                           */
/* ------------------------------------------------------------------------- */

/* 배열 길이는 StackType_t **word**다. CMSIS의 stack_size(byte)와 단위가 다르다. */
static StackType_t  s_control_stack[TASK_STACK_WORDS_CONTROL];
static StaticTask_t s_control_tcb;
static StackType_t  s_health_stack[TASK_STACK_WORDS_HEALTH];
static StaticTask_t s_health_tcb;
static StackType_t  s_io_stack[TASK_STACK_WORDS_IO];
static StaticTask_t s_io_tcb;

static TaskHandle_t s_control_task;
static TaskHandle_t s_health_task;
static TaskHandle_t s_io_task;

#if U3_SMOKE_TEST
/* SmokeTask는 U3 전용이다. handle을 여기 두는 이유는 HealthTask의 stack 계측이
   task 본체보다 앞에 있기 때문이다. */
static StackType_t  s_smoke_stack[TASK_STACK_WORDS_SMOKE];
static StaticTask_t s_smoke_tcb;
static TaskHandle_t s_smoke_task;
#endif

#if MICRO_ROS
/* MicroRosTask (P3). 본체는 micro_ros_app.c에 있고 여기서는 정적 메모리와
   생성만 소유한다 — RTOS 객체를 만드는 자리를 이 파일 하나로 유지한다. */
static StackType_t  s_micro_ros_stack[TASK_STACK_WORDS_MICRO_ROS];
static StaticTask_t s_micro_ros_tcb;
static TaskHandle_t s_micro_ros_task;
#endif

/* 길이 1 mailbox 3개. multi-field/int64 snapshot을 통째로 옮기는 유일한 수단이다. */
static uint8_t       s_cmd_storage[sizeof(control_command_t)];
static StaticQueue_t s_cmd_queue_cb;
static QueueHandle_t s_cmd_queue;

static uint8_t       s_result_storage[sizeof(control_apply_result_t)];
static StaticQueue_t s_result_queue_cb;
static QueueHandle_t s_result_queue;

static uint8_t       s_status_storage[sizeof(control_status_t)];
static StaticQueue_t s_status_queue_cb;
static QueueHandle_t s_status_queue;

/* micro-ROS `/cmd_vel` 전용 길이 1 mailbox. 레거시 명령 큐와 나눈 근거는
   app_link.h의 app_link_post_cmd_vel() 주석에 있다. 이쪽에는 결과 큐가 없다. */
static uint8_t       s_cmdvel_storage[sizeof(control_command_t)];
static StaticQueue_t s_cmdvel_queue_cb;
static QueueHandle_t s_cmdvel_queue;

/* idle/timer task 메모리. 생성 wrapper의 weak 기본 구현에 기대지 않는다. */
static StackType_t  s_idle_stack[TASK_STACK_WORDS_IDLE];
static StaticTask_t s_idle_tcb;
static StackType_t  s_timer_stack[configTIMER_TASK_STACK_DEPTH];
static StaticTask_t s_timer_tcb;

static control_core_t  s_core;
static health_monitor_t s_health;

/* ISR/task가 함께 OR로 쌓는 단일 word. 소비는 ControlTask 하나뿐이다. */
static volatile uint32_t s_pending_fault_flags;

/* ------------------------------------------------------------------------- */
/* 치명 경로                                                                  */
/* ------------------------------------------------------------------------- */

void rtos_app_fatal(void)
{
  /* configASSERT/stack overflow/malloc hook에서 온다. RTOS 상태를 믿을 수 없으므로
     lock-free·allocation-free인 kill만 부르고 멈춘다. */
  platform_motor_kill();
  __disable_irq();
  for (;;)
  {
  }
}

/* ------------------------------------------------------------------------- */
/* ISR                                                                        */
/* ------------------------------------------------------------------------- */

void rtos_app_on_control_tick_isr(void)
{
  BaseType_t woken = pdFALSE;

  g_control_tick_cycles = platform_dwt_cycles();
  g_control_tick_seq++;

  if (s_control_task == NULL)
  {
    return;
  }

  (void)xTaskNotifyFromISR(s_control_task, CTRL_NOTIFY_TICK, eSetBits, &woken);
  portYIELD_FROM_ISR(woken);
}

/* ------------------------------------------------------------------------- */
/* app_link 구현                                                              */
/* ------------------------------------------------------------------------- */

static void latch_pending_fault(uint32_t fault_flags)
{
  uint32_t state;

  if (fault_flags == MOTOR_FAULT_NONE)
  {
    return;
  }

  state = platform_critical_enter();
  s_pending_fault_flags |= fault_flags;
  platform_critical_exit(state);
}

static uint32_t take_pending_faults(void)
{
  uint32_t flags;
  uint32_t state = platform_critical_enter();

  flags = s_pending_fault_flags;
  s_pending_fault_flags = 0U;
  platform_critical_exit(state);
  return flags;
}

bool app_link_post_command(const control_command_t *cmd)
{
  if (s_cmd_queue == NULL)
  {
    return false;
  }

  /* 길이 1 overwrite다. 아직 소비되지 않은 이전 명령은 사라지고 그 seq의 결과는
     오지 않는다 — 호출자가 seq 불일치를 err로 처리한다. */
  if (xQueueOverwrite(s_cmd_queue, cmd) != pdPASS)
  {
    return false;
  }

  (void)xTaskNotify(s_control_task, CTRL_NOTIFY_COMMAND, eSetBits);
  return true;
}

bool app_link_post_cmd_vel(const control_command_t *cmd)
{
  if ((s_cmdvel_queue == NULL) || (s_control_task == NULL))
  {
    return false;
  }

  /* overwrite다. 아직 소비되지 않은 이전 /cmd_vel은 사라진다 — KEEP_LAST(1)
     BEST_EFFORT 계약과 같은 뜻이며, 큐에 쌓인 속도 지령은 이미 사용자의 의도가
     아니다 (base_contract.md QoS 절). */
  if (xQueueOverwrite(s_cmdvel_queue, cmd) != pdPASS)
  {
    return false;
  }

  (void)xTaskNotify(s_control_task, CTRL_NOTIFY_CMDVEL, eSetBits);
  return true;
}

bool app_link_wait_applied(uint32_t seq, uint32_t timeout_ms,
                           control_apply_result_t *out)
{
  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

  if (s_result_queue == NULL)
  {
    return false;
  }

  for (;;)
  {
    /* 부호 있는 뺄셈이라 tick이 감겨도 성립한다. */
    int32_t left = (int32_t)(deadline - xTaskGetTickCount());
    TickType_t remaining = (left > 0) ? (TickType_t)left : 0U;

    if (xQueueReceive(s_result_queue, out, remaining) != pdTRUE)
    {
      return false;
    }

    if (out->seq == seq)
    {
      return true;
    }

    /* 뒤처진 다른 seq의 결과다. 시간이 남았으면 계속 기다리고, 다 썼으면 그대로
       돌려준다 — 호출자가 seq 불일치를 err로 판정한다. */
    if (remaining == 0U)
    {
      return true;
    }
  }
}

bool app_link_get_status(control_status_t *out)
{
  if (s_status_queue == NULL)
  {
    return false;
  }

  /* peek이라 소비되지 않는다. LegacyIoTask와 HealthTask가 같은 snapshot을 본다. */
  return xQueuePeek(s_status_queue, out, 0U) == pdTRUE;
}

void app_link_urgent_stop(uint32_t fault_flags)
{
  latch_pending_fault(fault_flags);

  if (s_control_task == NULL)
  {
    return;
  }
  (void)xTaskNotify(s_control_task, CTRL_NOTIFY_FAULT, eSetBits);
}

void app_link_urgent_stop_from_isr(uint32_t fault_flags)
{
  BaseType_t woken = pdFALSE;

  latch_pending_fault(fault_flags);

  if (s_control_task == NULL)
  {
    return;
  }

  (void)xTaskNotifyFromISR(s_control_task, CTRL_NOTIFY_FAULT, eSetBits, &woken);
  portYIELD_FROM_ISR(woken);
}

void app_link_notify_io_from_isr(void)
{
  BaseType_t woken = pdFALSE;

  if (s_io_task == NULL)
  {
    return;
  }

  (void)xTaskNotifyFromISR(s_io_task, IO_NOTIFY_RX, eSetBits, &woken);
  portYIELD_FROM_ISR(woken);
}

/* ------------------------------------------------------------------------- */
/* ControlTask (P5)                                                           */
/* ------------------------------------------------------------------------- */

/**
  * @brief  **정상 운전에서 TIM1 출력을 건드리는 유일한 지점이다.**
  */
static void control_apply_output(const control_out_t *out)
{
  if (out->kind == CONTROL_OUT_KILL)
  {
    platform_motor_kill();
    return;
  }
  platform_motor_write(&out->ccr);
}

static void control_publish_status(uint32_t now_ms)
{
  control_status_t status;

  control_core_status(&s_core, now_ms, &status);
  (void)xQueueOverwrite(s_status_queue, &status);

  g_rtos_metrics.loop_count = s_core.loop_count;
  g_rtos_metrics.loop_overruns = s_core.loop_overruns;
  g_rtos_metrics.heartbeat = s_core.heartbeat;
  g_rtos_metrics.urgent_stop_count = s_core.urgent_stop_count;
  g_rtos_metrics.stale_command_count = s_core.stale_command_count;
  g_rtos_metrics.fault_flags = status.fault_flags;
  g_rtos_metrics.wd_seq = status.wd_seq;
  /* wd_ms - last_feed_ms가 MCU 내부에서 잰 실제 워치독 지연이다. 호스트에서
     본 정지 시각에는 관성 감속과 표본 간격이 섞이므로 그쪽으로 판정하지 않는다. */
  g_rtos_metrics.wd_ms = status.wd_ms;

  g_rtos_metrics.closed_loop = status.closed_loop ? 1U : 0U;
  g_rtos_metrics.last_feed_ms = status.last_feed_ms;
  g_rtos_metrics.target_tps_left = status.target_tps_left;
  g_rtos_metrics.target_tps_right = status.target_tps_right;
  g_rtos_metrics.measured_tps_left = status.measured_tps_left;
  g_rtos_metrics.measured_tps_right = status.measured_tps_right;
  g_rtos_metrics.duty_left_pm = status.duty_left_pm;
  g_rtos_metrics.duty_right_pm = status.duty_right_pm;
  g_rtos_metrics.integrator_left_mpm = status.integrator_left_mpm;
  g_rtos_metrics.integrator_right_mpm = status.integrator_right_mpm;
  g_rtos_metrics.saturated_left = status.saturated_left ? 1U : 0U;
  g_rtos_metrics.saturated_right = status.saturated_right ? 1U : 0U;
}

static void control_task(void *argument)
{
  (void)argument;

  /* 여기 도달했다는 것은 handle과 static 객체가 전부 준비됐다는 뜻이다. TIM6는
     이 시점 이후에만 인터럽트를 올린다. */
  if (!platform_control_timer_start())
  {
    Error_Handler();
  }

  for (;;)
  {
    control_out_t out;
    uint32_t notify = 0U;
    uint32_t entry_cycles;
    uint32_t now_ms;
    uint32_t elapsed;

    if (xTaskNotifyWait(0U, UINT32_MAX, &notify, portMAX_DELAY) != pdTRUE)
    {
      continue;
    }

#if IWDG_STALL_TEST
    /* HIL fault injection. SWD halt 중에는 IWDG가 freeze되므로 flag를 쓴 뒤 resume한다.
       마지막 PWM을 남기지 않도록 먼저 hardware 출력을 끊고 ControlTask만 멈춘다.
       HealthTask는 계속 돌아 heartbeat 정체를 판정한 뒤 refresh를 중단한다. */
    if (g_iwdg_test_stall_control != 0U)
    {
      g_rtos_metrics.iwdg_test_stall_seen = 1U;
      platform_motor_kill();
      vTaskSuspend(NULL);
      continue;
    }
#endif

    entry_cycles = platform_dwt_cycles();
    now_ms = platform_now_ms();

    /* 순서가 곧 안전 규칙이다. 명령을 먼저 반영하고, 같은 wake에 fault가 섞여
       있으면 그 fault가 출력을 다시 0으로 덮는다. 결과의 정오는 LegacyIoTask가
       RX fault epoch 재확인으로 가린다. */
    if ((notify & CTRL_NOTIFY_COMMAND) != 0U)
    {
      control_command_t cmd;

      while (xQueueReceive(s_cmd_queue, &cmd, 0U) == pdTRUE)
      {
        control_apply_result_t result;

        control_core_on_command(&s_core, now_ms, &cmd, &result, &out);
        control_apply_output(&out);
        (void)xQueueOverwrite(s_result_queue, &result);
      }
    }

    if ((notify & CTRL_NOTIFY_CMDVEL) != 0U)
    {
      control_command_t cmd;

      /* **결과를 게시하지 않는다.** s_result_queue는 레거시 ACK 상관관계 전용이고,
         여기에 /cmd_vel의 결과를 섞으면 LegacyIoTask가 남의 결과를 자기 seq로
         읽는다. /cmd_vel은 응답할 상대가 없으므로 결과를 버려도 잃는 것이 없다. */
      while (xQueueReceive(s_cmdvel_queue, &cmd, 0U) == pdTRUE)
      {
        control_apply_result_t discarded;

        control_core_on_command(&s_core, now_ms, &cmd, &discarded, &out);
        control_apply_output(&out);
      }
    }

    if ((notify & CTRL_NOTIFY_FAULT) != 0U)
    {
      control_core_on_stop(&s_core, now_ms, take_pending_faults(), &out);
      control_apply_output(&out);
    }

    if ((notify & CTRL_NOTIFY_TICK) != 0U)
    {
      uint32_t tick_seq;
      uint32_t tick_cycles;
      uint32_t latency;
      uint32_t state;

      /* sequence와 timestamp는 같은 tick의 짝이어야 한다. ISR이 둘 사이에 끼면
         지연이 엉뚱한 tick 기준으로 계산된다. */
      state = platform_critical_enter();
      tick_seq = g_control_tick_seq;
      tick_cycles = g_control_tick_cycles;
      platform_critical_exit(state);

      /* 진입 시각을 읽은 뒤 새 tick이 들어왔다면 뺄셈이 음수(= 거대한 unsigned)가
         된다. 그런 표본은 최댓값에 반영하지 않는다. */
      latency = entry_cycles - tick_cycles;
      if (((int32_t)latency >= 0) &&
          (latency > g_rtos_metrics.wake_latency_max_cycles))
      {
        g_rtos_metrics.wake_latency_max_cycles = latency;
      }
      g_rtos_metrics.tick_seq = tick_seq;

      control_core_on_tick(&s_core, now_ms, tick_seq,
                           platform_encoder_left_count(),
                           platform_encoder_right_count(), &out);
      control_apply_output(&out);
    }

    control_publish_status(now_ms);

    elapsed = platform_dwt_cycles() - entry_cycles;
    if (elapsed > g_rtos_metrics.wcet_cycles)
    {
      g_rtos_metrics.wcet_cycles = elapsed;
    }
  }
}

/* ------------------------------------------------------------------------- */
/* HealthTask (P4)                                                            */
/* ------------------------------------------------------------------------- */

static void health_collect_stacks(void)
{
  g_rtos_metrics.stack_free_control_words =
      (uint32_t)uxTaskGetStackHighWaterMark(s_control_task);
  g_rtos_metrics.stack_free_health_words =
      (uint32_t)uxTaskGetStackHighWaterMark(s_health_task);
  g_rtos_metrics.stack_free_io_words =
      (uint32_t)uxTaskGetStackHighWaterMark(s_io_task);
  g_rtos_metrics.stack_free_idle_words =
      (uint32_t)uxTaskGetStackHighWaterMark(xTaskGetIdleTaskHandle());
  g_rtos_metrics.stack_free_timer_words =
      (uint32_t)uxTaskGetStackHighWaterMark(xTimerGetTimerDaemonTaskHandle());
#if U3_SMOKE_TEST
  g_rtos_metrics.stack_free_smoke_words =
      (uint32_t)uxTaskGetStackHighWaterMark(s_smoke_task);
#endif
#if MICRO_ROS
  g_rtos_metrics.stack_free_micro_ros_words =
      (uint32_t)uxTaskGetStackHighWaterMark(s_micro_ros_task);
#endif
}

static void health_iwdg_refresh(void)
{
  uint32_t refresh_ms = platform_now_ms();

  if (!platform_iwdg_refresh())
  {
    g_rtos_metrics.iwdg_refresh_fail_count++;
    rtos_app_fatal();
  }

  if (g_rtos_metrics.iwdg_refresh_count == 0U)
  {
    /* IWDG는 boot 도중에 시작되므로 boot tick은 시작->첫 refresh의 보수적 상한이다. */
    g_rtos_metrics.iwdg_first_refresh_ms = refresh_ms;
  }
  g_rtos_metrics.iwdg_refresh_count++;
}

static void health_task(void *argument)
{
  TickType_t last_wake = xTaskGetTickCount();
  uint32_t last_led_ms = platform_now_ms();

  (void)argument;
  health_monitor_init(&s_health, 0U);

  /* MX_IWDG_Init()에서 scheduler 전에 시작된 counter를 HealthTask가 인수한다. */
  health_iwdg_refresh();

  for (;;)
  {
    control_status_t status;
    app_io_metrics_t io;
    uint32_t now_ms;
    bool refresh_allowed = false;

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(HEALTH_PERIOD_MS));
    now_ms = platform_now_ms();

    if (app_link_get_status(&status))
    {
      refresh_allowed = health_monitor_update(&s_health, status.heartbeat,
                                              HEALTH_PERIOD_MS,
                                              HEALTH_HEARTBEAT_STALL_MS);

      g_rtos_metrics.health_refresh_allowed = refresh_allowed ? 1U : 0U;
      g_rtos_metrics.health_stall_events = s_health.stall_events;
      g_rtos_metrics.health_max_stall_ms = s_health.max_stall_ms;
    }
    else
    {
      /* status mailbox가 없으면 ControlTask 진행 근거가 없으므로 먹이지 않는다. */
      g_rtos_metrics.health_refresh_allowed = 0U;
    }

    if (refresh_allowed)
    {
      /* IWDG refresh의 유일한 정상 call site다. */
      health_iwdg_refresh();
    }
    else
    {
      /* ControlTask 진행 근거가 사라진 fatal 경로다. reset을 기다리는 동안 마지막
         PWM이 남지 않도록 즉시 MOE/CCR을 끊고 IWDG는 의도적으로 먹이지 않는다. */
      platform_motor_kill();
    }

    app_get_io_metrics(&io);
    g_rtos_metrics.rx_overflow_count = io.rx_overflow_count;
    g_rtos_metrics.uart_error_count = io.uart_error_count;
    g_rtos_metrics.uart_error_last = io.uart_error_last;
    g_rtos_metrics.rx_rearm_fail_count = io.rx_rearm_fail_count;
    g_rtos_metrics.tx_fail_count = io.tx_fail_count;
    g_rtos_metrics.tx_build_fail_count = io.tx_build_fail_count;
    g_rtos_metrics.cmd_accepted_count = io.cmd_accepted_count;
    g_rtos_metrics.cmd_timeout_count = io.cmd_timeout_count;
    g_rtos_metrics.cmd_seq_mismatch_count = io.cmd_seq_mismatch_count;
    g_rtos_metrics.cmd_rejected_count = io.cmd_rejected_count;

#if MICRO_ROS
    {
      micro_ros_metrics_t mr;

      micro_ros_get_metrics(&mr);
      g_rtos_metrics.mr_agent_state = mr.agent_state;
      g_rtos_metrics.mr_connect_count = mr.connect_count;
      g_rtos_metrics.mr_disconnect_count = mr.disconnect_count;
      g_rtos_metrics.mr_ping_fail_count = mr.ping_fail_count;
      g_rtos_metrics.mr_create_fail_count = mr.create_fail_count;
      g_rtos_metrics.mr_spin_fail_count = mr.spin_fail_count;
      g_rtos_metrics.mr_time_sync_count = mr.time_sync_count;
      g_rtos_metrics.mr_time_sync_fail_count = mr.time_sync_fail_count;
      g_rtos_metrics.mr_epoch_synchronized = mr.epoch_synchronized;
      g_rtos_metrics.mr_cmd_vel_count = mr.cmd_vel_count;
      g_rtos_metrics.mr_cmd_vel_applied_count = mr.cmd_vel_applied_count;
      g_rtos_metrics.mr_cmd_vel_rejected_count = mr.cmd_vel_rejected_count;
      g_rtos_metrics.mr_cmd_vel_last_verdict = mr.cmd_vel_last_verdict;
      g_rtos_metrics.mr_cmd_vel_age_ms = mr.cmd_vel_age_ms;
      g_rtos_metrics.mr_joint_states_count = mr.joint_states_count;
      g_rtos_metrics.mr_joint_states_fail_count = mr.joint_states_fail_count;
      g_rtos_metrics.mr_joint_states_stale_count = mr.joint_states_stale_count;
      g_rtos_metrics.mr_base_status_count = mr.base_status_count;
      g_rtos_metrics.mr_base_status_fail_count = mr.base_status_fail_count;
      g_rtos_metrics.mr_pool_capacity = mr.pool_capacity;
      g_rtos_metrics.mr_pool_free = mr.pool_free;
      g_rtos_metrics.mr_pool_free_min = mr.pool_free_min;
      g_rtos_metrics.mr_pool_largest_free = mr.pool_largest_free;
      g_rtos_metrics.mr_pool_alloc_fail = mr.pool_alloc_fail;
      g_rtos_metrics.mr_pool_invalid_free = mr.pool_invalid_free;
      g_rtos_metrics.mr_pool_live_blocks = mr.pool_live_blocks;
      g_rtos_metrics.mr_pool_escaped_alloc = mr.pool_escaped_alloc;
      g_rtos_metrics.mr_uart1_error_count = mr.uart1_error_count;
      g_rtos_metrics.mr_uart1_error_last = mr.uart1_error_last;
      g_rtos_metrics.mr_uart1_rx_overflow_count = mr.uart1_rx_overflow_count;
      g_rtos_metrics.mr_uart1_rearm_count = mr.uart1_rearm_count;
      g_rtos_metrics.mr_uart1_tx_fail_count = mr.uart1_tx_fail_count;
    }
#endif

    health_collect_stacks();

    if ((uint32_t)(now_ms - last_led_ms) >= LD2_TOGGLE_PERIOD_MS)
    {
      last_led_ms += LD2_TOGGLE_PERIOD_MS;
      platform_led_toggle();
    }
  }
}

/* ------------------------------------------------------------------------- */
/* LegacyIoTask (P2)                                                          */
/* ------------------------------------------------------------------------- */

static void legacy_io_task(void *argument)
{
  (void)argument;

  /* 이 task가 USART2 수신의 유일한 소유자다. handle이 다 생긴 지금 처음 무장한다.
     실패해도 app_step()의 재무장 그물이 받는다. */
  (void)platform_uart_rx_arm();

  for (;;)
  {
    uint32_t notify = 0U;

    /* 개행 notification으로 깨어나고, timeout은 telemetry/워치독 통지/재무장을 위한
       그물이다. 어느 쪽이든 ControlTask보다 낮은 우선순위라 제어를 막을 수 없다. */
    (void)xTaskNotifyWait(0U, UINT32_MAX, &notify,
                          pdMS_TO_TICKS(LEGACY_IO_POLL_MS));
    app_step();
  }
}

/* ------------------------------------------------------------------------- */
/* SmokeTask (P1) — U3 전용, U3_SMOKE_TEST build에만 존재한다                  */
/* ------------------------------------------------------------------------- */

#if U3_SMOKE_TEST

static u3_smoke_t s_smoke;
static char       s_smoke_tx[U3_TX_LINE_CAP];
static uint8_t    s_smoke_rx[U3_SMOKE_READ_CHUNK];

static uint32_t s_smoke_seq;
static uint32_t s_smoke_build_fail;

/* T 스트림 주기. 7절에서 호스트가 `N`으로 바꿔 링크 부하율을 올린다 — 50 Hz 기본값은
   micro-ROS 실부하를 흉내 낼 뿐이라 고속 구간에서는 듀티가 너무 낮아 판정이 안 된다. */
static uint32_t s_stream_period_ms;

/* 보드레이트 전환 안전망(7절). 새 속도에서 유효한 줄을 하나라도 받으면 해제된다. */
static bool     s_baud_probation;
static uint32_t s_baud_probation_ms;

static void smoke_send(const char *tag, const int64_t *values, size_t count)
{
  size_t offset;
  size_t length;

  if (!line_build(s_smoke_tx, sizeof(s_smoke_tx), tag, values, count,
                  &offset, &length))
  {
    /* 시험 경로다. 모터를 끊지 않는다 — 이 build에서도 제어 동작은 그대로여야 한다. */
    s_smoke_build_fail++;
    return;
  }

  (void)platform_uart1_write((const uint8_t *)&s_smoke_tx[offset], length);
}

/**
  * @brief  50 Hz 상태 줄. **Pi -> MCU 방향의 판정값은 전부 여기 실린다.**
  * @note   에코가 돌아오는 길에서 또 유실될 수 있으므로, 그 방향의 정직한 측정은
  *         MCU 안의 계수기뿐이다. 호스트는 이 줄을 읽어 판정한다.
  */
static void smoke_send_status(uint32_t now_ms)
{
  platform_uart1_metrics_t link;
  u3_counters_t counters;
  int64_t values[U3_T_VALUE_COUNT];
  int32_t duty_left = g_rtos_metrics.duty_left_pm;
  int32_t duty_right = g_rtos_metrics.duty_right_pm;

  platform_uart1_metrics(&link);
  u3_smoke_counters(&s_smoke, &counters);

  s_smoke_seq++;
  values[0] = (int64_t)s_smoke_seq;
  values[1] = (int64_t)now_ms;
  values[2] = (int64_t)counters.rx_ping;
  values[3] = (int64_t)counters.rx_bad;
  values[4] = (int64_t)counters.rx_gap;
  values[5] = (int64_t)counters.rx_dup;
  values[6] = (int64_t)link.error_count;
  values[7] = (int64_t)link.error_last;
  values[8] = (int64_t)link.rx_overflow_count;
  values[9] = (int64_t)link.rearm_count;
  values[10] = (int64_t)g_rtos_metrics.loop_overruns;
  /* 6절 "CCR은 안전하게 0으로 수렴" 판정값. 0이면 좌우 duty가 모두 0이다. */
  values[11] = (int64_t)(((duty_left < 0) ? -duty_left : duty_left) +
                         ((duty_right < 0) ? -duty_right : duty_right));

  smoke_send("T", values, U3_T_VALUE_COUNT);
}

/**
  * @brief  보드레이트를 바꾼다. ACK는 **전환 전 속도로** 나가야 호스트가 받는다.
  */
static void smoke_apply_baud(uint32_t baud)
{
  int64_t value = (int64_t)baud;

  smoke_send("B", &value, 1U);

  /* HAL_UART_Transmit은 TC까지 기다리므로 마지막 비트는 이미 선을 떠났다. 이 지연은
     호스트가 ACK를 읽고 자기 포트를 다시 여는 시간이다. */
  vTaskDelay(pdMS_TO_TICKS(U3_BAUD_ACK_SETTLE_MS));

  if (!platform_uart1_set_baud(baud))
  {
    (void)platform_uart1_set_baud(U3_BAUD_DEFAULT);
    s_baud_probation = false;
    return;
  }

  s_baud_probation = (baud != U3_BAUD_DEFAULT);
  s_baud_probation_ms = platform_now_ms();
}

static void smoke_task(void *argument)
{
  TickType_t last_wake = xTaskGetTickCount();
  uint32_t last_stream_ms;

  (void)argument;

  u3_smoke_init(&s_smoke);
  s_smoke_seq = 0U;
  s_smoke_build_fail = 0U;
  s_stream_period_ms = U3_STREAM_PERIOD_MS;
  s_baud_probation = false;
  s_baud_probation_ms = 0U;

  /* 이 task가 USART1의 유일한 소유자다. handle이 다 생긴 지금 처음 무장한다. */
  (void)platform_uart1_start();
  last_stream_ms = platform_now_ms();

  for (;;)
  {
    size_t received;
    size_t i;
    uint32_t now_ms;
    uint32_t pending_baud = 0U;

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(U3_SMOKE_POLL_MS));

    /* 한 번의 FE/NE/PE/ORE로 링크가 영구히 죽지 않게 하는 그물이다. */
    (void)platform_uart1_rearm_if_needed();

    received = platform_uart1_read(s_smoke_rx, sizeof(s_smoke_rx));
    for (i = 0U; i < received; i++)
    {
      uint32_t value = 0U;
      u3_event_t event = u3_smoke_push(&s_smoke, (char)s_smoke_rx[i], &value);

      if (event == U3_EVENT_NONE)
      {
        continue;
      }

      /* 형식이 맞는 줄이 하나라도 왔다면 이 속도에서 링크가 살아 있다는 뜻이다. */
      if (event != U3_EVENT_BAD)
      {
        s_baud_probation = false;
      }

      if (event == U3_EVENT_RESET)
      {
        /* "여기서부터 세라". 링크 계수기도 같이 되돌려야 단계별 판정이 성립한다. */
        platform_uart1_reset_metrics();
      }
      else if (event == U3_EVENT_BAUD)
      {
        /* 링에 남은 바이트를 마저 소비한 뒤에 바꾼다. */
        pending_baud = value;
      }
      else if (event == U3_EVENT_PERIOD)
      {
        s_stream_period_ms = value;
        last_stream_ms = platform_now_ms();
      }
      else if ((event == U3_EVENT_PING) &&
               ((u3_smoke_mode(&s_smoke) & U3_MODE_ECHO) != 0U))
      {
        int64_t echo = (int64_t)value;

        smoke_send("R", &echo, 1U);
      }
      else
      {
        /* MODE/BAD와 에코가 꺼진 PING은 계수기와 mode만 움직인다. */
      }
    }

    if (pending_baud != 0U)
    {
      smoke_apply_baud(pending_baud);
      last_wake = xTaskGetTickCount();
      last_stream_ms = platform_now_ms();
      continue;
    }

    now_ms = platform_now_ms();

    if (s_baud_probation &&
        ((uint32_t)(now_ms - s_baud_probation_ms) >= U3_BAUD_REVERT_MS))
    {
      s_baud_probation = false;
      (void)platform_uart1_set_baud(U3_BAUD_DEFAULT);
      last_wake = xTaskGetTickCount();
      last_stream_ms = platform_now_ms();
      continue;
    }

    if ((u3_smoke_mode(&s_smoke) & U3_MODE_STREAM) == 0U)
    {
      /* 꺼져 있는 동안 밀린 주기를 나중에 몰아 내보내지 않는다. */
      last_stream_ms = now_ms;
    }
    else if ((uint32_t)(now_ms - last_stream_ms) >= s_stream_period_ms)
    {
      /* 따라가고 있는 동안은 누산으로 주기를 정확히 지킨다. 블로킹 송신이 주기보다
         길어지는 저속·고부하 구간에서는 한 주기 넘게 밀리는데, 그때 밀린 몫을 몰아
         내보내면 버스트가 된다. 그런 경우에는 현재 시각으로 맞춰 실제 달성 주파수가
         스스로 낮아지게 두고, 호스트가 그 값을 그대로 잰다. */
      last_stream_ms += s_stream_period_ms;
      if ((uint32_t)(now_ms - last_stream_ms) >= s_stream_period_ms)
      {
        last_stream_ms = now_ms;
      }
      smoke_send_status(now_ms);
    }
    else
    {
      /* 아직 주기가 안 됐다. */
    }
  }
}

#endif /* U3_SMOKE_TEST */

/* ------------------------------------------------------------------------- */
/* 생성                                                                       */
/* ------------------------------------------------------------------------- */

void rtos_app_init(void)
{
  s_pending_fault_flags = 0U;
  g_control_tick_seq = 0U;
  g_control_tick_cycles = 0U;
  g_iwdg_test_stall_control = 0U;
  g_rtos_metrics.boot_reset_flags = platform_boot_reset_flags();
  g_rtos_metrics.boot_was_iwdg_reset =
      platform_boot_was_iwdg_reset() ? 1U : 0U;
  g_rtos_metrics.iwdg_test_mode = (uint32_t)IWDG_STALL_TEST;

  /* 엔코더는 이미 platform_init()에서 돌고 있다. 현재 카운터를 기준으로 잡아
     첫 델타가 0이 되게 한다. */
  control_core_init(&s_core, platform_now_ms(),
                    platform_encoder_left_count(),
                    platform_encoder_right_count());

  s_cmd_queue = xQueueCreateStatic(1U, sizeof(control_command_t),
                                   s_cmd_storage, &s_cmd_queue_cb);
  s_result_queue = xQueueCreateStatic(1U, sizeof(control_apply_result_t),
                                      s_result_storage, &s_result_queue_cb);
  s_status_queue = xQueueCreateStatic(1U, sizeof(control_status_t),
                                      s_status_storage, &s_status_queue_cb);
  s_cmdvel_queue = xQueueCreateStatic(1U, sizeof(control_command_t),
                                      s_cmdvel_storage, &s_cmdvel_queue_cb);

  if ((s_cmd_queue == NULL) || (s_result_queue == NULL) ||
      (s_status_queue == NULL) || (s_cmdvel_queue == NULL))
  {
    Error_Handler();
  }

  /* 첫 status를 미리 채워 둔다. LegacyIoTask가 첫 바퀴에서 빈 mailbox를 만나
     telemetry를 건너뛰지 않게 한다. */
  control_publish_status(platform_now_ms());

  s_control_task = xTaskCreateStatic(control_task, "Control",
                                     TASK_STACK_WORDS_CONTROL, NULL,
                                     TASK_PRIO_CONTROL, s_control_stack,
                                     &s_control_tcb);
  s_health_task = xTaskCreateStatic(health_task, "Health",
                                    TASK_STACK_WORDS_HEALTH, NULL,
                                    TASK_PRIO_HEALTH, s_health_stack,
                                    &s_health_tcb);
  s_io_task = xTaskCreateStatic(legacy_io_task, "LegacyIo",
                                TASK_STACK_WORDS_IO, NULL,
                                TASK_PRIO_LEGACY_IO, s_io_stack, &s_io_tcb);

  if ((s_control_task == NULL) || (s_health_task == NULL) || (s_io_task == NULL))
  {
    Error_Handler();
  }

#if U3_SMOKE_TEST
  s_smoke_task = xTaskCreateStatic(smoke_task, "Smoke",
                                   TASK_STACK_WORDS_SMOKE, NULL,
                                   TASK_PRIO_SMOKE, s_smoke_stack, &s_smoke_tcb);
  if (s_smoke_task == NULL)
  {
    Error_Handler();
  }
#endif

#if MICRO_ROS
  s_micro_ros_task = xTaskCreateStatic(micro_ros_task, "MicroRos",
                                       TASK_STACK_WORDS_MICRO_ROS, NULL,
                                       TASK_PRIO_MICRO_ROS, s_micro_ros_stack,
                                       &s_micro_ros_tcb);
  if (s_micro_ros_task == NULL)
  {
    Error_Handler();
  }
#endif
}

/* ------------------------------------------------------------------------- */
/* static 메모리 공급 (idle / timer)                                          */
/* ------------------------------------------------------------------------- */

/* cmsis_os2.c에 __WEAK 기본 구현이 있지만, 정적 메모리를 application이 명시적으로
   소유하도록 strong definition으로 덮는다. 길이 단위는 word다. */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &s_idle_tcb;
  *ppxIdleTaskStackBuffer = &s_idle_stack[0];
  *pulIdleTaskStackSize = (uint32_t)TASK_STACK_WORDS_IDLE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
  *ppxTimerTaskTCBBuffer = &s_timer_tcb;
  *ppxTimerTaskStackBuffer = &s_timer_stack[0];
  *pulTimerTaskStackSize = (uint32_t)configTIMER_TASK_STACK_DEPTH;
}
