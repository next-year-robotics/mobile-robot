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

/* ControlTask direct notification bit. TIM6/명령/fault가 같은 task를 깨운다. */
#define CTRL_NOTIFY_TICK     (1UL << 0)
#define CTRL_NOTIFY_COMMAND  (1UL << 1)
#define CTRL_NOTIFY_FAULT    (1UL << 2)

#define IO_NOTIFY_RX         (1UL << 0)

rtos_metrics_t g_rtos_metrics;
volatile uint32_t g_control_tick_seq;
volatile uint32_t g_control_tick_cycles;

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
}

static void health_task(void *argument)
{
  TickType_t last_wake = xTaskGetTickCount();
  uint32_t last_led_ms = platform_now_ms();

  (void)argument;
  health_monitor_init(&s_health, 0U);

  for (;;)
  {
    control_status_t status;
    app_io_metrics_t io;
    uint32_t now_ms;

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(HEALTH_PERIOD_MS));
    now_ms = platform_now_ms();

    if (app_link_get_status(&status))
    {
      bool refresh_allowed = health_monitor_update(&s_health, status.heartbeat,
                                                   HEALTH_PERIOD_MS,
                                                   HEALTH_HEARTBEAT_STALL_MS);

      /* 후속 단계에서 **HealthTask만** IWDG를 refresh한다. 이번 단계는 판정을
         기록만 하고 peripheral을 켜지 않는다. */
      g_rtos_metrics.health_refresh_allowed = refresh_allowed ? 1U : 0U;
      g_rtos_metrics.health_stall_events = s_health.stall_events;
      g_rtos_metrics.health_max_stall_ms = s_health.max_stall_ms;
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
/* 생성                                                                       */
/* ------------------------------------------------------------------------- */

void rtos_app_init(void)
{
  s_pending_fault_flags = 0U;
  g_control_tick_seq = 0U;
  g_control_tick_cycles = 0U;

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

  if ((s_cmd_queue == NULL) || (s_result_queue == NULL) || (s_status_queue == NULL))
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
