/**
  ******************************************************************************
  * @file    rtos_app.h
  * @brief   application이 소유하는 static FreeRTOS 실행 구조.
  * @note    이 헤더는 펌웨어 전용이다(순수 모듈은 포함하지 않는다). 인터페이스에는
  *          FreeRTOS 타입이 나오지 않으므로 생성된 Core/ 코드에서 그대로 부를 수 있다.
  *
  *          task/IRQ/데이터 소유권
  *            - ControlTask  P5 : TIM6 tick, 엔코더 샘플, 워치독, **TIM1 출력**
  *            - HealthTask   P4 : heartbeat/stack/fault 관측, LED
  *            - LegacyIoTask P2 : USART2 RX/TX, ASCII 파싱, ACK, 50 Hz telemetry
  ******************************************************************************
  */
#ifndef RTOS_APP_H
#define RTOS_APP_H

#include <stdint.h>

/**
  * @brief  디버거/SWD에서 관측하는 고정 크기 runtime 지표.
  * @note   문자열로 만들지 않는다. ControlTask는 여기에 숫자만 남긴다.
  */
typedef struct
{
  uint32_t tick_seq;                  /* TIM6 ISR이 올린 control tick sequence */
  uint32_t loop_count;                /* 실제로 처리한 control cycle 수 */
  uint32_t loop_overruns;             /* 병합되어 놓친 tick 수 (누적) */
  uint32_t wcet_cycles;               /* control cycle 최대 소요 cycle */
  uint32_t wake_latency_max_cycles;   /* TIM6 ISR -> ControlTask 진입 최대 지연 */
  uint32_t heartbeat;
  uint32_t urgent_stop_count;
  uint32_t stale_command_count;
  uint32_t fault_flags;
  uint32_t wd_seq;

  /* LegacyIoTask/ISR 계측 (app_main.c에서 복사해 온다) */
  uint32_t rx_overflow_count;
  uint32_t uart_error_count;
  uint32_t uart_error_last;
  uint32_t rx_rearm_fail_count;
  uint32_t tx_fail_count;
  uint32_t tx_build_fail_count;
  uint32_t cmd_accepted_count;
  uint32_t cmd_timeout_count;
  uint32_t cmd_seq_mismatch_count;
  uint32_t cmd_rejected_count;

  /* stack high-water mark. 단위는 **word**이며 남은 여유를 뜻한다. */
  uint32_t stack_free_control_words;
  uint32_t stack_free_health_words;
  uint32_t stack_free_io_words;
  uint32_t stack_free_idle_words;
  uint32_t stack_free_timer_words;

  uint32_t health_stall_events;
  uint32_t health_max_stall_ms;
  uint32_t health_refresh_allowed;    /* 이번 HealthTask 주기의 refresh 판정 */

  /* IWDG/reset 계측. reset_flags는 boot 직후 RCC_CSR의 reset-cause bit mask다. */
  uint32_t boot_reset_flags;
  uint32_t boot_was_iwdg_reset;
  uint32_t iwdg_first_refresh_ms;     /* boot HAL tick 기준 startup handoff 시각 */
  uint32_t iwdg_refresh_count;
  uint32_t iwdg_refresh_fail_count;
  uint32_t iwdg_test_mode;
  uint32_t iwdg_test_stall_seen;

  /* ---- 속도 폐루프 계측 (M4) ----
     "계측은 와이어가 아니라 SWD로"가 뼈대 단계에서 세운 원칙이다. 목표/적분기/
     포화는 여기서만 본다. 전부 정수이며 적분기는 ‰ x 1000으로 싣는다. */
  uint32_t closed_loop;               /* 현재 모드 (0 개루프 / 1 폐루프) */
  uint32_t last_feed_ms;              /* 마지막으로 워치독을 먹인 시각 */
  int32_t  target_tps_left;
  int32_t  target_tps_right;
  int32_t  measured_tps_left;
  int32_t  measured_tps_right;
  int32_t  duty_left_pm;              /* 실제로 CCR에 실린 duty */
  int32_t  duty_right_pm;
  int32_t  integrator_left_mpm;       /* 적분기 ‰ x 1000 */
  int32_t  integrator_right_mpm;
  uint32_t saturated_left;            /* 클램프 전 duty가 범위를 벗어났는가 */
  uint32_t saturated_right;
} rtos_metrics_t;

extern rtos_metrics_t g_rtos_metrics;

/** @brief TIM6 ISR이 올리는 control tick sequence. 생산자는 ISR 하나뿐이다. */
extern volatile uint32_t g_control_tick_seq;

/** @brief TIM6 ISR이 잡은 DWT cycle timestamp. wake latency의 기준이다. */
extern volatile uint32_t g_control_tick_cycles;

/**
  * @brief IWDG_STALL_TEST build에서 SWD로 1을 쓰면 ControlTask를 안전 정지시킨다.
  * @note  production build에도 관측 symbol은 남지만 test option이 0이면 무시한다.
  */
extern volatile uint32_t g_iwdg_test_stall_control;

/**
  * @brief  static task와 mailbox를 만든다. MX_FREERTOS_Init()에서 부른다.
  * @note   scheduler가 서기 전이며, 이 함수가 끝난 뒤에야 인터럽트 원이 켜진다.
  *         실패하면 돌아오지 않는다(Error_Handler).
  */
void rtos_app_init(void);

/**
  * @brief  TIM6 100 Hz update ISR 본체.
  * @note   sequence 증가, DWT timestamp, ControlTask direct notification, yield 예약이
  *         전부다. 계산도 파싱도 logging도 하지 않는다.
  */
void rtos_app_on_control_tick_isr(void);

/**
  * @brief  configASSERT/치명 경로의 정지. 모터를 먼저 끊고 멈춘다.
  */
void rtos_app_fatal(void);

#endif /* RTOS_APP_H */
