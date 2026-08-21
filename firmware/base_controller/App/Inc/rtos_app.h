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
  uint32_t health_refresh_allowed;    /* 향후 IWDG refresh 판정. 지금은 기록만 한다. */
} rtos_metrics_t;

extern rtos_metrics_t g_rtos_metrics;

/** @brief TIM6 ISR이 올리는 control tick sequence. 생산자는 ISR 하나뿐이다. */
extern volatile uint32_t g_control_tick_seq;

/** @brief TIM6 ISR이 잡은 DWT cycle timestamp. wake latency의 기준이다. */
extern volatile uint32_t g_control_tick_cycles;

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
