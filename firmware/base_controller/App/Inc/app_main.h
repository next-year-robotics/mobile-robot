/**
  ******************************************************************************
  * @file    app_main.h
  * @brief   LegacyIoTask의 orchestration. ASCII 수신/파싱/ACK/telemetry를 맡는다.
  * @note    이 파일은 TIM1/CCR/MOE를 직접 만지지 않는다. 모터 출력의 정상 경로
  *          소유자는 ControlTask 하나뿐이다. 여기서는 app_link로 명령을 넘기고 적용
  *          결과를 받은 뒤에만 ACK한다.
  ******************************************************************************
  */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdbool.h>
#include <stdint.h>

/** @brief 디버거/SWD에서 관측하는 통신 계측값. 고정 크기다. */
typedef struct
{
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
} app_io_metrics_t;

/**
  * @brief  수신/파서 상태와 주변장치를 초기화한다.
  * @note   scheduler 시작 전에 부른다. 여기서 인터럽트 원은 켜지지 않는다.
  * @retval 실패하면 false. main()은 Error_Handler()로 간다.
  */
bool app_init(void);

/**
  * @brief  LegacyIoTask 한 바퀴. RX 손상 동기화 -> 재무장 -> 상한 있는 명령 처리 ->
  *         워치독 통지 -> 50 Hz telemetry.
  */
void app_step(void);

/** @brief 계측값 snapshot. HealthTask가 읽어 간다. */
void app_get_io_metrics(app_io_metrics_t *out);

/* ---- ISR 문맥에서 adapter가 부르는 진입점 ---- */

/**
  * @brief  수신 바이트 하나.
  * @note   링이 차면 ControlTask에 urgent STOP을 걸고 RX fault epoch를 올린다. CCR을
  *         직접 쓰지 않는다. LegacyIoTask는 개행에서만 깨운다.
  */
void app_on_rx_byte(uint8_t byte);

/** @brief UART 하드웨어 오류. urgent STOP + 큐 flush/resync를 예약한다. */
void app_on_uart_error(uint32_t error_code);

/** @brief 수신 재무장 실패. 연속 실패가 이어지면 fault를 래치한다. */
void app_on_rx_rearm_failed(void);

#endif /* APP_MAIN_H */
