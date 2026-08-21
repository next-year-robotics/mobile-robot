/**
  ******************************************************************************
  * @file    app_main.h
  * @brief   애플리케이션 orchestration. main.c는 이 세 개만 부른다.
  ******************************************************************************
  */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdbool.h>
#include <stdint.h>

/**
  * @brief  애플리케이션 상태와 주변장치를 초기화한다.
  * @retval 실패하면 false. main()은 Error_Handler()로 간다.
  */
bool app_init(void);

/**
  * @brief  super-loop 한 바퀴.
  */
void app_step(void);

/* ---- ISR 문맥에서 adapter가 부르는 진입점 ---- */

/** @brief 수신 바이트 하나. 링이 차면 즉시 출력 0 + RX fault epoch를 기록한다. */
void app_on_rx_byte(uint8_t byte);

/** @brief UART 하드웨어 오류. 즉시 출력 0 + 큐 flush/resync를 예약한다. */
void app_on_uart_error(uint32_t error_code);

/** @brief 수신 재무장 실패. 연속 실패가 이어지면 fault를 래치한다. */
void app_on_rx_rearm_failed(void);

#endif /* APP_MAIN_H */
