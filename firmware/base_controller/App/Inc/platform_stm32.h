/**
  ******************************************************************************
  * @file    platform_stm32.h
  * @brief   TIM/UART/레지스터 접근을 감싸는 얇은 STM32 adapter.
  * @note    인터페이스에는 HAL 타입이 나오지 않는다. 애플리케이션은 이 헤더만
  *          보고, HAL 레지스터 기록은 platform_stm32.c 안에만 존재한다.
  ******************************************************************************
  */
#ifndef PLATFORM_STM32_H
#define PLATFORM_STM32_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motor_output.h"

/**
  * @brief  TIM1 ARR 대조, CCR 0 확정, PWM/엔코더 기동, UART 수신 무장.
  * @retval 하나라도 실패하면 false. 호출자는 Error_Handler()로 간다.
  */
bool platform_init(void);

uint32_t platform_now_ms(void);

/**
  * @brief  네 CCR을 기록한다. **정상 경로에서 TIM1 CCR을 쓰는 유일한 지점이다.**
  * @note   채널마다 반대쪽을 먼저 0으로 만든 뒤 이쪽을 올린다. 한 채널의 두
  *         입력이 동시에 0이 아닌 순간이 생기지 않는다.
  */
void platform_motor_write(const motor_ccr_t *out);

/**
  * @brief  복구 가능한 통신 손상에서 네 CCR만 즉시 0으로 만든다.
  * @note   ISR-safe, allocation/lock/blocking-free. MOE는 유지하므로 parser가 newline에
  *         재동기화된 뒤 새 정상 명령으로 운전을 재개할 수 있다.
  */
void platform_motor_zero(void);

/**
  * @brief  강제 정지. 네 CCR을 0으로 만들고 MOE를 내린다.
  * @note   allocation-free, lock-free, idempotent이며 HAL 블로킹 API를 쓰지
  *         않는다. Error_Handler와 모든 치명 fault 핸들러에서 부를 수 있다.
  */
void platform_motor_kill(void);

uint16_t platform_encoder_left_count(void);
uint16_t platform_encoder_right_count(void);

/**
  * @brief  한 줄을 USART2로 내보낸다. **유한 타임아웃**을 쓴다.
  * @retval 타임아웃/오류면 false. 호출자는 fault를 래치한다.
  */
bool platform_uart_send(const char *data, size_t length);

bool platform_uart_rx_is_armed(void);
bool platform_uart_rx_arm(void);

/** 짧은 ISR/main 공유 상태 갱신용 critical section. 반환값은 이전 PRIMASK다. */
uint32_t platform_critical_enter(void);
void platform_critical_exit(uint32_t state);

void platform_led_toggle(void);

#endif /* PLATFORM_STM32_H */
