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
  * @brief  RCC reset-cause bits를 boot 초기에 보존하고 hardware flags를 clear한다.
  * @note   HAL_Init() 직후, SystemClock_Config()/IWDG 시작 전에 한 번 호출한다.
  */
void platform_boot_snapshot_reset_flags(void);

/** @brief boot 때 보존한 RCC_CSR reset-cause bit mask. */
uint32_t platform_boot_reset_flags(void);

/** @brief 직전 reset 원인에 IWDG가 포함되어 있었는가. */
bool platform_boot_was_iwdg_reset(void);

/**
  * @brief IWDG counter를 refresh한다. HealthTask만 호출한다.
  * @retval HAL refresh 성공 여부.
  */
bool platform_iwdg_refresh(void);

/**
  * @brief  TIM1 ARR 대조, CCR 0 확정, PWM/엔코더 기동, DWT cycle counter 활성화.
  * @note   **인터럽트 원을 하나도 켜지 않는다.** scheduler가 서기 전에 task handle이
  *         없는 ISR이 뜨는 것을 막기 위해 UART 수신 무장과 TIM6 기동은 각 소유
  *         task의 첫머리로 옮겼다.
  * @retval 하나라도 실패하면 false. 호출자는 Error_Handler()로 간다.
  */
bool platform_init(void);

/**
  * @brief  TIM6 100 Hz control tick 인터럽트를 시작한다.
  * @note   **ControlTask handle과 static 객체가 전부 준비된 뒤에만** 부른다.
  *         호출자는 ControlTask 자신이다.
  */
bool platform_control_timer_start(void);

uint32_t platform_now_ms(void);

/**
  * @brief  DWT cycle counter. platform_init()에서 scheduler 전에 한 번 켠다.
  * @note   wrap-safe uint32 뺄셈으로만 쓴다. 84 MHz에서 약 51 s마다 감긴다.
  */
uint32_t platform_dwt_cycles(void);

/**
  * @brief  네 CCR을 기록한다. **정상 경로에서 TIM1 CCR을 쓰는 유일한 지점이다.**
  * @note   채널마다 반대쪽을 먼저 0으로 만든 뒤 이쪽을 올린다. 한 채널의 두
  *         입력이 동시에 0이 아닌 순간이 생기지 않는다.
  */
void platform_motor_write(const motor_ccr_t *out);

/**
  * @brief  네 CCR만 즉시 0으로 만든다. MOE는 유지한다.
  * @note   RTOS 전환 뒤 **허용된 call site는 platform_motor_kill() 내부뿐이다.**
  *         복구 가능한 RX 손상에서 ISR이 직접 CCR을 쓰던 경로는 없앴다 — 이제 ISR은
  *         ControlTask에 urgent STOP을 걸고, 최고 우선순위인 ControlTask가
  *         platform_motor_write()로 0을 반영한다. 정상 경로의 TIM1 소유자를 하나로
  *         유지하기 위한 규칙이다.
  *         ISR-safe, allocation/lock/blocking-free인 성질은 그대로 유지한다.
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

/**
  * @brief  USART2 수신 인터럽트를 무장한다.
  * @note   **LegacyIoTask가 유일한 소유자다.** scheduler 시작 뒤 그 task의 첫머리에서
  *         처음 무장하고, 이후 재무장도 같은 task와 RX/Error callback에서만 한다.
  */
bool platform_uart_rx_arm(void);

/** 짧은 ISR/main 공유 상태 갱신용 critical section. 반환값은 이전 PRIMASK다. */
uint32_t platform_critical_enter(void);
void platform_critical_exit(uint32_t state);

void platform_led_toggle(void);

#endif /* PLATFORM_STM32_H */
