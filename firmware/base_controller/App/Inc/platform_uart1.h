/**
  ******************************************************************************
  * @file    platform_uart1.h
  * @brief   USART1(PB6/PB7) 직결 링크의 얇은 STM32 adapter. HAL 타입은 나오지 않는다.
  * @note    **USART2 경로와 완전히 분리돼 있다.** USART2는 M3/M4 ASCII와 ST-LINK
  *          디버그 콘솔로 남고, 이 파일은 RPi5로 가는 GPIO UART만 소유한다
  *          (`interface/base_contract.md` 물리 트랜스포트 절).
  *
  *          수신은 **circular DMA**다. 인터럽트를 바이트마다 받지 않으므로 보드레이트를
  *          올려도 ISR 부하가 늘지 않는다 — 7절 스윕이 3 Mbaud까지 올라갈 수 있는
  *          이유다. 소비자는 NDTR로 생산 위치를 읽어 자기 tail부터 긁어 간다.
  *
  *          송신은 **유한 타임아웃 블로킹**이다. TX DMA는 생성만 돼 있고 여기서는
  *          쓰지 않는다 — 8절 micro-ROS 트랜스포트에서 도입한다.
  ******************************************************************************
  */
#ifndef PLATFORM_UART1_H
#define PLATFORM_UART1_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct
{
  uint32_t error_count;        /* ErrorCallback 진입 횟수 */
  uint32_t error_last;         /* 마지막 유효 HAL_UART_ERROR_* mask */
  uint32_t error_pe;
  uint32_t error_ne;
  uint32_t error_fe;
  uint32_t error_ore;
  uint32_t error_dma;
  uint32_t rearm_count;        /* DMA 수신을 다시 건 횟수 */
  uint32_t rearm_fail_count;
  uint32_t rx_overflow_count;  /* 링이 고수위를 넘은 횟수 (아래 주석 참조) */
  uint32_t tx_fail_count;
  uint32_t baud;               /* 현재 적용된 보드레이트 */
} platform_uart1_metrics_t;

/**
  * @brief  계수기를 0으로 두고 circular DMA 수신을 처음 건다.
  * @note   소유 task의 첫머리에서만 부른다. scheduler 전에 부르지 않는다.
  */
bool platform_uart1_start(void);

/** @brief HAL 기준으로 수신이 무장돼 있는가. */
bool platform_uart1_is_armed(void);

/**
  * @brief  수신이 풀려 있으면 다시 건다.
  * @note   **이 그물이 없으면 링크가 한 번의 FE/NE/PE/ORE로 영구히 죽는다.** HAL은
  *         DMA 수신 중 오류를 blocking error로 보고 전송을 끊기 때문이다. ErrorCallback
  *         안에서 되살리지 않고(그 시점에는 DMA abort가 아직 진행 중일 수 있다)
  *         소유 task가 주기적으로 여기서 확인한다.
  * @retval 무장 상태면 true.
  */
bool platform_uart1_rearm_if_needed(void);

/**
  * @brief  DMA 링에 쌓인 바이트를 최대 max개까지 긁어 온다. 블로킹하지 않는다.
  * @retval 실제로 복사한 바이트 수.
  */
size_t platform_uart1_read(uint8_t *dst, size_t max);

/**
  * @brief  한 줄을 USART1로 내보낸다. **유한 타임아웃**을 쓴다.
  * @retval 타임아웃/오류면 false (tx_fail_count가 오른다).
  */
bool platform_uart1_write(const uint8_t *src, size_t length);

/**
  * @brief  USART1 보드레이트를 바꾸고 수신을 다시 건다. 7절 스윕 전용이다.
  * @note   호출 전에 송신이 비어 있어야 한다 — 이 함수는 진행 중인 프레임을 기다리지
  *         않는다. 호출자가 ACK를 먼저 보내고 짧게 쉰 뒤 부른다.
  */
bool platform_uart1_set_baud(uint32_t baud);

/**
  * @brief  baud를 뺀 모든 계수기를 0으로 되돌린다.
  * @note   "여기서부터 세라"는 뜻이다. 호스트가 Z를 보낼 때와 수신을 처음 걸 때 부른다.
  *         그래야 `rearm_count == 0`이 "복구가 한 번도 필요 없었다"라는 판정이 된다.
  */
void platform_uart1_reset_metrics(void);

void platform_uart1_metrics(platform_uart1_metrics_t *out);

/**
  * @brief  USART1 ErrorCallback 본체. platform_stm32.c의 공용 callback이 위임한다.
  * @note   ISR이다. 계수만 하고 재무장은 하지 않는다.
  */
void platform_uart1_on_error_isr(uint32_t error_code);

#endif /* PLATFORM_UART1_H */
