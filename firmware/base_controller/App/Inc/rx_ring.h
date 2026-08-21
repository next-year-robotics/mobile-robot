/**
  ******************************************************************************
  * @file    rx_ring.h
  * @brief   단일 생산자(ISR) / 단일 소비자(super-loop) 수신 링버퍼.
  * @note    HAL/RTOS 비의존. 포화는 fail-closed 사건이다. 생산자는 drop_count만
  *          올리고, 애플리케이션은 별도 fault epoch를 보고 출력 0 + 링 전체 폐기 +
  *          다음 개행까지 parser discard를 수행한다. 여러 갭 위치를 하나로 압축해
  *          보존하지 않는다 — 그러면 뒤쪽 갭이 유실될 수 있다.
  ******************************************************************************
  */
#ifndef RX_RING_H
#define RX_RING_H

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"

typedef struct
{
  volatile uint8_t  buf[CMD_RX_RING_SIZE];
  volatile uint16_t head;        /* 생산자만 쓴다 */
  volatile uint16_t tail;        /* 소비자만 쓴다 */
  volatile uint32_t drop_count;  /* 생산자: 버린 바이트 수 (단조) */
} rx_ring_t;

void rx_ring_init(rx_ring_t *r);

/**
  * @brief  생산자(ISR). 가득 차면 버리고 drop_count를 올린다. 기다리지 않는다.
  * @retval 넣었으면 true.
  */
bool rx_ring_push(rx_ring_t *r, uint8_t byte);

/**
  * @brief  소비자. 비었으면 false.
  */
bool rx_ring_pop(rx_ring_t *r, uint8_t *out);

/**
  * @brief  현재 저장된 바이트를 전부 폐기한다.
  * @note   ISR 생산자와 동시에 부를 때는 호출자가 짧은 critical section으로 감싼다.
  *         parser 재동기화는 이 함수의 책임이 아니다.
  */
void rx_ring_discard_all(rx_ring_t *r);

#endif /* RX_RING_H */
