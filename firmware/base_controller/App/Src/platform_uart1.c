/**
  ******************************************************************************
  * @file    platform_uart1.c
  * @brief   USART1 circular-DMA 링크 adapter. HAL 레지스터 기록은 이 파일 안에만 있다.
  ******************************************************************************
  */
#include "platform_uart1.h"

#include "app_config.h"

#include "main.h"
#include "usart.h"

/* DMA 링 크기. 2의 거듭제곱이어야 인덱스를 & 로 감을 수 있다.
   소비자가 U3_SMOKE_POLL_MS마다 긁어 가므로, 이 크기는 "한 주기 동안 들어올 수 있는
   최대 바이트"보다 넉넉해야 한다. 2048 B는 3 Mbaud에서도 6.8 ms 분량이라 1 ms 주기에
   6배 이상 여유가 있다. */
#define UART1_RX_DMA_SIZE       2048U

/* 고수위. 링이 이만큼 차 있는 것을 관측하면 유실이 임박한 것으로 본다. circular DMA는
   생산자가 소비자를 앞질러도 아무 신호를 남기지 않으므로, 넘친 뒤가 아니라 **넘치기
   전에** 세는 것이 유일하게 정직한 관측이다. */
#define UART1_RX_HIGH_WATER     (UART1_RX_DMA_SIZE - (UART1_RX_DMA_SIZE / 8U))

/* 최악의 T 줄은 134 B다. 230400 8N1에서 약 5.8 ms이므로 20 ms면 충분한 상한이다.
   여기서 걸리면 링크가 아니라 주변장치가 죽은 것이다. */
#define UART1_TX_TIMEOUT_MS       20U

_Static_assert((UART1_RX_DMA_SIZE & (UART1_RX_DMA_SIZE - 1U)) == 0U,
               "UART1_RX_DMA_SIZE는 2의 거듭제곱이어야 한다");

/* DMA가 소비자와 무관하게 채운다. 컴파일러가 값을 캐싱하지 못하게 volatile로 둔다. */
static volatile uint8_t s_rx_dma[UART1_RX_DMA_SIZE];
static uint16_t s_rx_tail;

static platform_uart1_metrics_t s_metrics;

static bool rx_arm(void)
{
  /* DMA는 항상 버퍼 처음부터 다시 채운다. tail도 같이 되돌리지 않으면 재무장 직후
     쓰레기를 유효 바이트로 읽는다. */
  s_rx_tail = 0U;

  if (HAL_UART_Receive_DMA(&huart1, (uint8_t *)s_rx_dma, UART1_RX_DMA_SIZE) != HAL_OK)
  {
    s_metrics.rearm_fail_count++;
    return false;
  }

  s_metrics.rearm_count++;
  return true;
}

void platform_uart1_reset_metrics(void)
{
  uint32_t baud = s_metrics.baud;

  s_metrics.error_count = 0U;
  s_metrics.error_last = 0U;
  s_metrics.error_pe = 0U;
  s_metrics.error_ne = 0U;
  s_metrics.error_fe = 0U;
  s_metrics.error_ore = 0U;
  s_metrics.error_dma = 0U;
  s_metrics.rearm_count = 0U;
  s_metrics.rearm_fail_count = 0U;
  s_metrics.rx_overflow_count = 0U;
  s_metrics.tx_fail_count = 0U;
  s_metrics.baud = baud;
}

bool platform_uart1_start(void)
{
  uint32_t i;
  bool armed;

  for (i = 0U; i < UART1_RX_DMA_SIZE; i++)
  {
    s_rx_dma[i] = 0U;
  }

  s_metrics.baud = huart1.Init.BaudRate;
  armed = rx_arm();

  /* 처음 거는 것은 "복구"가 아니다. 무장한 **뒤에** 0으로 두어야 rearm_count가
     "링크가 죽어서 되살린 횟수"라는 뜻을 갖는다. */
  platform_uart1_reset_metrics();
  return armed;
}

bool platform_uart1_is_armed(void)
{
  return huart1.RxState == HAL_UART_STATE_BUSY_RX;
}

bool platform_uart1_rearm_if_needed(void)
{
  if (platform_uart1_is_armed())
  {
    return true;
  }

  /* ORE는 SR 다음 DR을 읽어야 지워진다. 서 있는 채로 재무장하면 첫 바이트부터 다시
     오류가 난다. 수신이 이미 끊긴 상태이므로 여기서 DR을 읽어도 삼킬 정상 바이트가 없다. */
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart1);
  }
  huart1.ErrorCode = HAL_UART_ERROR_NONE;

  return rx_arm();
}

size_t platform_uart1_read(uint8_t *dst, size_t max)
{
  uint16_t head;
  uint16_t avail;
  size_t count;
  size_t i;

  if ((huart1.hdmarx == NULL) || (max == 0U))
  {
    return 0U;
  }

  /* NDTR은 SIZE에서 1까지 내려가고 SIZE로 재장전된다. 그래서 head는 [0, SIZE-1]이다. */
  head = (uint16_t)((UART1_RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx)) &
                    (UART1_RX_DMA_SIZE - 1U));
  avail = (uint16_t)((head - s_rx_tail) & (UART1_RX_DMA_SIZE - 1U));

  if (avail >= UART1_RX_HIGH_WATER)
  {
    s_metrics.rx_overflow_count++;
  }

  count = ((size_t)avail < max) ? (size_t)avail : max;
  for (i = 0U; i < count; i++)
  {
    dst[i] = s_rx_dma[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) & (UART1_RX_DMA_SIZE - 1U));
  }

  return count;
}

bool platform_uart1_write(const uint8_t *src, size_t length)
{
  if (HAL_UART_Transmit(&huart1, src, (uint16_t)length,
                        UART1_TX_TIMEOUT_MS) != HAL_OK)
  {
    s_metrics.tx_fail_count++;
    return false;
  }
  return true;
}

bool platform_uart1_set_baud(uint32_t baud)
{
  /* 진행 중인 DMA/IT를 먼저 끊는다. HAL_UART_Init은 gState가 RESET일 때만 MspInit을
     다시 부르므로 GPIO/DMA 배선은 그대로 살아 있다. */
  (void)HAL_UART_Abort(&huart1);

  huart1.Init.BaudRate = baud;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    return false;
  }

  s_metrics.baud = baud;
  return rx_arm();
}

void platform_uart1_metrics(platform_uart1_metrics_t *out)
{
  *out = s_metrics;
}

void platform_uart1_on_error_isr(uint32_t error_code)
{
  /* HAL이 복구 경계에서 ErrorCode를 이미 지운 뒤 부르는 경우가 있다. 0으로 마지막
     유효 진단값을 덮어쓰지 않는다 (USART2 경로와 같은 규칙이다). */
  if (error_code != 0U)
  {
    s_metrics.error_last = error_code;
  }
  s_metrics.error_count++;

  if ((error_code & HAL_UART_ERROR_PE) != 0U)
  {
    s_metrics.error_pe++;
  }
  if ((error_code & HAL_UART_ERROR_NE) != 0U)
  {
    s_metrics.error_ne++;
  }
  if ((error_code & HAL_UART_ERROR_FE) != 0U)
  {
    s_metrics.error_fe++;
  }
  if ((error_code & HAL_UART_ERROR_ORE) != 0U)
  {
    s_metrics.error_ore++;
  }
  if ((error_code & HAL_UART_ERROR_DMA) != 0U)
  {
    s_metrics.error_dma++;
  }

  /* 재무장은 여기서 하지 않는다. HAL은 blocking error에서 HAL_DMA_Abort_IT를 걸고
     그 완료 callback에서 여기로 오기도 하므로, 이 시점의 상태를 믿고 다시 걸면
     조용히 실패한다. 소유 task가 platform_uart1_rearm_if_needed()로 받는다. */
}
