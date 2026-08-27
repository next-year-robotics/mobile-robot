/**
  ******************************************************************************
  * @file    u3_smoke.h
  * @brief   U3 양방향 smoke test의 줄 규약과 계수기. HAL/RTOS 비의존 순수 모듈.
  * @note    micro-ROS를 얹기 전에 USART1 링크 자체를 검증한다. 제어·IWDG 경로는
  *          건드리지 않는다.
  *
  *            호스트 -> MCU   P <seq>\n      순번 있는 ping. seq는 uint32
  *                            S <mask>\n     bit0 = T 스트림, bit1 = R 에코
  *                            Z\n            수신 계수기와 순번 기준선 초기화
  *                            B <baud>\n     USART1 보드레이트 전환 (스윕용)
  *                            N <ms>\n       T 스트림 주기 = 부하 조절 (스윕용)
  *
  *            MCU -> 호스트   T <12개 값>\n  50 Hz 상태 스트림
  *                            R <seq>\n      P에 대한 같은 순번 에코
  *                            B <baud>\n     보드레이트 전환 ACK (전환 전 속도로)
  *
  *          누락·중복 판정은 MCU가 한다. 에코가 돌아오는 경로에서 또 유실될 수 있으므로
  *          Pi -> MCU 방향의 정직한 측정값은 MCU 안의 계수기뿐이다.
  ******************************************************************************
  */
#ifndef U3_SMOKE_H
#define U3_SMOKE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 가장 긴 수신 줄은 "B 3000000"(9자)이다. 여유를 두고 32로 잡는다. */
#define U3_RX_LINE_CAP            32U

/* T 줄의 값 개수. 전부 uint32라 값 하나의 최악 폭은 10자 + 앞 공백이다. */
#define U3_T_VALUE_COUNT          12U
#define U3_TX_WORST_LINE_LEN      (1U + (U3_T_VALUE_COUNT * 11U) + 1U)
#define U3_TX_LINE_CAP           160U

_Static_assert(U3_TX_LINE_CAP >= U3_TX_WORST_LINE_LEN,
               "U3_TX_LINE_CAP이 최악의 T 줄보다 작다");

/* T 스트림 주기의 허용 범위. 하한은 SmokeTask poll 주기, 상한은 1초다. 0을 허용하면
   주기가 아니라 "쉬지 않고 보내기"가 되어 판정 기준이 사라진다. */
#define U3_PERIOD_MIN_MS           1U
#define U3_PERIOD_MAX_MS        1000U

#define U3_MODE_STREAM          0x01U   /* T 스트림 */
#define U3_MODE_ECHO            0x02U   /* P -> R 에코 */
#define U3_MODE_ALL             (U3_MODE_STREAM | U3_MODE_ECHO)

typedef enum
{
  U3_EVENT_NONE = 0,   /* 줄이 아직 끝나지 않았다 / 빈 줄을 흘렸다 */
  U3_EVENT_PING,       /* 유효한 P — out_value에 seq가 들어간다 */
  U3_EVENT_MODE,       /* S 처리 완료 — out_value에 새 mask가 들어간다 */
  U3_EVENT_RESET,      /* Z 처리 완료 */
  U3_EVENT_BAUD,       /* B 처리 요청 — out_value에 목표 baud가 들어간다 */
  U3_EVENT_PERIOD,     /* N 처리 요청 — out_value에 새 T 주기[ms]가 들어간다 */
  U3_EVENT_BAD         /* 형식이 깨진 줄 */
} u3_event_t;

typedef struct
{
  uint32_t rx_ping;    /* 유효한 P 줄 수 */
  uint32_t rx_bad;     /* 형식이 깨진 줄 수 (= 문자 깨짐의 관측값) */
  uint32_t rx_gap;     /* 건너뛴 순번의 개수 합 */
  uint32_t rx_dup;     /* 순번이 되돌아온 줄 수 (중복/역전) */
} u3_counters_t;

typedef struct
{
  char     line[U3_RX_LINE_CAP];
  uint16_t length;
  bool     overlong;      /* 이 줄은 cap을 넘었다 — 개행까지 버리고 bad로 센다 */
  bool     have_seq;      /* 순번 기준선이 잡혔는가 */
  uint32_t expect_seq;    /* 다음에 올 것으로 기대하는 P 순번 */
  uint8_t  mode;
  u3_counters_t counters;
} u3_smoke_t;

/** @brief mode를 U3_MODE_ALL로 두고 시작한다. 꽂자마자 T가 나와야 관측이 쉽다. */
void u3_smoke_init(u3_smoke_t *s);

/**
  * @brief  한 바이트를 밀어 넣는다.
  * @param  out_value  PING/MODE/BAUD일 때만 채워진다.
  * @note   '\r'은 무시하고 '\n'에서만 줄을 판정한다. 빈 줄은 사건이 아니다.
  */
u3_event_t u3_smoke_push(u3_smoke_t *s, char c, uint32_t *out_value);

uint8_t u3_smoke_mode(const u3_smoke_t *s);
void u3_smoke_counters(const u3_smoke_t *s, u3_counters_t *out);

#endif /* U3_SMOKE_H */
