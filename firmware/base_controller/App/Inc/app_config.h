/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   M3 개루프 펌웨어의 확정 상수. HAL/RTOS에 의존하지 않는다.
  * @note    이 헤더는 순수 모듈과 STM32 adapter가 함께 쓴다. 여기에 HAL 헤더나
  *          main.h를 들이지 않는다 — 들어오는 순간 host 단위 테스트가 깨진다.
  ******************************************************************************
  */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* ---- 주기 (super-loop) ---- */
#define LD2_TOGGLE_PERIOD_MS      500U
#define ENCODER_SAMPLE_PERIOD_MS   10U
#define TICKS_REPORT_PERIOD_MS     20U  /* 50 Hz */

/* ---- 엔코더 ---- */

/* 로봇 전진 시 좌우 누적 틱이 모두 +가 되게 하는 보정. 2026-08-20 M2 실측:
   왼쪽 원시 부호 +, 오른쪽 원시 부호 -. 두 바퀴가 마주 보게 달려 있어 생기는
   차이이므로 배선이 아니라 여기서 잡는다. */
#define ENCODER_SIGN_LEFT         (+1)
#define ENCODER_SIGN_RIGHT        (-1)

/* ---- 모터 ---- */

/* TIM1 ARR. .ioc의 TIM1.Period와 반드시 같아야 하며 platform_init()에서 대조한다. */
#define MOTOR_PWM_ARR             4666U

/* duty 지령 단위는 per-mille 정수다. MCU에서 부동소수 파싱을 하지 않는다. */
#define MOTOR_DUTY_MAX_PM         1000

/* 양수 duty = 로봇 전진이 되게 하는 보정. 두 바퀴가 마주 보게 달려 있어 한쪽이
   -1이다. M3.md 5.3에서 실측 확정 (2026-08-21): 좌 +100 지령에 후진(-622 tps),
   우 +100 지령에 전진. 배선을 바꾸지 않고 여기서 잡는다. */
#define MOTOR_SIGN_LEFT           (-1)
#define MOTOR_SIGN_RIGHT          (+1)

/* ---- 명령 워치독 ---- */

/* 마지막 유효 명령으로부터 이 시간이 지나면 duty 0. 호스트 툴이 죽거나 USB가
   빠져도 모터가 계속 돌지 않게 한다. limits.md의 확정값 200 ms는 /cmd_vel 주기
   기준이며 M4/M5에서 적용한다. M3는 사람이 개입하는 수동 시험이라 500 ms다. */
#define CMD_WATCHDOG_MS           500U

/* ---- 통신 버퍼 ---- */
#define CMD_RX_RING_SIZE          64U  /* 2의 거듭제곱이어야 한다 */
#define CMD_LINE_CAP              32U
#define TX_LINE_CAP               64U

/* 한 번의 super-loop에서 소비할 RX 바이트/줄 상한. 호스트가 쓰레기를 최고속으로
   퍼부어도 command_poll이 워치독 평가를 굶기지 못하게 하는 유일한 장치다.
   230400 8N1에서 64바이트는 약 2.8 ms 분량이라 500 ms 워치독에 여유가 크다. */
#define CMD_POLL_MAX_BYTES        64U
#define CMD_POLL_MAX_LINES         4U

/* 블로킹 송신의 유한 타임아웃. 최악의 줄 59 B는 230400 8N1에서 약 2.6 ms다.
   여기서 걸리면 주변장치가 죽은 것으로 보고 fault를 래치한다. */
#define UART_TX_TIMEOUT_MS         10U

/* 수신 재무장이 이만큼 연속 실패하면 링크가 죽은 것으로 보고 fault를 래치한다. */
#define CMD_RX_REARM_FAIL_LIMIT     8U

/* ---- 출력 줄 최악 길이 ---- */

/* 가장 긴 태그는 "ticks"(5). 타임스탬프는 uint32(최대 10자리), 누적 틱은
   int64(부호 포함 최대 20자). 값마다 앞에 공백 하나가 붙고 줄 끝에 '\n'이 온다. */
#define LINE_MAX_TAG_LEN           5U
#define LINE_U32_MAX_CHARS        10U
#define LINE_I64_MAX_CHARS        20U
#define TX_WORST_LINE_LEN         (LINE_MAX_TAG_LEN                     \
                                   + (1U + LINE_U32_MAX_CHARS)          \
                                   + (2U * (1U + LINE_I64_MAX_CHARS))   \
                                   + 1U)

/* ---- 불변식 ---- */
_Static_assert((CMD_RX_RING_SIZE & (CMD_RX_RING_SIZE - 1U)) == 0U,
               "CMD_RX_RING_SIZE는 2의 거듭제곱이어야 한다 (인덱스를 & 로 감는다)");
_Static_assert(CMD_RX_RING_SIZE >= 2U && CMD_RX_RING_SIZE <= 0x8000U,
               "CMD_RX_RING_SIZE가 uint16 인덱스 범위를 벗어난다");
_Static_assert(CMD_LINE_CAP >= 16U,
               "CMD_LINE_CAP이 'duty -1000 -1000' 을 담지 못한다");
_Static_assert(TX_LINE_CAP >= TX_WORST_LINE_LEN,
               "TX_LINE_CAP이 최악의 ticks 줄보다 작다");
_Static_assert((MOTOR_SIGN_LEFT == 1) || (MOTOR_SIGN_LEFT == -1),
               "MOTOR_SIGN_LEFT는 +1 또는 -1이어야 한다");
_Static_assert((MOTOR_SIGN_RIGHT == 1) || (MOTOR_SIGN_RIGHT == -1),
               "MOTOR_SIGN_RIGHT는 +1 또는 -1이어야 한다");
_Static_assert((ENCODER_SIGN_LEFT == 1) || (ENCODER_SIGN_LEFT == -1),
               "ENCODER_SIGN_LEFT는 +1 또는 -1이어야 한다");
_Static_assert((ENCODER_SIGN_RIGHT == 1) || (ENCODER_SIGN_RIGHT == -1),
               "ENCODER_SIGN_RIGHT는 +1 또는 -1이어야 한다");
_Static_assert(MOTOR_DUTY_MAX_PM > 0, "MOTOR_DUTY_MAX_PM은 양수여야 한다");
_Static_assert(CMD_POLL_MAX_BYTES > 0U && CMD_POLL_MAX_LINES > 0U,
               "command_poll 상한이 0이면 명령이 영영 처리되지 않는다");

#endif /* APP_CONFIG_H */
