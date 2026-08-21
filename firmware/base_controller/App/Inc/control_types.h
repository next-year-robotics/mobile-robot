/**
  ******************************************************************************
  * @file    control_types.h
  * @brief   ControlTask와 나머지 task가 주고받는 고정 크기 snapshot 타입.
  * @note    HAL/RTOS 비의존. 여기에 FreeRTOS 타입이나 HAL 헤더를 들이지 않는다 —
  *          들어오는 순간 host 단위 테스트가 깨진다.
  *
  *          task 사이의 multi-field/int64 전달은 전부 이 구조체 단위로만 한다.
  *          개별 필드를 volatile로 흩뿌려 동기화하지 않는다.
  ******************************************************************************
  */
#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "motor_output.h"

/**
  * @brief  LegacyIoTask -> ControlTask 명령 snapshot.
  * @note   좌/우 duty, 수락 시각, 기한, 출처 sequence가 한 덩어리로 움직인다.
  *         길이 1 mailbox로 옮기므로 필드가 서로 다른 명령에서 섞일 수 없다.
  */
typedef struct
{
  uint32_t seq;          /* 1부터 단조 증가. 적용 결과 correlation의 유일한 키다. */
  int32_t  left_pm;      /* 클램프 전 원시값. 클램프는 ControlTask가 한다. */
  int32_t  right_pm;
  uint32_t accepted_ms;  /* LegacyIoTask가 줄을 유효로 판정한 시각 */
  uint32_t deadline_ms;  /* accepted_ms + CMD_WATCHDOG_MS. 넘겨 도착하면 거절이다. */
  bool     is_stop;
} control_command_t;

typedef enum
{
  CONTROL_APPLY_NONE = 0,
  CONTROL_APPLY_OK,       /* 이 seq의 duty가 실제로 CCR에 반영됐다 */
  CONTROL_APPLY_REJECTED  /* fault latch 또는 기한 초과로 반영되지 않았다 */
} control_apply_kind_t;

/**
  * @brief  ControlTask -> LegacyIoTask 적용 결과.
  * @note   `ok`는 이 결과가 CONTROL_APPLY_OK이고 seq가 일치할 때만 나간다.
  *         "queue에 넣었다"는 사실은 ACK 근거가 아니다.
  */
typedef struct
{
  uint32_t             seq;
  control_apply_kind_t kind;
  uint32_t             applied_ms;
  int32_t              left_pm;   /* 클램프 뒤 실제 적용값 */
  int32_t              right_pm;
} control_apply_result_t;

/**
  * @brief  ControlTask가 게시하는 상태 snapshot.
  */
typedef struct
{
  uint32_t timestamp_ms;
  int64_t  ticks_left;        /* 누적 틱. 50 Hz telemetry의 의미를 그대로 유지한다. */
  int64_t  ticks_right;
  int32_t  applied_left_pm;   /* 현재 지령 duty (워치독 만료 시에도 남는다) */
  int32_t  applied_right_pm;
  uint32_t applied_seq;       /* 마지막으로 출력에 반영된 command sequence */
  uint32_t fault_flags;       /* MOTOR_FAULT_* 래치 */
  uint32_t wd_seq;            /* 워치독 만료 edge 카운터 (단조) */
  uint32_t wd_ms;             /* 마지막 만료 edge를 관측한 시각 */
  uint32_t heartbeat;         /* control cycle마다 +1. HealthTask가 관측한다. */
  uint32_t loop_count;
  uint32_t loop_overruns;
  bool     output_allowed;    /* 지금 duty가 CCR에 실릴 수 있는가 */
} control_status_t;

/**
  * @brief  ControlTask가 만들어 낸 출력 지시. 레지스터 기록은 호출자가 한다.
  * @note   순수 모듈이 HAL을 부르지 않게 하는 유일한 장치다.
  */
typedef enum
{
  CONTROL_OUT_WRITE = 0,  /* platform_motor_write(&ccr) */
  CONTROL_OUT_KILL        /* platform_motor_kill() — 래치된 fault */
} control_out_kind_t;

typedef struct
{
  control_out_kind_t kind;
  motor_ccr_t        ccr;
} control_out_t;

#endif /* CONTROL_TYPES_H */
