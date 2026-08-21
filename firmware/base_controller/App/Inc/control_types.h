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
  * @brief  ControlTask가 받는 지령의 종류.
  * @note   duty와 vel은 **모드**다. 한쪽으로 지령하면 그 모드가 유지되며, 모드가
  *         바뀌는 순간 적분기를 리셋한다. stop은 모드를 바꾸지 않고 목표만 0으로
  *         만든다 (M4.md 4.1).
  */
typedef enum
{
  CONTROL_CMD_STOP = 0,  /* 양쪽 목표 0. 모드는 유지한다. */
  CONTROL_CMD_DUTY,      /* 개루프. left/right는 per-mille duty다. */
  CONTROL_CMD_VEL        /* 폐루프. left/right는 목표 tps다. */
} control_cmd_kind_t;

/**
  * @brief  LegacyIoTask -> ControlTask 명령 snapshot.
  * @note   좌/우 값, 수락 시각, 기한, 출처 sequence가 한 덩어리로 움직인다.
  *         길이 1 mailbox로 옮기므로 필드가 서로 다른 명령에서 섞일 수 없다.
  */
typedef struct
{
  uint32_t           seq;      /* 1부터 단조 증가. 적용 결과 correlation의 유일한 키다. */
  control_cmd_kind_t kind;
  int32_t            left;     /* 클램프 전 원시값. duty면 ‰, vel이면 tps. */
  int32_t            right;
  uint32_t           accepted_ms;  /* LegacyIoTask가 줄을 유효로 판정한 시각 */
  uint32_t           deadline_ms;  /* accepted_ms + CMD_WATCHDOG_MS. 넘겨 도착하면 거절 */
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
  *
  *         `left`/`right`는 **ACK에 실릴 값**이며 단위는 명령 종류를 따른다:
  *         duty면 클램프 뒤 ‰, vel이면 클램프 뒤 목표 tps, stop이면 0이다.
  *         `ok <ms> <a> <b>` 형식을 바꾸지 않기 위한 것이다 — 호스트 툴의
  *         "보낸 값과 ok의 값이 다르면 즉시 정지" 검사를 명령마다 따로 쓰지 않는다.
  */
typedef struct
{
  uint32_t             seq;
  control_apply_kind_t kind;
  uint32_t             applied_ms;
  int32_t              left;   /* 클램프 뒤 값. 단위는 명령 종류를 따른다. */
  int32_t              right;
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

  /* ---- 속도 폐루프 (M4) ---- */

  /* 마지막으로 워치독을 먹인 시각. MCU 내부 만료 판정을 SWD로 확인하는 기준이다.
     호스트 wall-clock에는 UART와 스케줄링 지연이 더해지므로 그쪽으로 판정하지 않는다. */
  uint32_t last_feed_ms;

  int32_t  target_tps_left;    /* 클램프 뒤 목표. 개루프에서는 0이다. */
  int32_t  target_tps_right;
  int32_t  measured_tps_left;  /* 이번 표본의 측정 속도(반올림). 와이어는 정수다. */
  int32_t  measured_tps_right;
  int32_t  duty_left_pm;       /* 이번 cycle에 **실제로 CCR에 실린** duty */
  int32_t  duty_right_pm;
  /* 적분기 값. rtos_metrics/telemetry를 정수로 유지하기 위해 ‰ x 1000으로 싣는다. */
  int32_t  integrator_left_mpm;
  int32_t  integrator_right_mpm;
  bool     saturated_left;     /* 클램프 전 duty가 범위를 벗어났는가 */
  bool     saturated_right;
  bool     closed_loop;        /* 현재 모드 */
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
