/**
  ******************************************************************************
  * @file    control_core.h
  * @brief   100 Hz ControlTask 한 cycle의 순수 상태기계.
  * @note    HAL/RTOS 비의존. 레지스터를 만지지 않고 "무엇을 써야 하는지"만 만든다.
  *          개루프 duty 경로와 속도 폐루프를 모드로 나란히 갖는다.
  *          FF/PI 자체는 speed_control.c에 있고 여기서는 표본을 만들고 언제
  *          적분을 진행할지를 정한다.
  ******************************************************************************
  */
#ifndef CONTROL_CORE_H
#define CONTROL_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "control_types.h"
#include "encoder_accumulator.h"
#include "motor_safety.h"
#include "speed_control.h"

/**
  * @brief  좌우 계수. app_config.h의 확정값으로 초기화된다. 튜닝 중 SWD로 덮어쓸 수
  *         있도록 const가 아니다.
  * @note   확정된 값은 반드시 app_config.h로 되돌려 커밋한다. SWD로만 존재하는
  *         게인은 재현할 수 없다.
  */
extern speed_gains_t g_speed_gains_left;
extern speed_gains_t g_speed_gains_right;

typedef struct
{
  motor_safety_t  safety;
  encoder_accum_t enc_left;
  encoder_accum_t enc_right;
  speed_control_t spd_left;
  speed_control_t spd_right;
  /* 이번 표본의 측정 속도. tick_delta로 나눈 뒤의 값이다. */
  float    measured_tps_left;
  float    measured_tps_right;

  /* 이번 cycle에 CCR로 나갈 duty(클램프 후). 개루프에서는 호스트 지령이고
     폐루프에서는 FF+PI의 출력이다. */
  int32_t  cmd_left_pm;
  int32_t  cmd_right_pm;
  /* 워치독/fault로 출력이 막히면 0이 된다. telemetry가 싣는 값은 이쪽이다. */
  int32_t  out_left_pm;
  int32_t  out_right_pm;
  bool     closed_loop;         /* 마지막 duty/vel 지령이 정한 모드 */
  uint32_t applied_seq;

  uint32_t tick_seq_seen;       /* 마지막으로 처리한 TIM6 tick sequence */
  uint32_t loop_count;
  uint32_t loop_overruns;       /* 병합되어 놓친 tick 수 (누적) */
  uint32_t urgent_stop_count;
  uint32_t stale_command_count;
  uint32_t heartbeat;
  uint32_t wd_seq;
  uint32_t wd_ms;
  bool     output_allowed;
} control_core_t;

/**
  * @brief  초기화. 엔코더는 현재 카운터를 기준으로 잡아 첫 델타가 0이 되게 한다.
  * @note   호출 시점에 엔코더 타이머가 이미 돌고 있어야 한다.
  */
void control_core_init(control_core_t *c, uint32_t now_ms,
                       uint16_t enc_left_raw, uint16_t enc_right_raw);

/**
  * @brief  TIM6 100 Hz tick 처리.
  * @param  tick_seq  ISR이 올린 sequence의 snapshot. 직전 값과의 차이로 병합/누락을
  *                   검출한다(부호 없는 뺄셈이라 wrap-safe다).
  * @note   워치독 평가가 100 Hz로 양자화되므로 정지는 마지막 유효 명령 뒤
  *         CMD_WATCHDOG_MS ~ CMD_WATCHDOG_MS + CONTROL_PERIOD_MS 사이에 일어난다.
  *
  *         속도 추정은 tick_delta로 나눈다.
  *           measured_tps = delta_ticks * CONTROL_RATE_HZ / tick_delta
  *         병합된 tick에서는 출력만 다시 계산하고 적분기는 진행시키지 않는다. 긴 구간의
  *         평균 하나로 과거의 여러 제어 출력을 재현할 수 없기 때문이다.
  */
void control_core_on_tick(control_core_t *c, uint32_t now_ms, uint32_t tick_seq,
                          uint16_t enc_left_raw, uint16_t enc_right_raw,
                          control_out_t *out);

/**
  * @brief  명령 하나를 적용한다. 결과는 반드시 seq와 함께 돌려준다.
  * @note   fault 래치나 기한 초과면 duty를 반영하지 않고 CONTROL_APPLY_REJECTED다.
  *         호출자는 이 결과가 OK일 때만 `ok`를 보낼 수 있다.
  *
  *         폐루프에서는 새 FF + 새 P + 기존 I로 즉시 출력을 다시 계산한다. `Ki_cycle
  *         * error`를 더하지는 않는다. 진행되는 적분의 dT는 정확히 한 주기로
  *         고정한다. 그래야 명령이 비주기적으로 들어와도 적분 이득이 지령 주기에 따라
  *         달라지지 않는다.
  */
void control_core_on_command(control_core_t *c, uint32_t now_ms,
                             const control_command_t *cmd,
                             control_apply_result_t *result, control_out_t *out);

/**
  * @brief  긴급 정지. 지령 duty를 0으로 지우고 필요하면 fault를 래치한다.
  * @param  fault_flags  MOTOR_FAULT_NONE이면 래치 없이 출력만 0으로 만든다
  *                      (RX 손상처럼 개행 재동기화로 복구 가능한 경우).
  */
void control_core_on_stop(control_core_t *c, uint32_t now_ms, uint32_t fault_flags,
                          control_out_t *out);

/**
  * @brief  현재 상태를 고정 크기 snapshot으로 만든다.
  */
void control_core_status(const control_core_t *c, uint32_t now_ms,
                         control_status_t *out);

#endif /* CONTROL_CORE_H */
