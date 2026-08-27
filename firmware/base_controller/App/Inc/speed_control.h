/**
  ******************************************************************************
  * @file    speed_control.h
  * @brief   바퀴 하나의 속도 폐루프 — 피드포워드 + PI + 조건부 적분(anti-windup).
  * @note    HAL/RTOS 비의존. 레지스터도 시간도 만지지 않는다. 한 cycle의 입력
  *          (목표 tps, 측정 tps, "이번이 정상 cycle인가")만 받아 duty를 만든다.
  *
  *          연산은 `float`다. 회귀 계수를 1:1로 적을 수 있고 Q 포맷 스케일이 코드에
  *          퍼지지 않는다. 다만 와이어에는 float가 나가지 않는다. 출력은 정수 per-mille다.
  *
  *          구현하는 식은 이렇다.
  *            duty_ff  = run_intercept * sign(target) + kff * target   (target != 0)
  *            error    = target - measured
  *            I_next   = I + ki_cycle * error          (정상 cycle에서만)
  *            duty_raw = duty_ff + kp * error + I
  *            duty     = clamp(duty_raw, +-duty_max_pm)
  ******************************************************************************
  */
#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/**
  * @brief  바퀴 하나의 확정 계수. 좌우가 각자 자기 값을 가진다.
  * @note   `deadband_*`는 여기 들어오지 않는다. 정지마찰과 운동마찰은 다른
  *         물리량이다. 전자를 상시 피드포워드에 넣으면 상수 바이어스가 전 운전점에
  *         걸려 적분기가 늘 그것을 상쇄한다.
  */
typedef struct
{
  float   kff;            /* ‰/tps — duty-tps 회귀 기울기의 역수 */
  float   run_intercept;  /* ‰ — 같은 회귀의 x절편(운동마찰) */
  float   kp;             /* ‰/tps */
  float   ki_cycle;       /* ‰/tps — 정상 cycle(CONTROL_PERIOD_MS) 하나당 */
  float   i_max_pm;       /* ‰ — 적분기 절댓값 상한 */
  int32_t duty_max_pm;    /* ‰ — 출력 클램프 */
} speed_gains_t;

typedef struct
{
  float   integrator_pm;  /* ‰. 적분은 정상 cycle에서만 진행한다. */
  int32_t target_tps;     /* 클램프가 끝난 목표 */
  int32_t duty_pm;        /* 마지막 출력(클램프 뒤) */
  bool    saturated;      /* 클램프 전 duty가 범위를 벗어났는가 */
} speed_control_t;

/**
  * @brief  전 필드를 0으로 만든다.
  */
void speed_control_init(speed_control_t *s);

/**
  * @brief  적분기와 출력을 지운다. 목표는 건드리지 않는다.
  * @note   코스팅(목표 0), 워치독 만료, stop, urgent stop, 모드 전환에서 부른다.
  */
void speed_control_reset(speed_control_t *s);

/**
  * @brief  목표를 갱신한다. 부호가 뒤집히면 적분기를 먼저 지운다.
  * @note   이전 방향에서 쌓인 I가 작은 반대 방향 지령의 FF+P를 이겨 잠시 기존
  *         방향 duty를 내는 일을 막는다. 같은 부호의 갱신과 주기 refresh에서는
  *         적분기를 유지한다.
  */
void speed_control_set_target(speed_control_t *s, int32_t target_tps);

/**
  * @brief  피드포워드 항만 계산한다(테스트/검증용 순수 함수).
  */
float speed_control_feedforward(const speed_gains_t *g, int32_t target_tps);

/**
  * @brief  한 표본을 반영해 duty를 다시 만든다.
  * @param  measured_tps        이번 표본의 측정 속도
  * @param  advance_integrator  정상 cycle(tick_delta == 1)에서만 true.
  *                             명령 도착 경로와 병합 tick에서는 false다 —
  *                             진행되는 적분의 dT를 정확히 한 주기로 고정한다.
  * @retval 클램프 뒤 duty (per-mille). `s->saturated`는 클램프 전 값 기준이다.
  * @note   목표가 0이면 PI를 돌리지 않고 duty 0 + 적분기 리셋이다. 관성으로 굴러가는
  *         바퀴에 역방향 duty를 걸지 않는다. 정지는 코스팅이다.
  */
int32_t speed_control_update(speed_control_t *s, const speed_gains_t *g,
                             float measured_tps, bool advance_integrator);

#endif /* SPEED_CONTROL_H */
