/**
  ******************************************************************************
  * @file    motor_safety.h
  * @brief   명령 워치독(wrap-safe timeout)과 fault latch. 순수 상태기계.
  * @note    HAL/RTOS 비의존. 실제 출력 차단은 호출자가 이 판정을 보고 수행한다.
  ******************************************************************************
  */
#ifndef MOTOR_SAFETY_H
#define MOTOR_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

/* 래치되는 fault 원인. 한 번 서면 리셋 전까지 내려가지 않는다. */
#define MOTOR_FAULT_NONE      0x00U
#define MOTOR_FAULT_TX        0x01U  /* 유한 타임아웃 안에 송신이 끝나지 않았다 */
#define MOTOR_FAULT_RX_REARM  0x02U  /* 수신 재무장이 연속 실패했다 */
#define MOTOR_FAULT_TX_BUILD  0x04U  /* 출력 줄이 버퍼에 들어가지 않았다 */

typedef struct
{
  uint32_t last_cmd_ms;
  uint32_t timeout_ms;
  uint32_t fault_flags;
  bool     timed_out;
  bool     timeout_notify;
} motor_safety_t;

/**
  * @brief  초기화. 첫 명령 전에는 이미 만료된 상태로 두되 통지는 띄우지 않는다.
  * @note   부호 없는 뺄셈이라 now에서 timeout을 빼두면 (now - last)가 곧바로
  *         만료로 계산된다. 부팅 직후 wd 한 줄이 뜨는 것을 막으면서 출력은 0이다.
  */
void motor_safety_init(motor_safety_t *s, uint32_t now_ms, uint32_t timeout_ms);

/**
  * @brief  유효 명령을 받았다고 알린다. fault가 래치된 뒤에는 무시한다.
  */
void motor_safety_feed(motor_safety_t *s, uint32_t now_ms);

/**
  * @brief  fault를 래치한다. 두 번 호출해도 결과가 같다.
  */
void motor_safety_latch_fault(motor_safety_t *s, uint32_t flag);

/**
  * @brief  래치된 fault가 있는가.
  */
bool motor_safety_faulted(const motor_safety_t *s);

/**
  * @brief  지금 모터 출력을 허용해도 되는가. 만료 전이를 여기서 기록한다.
  * @note   uint32 tick이 감겨도 (now - last)의 부호 없는 뺄셈이 성립한다.
  */
bool motor_safety_output_allowed(motor_safety_t *s, uint32_t now_ms);

/**
  * @brief  워치독 만료 통지를 한 번만 꺼내 간다("wd" 한 줄을 위한 edge).
  */
bool motor_safety_take_notify(motor_safety_t *s);

#endif /* MOTOR_SAFETY_H */
