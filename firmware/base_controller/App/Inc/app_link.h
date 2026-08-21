/**
  ******************************************************************************
  * @file    app_link.h
  * @brief   LegacyIoTask <-> ControlTask 사이의 얇은 seam.
  * @note    인터페이스에 FreeRTOS 타입이 나오지 않는다. 펌웨어에서는
  *          App/Src/rtos_app.c가 static queue/notification으로 구현하고,
  *          host 테스트는 같은 이름의 fake로 구현한다 — platform_stm32.h와 같은 규칙이다.
  ******************************************************************************
  */
#ifndef APP_LINK_H
#define APP_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "control_types.h"

/**
  * @brief  명령 snapshot을 길이 1 mailbox에 게시하고 ControlTask를 깨운다.
  * @note   overwrite다. 아직 소비되지 않은 이전 명령은 사라지며, 그 seq의 결과는
  *         영영 오지 않는다 — 호출자는 seq 불일치를 err로 처리해야 한다.
  * @retval 게시하지 못했으면 false.
  */
bool app_link_post_command(const control_command_t *cmd);

/**
  * @brief  해당 seq의 적용 결과를 **상한 있는 시간** 동안 기다린다.
  * @retval 시간 안에 결과를 하나라도 받았으면 true (seq 일치 여부는 호출자가 본다).
  */
bool app_link_wait_applied(uint32_t seq, uint32_t timeout_ms,
                           control_apply_result_t *out);

/**
  * @brief  ControlTask가 게시한 최신 상태 snapshot을 읽는다(소비하지 않는다).
  */
bool app_link_get_status(control_status_t *out);

/**
  * @brief  task 문맥의 긴급 정지 요청. ControlTask가 즉시 선점해 출력을 0으로 만든다.
  * @param  fault_flags  MOTOR_FAULT_NONE이면 래치 없는 복구 가능 정지다.
  */
void app_link_urgent_stop(uint32_t fault_flags);

/**
  * @brief  ISR 문맥의 긴급 정지 요청.
  * @note   ISR은 CCR을 직접 쓰지 않는다. 최고 우선순위 ControlTask를 깨우고
  *         yield만 예약한다.
  */
void app_link_urgent_stop_from_isr(uint32_t fault_flags);

/**
  * @brief  ISR이 LegacyIoTask를 깨운다(수신 프레임 경계).
  */
void app_link_notify_io_from_isr(void);

#endif /* APP_LINK_H */
