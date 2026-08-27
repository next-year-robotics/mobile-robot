/**
  ******************************************************************************
  * @file    control_ack.h
  * @brief   적용 결과 -> ACK 판정. 순수 함수.
  * @note    HAL/RTOS 비의존. "명령을 queue에 넣었다"와 "출력이 실제로 반영됐다"를
  *          구분하는 지점이다. 적용되지 않은 duty에 `ok`를 보내지 않게 하는 유일한
  *          판정이다.
  ******************************************************************************
  */
#ifndef CONTROL_ACK_H
#define CONTROL_ACK_H

#include <stdbool.h>
#include <stdint.h>

#include "control_types.h"

typedef enum
{
  CONTROL_ACK_OK = 0,          /* `ok <ms> <left> <right>` */
  CONTROL_ACK_ERR_TIMEOUT,     /* bounded 대기 안에 결과가 오지 않았다 */
  CONTROL_ACK_ERR_SEQ,         /* mailbox overwrite 등으로 다른 seq의 결과가 왔다 */
  CONTROL_ACK_ERR_REJECTED,    /* ControlTask가 적용을 거절했다 */
  CONTROL_ACK_ERR_RX_FAULT     /* 적용 뒤 RX 손상이 출력을 다시 0으로 만들었다 */
} control_ack_t;

/**
  * @brief  ACK를 정한다.
  * @param  received          bounded 대기에서 결과를 받았는가
  * @param  awaited_seq       이 ACK가 기다리던 command sequence
  * @param  result            received가 true일 때만 읽는다
  * @param  rx_fault_pending  결과를 받은 뒤 새 RX fault epoch가 관측됐는가
  * @retval CONTROL_ACK_OK 이외에는 전부 `err`다.
  */
control_ack_t control_ack_decide(bool received, uint32_t awaited_seq,
                                 const control_apply_result_t *result,
                                 bool rx_fault_pending);

#endif /* CONTROL_ACK_H */
