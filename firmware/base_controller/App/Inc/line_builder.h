/**
  ******************************************************************************
  * @file    line_builder.h
  * @brief   "<tag> <v0> <v1> ...\n" 한 줄을 buffer 뒤에서부터 채우는 순수 빌더.
  * @note    HAL/RTOS 비의존. 실제 capacity를 검사하고 부족하면 실패를 반환한다.
  ******************************************************************************
  */
#ifndef LINE_BUILDER_H
#define LINE_BUILDER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
  * @brief  buf의 끝에서부터 앞으로 한 줄을 조립한다.
  * @param  buf         출력 버퍼
  * @param  capacity    buf의 실제 크기
  * @param  tag         줄 머리 태그 ("ticks", "ok", "err", "wd")
  * @param  values      10진수로 쓸 값들
  * @param  count       values의 개수 (0 가능)
  * @param  out_offset  성공 시 줄의 시작 인덱스
  * @param  out_length  성공 시 줄의 길이 ('\n' 포함)
  * @retval 공간이 모자라거나 인자가 잘못되면 false. 실패 시 buf 내용은 쓰레기이며
  *         out_offset/out_length는 갱신되지 않는다 — 호출자는 송신하면 안 된다.
  */
bool line_build(char *buf, size_t capacity, const char *tag,
                const int64_t *values, size_t count,
                size_t *out_offset, size_t *out_length);

#endif /* LINE_BUILDER_H */
