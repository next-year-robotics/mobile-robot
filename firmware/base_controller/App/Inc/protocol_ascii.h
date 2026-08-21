/**
  ******************************************************************************
  * @file    protocol_ascii.h
  * @brief   byte -> frame -> command. discard/resync 상태기계를 포함한다.
  * @note    HAL/RTOS 비의존. M3 ASCII 규약을 그대로 구현한다.
  *
  *            호스트 -> MCU   duty <left_pm> <right_pm>\n   (-1000 ~ +1000)
  *                            stop\n
  *
  *          파싱에 실패한 줄은 명령으로 치지 않는다. 워치독을 갱신하지 않으므로
  *          호스트가 쓰레기를 보내는 동안에도 워치독이 계속 흐른다.
  ******************************************************************************
  */
#ifndef PROTOCOL_ASCII_H
#define PROTOCOL_ASCII_H

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"

typedef enum
{
  PROTO_CMD_NONE = 0,
  PROTO_CMD_DUTY,
  PROTO_CMD_STOP
} proto_cmd_kind_t;

typedef struct
{
  proto_cmd_kind_t kind;
  int32_t          left_pm;   /* 클램프 전 원시값. 클램프는 motor_output이 한다. */
  int32_t          right_pm;
} proto_command_t;

typedef enum
{
  PROTO_EVENT_NONE = 0,  /* 바이트를 더 받아야 한다 / 빈 줄을 흘렸다 */
  PROTO_EVENT_COMMAND,   /* 줄이 끝났고 유효하다 */
  PROTO_EVENT_REJECT     /* 줄이 끝났고 거절이다 -> err */
} proto_event_t;

typedef struct
{
  char     line[CMD_LINE_CAP];
  uint16_t length;
  bool     poisoned;       /* 현재 프레임이 손상됐다 — 개행까지 폐기한다 */
  uint32_t reject_count;
  uint32_t poison_count;
} protocol_t;

void protocol_init(protocol_t *p);

/**
  * @brief  현재 프레임을 손상으로 표시한다.
  * @note   RX 링 포화나 UART FE/NE/PE/ORE처럼 "바이트가 유실됐거나 믿을 수 없다"는
  *         사실이 밖에서 관측됐을 때 부른다. 이후 이 줄은 무슨 내용이 오든 실행되지
  *         않고 다음 개행에서 err로 끝난다. 재동기화 지점은 개행뿐이다.
  */
void protocol_poison(protocol_t *p);

/**
  * @brief  한 바이트를 밀어 넣는다.
  * @param  out  PROTO_EVENT_COMMAND일 때만 채워진다.
  */
proto_event_t protocol_push(protocol_t *p, char c, proto_command_t *out);

/**
  * @brief  '\0'로 끝나는 한 줄을 해석한다(테스트/재사용용 순수 함수).
  */
bool protocol_parse_line(const char *line, proto_command_t *out);

#endif /* PROTOCOL_ASCII_H */
