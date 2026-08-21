#include "protocol_ascii.h"

#include <string.h>

/* per-mille 지령이라 여섯 자리를 넘길 일이 없다. 넘치면 실패로 본다. */
#define PARSE_MAX_DIGITS  6

/**
  * @brief  공백을 건너뛰고 10진 정수 하나를 읽는다.
  * @retval 숫자가 없거나 자릿수가 넘치면 false.
  */
static bool parse_i32(const char **cursor, int32_t *out)
{
  const char *p = *cursor;
  int32_t value = 0;
  int32_t sign = 1;
  int digits = 0;

  while (*p == ' ')
  {
    p++;
  }

  if (*p == '-')
  {
    sign = -1;
    p++;
  }
  else if (*p == '+')
  {
    p++;
  }
  else
  {
    /* 부호 없음 */
  }

  while ((*p >= '0') && (*p <= '9'))
  {
    if (digits >= PARSE_MAX_DIGITS)
    {
      return false;
    }
    value = (value * 10) + (int32_t)(*p - '0');
    p++;
    digits++;
  }

  if (digits == 0)
  {
    return false;
  }

  *out = sign * value;
  *cursor = p;
  return true;
}

bool protocol_parse_line(const char *line, proto_command_t *out)
{
  const char *p = line;
  int32_t left = 0;
  int32_t right = 0;

  out->kind = PROTO_CMD_NONE;
  out->left_pm = 0;
  out->right_pm = 0;

  if ((strncmp(p, "duty", 4) == 0) && ((p[4] == ' ') || (p[4] == '\0')))
  {
    p += 4;
    if (!parse_i32(&p, &left) || !parse_i32(&p, &right))
    {
      return false;
    }

    /* 꼬리 문자까지 검사한다. "duty 1 2 3"은 거절이다. */
    while (*p == ' ')
    {
      p++;
    }
    if (*p != '\0')
    {
      return false;
    }

    out->kind = PROTO_CMD_DUTY;
    out->left_pm = left;
    out->right_pm = right;
    return true;
  }

  if (strcmp(p, "stop") == 0)
  {
    out->kind = PROTO_CMD_STOP;
    return true;
  }

  return false;
}

void protocol_init(protocol_t *p)
{
  p->length = 0U;
  p->poisoned = false;
  p->reject_count = 0U;
  p->poison_count = 0U;
  p->line[0] = '\0';
}

void protocol_poison(protocol_t *p)
{
  if (!p->poisoned)
  {
    p->poisoned = true;
    p->poison_count++;
  }
  /* 이미 모은 조각은 믿을 수 없다. 개행이 올 때까지 아무것도 모으지 않는다. */
  p->length = 0U;
}

proto_event_t protocol_push(protocol_t *p, char c, proto_command_t *out)
{
  if ((c == '\n') || (c == '\r'))
  {
    bool poisoned = p->poisoned;
    uint16_t length = p->length;

    /* 개행이 유일한 재동기화 지점이다. */
    p->poisoned = false;
    p->length = 0U;

    if (poisoned)
    {
      p->reject_count++;
      return PROTO_EVENT_REJECT;
    }

    if (length == 0U)
    {
      /* 빈 줄과 CRLF의 뒤쪽 문자를 무시한다. */
      return PROTO_EVENT_NONE;
    }

    p->line[length] = '\0';
    if (protocol_parse_line(p->line, out))
    {
      return PROTO_EVENT_COMMAND;
    }

    p->reject_count++;
    return PROTO_EVENT_REJECT;
  }

  if (p->poisoned)
  {
    return PROTO_EVENT_NONE;
  }

  if (p->length >= (CMD_LINE_CAP - 1U))
  {
    /* 줄이 버퍼보다 길다. 이 줄은 끝까지 버린다. */
    protocol_poison(p);
    return PROTO_EVENT_NONE;
  }

  p->line[p->length] = c;
  p->length++;
  return PROTO_EVENT_NONE;
}
