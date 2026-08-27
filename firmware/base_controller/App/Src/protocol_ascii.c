#include "protocol_ascii.h"

#include <stddef.h>
#include <string.h>

/* per-mille 지령(<= 4자리)과 tps 지령(<= 4자리)이라 여섯 자리를 넘길 일이 없다.
   넘치면 실패로 본다. */
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

/**
  * @brief  남은 꼬리가 공백뿐인지 검사한다.
  * @note   "duty 1 2 3"을 거절하는 지점이다. 값을 다 읽었다고 줄이 유효하지는 않다.
  */
static bool at_end(const char *p)
{
  while (*p == ' ')
  {
    p++;
  }
  return *p == '\0';
}

/** @brief "<tag> " 또는 "<tag>" 로 시작하는지 본다. */
static bool head_is(const char *p, const char *tag, size_t tag_len)
{
  return (strncmp(p, tag, tag_len) == 0) &&
         ((p[tag_len] == ' ') || (p[tag_len] == '\0'));
}

bool protocol_parse_line(const char *line, proto_command_t *out)
{
  const char *p = line;
  int32_t left = 0;
  int32_t right = 0;

  out->kind = PROTO_CMD_NONE;
  out->left = 0;
  out->right = 0;

  if (head_is(p, "duty", 4U) || head_is(p, "vel", 3U))
  {
    bool is_vel = (*p == 'v');

    p += is_vel ? 3 : 4;
    if (!parse_i32(&p, &left) || !parse_i32(&p, &right) || !at_end(p))
    {
      return false;
    }

    out->kind = is_vel ? PROTO_CMD_VEL : PROTO_CMD_DUTY;
    out->left = left;
    out->right = right;
    return true;
  }

  if (head_is(p, "spd", 3U))
  {
    p += 3;
    if (!parse_i32(&p, &left) || !at_end(p))
    {
      return false;
    }

    /* 0과 1 이외는 거절이다. "켜짐"의 표현을 하나로 유지한다. */
    if ((left != 0) && (left != 1))
    {
      return false;
    }

    out->kind = PROTO_CMD_SPD;
    out->left = left;
    out->right = left;
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
