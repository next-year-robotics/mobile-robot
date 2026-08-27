/**
  ******************************************************************************
  * @file    u3_smoke.c
  * @brief   U3 smoke test 줄 해석과 누락/중복 계수. 순수 모듈이다.
  ******************************************************************************
  */
#include "u3_smoke.h"

/** @brief 10진 uint32를 읽는다. 자리수 초과/비숫자는 실패다. */
static bool parse_u32(const char *s, const char **end, uint32_t *out)
{
  uint32_t value = 0U;
  uint32_t digits = 0U;

  while ((*s >= '0') && (*s <= '9'))
  {
    uint32_t digit = (uint32_t)(*s - '0');

    /* 곱하기 전에 넘칠지 판정한다. 넘친 뒤에 보면 이미 값이 감겨 있다. */
    if ((value > (0xFFFFFFFFU / 10U)) ||
        ((value == (0xFFFFFFFFU / 10U)) && (digit > (0xFFFFFFFFU % 10U))))
    {
      return false;
    }
    value = (value * 10U) + digit;
    digits++;
    s++;
  }

  if (digits == 0U)
  {
    return false;
  }

  *end = s;
  *out = value;
  return true;
}

static const char *skip_spaces(const char *s)
{
  while (*s == ' ')
  {
    s++;
  }
  return s;
}

/** @brief 남은 것이 공백뿐인가. 뒤에 쓰레기가 붙은 줄은 유효한 줄이 아니다. */
static bool at_end(const char *s)
{
  return *skip_spaces(s) == '\0';
}

static void counters_reset(u3_smoke_t *s)
{
  s->counters.rx_ping = 0U;
  s->counters.rx_bad = 0U;
  s->counters.rx_gap = 0U;
  s->counters.rx_dup = 0U;
  s->have_seq = false;
  s->expect_seq = 0U;
}

void u3_smoke_init(u3_smoke_t *s)
{
  s->length = 0U;
  s->overlong = false;
  s->mode = U3_MODE_ALL;
  counters_reset(s);
}

/**
  * @brief  P 순번 하나를 계수기에 반영한다.
  * @note   기대값보다 크면 건너뛴 개수만큼 gap에 더한다. 작으면 중복이나 역전이다.
  *         기대값을 뒤로 되돌리지 않는다. 되돌리면 그 뒤 정상 순번이 전부 중복으로
  *         보인다.
  */
static void account_seq(u3_smoke_t *s, uint32_t seq)
{
  s->counters.rx_ping++;

  if (!s->have_seq)
  {
    s->have_seq = true;
    s->expect_seq = seq + 1U;
    return;
  }

  if (seq == s->expect_seq)
  {
    s->expect_seq = seq + 1U;
    return;
  }

  if (seq > s->expect_seq)
  {
    s->counters.rx_gap += (seq - s->expect_seq);
    s->expect_seq = seq + 1U;
    return;
  }

  s->counters.rx_dup++;
}

/** @brief '\0'로 끝나는 한 줄을 해석한다. */
static u3_event_t parse_line(u3_smoke_t *s, const char *line, uint32_t *out_value)
{
  const char *p = skip_spaces(line);
  char tag = *p;
  uint32_t value;

  if (tag == '\0')
  {
    return U3_EVENT_NONE;
  }
  p++;

  /* 태그는 한 글자다. 뒤에 다른 글자가 바로 붙으면 깨진 줄로 본다. */
  if ((*p != '\0') && (*p != ' '))
  {
    return U3_EVENT_BAD;
  }

  switch (tag)
  {
    case 'Z':
      if (!at_end(p))
      {
        return U3_EVENT_BAD;
      }
      /* 자기 자신도 세지 않는다. Z 이후가 그대로 0에서 시작해야 판정이 쉽다. */
      counters_reset(s);
      return U3_EVENT_RESET;

    case 'P':
      p = skip_spaces(p);
      if (!parse_u32(p, &p, &value) || !at_end(p))
      {
        return U3_EVENT_BAD;
      }
      account_seq(s, value);
      *out_value = value;
      return U3_EVENT_PING;

    case 'S':
      p = skip_spaces(p);
      if (!parse_u32(p, &p, &value) || !at_end(p) || (value > (uint32_t)U3_MODE_ALL))
      {
        return U3_EVENT_BAD;
      }
      s->mode = (uint8_t)value;
      *out_value = value;
      return U3_EVENT_MODE;

    case 'B':
      p = skip_spaces(p);
      if (!parse_u32(p, &p, &value) || !at_end(p))
      {
        return U3_EVENT_BAD;
      }
      *out_value = value;
      return U3_EVENT_BAUD;

    case 'N':
      p = skip_spaces(p);
      if (!parse_u32(p, &p, &value) || !at_end(p) ||
          (value < U3_PERIOD_MIN_MS) || (value > U3_PERIOD_MAX_MS))
      {
        return U3_EVENT_BAD;
      }
      *out_value = value;
      return U3_EVENT_PERIOD;

    default:
      break;
  }

  return U3_EVENT_BAD;
}

u3_event_t u3_smoke_push(u3_smoke_t *s, char c, uint32_t *out_value)
{
  u3_event_t event;

  if (c == '\r')
  {
    return U3_EVENT_NONE;
  }

  if (c != '\n')
  {
    if (s->length < (U3_RX_LINE_CAP - 1U))
    {
      s->line[s->length] = c;
      s->length++;
    }
    else
    {
      /* cap을 넘겼다. 잘린 앞부분이 우연히 유효한 줄로 보이지 않게 표시해 둔다. */
      s->overlong = true;
    }
    return U3_EVENT_NONE;
  }

  s->line[s->length] = '\0';

  if (s->overlong)
  {
    s->counters.rx_bad++;
    event = U3_EVENT_BAD;
  }
  else
  {
    event = parse_line(s, s->line, out_value);
    if (event == U3_EVENT_BAD)
    {
      s->counters.rx_bad++;
    }
  }

  s->length = 0U;
  s->overlong = false;
  return event;
}

uint8_t u3_smoke_mode(const u3_smoke_t *s)
{
  return s->mode;
}

void u3_smoke_counters(const u3_smoke_t *s, u3_counters_t *out)
{
  *out = s->counters;
}
