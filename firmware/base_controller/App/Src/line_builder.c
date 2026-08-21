#include "line_builder.h"

#include <string.h>

/**
  * @brief  pos 바로 앞에 한 글자를 쓰고 pos를 당긴다.
  * @retval 남은 공간이 없으면 false.
  */
static bool put_char(char *buf, size_t *pos, char c)
{
  if (*pos == 0U)
  {
    return false;
  }

  (*pos)--;
  buf[*pos] = c;
  return true;
}

/**
  * @brief  pos 바로 앞에 10진수를 쓴다.
  * @note   링커가 nano.specs를 쓰므로 printf 계열은 %lld를 다루지 못한다.
  *         64-bit 누적 틱은 직접 변환한다. INT64_MIN도 부호 반전 없이 다루도록
  *         크기는 부호 없는 타입으로 잡는다.
  */
static bool put_i64(char *buf, size_t *pos, int64_t value)
{
  uint64_t magnitude = (value < 0) ? (~(uint64_t)value + 1U) : (uint64_t)value;

  do
  {
    if (!put_char(buf, pos, (char)('0' + (uint32_t)(magnitude % 10U))))
    {
      return false;
    }
    magnitude /= 10U;
  } while (magnitude != 0U);

  if (value < 0)
  {
    return put_char(buf, pos, '-');
  }

  return true;
}

bool line_build(char *buf, size_t capacity, const char *tag,
                const int64_t *values, size_t count,
                size_t *out_offset, size_t *out_length)
{
  size_t pos = capacity;
  size_t tag_length;
  size_t i = count;

  if ((buf == NULL) || (tag == NULL) || (out_offset == NULL)
      || (out_length == NULL))
  {
    return false;
  }
  if ((count > 0U) && (values == NULL))
  {
    return false;
  }

  if (!put_char(buf, &pos, '\n'))
  {
    return false;
  }

  /* 뒤에서부터 채우므로 값도 역순으로 쓴다. */
  while (i-- > 0U)
  {
    if (!put_i64(buf, &pos, values[i]))
    {
      return false;
    }
    if (!put_char(buf, &pos, ' '))
    {
      return false;
    }
  }

  tag_length = strlen(tag);
  if (tag_length > pos)
  {
    return false;
  }
  pos -= tag_length;
  memcpy(&buf[pos], tag, tag_length);

  *out_offset = pos;
  *out_length = capacity - pos;
  return true;
}
