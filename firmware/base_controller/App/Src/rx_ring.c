#include "rx_ring.h"

void rx_ring_init(rx_ring_t *r)
{
  r->head = 0U;
  r->tail = 0U;
  r->drop_count = 0U;
}

bool rx_ring_push(rx_ring_t *r, uint8_t byte)
{
  uint16_t head = r->head;
  uint16_t next = (uint16_t)((head + 1U) & (CMD_RX_RING_SIZE - 1U));

  if (next == r->tail)
  {
    r->drop_count++;
    return false;
  }

  r->buf[head] = byte;
  r->head = next;
  return true;
}

bool rx_ring_pop(rx_ring_t *r, uint8_t *out)
{
  uint16_t tail = r->tail;

  if (tail == r->head)
  {
    return false;
  }

  *out = r->buf[tail];
  r->tail = (uint16_t)((tail + 1U) & (CMD_RX_RING_SIZE - 1U));
  return true;
}

void rx_ring_discard_all(rx_ring_t *r)
{
  r->tail = r->head;
}
