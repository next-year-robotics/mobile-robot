/**
  ******************************************************************************
  * @file    static_pool.c
  * @brief   first-fit + 인접 병합 allocator. FreeRTOS heap_4와 같은 알고리즘이다.
  * @note    잠금이 없다. 소유 task 하나에서만 부른다 (static_pool.h 참조).
  ******************************************************************************
  */
#include "static_pool.h"

#include <string.h>

/* header 크기를 정렬 배수로 올린다. base가 정렬돼 있고 header가 정렬 배수면
   payload도 자동으로 정렬된다 — payload 정렬을 따로 계산하지 않는 이유다. */
#define POOL_HEADER  ((sizeof(static_pool_block_t) + (STATIC_POOL_ALIGN - 1U)) \
                      & ~((size_t)(STATIC_POOL_ALIGN - 1U)))

/* 분할 뒤 남는 조각이 이보다 작으면 쪼개지 않는다. header도 못 담는 조각을
   free 리스트에 넣으면 리스트만 길어지고 쓸 수는 없다. */
#define POOL_MIN_BLOCK  (POOL_HEADER * 2U)

/* size의 최상위 bit를 "사용 중" 표시로 쓴다. 그래서 한 블록은 SIZE_MAX/2를
   넘을 수 없고, alloc()이 그 경계를 먼저 막는다. */
#define POOL_ALLOC_BIT  ((size_t)1U << ((sizeof(size_t) * 8U) - 1U))

static size_t align_up(size_t value)
{
  return (value + (STATIC_POOL_ALIGN - 1U)) & ~((size_t)(STATIC_POOL_ALIGN - 1U));
}

/**
  * @brief  주소 순 free 리스트에 넣고 앞뒤와 붙는지 본다.
  * @note   병합을 삽입과 같은 자리에서 하는 것이 heap_4의 요령이다. 리스트가 주소
  *         순이므로 인접 여부를 포인터 산술 두 번으로 판정할 수 있다.
  */
static void insert_free(static_pool_t *p, static_pool_block_t *block)
{
  static_pool_block_t *iter;
  uint8_t *bytes;

  for (iter = &p->start; iter->next < block; iter = iter->next)
  {
    /* block보다 주소가 작은 마지막 블록을 찾는다. */
  }

  /* 앞 블록과 물리적으로 붙어 있으면 하나로 만든다. */
  bytes = (uint8_t *)iter;
  if ((iter != &p->start) && ((bytes + iter->size) == (uint8_t *)block))
  {
    iter->size += block->size;
    block = iter;
  }

  /* 뒤 블록과도 붙어 있으면 마저 흡수한다. end sentinel은 흡수하지 않는다. */
  bytes = (uint8_t *)block;
  if ((bytes + block->size) == (uint8_t *)iter->next)
  {
    if (iter->next != p->end)
    {
      block->size += iter->next->size;
      block->next = iter->next->next;
    }
    else
    {
      block->next = p->end;
    }
  }
  else
  {
    block->next = iter->next;
  }

  if (iter != block)
  {
    iter->next = block;
  }
}

void static_pool_init(static_pool_t *p, void *memory, size_t bytes)
{
  uintptr_t raw;
  uintptr_t base;
  uintptr_t end;
  size_t usable;
  static_pool_block_t *first;

  memset(p, 0, sizeof(*p));

  if ((memory == NULL) || (bytes == 0U))
  {
    return;
  }

  raw = (uintptr_t)memory;
  base = (raw + (STATIC_POOL_ALIGN - 1U)) & ~((uintptr_t)(STATIC_POOL_ALIGN - 1U));

  if ((size_t)(base - raw) >= bytes)
  {
    return;
  }
  usable = bytes - (size_t)(base - raw);

  /* 최상단에 크기 0인 sentinel을 둔다. free 리스트의 끝이며 절대 병합되지 않는다. */
  if (usable < (POOL_HEADER + POOL_MIN_BLOCK))
  {
    return;
  }
  end = (base + usable - POOL_HEADER) & ~((uintptr_t)(STATIC_POOL_ALIGN - 1U));

  p->base = (uint8_t *)base;
  p->end = (static_pool_block_t *)end;
  p->end->size = 0U;
  p->end->next = NULL;

  first = (static_pool_block_t *)base;
  first->size = (size_t)(end - base);
  first->next = p->end;

  p->start.next = first;
  p->start.size = 0U;

  p->capacity = first->size;
  p->free_bytes = first->size;
  p->free_bytes_min = first->size;
}

void *static_pool_alloc(static_pool_t *p, size_t bytes)
{
  static_pool_block_t *prev;
  static_pool_block_t *block;
  static_pool_block_t *split;
  size_t need;

  if ((p->capacity == 0U) || (bytes >= (POOL_ALLOC_BIT - POOL_HEADER)))
  {
    p->alloc_fail_count++;
    return NULL;
  }

  need = align_up(bytes + POOL_HEADER);
  if (need < POOL_MIN_BLOCK)
  {
    need = POOL_MIN_BLOCK;
  }

  prev = &p->start;
  block = p->start.next;
  while ((block != p->end) && (block->size < need))
  {
    prev = block;
    block = block->next;
  }

  if (block == p->end)
  {
    p->alloc_fail_count++;
    return NULL;
  }

  prev->next = block->next;

  if ((block->size - need) >= POOL_MIN_BLOCK)
  {
    split = (static_pool_block_t *)((uint8_t *)block + need);
    split->size = block->size - need;
    split->next = NULL;
    block->size = need;
    insert_free(p, split);
  }

  p->free_bytes -= block->size;
  if (p->free_bytes < p->free_bytes_min)
  {
    p->free_bytes_min = p->free_bytes;
  }

  block->size |= POOL_ALLOC_BIT;
  block->next = NULL;
  p->alloc_count++;
  p->live_blocks++;

  return (void *)((uint8_t *)block + POOL_HEADER);
}

/**
  * @brief  포인터가 이 pool의 살아 있는 블록을 가리키는지 본다.
  * @note   남의 allocator에서 온 포인터나 이중 해제를 조용히 삼키지 않기 위한
  *         최소한의 확인이다. 완전한 검증은 아니지만 실제로 나는 실수는 잡는다.
  */
static static_pool_block_t *live_block(const static_pool_t *p, const void *ptr)
{
  const uint8_t *bytes = (const uint8_t *)ptr;
  static_pool_block_t *block;

  if ((p->capacity == 0U) || (ptr == NULL))
  {
    return NULL;
  }
  if ((bytes < (p->base + POOL_HEADER)) || (bytes >= (const uint8_t *)p->end))
  {
    return NULL;
  }
  if ((((uintptr_t)bytes) & (STATIC_POOL_ALIGN - 1U)) != 0U)
  {
    return NULL;
  }

  block = (static_pool_block_t *)(void *)(bytes - POOL_HEADER);
  if ((block->size & POOL_ALLOC_BIT) == 0U)
  {
    return NULL;
  }
  return block;
}

void static_pool_free(static_pool_t *p, void *ptr)
{
  static_pool_block_t *block;

  if (ptr == NULL)
  {
    return;
  }

  block = live_block(p, ptr);
  if (block == NULL)
  {
    p->invalid_free_count++;
    return;
  }

  block->size &= ~POOL_ALLOC_BIT;
  block->next = NULL;
  p->free_bytes += block->size;
  p->free_count++;
  p->live_blocks--;
  insert_free(p, block);
}

size_t static_pool_payload_size(const static_pool_t *p, const void *ptr)
{
  const static_pool_block_t *block = live_block(p, ptr);

  if (block == NULL)
  {
    return 0U;
  }
  return (block->size & ~POOL_ALLOC_BIT) - POOL_HEADER;
}

void *static_pool_realloc(static_pool_t *p, void *ptr, size_t bytes)
{
  size_t old;
  void *fresh;

  if (ptr == NULL)
  {
    return static_pool_alloc(p, bytes);
  }
  if (bytes == 0U)
  {
    static_pool_free(p, ptr);
    return NULL;
  }

  old = static_pool_payload_size(p, ptr);
  if (old == 0U)
  {
    /* 우리 블록이 아니다. 복사할 길이를 모르므로 손대지 않는다. */
    p->invalid_free_count++;
    return NULL;
  }

  /* 축소는 제자리에서 끝낸다. 뒤를 잘라 free 리스트에 넣으면 조각이 하나 더
     생기는데, micro-ROS의 realloc은 대부분 증가 방향이라 이득이 없다. */
  if (old >= bytes)
  {
    return ptr;
  }

  fresh = static_pool_alloc(p, bytes);
  if (fresh == NULL)
  {
    /* **원래 블록은 살아 있다.** rcutils reallocate 계약이 그렇다. */
    return NULL;
  }

  memcpy(fresh, ptr, old);
  static_pool_free(p, ptr);
  return fresh;
}

void *static_pool_calloc(static_pool_t *p, size_t n, size_t size)
{
  size_t total;
  void *ptr;

  if ((n != 0U) && (size > (((size_t)-1) / n)))
  {
    p->alloc_fail_count++;
    return NULL;
  }

  total = n * size;
  ptr = static_pool_alloc(p, total);
  if (ptr != NULL)
  {
    memset(ptr, 0, total);
  }
  return ptr;
}

void static_pool_metrics(const static_pool_t *p, static_pool_metrics_t *out)
{
  const static_pool_block_t *block;
  size_t largest = 0U;

  for (block = p->start.next; (block != NULL) && (block != p->end);
       block = block->next)
  {
    if (block->size > largest)
    {
      largest = block->size;
    }
  }

  out->capacity_bytes = (uint32_t)p->capacity;
  out->free_bytes = (uint32_t)p->free_bytes;
  out->free_bytes_min = (uint32_t)p->free_bytes_min;
  out->largest_free_block = (uint32_t)largest;
  out->alloc_count = p->alloc_count;
  out->free_count = p->free_count;
  out->alloc_fail_count = p->alloc_fail_count;
  out->invalid_free_count = p->invalid_free_count;
  out->live_blocks = p->live_blocks;
}
