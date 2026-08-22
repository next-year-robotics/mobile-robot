/**
  ******************************************************************************
  * @file    static_pool.h
  * @brief   고정 배열 하나 위에서 도는 first-fit 병합 allocator. 순수 모듈이다.
  * @note    **왜 이게 필요한가.** micro-ROS는 rcutils allocator를 통해 init과
  *          재접속 경로에서 동적 할당을 한다(노드 이름, 타입 지원, executor handle
  *          배열, rmw 옵션). 그 호출을 보낼 곳이 이 프로젝트에는 원래 없다:
  *
  *            - FreeRTOS heap : `configSUPPORT_DYNAMIC_ALLOCATION == 0`이고
  *                              heap_4.c는 링크 대상에서 빠져 있다. 금지다.
  *            - newlib malloc : `_sbrk`를 쓰고, FreeRTOS 문맥에서 재진입 보호가
  *                              없으며, 얼마나 썼는지 관측할 방법이 없다.
  *
  *          그래서 **micro-ROS 전용 pool을 하나 두고 그 안에서만 놀게 한다.**
  *          상한이 링커 시점에 박히고, 최저 여유와 실패 횟수를 SWD로 볼 수 있다
  *          (M5.md 10절 "reconnect 반복 뒤 minimum-ever-free heap이 안정화되고
  *          allocation failure 0").
  *
  *          알고리즘은 FreeRTOS heap_4와 같다 — 주소 순 free 리스트, 분할, 인접
  *          병합. 새로 발명한 것이 아니라 **FreeRTOS 의존성을 걷어 내고 host에서
  *          ASan으로 돌릴 수 있게 만든 것**이다.
  *
  *          **소유자는 한 task뿐이다(MicroRosTask).** 내부에 잠금이 없다. 다른
  *          task에서 부르면 free 리스트가 깨진다. 계측값도 소유 task가 복사한다.
  ******************************************************************************
  */
#ifndef STATIC_POOL_H
#define STATIC_POOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief payload 정렬. double(8 B)을 담는 ROS 메시지가 있으므로 8이다. */
#define STATIC_POOL_ALIGN  8U

typedef struct static_pool_block
{
  struct static_pool_block *next;  /* 주소 순 free 리스트 */
  size_t                    size;  /* header 포함 총 크기. 최상위 bit = 사용 중 */
} static_pool_block_t;

typedef struct
{
  static_pool_block_t  start;      /* 리스트 head. pool 밖에 있다. */
  static_pool_block_t *end;        /* pool 최상단 sentinel */
  uint8_t             *base;
  size_t               capacity;   /* 정렬 보정 뒤 실제로 쓸 수 있는 바이트 */
  size_t               free_bytes;
  size_t               free_bytes_min;   /* minimum-ever-free */
  uint32_t             alloc_count;
  uint32_t             free_count;
  uint32_t             alloc_fail_count;
  uint32_t             invalid_free_count;  /* 이중 해제 / 남의 포인터 */
  uint32_t             live_blocks;
} static_pool_t;

/**
  * @brief  SWD로 읽는 고정 크기 계측값.
  * @note   `free_bytes`는 block header를 포함한 값이다. 사용자 payload 합계가
  *         아니라 "pool에 남은 공간"이다.
  */
typedef struct
{
  uint32_t capacity_bytes;
  uint32_t free_bytes;
  uint32_t free_bytes_min;      /* 이 값이 안정화되면 누수가 없다는 뜻이다 */
  uint32_t largest_free_block;  /* 이 값이 계속 줄면 단편화다 */
  uint32_t alloc_count;
  uint32_t free_count;
  uint32_t alloc_fail_count;
  uint32_t invalid_free_count;
  uint32_t live_blocks;
} static_pool_metrics_t;

/**
  * @brief  pool을 세운다. `memory`는 호출자가 소유하는 정적 배열이다.
  * @note   정렬 보정으로 앞부분이 버려질 수 있으므로 capacity < bytes일 수 있다.
  *         너무 작으면 capacity가 0이 되고 모든 할당이 실패한다.
  */
void static_pool_init(static_pool_t *p, void *memory, size_t bytes);

/**
  * @brief  first-fit 할당. 실패하면 NULL이며 `alloc_fail_count`가 오른다.
  * @note   `bytes == 0`도 유일한 포인터를 돌려준다. malloc(0)의 관례이고, rcl에는
  *         길이 0 sequence를 할당한 뒤 NULL 검사로 실패를 판정하는 자리가 있다.
  */
void *static_pool_alloc(static_pool_t *p, size_t bytes);

/** @brief NULL은 무시한다. pool 밖 포인터와 이중 해제는 계수하고 무시한다. */
void static_pool_free(static_pool_t *p, void *ptr);

/**
  * @brief  realloc. 축소는 제자리에서 끝낸다(조각을 더 만들지 않는다).
  * @note   실패하면 NULL을 돌려주고 **원래 블록은 그대로 살아 있다.**
  */
void *static_pool_realloc(static_pool_t *p, void *ptr, size_t bytes);

/** @brief 0으로 채운 할당. `n * size` 곱셈 넘침을 막는다. */
void *static_pool_calloc(static_pool_t *p, size_t n, size_t size);

/** @brief 이 포인터가 실제로 쓸 수 있는 payload 바이트. 아니면 0. */
size_t static_pool_payload_size(const static_pool_t *p, const void *ptr);

void static_pool_metrics(const static_pool_t *p, static_pool_metrics_t *out);

#endif /* STATIC_POOL_H */
