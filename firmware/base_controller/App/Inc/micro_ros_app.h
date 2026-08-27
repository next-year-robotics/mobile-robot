/**
  ******************************************************************************
  * @file    micro_ros_app.h
  * @brief   MicroRosTask 본체와 그 계측값. 펌웨어 전용이다.
  * @note    rcl/rclc/rmw를 부르는 코드는 이 모듈 안에만 있다. 인터페이스에는
  *          micro-ROS 타입이 나오지 않으므로 rtos_app.c는 이 헤더만 보고 task를
  *          띄운다 (platform_stm32.h / app_main.h와 같은 규칙이다).
  *
  *          순수 규칙은 여기 없다. 재접속 상태기계는 agent_link.c, 역기구학은
  *          base_kinematics.c, allocator는 static_pool.c에 있고 셋 다 host에서
  *          단위 테스트한다. 이 파일에 남은 것은 micro-ROS API를 그 순서대로 부르는
  *          배선뿐이다.
  ******************************************************************************
  */
#ifndef MICRO_ROS_APP_H
#define MICRO_ROS_APP_H

#include <stdint.h>

/** @brief SWD로 읽는 고정 크기 계측값. */
typedef struct
{
  uint32_t agent_state;             /* agent_state_t */
  uint32_t connect_count;
  uint32_t disconnect_count;
  uint32_t ping_fail_count;
  uint32_t create_fail_count;
  uint32_t spin_fail_count;

  uint32_t time_sync_count;
  uint32_t time_sync_fail_count;
  uint32_t epoch_synchronized;

  uint32_t cmd_vel_count;           /* 콜백이 불린 횟수 */
  uint32_t cmd_vel_applied_count;   /* 실제로 ControlTask에 넘긴 횟수 */
  uint32_t cmd_vel_rejected_count;
  uint32_t cmd_vel_last_verdict;    /* cmd_stamp_verdict_t */
  uint32_t cmd_vel_age_ms;          /* 마지막으로 받아들인 명령의 나이 */

  uint32_t joint_states_count;
  uint32_t joint_states_fail_count;
  uint32_t joint_states_stale_count; /* snapshot 나이가 상한에 걸린 횟수 */
  uint32_t base_status_count;
  uint32_t base_status_fail_count;

  /* micro-ROS 전용 pool (static_pool.h). `pool_free_min`이 안정화되고
     `pool_alloc_fail`이 0이면 누수가 없다. */
  uint32_t pool_capacity;
  uint32_t pool_free;
  uint32_t pool_free_min;
  uint32_t pool_largest_free;
  uint32_t pool_alloc_fail;
  uint32_t pool_invalid_free;
  uint32_t pool_live_blocks;
  /* rcutils allocator를 우회해 libc malloc으로 온 할당 수. 0이어야 한다. */
  uint32_t pool_escaped_alloc;

  /* USART1 트랜스포트. platform_uart1의 계수기를 그대로 옮긴 값이다. */
  uint32_t uart1_error_count;
  uint32_t uart1_error_last;
  uint32_t uart1_rx_overflow_count;
  uint32_t uart1_rearm_count;
  uint32_t uart1_tx_fail_count;
} micro_ros_metrics_t;

/**
  * @brief  MicroRosTask 본체. rtos_app.c가 xTaskCreateStatic으로 띄운다.
  * @note   USART1의 유일한 소유자다. DMA 수신 기동과 재무장이 이 task 안에서만
  *         일어난다.
  */
void micro_ros_task(void *argument);

/** @brief 계측 snapshot. HealthTask가 읽어 g_rtos_metrics로 옮긴다. */
void micro_ros_get_metrics(micro_ros_metrics_t *out);

#endif /* MICRO_ROS_APP_H */
