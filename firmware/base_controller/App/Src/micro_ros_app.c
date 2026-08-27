/**
  ******************************************************************************
  * @file    micro_ros_app.c
  * @brief   micro-ROS 트랜스포트 · 엔티티 · executor 배선. MicroRosTask 본체.
  ******************************************************************************
  */
#include "micro_ros_app.h"

#include "agent_link.h"
#include "app_config.h"
#include "app_link.h"
#include "base_config.h"
#include "base_kinematics.h"
#include "control_types.h"
#include "main.h"
#include "motor_safety.h"
#include "platform_stm32.h"
#include "platform_uart1.h"
#include "rtos_app.h"
#include "static_pool.h"

#include "FreeRTOS.h"
#include "task.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include <geometry_msgs/msg/twist_stamped.h>
#include <mr_msgs/msg/base_status.h>
#include <sensor_msgs/msg/joint_state.h>

#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MICRO_ROS_NODE_NAME     "base_controller"
#define TOPIC_CMD_VEL           "/cmd_vel"
#define TOPIC_JOINT_STATES      "/joint_states"
#define TOPIC_BASE_STATUS       "/base/status"

#define NS_PER_MS               1000000LL
#define NS_PER_S                1000000000LL

/* ------------------------------------------------------------------------- */
/* micro-ROS 전용 heap                                                        */
/* ------------------------------------------------------------------------- */

/* 8바이트 정렬. ROS 메시지에 double이 들어 있어 payload 정렬이 8이어야 한다. */
static uint8_t s_pool_memory[MICRO_ROS_POOL_BYTES] __attribute__((aligned(8)));
static static_pool_t s_pool;

static void *mr_allocate(size_t size, void *state)
{
  (void)state;
  return static_pool_alloc(&s_pool, size);
}

static void mr_deallocate(void *pointer, void *state)
{
  (void)state;
  static_pool_free(&s_pool, pointer);
}

static void *mr_reallocate(void *pointer, size_t size, void *state)
{
  (void)state;
  return static_pool_realloc(&s_pool, pointer, size);
}

static void *mr_zero_allocate(size_t count, size_t size, void *state)
{
  (void)state;
  return static_pool_calloc(&s_pool, count, size);
}

/* ---- newlib heap 차단 ----
   rcutils_set_default_allocator()로 갈아 끼우므로 정상 경로에서 libc heap이 쓰이는 일은
   없다. 다만 링커가 rcutils 기본 구현 때문에 newlib malloc과 _sbrk heap을 image에 끌고
   들어오므로, 쓰일 자리 자체가 남는다.

   네 심볼을 덮어 pool 하나로 모으면 어떤 경로로 새더라도 상한 안에 머문다.
   `escaped_alloc_count`가 0이 아닌 것으로 우회 할당이 드러난다. */

static uint32_t s_escaped_alloc_count;

void *malloc(size_t size)
{
  s_escaped_alloc_count++;
  return static_pool_alloc(&s_pool, size);
}

void *calloc(size_t count, size_t size)
{
  s_escaped_alloc_count++;
  return static_pool_calloc(&s_pool, count, size);
}

void *realloc(void *pointer, size_t size)
{
  s_escaped_alloc_count++;
  return static_pool_realloc(&s_pool, pointer, size);
}

void free(void *pointer)
{
  static_pool_free(&s_pool, pointer);
}

/* ------------------------------------------------------------------------- */
/* clock_gettime — rcutils가 CLOCK_MONOTONIC을 요구한다                        */
/* ------------------------------------------------------------------------- */

/**
  * @brief  newlib에 없는 monotonic clock을 FreeRTOS tick으로 채운다.
  * @note   libmicroros.a가 이 심볼을 필요로 한다. 라이브러리를 `-DCLOCK_MONOTONIC=0`
  *         으로 빌드했으므로 clock_id는 무시해도 된다. 분해능은 tick 하나(1 ms)다.
  *         micro-ROS는 이 시계를 내부 타임아웃에만 쓴다. `/joint_states`의 stamp는
  *         여기가 아니라 `rmw_uros_epoch_nanos()`에서 온다.
  *
  *         overflow count를 vTaskSetTimeOutState로 얻는다. 49.7일마다 감기는 tick을
  *         그대로 쓰면 그 시점에 micro-ROS의 모든 타임아웃이 한 번 어긋난다.
  */
int clock_gettime(clockid_t clock_id, struct timespec *tp)
{
  TimeOut_t state;
  uint64_t ticks;
  int64_t nanos;

  (void)clock_id;
  if (tp == NULL)
  {
    return -1;
  }

  vTaskSetTimeOutState(&state);
  ticks = ((uint64_t)state.xOverflowCount) << (sizeof(TickType_t) * 8U);
  ticks += (uint64_t)state.xTimeOnEntering;

  nanos = (int64_t)ticks * (NS_PER_S / (int64_t)configTICK_RATE_HZ);
  tp->tv_sec = (time_t)(nanos / NS_PER_S);
  tp->tv_nsec = (long)(nanos % NS_PER_S);
  return 0;
}

/* ------------------------------------------------------------------------- */
/* custom transport — USART1                                                  */
/* ------------------------------------------------------------------------- */

/**
  * @brief  트랜스포트는 이미 열려 있다.
  * @note   DMA 수신은 task 첫머리에서 platform_uart1_start()로 한 번만 건다. 여기서
  *         다시 걸지 않는 이유: 재접속마다 열고 닫으면 그때마다 DMA 링의 tail이
  *         되감긴다. 링에 남아 있던 프레임 조각이 유효 바이트로 읽힌다. 링크는 세션과
  *         무관하게 늘 살아 있는 것이 이 설계의 전제다.
  */
static bool transport_open(struct uxrCustomTransport *transport)
{
  (void)transport;
  return platform_uart1_is_armed() || platform_uart1_rearm_if_needed();
}

static bool transport_close(struct uxrCustomTransport *transport)
{
  /* 물리 링크를 끊지 않는다. 위 주석 참조. */
  (void)transport;
  return true;
}

static size_t transport_write(struct uxrCustomTransport *transport,
                              const uint8_t *buffer, size_t length,
                              uint8_t *error_code)
{
  (void)transport;

  if ((length == 0U) || (buffer == NULL))
  {
    return 0U;
  }

  /* 유한 타임아웃 블로킹이다. MTU 512 B는 921600에서 5.6 ms이고 상한은 20 ms라
     3.5배 여유가 있다. TX DMA는 쓰지 않는다. MicroRosTask가
     ControlTask/HealthTask보다 낮은 우선순위라 이 대기가 제어나 IWDG를 막을 수 없기
     때문이다. */
  if (!platform_uart1_write(buffer, length))
  {
    if (error_code != NULL)
    {
      *error_code = 1U;
    }
    return 0U;
  }
  return length;
}

static size_t transport_read(struct uxrCustomTransport *transport,
                             uint8_t *buffer, size_t length, int timeout,
                             uint8_t *error_code)
{
  size_t got;
  uint32_t deadline_ms;

  (void)transport;
  (void)error_code;

  if ((buffer == NULL) || (length == 0U))
  {
    return 0U;
  }

  /* 한 번의 FE/NE/PE/ORE로 링크가 영구히 죽지 않게 하는 그물. HAL은 DMA 수신 중
     오류를 blocking error로 보고 수신을 끊으므로 소유 task가 여기서 되살린다
     (platform_uart1.h). */
  (void)platform_uart1_rearm_if_needed();

  got = platform_uart1_read(buffer, length);
  if ((got > 0U) || (timeout <= 0))
  {
    return got;
  }

  deadline_ms = platform_now_ms() + (uint32_t)timeout;
  while ((int32_t)(platform_now_ms() - deadline_ms) < 0)
  {
    /* IDLE 인터럽트 대신 tick 단위 폴링이다. 921600에서 1 ms는 92 B이고 링은
       2048 B라 22배 여유가 있다. 움직이는 부품을 늘리지 않는 쪽을 골랐다. */
    vTaskDelay(pdMS_TO_TICKS(MICRO_ROS_TRANSPORT_POLL_MS));

    (void)platform_uart1_rearm_if_needed();
    got = platform_uart1_read(buffer, length);
    if (got > 0U)
    {
      break;
    }
  }

  return got;
}

/* ------------------------------------------------------------------------- */
/* 정적 메시지 메모리                                                          */
/* ------------------------------------------------------------------------- */

/* `/joint_states`. 가변 길이 배열의 실체를 전부 여기서 미리 잡는다. 발행 경로에서는
   단 한 바이트도 할당하지 않는다. */
static sensor_msgs__msg__JointState s_joint_state;
static rosidl_runtime_c__String     s_joint_names[2];
static char s_joint_name_left[sizeof(BASE_JOINT_NAME_LEFT)];
static char s_joint_name_right[sizeof(BASE_JOINT_NAME_RIGHT)];
static char s_joint_frame_id[1];
static double s_joint_position[2];
static double s_joint_velocity[2];

/* `/base/status`. 가변 길이는 header.frame_id 하나뿐이고 비워 둔다. */
static mr_msgs__msg__BaseStatus s_base_status;
static char s_base_status_frame_id[1];

/* `/cmd_vel`. frame_id는 계약상 무시하지만 역직렬화가 쓸 자리는 있어야 한다. */
static geometry_msgs__msg__TwistStamped s_cmd_vel;
static char s_cmd_vel_frame_id[CMD_VEL_FRAME_ID_CAP];

static void bind_string(rosidl_runtime_c__String *s, char *storage,
                        size_t capacity, const char *value)
{
  s->data = storage;
  s->capacity = capacity;
  if (value != NULL)
  {
    (void)strncpy(storage, value, capacity - 1U);
    storage[capacity - 1U] = '\0';
    s->size = strlen(storage);
  }
  else
  {
    storage[0] = '\0';
    s->size = 0U;
  }
}

static void message_memory_init(void)
{
  memset(&s_joint_state, 0, sizeof(s_joint_state));
  memset(&s_cmd_vel, 0, sizeof(s_cmd_vel));

  /* JointState는 frame을 쓰지 않는다. 빈 문자열이지만 capacity 1은 있어야 직렬화가
     '\0'을 쓸 자리를 갖는다. */
  bind_string(&s_joint_state.header.frame_id, s_joint_frame_id,
              sizeof(s_joint_frame_id), "");

  /* 이 순서가 계약이다. left가 먼저다. */
  bind_string(&s_joint_names[0], s_joint_name_left, sizeof(s_joint_name_left),
              BASE_JOINT_NAME_LEFT);
  bind_string(&s_joint_names[1], s_joint_name_right, sizeof(s_joint_name_right),
              BASE_JOINT_NAME_RIGHT);

  s_joint_state.name.data = s_joint_names;
  s_joint_state.name.size = 2U;
  s_joint_state.name.capacity = 2U;

  s_joint_state.position.data = s_joint_position;
  s_joint_state.position.size = 2U;
  s_joint_state.position.capacity = 2U;

  s_joint_state.velocity.data = s_joint_velocity;
  s_joint_state.velocity.size = 2U;
  s_joint_state.velocity.capacity = 2U;

  /* effort는 길이 0이다. 토크 센서가 없으므로 0을 채워 넣지 않는다. 0이 흘러 나가면
     누군가는 그걸 측정값으로 믿는다. */
  s_joint_state.effort.data = NULL;
  s_joint_state.effort.size = 0U;
  s_joint_state.effort.capacity = 0U;

  bind_string(&s_cmd_vel.header.frame_id, s_cmd_vel_frame_id,
              sizeof(s_cmd_vel_frame_id), "");

  memset(&s_base_status, 0, sizeof(s_base_status));
  bind_string(&s_base_status.header.frame_id, s_base_status_frame_id,
              sizeof(s_base_status_frame_id), "");
}

/* ------------------------------------------------------------------------- */
/* 상태                                                                       */
/* ------------------------------------------------------------------------- */

static rclc_support_t     s_support;
static rcl_node_t         s_node;
static rcl_allocator_t    s_allocator;
static rcl_publisher_t    s_pub_joint_states;
static rcl_publisher_t    s_pub_base_status;
static rcl_subscription_t s_sub_cmd_vel;
static rclc_executor_t    s_executor;
static bool               s_entities_created;

static agent_link_t       s_link;
static micro_ros_metrics_t s_metrics;

/* /cmd_vel이 ControlTask로 갈 때 쓰는 sequence. 레거시 ASCII 경로와 값이 겹치지
   않게 최상위 bit를 세워 둔다. g_rtos_metrics.applied_seq만 보고도 마지막 명령이
   어느 경로에서 왔는지 알 수 있다. */
#define CMD_VEL_SEQ_BASE  0x80000000UL
static uint32_t s_cmd_vel_seq;

static uint32_t s_next_joint_states_ms;
static uint32_t s_next_base_status_ms;
static uint32_t s_next_time_sync_ms;

/* ------------------------------------------------------------------------- */
/* /cmd_vel 콜백                                                              */
/* ------------------------------------------------------------------------- */

static void cmd_vel_callback(const void *message)
{
  const geometry_msgs__msg__TwistStamped *msg =
      (const geometry_msgs__msg__TwistStamped *)message;
  cmd_stamp_verdict_t verdict;
  control_command_t cmd;
  int64_t stamp_ns;
  uint32_t age_ms = 0U;
  uint32_t now_ms;
  int32_t tps_left;
  int32_t tps_right;

  s_metrics.cmd_vel_count++;

  stamp_ns = ((int64_t)msg->header.stamp.sec * NS_PER_S) +
             (int64_t)msg->header.stamp.nanosec;

  verdict = agent_link_check_stamp(rmw_uros_epoch_synchronized(), stamp_ns,
                                   rmw_uros_epoch_nanos(), CMD_VEL_MAX_AGE_MS,
                                   &age_ms);
  s_metrics.cmd_vel_last_verdict = (uint32_t)verdict;

  if (verdict != CMD_STAMP_OK)
  {
    /* 거절한 명령은 워치독을 먹이지 않는다. 그래야 낡은 명령만 계속 오는 상황에서
       MCU가 스스로 정지한다. */
    s_metrics.cmd_vel_rejected_count++;
    return;
  }

  if (!base_kin_twist_to_tps(&g_base_geometry,
                             (float)msg->twist.linear.x,
                             (float)msg->twist.angular.z,
                             &tps_left, &tps_right))
  {
    /* NaN/Inf가 왔다. 제어 경로에 들이지 않는다. */
    s_metrics.cmd_vel_rejected_count++;
    return;
  }

  now_ms = platform_now_ms();
  s_cmd_vel_seq++;

  cmd.seq = CMD_VEL_SEQ_BASE | s_cmd_vel_seq;
  cmd.kind = CONTROL_CMD_VEL;
  cmd.left = tps_left;
  cmd.right = tps_right;

  /* 명령의 나이만큼 기한을 앞당긴다. accepted_ms를 지금으로 두면 200 ms 늦게 도착한
     명령이 200 ms를 새로 얻고 생성 시각 기준 판정의 의미가 그 자리에서 사라진다.
     이미 흐른 시간을 빼서 넘긴다. */
  cmd.accepted_ms = now_ms - age_ms;
  cmd.deadline_ms = cmd.accepted_ms + CMD_WATCHDOG_MS;

  if (app_link_post_cmd_vel(&cmd))
  {
    s_metrics.cmd_vel_applied_count++;
    s_metrics.cmd_vel_age_ms = age_ms;
  }
  else
  {
    s_metrics.cmd_vel_rejected_count++;
  }
}

/* ------------------------------------------------------------------------- */
/* 엔티티 생성 / 정리                                                          */
/* ------------------------------------------------------------------------- */

static bool entities_create(void)
{
  rcl_ret_t ret;

  /* 생성 전에 0으로 둔다. 중간에 실패하면 호출자가 곧바로 entities_destroy()를
     부른다. 그때 아직 만들지 않은 엔티티에도 fini가 간다. rcl의 fini는 impl ==
     NULL을 안전하게 처리하도록 돼 있지만 그 성질에 기대는 대신 상태를 확정해 둔다.
     부분 실패 경로는 자주 지나가지 않아 틀려도 늦게 발견된다. */
  memset(&s_support, 0, sizeof(s_support));
  memset(&s_node, 0, sizeof(s_node));
  memset(&s_pub_joint_states, 0, sizeof(s_pub_joint_states));
  memset(&s_pub_base_status, 0, sizeof(s_pub_base_status));
  memset(&s_sub_cmd_vel, 0, sizeof(s_sub_cmd_vel));
  memset(&s_executor, 0, sizeof(s_executor));

  s_allocator = rcl_get_default_allocator();

  ret = rclc_support_init(&s_support, 0, NULL, &s_allocator);
  if (ret != RCL_RET_OK)
  {
    return false;
  }
  s_entities_created = true;   /* 여기부터는 실패해도 정리 대상이다 */

  ret = rclc_node_init_default(&s_node, MICRO_ROS_NODE_NAME, "", &s_support);
  if (ret != RCL_RET_OK)
  {
    return false;
  }

  ret = rclc_publisher_init_best_effort(
      &s_pub_joint_states, &s_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      TOPIC_JOINT_STATES);
  if (ret != RCL_RET_OK)
  {
    return false;
  }

  ret = rclc_publisher_init_best_effort(
      &s_pub_base_status, &s_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(mr_msgs, msg, BaseStatus),
      TOPIC_BASE_STATUS);
  if (ret != RCL_RET_OK)
  {
    return false;
  }

  ret = rclc_subscription_init_best_effort(
      &s_sub_cmd_vel, &s_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
      TOPIC_CMD_VEL);
  if (ret != RCL_RET_OK)
  {
    return false;
  }

  ret = rclc_executor_init(&s_executor, &s_support.context, 1U, &s_allocator);
  if (ret != RCL_RET_OK)
  {
    return false;
  }

  ret = rclc_executor_add_subscription(&s_executor, &s_sub_cmd_vel, &s_cmd_vel,
                                       &cmd_vel_callback, ON_NEW_DATA);
  if (ret != RCL_RET_OK)
  {
    return false;
  }

  /* 시간 동기는 여기서 처음 시도하고 실패하면 CONNECTED 안에서 재시도한다. 동기
     전에는 어떤 /cmd_vel도 실행하지 않으므로 연결 자체를 막을 필요는 없다. */
  s_next_time_sync_ms = platform_now_ms();
  return true;
}

static bool entities_destroy(void)
{
  bool ok = true;

  if (!s_entities_created)
  {
    return true;
  }

  /* agent가 사라진 뒤라 fini가 응답을 기다리다 실패할 수 있다. 그래도 끝까지 전부
     부순다. 하나가 실패했다고 멈추면 나머지가 pool에 남아 재접속마다 누수가 쌓인다. */
  if (rcl_publisher_fini(&s_pub_joint_states, &s_node) != RCL_RET_OK)
  {
    ok = false;
  }
  if (rcl_publisher_fini(&s_pub_base_status, &s_node) != RCL_RET_OK)
  {
    ok = false;
  }
  if (rcl_subscription_fini(&s_sub_cmd_vel, &s_node) != RCL_RET_OK)
  {
    ok = false;
  }
  if (rclc_executor_fini(&s_executor) != RCL_RET_OK)
  {
    ok = false;
  }
  if (rcl_node_fini(&s_node) != RCL_RET_OK)
  {
    ok = false;
  }
  if (rclc_support_fini(&s_support) != RCL_RET_OK)
  {
    ok = false;
  }

  s_entities_created = false;
  return ok;
}

/* ------------------------------------------------------------------------- */
/* 발행                                                                       */
/* ------------------------------------------------------------------------- */

static void publish_joint_states(uint32_t now_ms)
{
  control_status_t status;
  int64_t epoch_ns;
  int64_t stamp_ns;
  int32_t age_ms;

  if (!app_link_get_status(&status))
  {
    return;
  }

  /* snapshot이 만들어진 시각으로 stamp를 되돌린다. 지금 시각을 그대로 쓰면 최대 한
     control cycle(10 ms)만큼 늦은 데이터에 최신 시각을 붙이는 셈이다. `ros2 topic
     delay`는 그 오차를 못 본다.

     나이는 부호 있는 뺄셈으로 잰다. now_ms를 읽은 뒤 최고 우선순위인 ControlTask가
     선점해 더 새로운 snapshot을 게시하면 status.timestamp_ms가 now_ms보다 커진다.
     그때 unsigned로 빼면 약 4.29e9 ms(49.7일)가 나와 stamp가 49.7일 과거로
     날아간다. 음수 나이는 0으로 눌러 지금으로 본다. */
  age_ms = (int32_t)(now_ms - status.timestamp_ms);
  if (age_ms < 0)
  {
    age_ms = 0;
  }
  else if (age_ms > JOINT_STATES_MAX_AGE_MS)
  {
    /* snapshot이 여기까지 늙었다면 ControlTask가 멈췄다. 그런 값에 정확한 과거
       시각을 붙여 오도메트리가 조용히 적분하게 두지 않는다. */
    age_ms = JOINT_STATES_MAX_AGE_MS;
    s_metrics.joint_states_stale_count++;
  }

  epoch_ns = rmw_uros_epoch_nanos();
  stamp_ns = epoch_ns - ((int64_t)age_ms * NS_PER_MS);

  s_joint_state.header.stamp.sec = (int32_t)(stamp_ns / NS_PER_S);
  s_joint_state.header.stamp.nanosec = (uint32_t)(stamp_ns % NS_PER_S);

  s_joint_position[0] = base_kin_ticks_to_rad(&g_base_geometry, status.ticks_left);
  s_joint_position[1] = base_kin_ticks_to_rad(&g_base_geometry, status.ticks_right);
  s_joint_velocity[0] =
      (double)base_kin_tps_to_radps(&g_base_geometry, status.measured_tps_left);
  s_joint_velocity[1] =
      (double)base_kin_tps_to_radps(&g_base_geometry, status.measured_tps_right);

  if (rcl_publish(&s_pub_joint_states, &s_joint_state, NULL) == RCL_RET_OK)
  {
    s_metrics.joint_states_count++;
  }
  else
  {
    /* 발행 실패로 엔티티를 부수지 않는다. 링크 생사 판정은 ping 하나가 맡는다.
       판정자가 둘이면 어느 쪽이 옳은지 알 수 없다. */
    s_metrics.joint_states_fail_count++;
  }
}

/**
  * @brief  `/base/status` 10 Hz.
  * @note   여기 실리는 값은 전부 ControlTask가 게시한 snapshot 하나에서 온다.
  *         필드마다 따로 읽으면 서로 다른 cycle의 값이 한 메시지에 섞인다.
  */
static void publish_base_status(uint32_t now_ms)
{
  control_status_t status;
  int64_t stamp_ns;
  uint32_t cmd_age_ms;

  if (!app_link_get_status(&status))
  {
    return;
  }

  stamp_ns = rmw_uros_epoch_nanos();
  s_base_status.header.stamp.sec = (int32_t)(stamp_ns / NS_PER_S);
  s_base_status.header.stamp.nanosec = (uint32_t)(stamp_ns % NS_PER_S);

  s_base_status.agent_state = (uint8_t)s_link.state;

  /* `last_feed_ms`는 이미 stamp 시각이다. cmd_vel 콜백이 accepted_ms를 명령
     나이만큼 앞당겨 넘기므로 여기서 재는 것이 곧 `now - header.stamp`다. 계약이
     요구하는 값 그대로이고 epoch 동기 여부와 무관하게 성립한다. */
  cmd_age_ms = (uint32_t)(now_ms - status.last_feed_ms);
  s_base_status.cmd_age_s = (float)cmd_age_ms * 0.001f;
  s_base_status.cmd_watchdog_ok = (cmd_age_ms <= CMD_WATCHDOG_MS);

  s_base_status.duty_left = (float)status.duty_left_pm / (float)MOTOR_DUTY_MAX_PM;
  s_base_status.duty_right = (float)status.duty_right_pm / (float)MOTOR_DUTY_MAX_PM;
  s_base_status.saturated_left = status.saturated_left;
  s_base_status.saturated_right = status.saturated_right;

  s_base_status.loop_count = status.loop_count;
  s_base_status.loop_overruns = status.loop_overruns;

  if (rcl_publish(&s_pub_base_status, &s_base_status, NULL) == RCL_RET_OK)
  {
    s_metrics.base_status_count++;
  }
  else
  {
    s_metrics.base_status_fail_count++;
  }
}

/* ------------------------------------------------------------------------- */
/* CONNECTED 한 바퀴                                                          */
/* ------------------------------------------------------------------------- */

static bool spin_once(uint32_t now_ms)
{
  rcl_ret_t ret;

  if ((int32_t)(now_ms - s_next_time_sync_ms) >= 0)
  {
    /* 최초 동기와 주기 재동기를 같은 자리에서 한다. 다른 것은 타임아웃과 다음
       시각뿐이다. 아직 동기 전이면 자주 다시 시도하고 이미 동기됐으면 30초 뒤에
       짧게 갱신한다. 주기의 근거는 app_config.h에 있다. */
    bool was_synced = rmw_uros_epoch_synchronized();
    int timeout_ms = was_synced ? (int)AGENT_TIME_SYNC_RESYNC_TIMEOUT_MS
                                : (int)AGENT_TIME_SYNC_TIMEOUT_MS;

    if (rmw_uros_sync_session(timeout_ms) == RMW_RET_OK)
    {
      s_metrics.time_sync_count++;
      s_next_time_sync_ms = now_ms + AGENT_TIME_SYNC_PERIOD_MS;
    }
    else
    {
      s_metrics.time_sync_fail_count++;
      s_next_time_sync_ms = now_ms + AGENT_TIME_SYNC_RETRY_MS;
    }
  }

  /* timeout 0. 기다리는 일은 이 task의 vTaskDelayUntil이 한다. */
  ret = rclc_executor_spin_some(&s_executor, 0U);
  if ((ret != RCL_RET_OK) && (ret != RCL_RET_TIMEOUT))
  {
    return false;
  }

  if ((int32_t)(now_ms - s_next_joint_states_ms) >= 0)
  {
    /* 밀린 주기를 몰아 내보내지 않는다. 늦었으면 지금부터 다시 센다. */
    s_next_joint_states_ms += JOINT_STATES_PERIOD_MS;
    if ((int32_t)(now_ms - s_next_joint_states_ms) >= 0)
    {
      s_next_joint_states_ms = now_ms + JOINT_STATES_PERIOD_MS;
    }
    publish_joint_states(now_ms);
  }

  if ((int32_t)(now_ms - s_next_base_status_ms) >= 0)
  {
    s_next_base_status_ms += BASE_STATUS_PERIOD_MS;
    if ((int32_t)(now_ms - s_next_base_status_ms) >= 0)
    {
      s_next_base_status_ms = now_ms + BASE_STATUS_PERIOD_MS;
    }
    publish_base_status(now_ms);
  }

  return true;
}

/* ------------------------------------------------------------------------- */
/* 계측                                                                       */
/* ------------------------------------------------------------------------- */

static void collect_metrics(void)
{
  static_pool_metrics_t pool;
  platform_uart1_metrics_t link;

  static_pool_metrics(&s_pool, &pool);
  platform_uart1_metrics(&link);

  s_metrics.agent_state = (uint32_t)s_link.state;
  s_metrics.connect_count = s_link.connect_count;
  s_metrics.disconnect_count = s_link.disconnect_count;
  s_metrics.ping_fail_count = s_link.ping_fail_count;
  s_metrics.create_fail_count = s_link.create_fail_count;
  s_metrics.spin_fail_count = s_link.spin_fail_count;
  s_metrics.epoch_synchronized = rmw_uros_epoch_synchronized() ? 1U : 0U;

  s_metrics.pool_capacity = pool.capacity_bytes;
  s_metrics.pool_free = pool.free_bytes;
  s_metrics.pool_free_min = pool.free_bytes_min;
  s_metrics.pool_largest_free = pool.largest_free_block;
  s_metrics.pool_alloc_fail = pool.alloc_fail_count;
  s_metrics.pool_invalid_free = pool.invalid_free_count;
  s_metrics.pool_live_blocks = pool.live_blocks;

  s_metrics.uart1_error_count = link.error_count;
  s_metrics.uart1_error_last = link.error_last;
  s_metrics.uart1_rx_overflow_count = link.rx_overflow_count;
  s_metrics.uart1_rearm_count = link.rearm_count;
  s_metrics.uart1_tx_fail_count = link.tx_fail_count;
  s_metrics.pool_escaped_alloc = s_escaped_alloc_count;
}

void micro_ros_get_metrics(micro_ros_metrics_t *out)
{
  *out = s_metrics;
}

/* ------------------------------------------------------------------------- */
/* task 본체                                                                  */
/* ------------------------------------------------------------------------- */

void micro_ros_task(void *argument)
{
  TickType_t last_wake = xTaskGetTickCount();
  rcutils_allocator_t allocator;

  (void)argument;

  memset(&s_metrics, 0, sizeof(s_metrics));
  s_cmd_vel_seq = 0U;
  s_escaped_alloc_count = 0U;
  s_entities_created = false;

  /* allocator를 먼저 세운다. rcl은 첫 호출부터 할당한다. 여기가 늦으면 그 할당이
     newlib malloc으로 새어 나가고 그때는 이미 관측할 방법이 없다. */
  static_pool_init(&s_pool, s_pool_memory, sizeof(s_pool_memory));

  allocator = rcutils_get_zero_initialized_allocator();
  allocator.allocate = mr_allocate;
  allocator.deallocate = mr_deallocate;
  allocator.reallocate = mr_reallocate;
  allocator.zero_allocate = mr_zero_allocate;
  allocator.state = NULL;
  if (!rcutils_set_default_allocator(&allocator))
  {
    /* Error_Handler()가 아니라 rtos_app_fatal()이다. 둘 다 멈추지만 이쪽은 모터를
       먼저 끊는다. 여기 도달했으면 micro-ROS를 세울 수 없다. 그 상태로 마지막 PWM을
       남겨 둘 이유가 없다. */
    rtos_app_fatal();
  }

  message_memory_init();

  /* 이 task가 USART1의 유일한 소유자다. handle이 다 생긴 지금 처음 무장한다. */
  (void)platform_uart1_start();

  if (rmw_uros_set_custom_transport(true, NULL, transport_open, transport_close,
                                    transport_write,
                                    transport_read) != RMW_RET_OK)
  {
    rtos_app_fatal();
  }

  agent_link_init(&s_link, platform_now_ms());
  s_next_joint_states_ms = platform_now_ms();
  s_next_base_status_ms = platform_now_ms();

  for (;;)
  {
    agent_action_t action;
    uint32_t now_ms;
    bool ok = true;

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MICRO_ROS_POLL_MS));

    /* 세션과 무관하게 링크는 늘 살아 있어야 한다. transport_read를 타지 않는
       WAIT 주기에도 재무장 그물을 건다. */
    (void)platform_uart1_rearm_if_needed();

    now_ms = platform_now_ms();
    action = agent_link_next(&s_link, now_ms);

    switch (action)
    {
      case AGENT_ACTION_PING:
        ok = (rmw_uros_ping_agent((int)AGENT_PING_TIMEOUT_MS,
                                  (uint8_t)AGENT_PING_ATTEMPTS) == RMW_RET_OK);
        break;

      case AGENT_ACTION_CREATE:
        ok = entities_create();
        if (!ok)
        {
          /* 절반만 만들어진 것을 그대로 두면 pool이 샌다. */
          (void)entities_destroy();
        }
        break;

      case AGENT_ACTION_SPIN:
        ok = spin_once(now_ms);
        break;

      case AGENT_ACTION_DESTROY:
        ok = entities_destroy();
        break;

      case AGENT_ACTION_WAIT:
      default:
        break;
    }

    /* ping/spin/create가 시간을 먹었을 수 있다. 상태 전이 시각은 지금이 맞다. */
    agent_link_report(&s_link, platform_now_ms(), action, ok);

    if (agent_link_take_stop_request(&s_link))
    {
      /* 래치하지 않는 정지다. reset이나 버튼 없이 재접속만으로 되살아나야 한다.
         정지 자체는 MCU 내부 200 ms 워치독도 따로 보장한다. */
      app_link_urgent_stop(MOTOR_FAULT_NONE);
    }

    collect_metrics();
  }
}
