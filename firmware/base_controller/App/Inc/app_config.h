/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   펌웨어 확정 상수. HAL/RTOS에 의존하지 않는다.
  * @note    순수 모듈과 STM32 adapter가 함께 쓰는 헤더다. 여기에 HAL 헤더나
  *          main.h를 들이면 host 단위 테스트가 깨진다.
  ******************************************************************************
  */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* ---- 주기 ---- */
#define LD2_TOGGLE_PERIOD_MS      500U
#define ENCODER_SAMPLE_PERIOD_MS   10U  /* == CONTROL_PERIOD_MS. ControlTask가 샘플한다. */
#define TICKS_REPORT_PERIOD_MS     20U  /* 50 Hz */

/* ---- RTOS 실행 구조 ---- */

/* TIM6가 만드는 control tick. .ioc의 TIM6 8399/99(84 MHz/8400/100)와 같아야 한다. */
#define CONTROL_RATE_HZ           100U
#define CONTROL_PERIOD_MS          10U

/* native FreeRTOS numeric priority다. FreeRTOSConfig.h의 configMAX_PRIORITIES=56은
   ST CMSIS-RTOS v2 wrapper의 compile-time 요구값일 뿐이고 아래 값과 무관하다. */
#define TASK_PRIO_CONTROL           5U
#define TASK_PRIO_HEALTH            4U
#define TASK_PRIO_LEGACY_IO         2U

/* 길이 단위는 StackType_t word다 (1 word = 4 B). CMSIS osThreadAttr_t의 stack_size는
   byte라 값이 다르다. 이 프로젝트는 task를 native xTaskCreateStatic으로만 만들므로
   전부 word로 읽는다. */
#define TASK_STACK_WORDS_CONTROL   384U  /* 1,536 B */
#define TASK_STACK_WORDS_HEALTH    256U  /* 1,024 B */
#define TASK_STACK_WORDS_IO        512U  /* 2,048 B */
#define TASK_STACK_WORDS_IDLE      256U  /* 1,024 B */

#define HEALTH_PERIOD_MS           50U

/* HIL 전용 build option. 평소에는 0이다. CMake의 -DIWDG_STALL_TEST=ON일 때만
   ControlTask 정지 주입 hook을 컴파일한다. */
#ifndef IWDG_STALL_TEST
#define IWDG_STALL_TEST              0
#endif

/* HIL 전용 build option. CMake의 -DU3_SMOKE_TEST=ON일 때만 SmokeTask와 u3_smoke.c가
   들어간다. production image에는 USART1로 한 바이트도 나가지 않는다. */
#ifndef U3_SMOKE_TEST
#define U3_SMOKE_TEST                0
#endif

/* SmokeTask는 가장 낮은 우선순위다. 시험 트래픽이 제어·관측·레거시 IO 중 어느 것도
   지연시킬 수 없어야 "제어는 그대로"라는 전제가 성립한다. */
#define TASK_PRIO_SMOKE             1U
#define TASK_STACK_WORDS_SMOKE    384U  /* 1,536 B */

/* USART1 DMA 링을 긁는 주기. 보드레이트가 올라가도 이 주기만 지키면 링이 넘치지 않는다. */
#define U3_SMOKE_POLL_MS            1U

/* T 스트림 주기. `/joint_states` 계약과 같은 50 Hz로 두어 실제 부하를 흉내 낸다. */
#define U3_STREAM_PERIOD_MS        20U

/* 한 주기에 긁어 갈 최대 바이트. 3 Mbaud의 1 ms 분량(약 300 B)보다 커야 한다. */
#define U3_SMOKE_READ_CHUNK       512U

/* 보드레이트 스윕 안전망. 속도를 바꾼 뒤 이 시간 안에 유효한 줄이 하나도 오지 않으면
   기본 속도로 되돌린다. 되돌리지 않으면 실패한 단계마다 보드 리셋이 필요하다.

   `.ioc`의 `USART1.BaudRate`와 같아야 한다. 갈리면 자동 복귀가 엉뚱한 속도로 가서
   링크가 살아나지 않는다. */
#define U3_BAUD_DEFAULT        921600U
#define U3_BAUD_REVERT_MS        5000U
/* ACK를 보낸 뒤 호스트가 자기 포트를 다시 여는 데 주는 시간. */
#define U3_BAUD_ACK_SETTLE_MS      50U

/* LegacyIoTask는 개행 notification으로 깨어난다. 이 timeout은 telemetry/워치독
   통지/재무장을 위한 그물이다. 명령 지연이 아니라 주기 작업의 jitter 상한이다. */
#define LEGACY_IO_POLL_MS           2U

/* ControlTask가 명령을 반영하고 결과를 돌려줄 때까지 기다리는 상한. ControlTask는
   최고 우선순위라 정상적으로는 게시 즉시 선점해 결과가 이미 와 있다. 이 시간을
   넘기면 적용 근거가 없으므로 err다. */
#define CMD_APPLY_TIMEOUT_MS       30U

/* HealthTask가 control heartbeat 정체로 보고 IWDG refresh를 중단하는 한계. */
#define HEALTH_HEARTBEAT_STALL_MS 100U

/* ---- micro-ROS ---- */

/* CMake의 -DMICRO_ROS=ON일 때만 MicroRosTask와 libmicroros.a가 들어간다.
   U3_SMOKE_TEST와는 동시에 켤 수 없다. 둘 다 USART1의 소유자를 자처한다. */
#ifndef MICRO_ROS
#define MICRO_ROS                    0
#endif

/* ControlTask보다 낮고 LegacyIoTask보다 높다.
     - ControlTask(P5)보다 낮아야 하는 이유: micro-ROS는 blocking 송신과 상한이
       분명치 않은 executor를 품고 있다. 100 Hz 제어를 막을 수 없어야 한다.
     - HealthTask(P4)보다 낮아야 하는 이유: reconnect storm 중에도 IWDG는 계속
       먹어야 한다.
     - LegacyIoTask(P2)보다 높아야 하는 이유: 이제 실제 주행 명령이 이 경로로
       온다. 디버그용 ASCII 텔레메트리가 /cmd_vel을 밀어내면 안 된다. */
#define TASK_PRIO_MICRO_ROS         3U

/* rcl/rclc의 호출 깊이가 있어 LegacyIoTask보다 넉넉히 잡는다. 실제 사용량은
   HealthTask가 재는 stack high-water mark로 확인하고 줄인다. */
#define TASK_STACK_WORDS_MICRO_ROS 1536U  /* 6,144 B */

/* MicroRosTask 한 바퀴의 상한. 이 주기가 곧 `/joint_states` 타이밍 분해능이다. */
#define MICRO_ROS_POLL_MS            2U

/* 계약이 정한 발행 주기. */
#define JOINT_STATES_PERIOD_MS      20U  /* 50 Hz */
#define BASE_STATUS_PERIOD_MS      100U  /* 10 Hz */

/* `/joint_states` stamp를 되돌릴 수 있는 최대 나이. 정상값은 한 control cycle
   (10 ms) 이내다. 여기에 닿았다면 ControlTask가 멈춘 것이므로 더 과거로 보내는
   대신 상한에서 자르고 계수한다. */
#define JOINT_STATES_MAX_AGE_MS    100

/* micro-ROS 전용 pool. FreeRTOS heap도 newlib malloc도 쓰지 않는 이유는 static_pool.h에
   있다. 실사용량의 여러 배로 잡아 둔 값이다. 딱 맞게 깎으면 토픽을 하나 늘리는 순간
   `alloc_fail`이 뜨는데, 그 실패는 "엔티티 생성 실패 -> 재접속 반복"으로 나타나 원인을
   엉뚱하게 트랜스포트에서 찾게 된다. 토픽을 늘렸다면 `mr_pool_free_min`으로 여유를
   다시 확인한다. */
#define MICRO_ROS_POOL_BYTES      8192U  /* 8 KiB */

/* custom transport read의 폴링 주기. platform_uart1의 DMA 링을 긁는 간격이며
   U3_SMOKE_POLL_MS와 같은 근거다(링 2048 B는 921600에서 22 ms 분량). */
#define MICRO_ROS_TRANSPORT_POLL_MS  1U

/* agent 연결 상태기계 (agent_link.c). */
#define AGENT_BACKOFF_MIN_MS       500U  /* 첫 재시도 간격 */
#define AGENT_BACKOFF_MAX_MS      4000U  /* 지수 증가의 상한 */
#define AGENT_ALIVE_PING_MS        500U  /* CONNECTED 중 생존 확인 주기 */
#define AGENT_PING_RETRY_MS        100U  /* 한 번 놓쳤을 때의 재확인 간격 */
#define AGENT_PING_FAIL_LIMIT        2U  /* 이만큼 연속 실패하면 끊긴 것으로 본다 */

/* rmw_uros_ping_agent에 주는 인자. ping 하나가 이보다 오래 executor를 막지 않는다. */
#define AGENT_PING_TIMEOUT_MS       30U
#define AGENT_PING_ATTEMPTS          1U

/* rmw_uros_sync_session 타임아웃. 동기 전에는 어떤 /cmd_vel도 실행하지 않으므로
   연결 직후 반드시 한 번 성공해야 로봇이 움직인다. */
#define AGENT_TIME_SYNC_TIMEOUT_MS 200U
#define AGENT_TIME_SYNC_RETRY_MS  1000U

/* 동기는 한 번으로 끝나지 않는다. `rmw_uros_epoch_nanos()`는 동기 시점의 offset에
   MCU 자기 시계를 더한 값이다. MCU tick과 Pi 시계는 결이 달라 그 offset이 시간에
   비례해 낡는다. 그대로 두면 링크는 멀쩡한데 stamp만 늙어 `ros2 topic delay`가
   조용히 벌어진다. 30초 주기면 누적 오차가 1 ms 아래로 남는다. 재동기 타임아웃은
   최초 동기보다 짧다. 세션이 이미 서 있으면 오래 기다려 봐야 발행만 밀리기
   때문이다. */
#define AGENT_TIME_SYNC_PERIOD_MS 30000U
#define AGENT_TIME_SYNC_RESYNC_TIMEOUT_MS 50U

/* `/cmd_vel`의 나이 상한. MCU 내부 명령 워치독과 같은 값이어야 한다. 생성 시각 기준
   판정과 내부 워치독이 서로 다른 기한을 쓰면 둘 중 느슨한 쪽이 실제 기한이 된다. */
#define CMD_VEL_MAX_AGE_MS  CMD_WATCHDOG_MS

/* `/cmd_vel`의 header.frame_id를 받을 자리. 계약상 MCU는 이 값을 무시하지만 정적
   할당에서는 역직렬화가 쓸 공간을 미리 줘야 한다. 넘치면 그 메시지 하나가 버려질 뿐
   링크는 살아 있다. */
#define CMD_VEL_FRAME_ID_CAP        32U

/* ---- 엔코더 ---- */

/* 로봇 전진 시 좌우 누적 틱이 모두 +가 되게 하는 보정. 두 바퀴가 마주 보게 달려 있어
   원시 부호가 반대다. 배선이 아니라 여기서 잡는다. */
#define ENCODER_SIGN_LEFT         (+1)
#define ENCODER_SIGN_RIGHT        (-1)

/* ---- 모터 ---- */

/* TIM1 ARR. .ioc의 TIM1.Period와 반드시 같아야 하며 platform_init()에서 대조한다. */
#define MOTOR_PWM_ARR             4666U

/* duty 지령 단위는 per-mille 정수다. MCU에서 부동소수 파싱을 하지 않는다. */
#define MOTOR_DUTY_MAX_PM         1000

/* 양수 duty = 로봇 전진이 되게 하는 보정. 엔코더 부호와 마찬가지로 두 바퀴가 마주 보게
   달려 있어 한쪽이 -1이다. 배선을 바꾸지 않고 여기서 잡는다. */
#define MOTOR_SIGN_LEFT           (-1)
#define MOTOR_SIGN_RIGHT          (+1)

/* ---- 속도 폐루프 ---- */

/* `vel` 지령의 클램프. 운용 상한이자 PI 헤드룸 20 %와 피드포워드 선형 회귀 구간
   (<= 750 ‰)의 상한을 겸한다. */
#define VEL_TARGET_MAX_TPS        6000

/* 피드포워드 계수. duty-tps 회귀의 기울기 역수와 x절편이다.

     duty_ff = run_intercept * sign(target) + kff * target

   deadband_*(약 50 ‰)는 이 식에 들어가지 않는다. 정지마찰과 운동마찰은 다른
   물리량이다. 전자를 상시 피드포워드에 넣으면 약 30 ‰의 상수 바이어스가 전 운전점에
   걸려 적분기가 늘 그것을 상쇄하게 된다. */
#define KFF_LEFT                  0.1187f   /* ‰/tps */
#define KFF_RIGHT                 0.1194f   /* ‰/tps */
#define RUN_INTERCEPT_LEFT        18.4f     /* ‰ */
#define RUN_INTERCEPT_RIGHT       23.8f     /* ‰ */

/* PI 게인. 추측값이 아니라 플랜트 시정수에서 산술로 나온 값이어야 한다.
     Ti = tau,  Kp = kff * tau / tau_cl,  Ki_cycle = Kp * T / Ti

   다시 튜닝할 때는 한 번에 하나씩 바꾼다. FF 단독(Kp = Ki_cycle = 0)으로 기준을
   잡고, P를 얹고(tau_cl = 2 tau), 마지막에 I를 넣는다. 아래 값은 tau_cl = 2 tau에서
   나왔다. */
#define SPEED_KP_LEFT             0.05935f  /* ‰/tps — KFF_LEFT / 2 */
#define SPEED_KP_RIGHT            0.05970f  /* ‰/tps — KFF_RIGHT / 2 */
#define SPEED_KI_CYCLE_LEFT       0.008479f /* ‰/tps per cycle */
#define SPEED_KI_CYCLE_RIGHT      0.008178f /* ‰/tps per cycle */

/* 적분기 절댓값 상한. 운용 상한에서 FF가 이미 730 ‰ 남짓을 쓰므로 포화까지 남은
   여유가 약 260 ‰다. 적분기가 그 여유를 넘게 두면 anti-windup과 별개로 클램프
   자체가 의미를 잃는다. 반대로 적분기가 이 값을 지속적으로 쓴다면 게인이 아니라
   모델이나 하드웨어 문제이므로 튜닝을 멈추고 원인을 본다. */
#define SPEED_I_MAX_PM            250.0f    /* ‰ */

/* ---- 명령 워치독 ---- */

/* 마지막 유효 명령으로부터 이 시간이 지나면 duty 0. 호스트 툴이 죽거나 USB가 빠져도
   모터가 계속 돌지 않게 한다. /cmd_vel 10 Hz 발행에서 2주기를 놓칠 여유다. */
#define CMD_WATCHDOG_MS           200U

/* ---- 통신 버퍼 ---- */
#define CMD_RX_RING_SIZE          64U  /* 2의 거듭제곱이어야 한다 */
#define CMD_LINE_CAP              32U
#define TX_LINE_CAP               64U

/* 한 번의 super-loop에서 소비할 RX 바이트/줄 상한. 호스트가 쓰레기를 최고속으로
   퍼부어도 command_poll이 워치독 평가를 굶기지 못하게 하는 유일한 장치다.
   230400 8N1에서 64바이트는 약 2.8 ms 분량이다. */
#define CMD_POLL_MAX_BYTES        64U
#define CMD_POLL_MAX_LINES         4U

/* 블로킹 송신의 유한 타임아웃. 최악의 줄 59 B는 230400 8N1에서 약 2.6 ms다.
   여기서 걸리면 주변장치가 죽은 것으로 보고 fault를 래치한다. */
#define UART_TX_TIMEOUT_MS         10U

/* 수신 재무장이 이만큼 연속 실패하면 링크가 죽은 것으로 보고 fault를 래치한다. */
#define CMD_RX_REARM_FAIL_LIMIT     8U

/* ---- 출력 줄 최악 길이 ---- */

/* 가장 긴 태그는 "ticks"(5). 타임스탬프는 uint32(최대 10자리), 누적 틱은
   int64(부호 포함 최대 20자). 값마다 앞에 공백 하나가 붙고 줄 끝에 '\n'이 온다. */
#define LINE_MAX_TAG_LEN           5U
#define LINE_U32_MAX_CHARS        10U
#define LINE_I32_MAX_CHARS        11U
#define LINE_I64_MAX_CHARS        20U
#define TX_WORST_LINE_LEN         (LINE_MAX_TAG_LEN                     \
                                   + (1U + LINE_U32_MAX_CHARS)          \
                                   + (2U * (1U + LINE_I64_MAX_CHARS))   \
                                   + 1U)

/* `spd <ms> <tps_l> <duty_l> <tps_r> <duty_r>` — 값 4개다. 여기 실리는 값은 전부 int32
   범위 안이라 i64가 아니라 i32 폭으로 잡는다. 값을 하나 더 늘리면 87 B가 되어
   TX_LINE_CAP 확장과 UART_TX_TIMEOUT_MS 여유 재계산이 따라온다. */
#define TX_WORST_SPD_LINE_LEN     (3U                                   \
                                   + (1U + LINE_U32_MAX_CHARS)          \
                                   + (4U * (1U + LINE_I32_MAX_CHARS))   \
                                   + 1U)

/* ---- 불변식 ---- */
_Static_assert((CMD_RX_RING_SIZE & (CMD_RX_RING_SIZE - 1U)) == 0U,
               "CMD_RX_RING_SIZE는 2의 거듭제곱이어야 한다 (인덱스를 & 로 감는다)");
_Static_assert(CMD_RX_RING_SIZE >= 2U && CMD_RX_RING_SIZE <= 0x8000U,
               "CMD_RX_RING_SIZE가 uint16 인덱스 범위를 벗어난다");
_Static_assert(CMD_LINE_CAP >= 16U,
               "CMD_LINE_CAP이 'duty -1000 -1000' 을 담지 못한다");
_Static_assert(TX_LINE_CAP >= TX_WORST_LINE_LEN,
               "TX_LINE_CAP이 최악의 ticks 줄보다 작다");
_Static_assert((MOTOR_SIGN_LEFT == 1) || (MOTOR_SIGN_LEFT == -1),
               "MOTOR_SIGN_LEFT는 +1 또는 -1이어야 한다");
_Static_assert((MOTOR_SIGN_RIGHT == 1) || (MOTOR_SIGN_RIGHT == -1),
               "MOTOR_SIGN_RIGHT는 +1 또는 -1이어야 한다");
_Static_assert((ENCODER_SIGN_LEFT == 1) || (ENCODER_SIGN_LEFT == -1),
               "ENCODER_SIGN_LEFT는 +1 또는 -1이어야 한다");
_Static_assert((ENCODER_SIGN_RIGHT == 1) || (ENCODER_SIGN_RIGHT == -1),
               "ENCODER_SIGN_RIGHT는 +1 또는 -1이어야 한다");
_Static_assert(MOTOR_DUTY_MAX_PM > 0, "MOTOR_DUTY_MAX_PM은 양수여야 한다");
_Static_assert(CMD_POLL_MAX_BYTES > 0U && CMD_POLL_MAX_LINES > 0U,
               "command_poll 상한이 0이면 명령이 영영 처리되지 않는다");
_Static_assert(ENCODER_SAMPLE_PERIOD_MS == CONTROL_PERIOD_MS,
               "엔코더 샘플은 ControlTask cycle에서만 일어난다");
_Static_assert(TASK_PRIO_CONTROL > TASK_PRIO_HEALTH,
               "ControlTask가 HealthTask보다 낮으면 통신/관측이 제어를 막을 수 있다");
_Static_assert(TASK_PRIO_HEALTH > TASK_PRIO_LEGACY_IO,
               "LegacyIoTask는 가장 낮아야 한다 — UART 블로킹 송신이 여기서 일어난다");
_Static_assert(TASK_PRIO_LEGACY_IO > TASK_PRIO_SMOKE,
               "SmokeTask가 LegacyIoTask보다 높으면 시험 트래픽이 명령/텔레메트리를 밀어낸다");
_Static_assert(TASK_PRIO_SMOKE > 0U,
               "SmokeTask가 idle 우선순위면 T 스트림이 굶는다");
_Static_assert(!(MICRO_ROS && U3_SMOKE_TEST),
               "micro-ROS와 U3 smoke는 둘 다 USART1 소유자다. 동시에 켤 수 없다");
_Static_assert(TASK_PRIO_HEALTH > TASK_PRIO_MICRO_ROS,
               "MicroRosTask가 HealthTask보다 높으면 reconnect storm이 IWDG를 굶긴다");
_Static_assert(TASK_PRIO_CONTROL > TASK_PRIO_MICRO_ROS,
               "MicroRosTask가 ControlTask보다 높으면 통신이 100 Hz 제어를 막는다");
_Static_assert(TASK_PRIO_MICRO_ROS > TASK_PRIO_LEGACY_IO,
               "레거시 ASCII 텔레메트리가 /cmd_vel을 밀어내면 안 된다");
_Static_assert(JOINT_STATES_PERIOD_MS >= MICRO_ROS_POLL_MS,
               "발행 주기가 task 주기보다 짧으면 한 바퀴에 두 번 발행이 밀린다");
_Static_assert(BASE_STATUS_PERIOD_MS >= JOINT_STATES_PERIOD_MS,
               "/base/status(10 Hz)가 /joint_states(50 Hz)보다 잦으면 계약 위반이다");
_Static_assert(CMD_VEL_MAX_AGE_MS == CMD_WATCHDOG_MS,
               "stamp 기준 기한과 MCU 내부 워치독이 갈리면 느슨한 쪽이 실제 기한이 된다");
_Static_assert(AGENT_BACKOFF_MAX_MS >= AGENT_BACKOFF_MIN_MS,
               "backoff 상한이 하한보다 작으면 지수 증가가 즉시 잘린다");
_Static_assert(AGENT_PING_FAIL_LIMIT >= 1U,
               "ping 실패 허용치가 0이면 상태기계가 진행하지 못한다");
_Static_assert(U3_STREAM_PERIOD_MS >= U3_SMOKE_POLL_MS,
               "T 스트림 주기가 poll 주기보다 짧으면 한 주기에 두 줄이 밀린다");
_Static_assert(CMD_APPLY_TIMEOUT_MS >= (3U * CONTROL_PERIOD_MS),
               "적용 대기 상한이 control cycle 몇 번보다 짧으면 정상 명령이 err가 된다");
_Static_assert(CMD_WATCHDOG_MS > CONTROL_PERIOD_MS,
               "워치독이 control 주기보다 짧으면 명령이 반영되기 전에 만료된다");
_Static_assert(TX_LINE_CAP >= TX_WORST_SPD_LINE_LEN,
               "TX_LINE_CAP이 최악의 spd 줄보다 작다");
_Static_assert(VEL_TARGET_MAX_TPS > 0 && VEL_TARGET_MAX_TPS <= 8000,
               "vel 목표 상한이 모터의 물리 상한 밖이다");

/* 게인 범위. 폐루프에서 부호가 틀리면 튜닝으로 나타나지 않고 첫 지령에서 폭주한다.
   음의 게인은 그 자체로 양의 피드백이므로 컴파일에서 막는다. */
_Static_assert(SPEED_KP_LEFT >= 0.0f && SPEED_KP_RIGHT >= 0.0f,
               "Kp가 음수면 error가 커지는 방향으로 duty를 낸다 (양의 피드백)");
_Static_assert(SPEED_KI_CYCLE_LEFT >= 0.0f && SPEED_KI_CYCLE_RIGHT >= 0.0f,
               "Ki_cycle이 음수면 적분이 오차를 키운다 (양의 피드백)");
_Static_assert(SPEED_I_MAX_PM > 0.0f && SPEED_I_MAX_PM < (float)MOTOR_DUTY_MAX_PM,
               "적분기 상한이 duty 클램프 이상이면 클램프가 의미를 잃는다");
_Static_assert(KFF_LEFT > 0.0f && KFF_RIGHT > 0.0f,
               "kff는 duty-tps 회귀 기울기의 역수이므로 양수다");

/* 최저 시험점의 FF가 정지마찰(좌 50.8 / 우 49.2 ‰)을 충분히 넘는지. 못 넘으면
   기동 킥 없이는 출발하지 못한다. */
_Static_assert((RUN_INTERCEPT_LEFT + (KFF_LEFT * 1000.0f)) > (2.0f * 50.8f),
               "좌 최저 시험점 1000 tps의 FF가 정지마찰의 2배에 못 미친다");
_Static_assert((RUN_INTERCEPT_RIGHT + (KFF_RIGHT * 1000.0f)) > (2.0f * 49.2f),
               "우 최저 시험점 1000 tps의 FF가 정지마찰의 2배에 못 미친다");

/* 운용 상한에서 피드포워드가 회귀 선형 구간(<= 750 ‰) 안에 남는지. */
_Static_assert((RUN_INTERCEPT_LEFT + (KFF_LEFT * (float)VEL_TARGET_MAX_TPS)) <= 750.0f,
               "좌 운용 상한의 FF가 회귀 선형 구간을 벗어난다");
_Static_assert((RUN_INTERCEPT_RIGHT + (KFF_RIGHT * (float)VEL_TARGET_MAX_TPS)) <= 750.0f,
               "우 운용 상한의 FF가 회귀 선형 구간을 벗어난다");

#endif /* APP_CONFIG_H */
