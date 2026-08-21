/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   M3 개루프 펌웨어의 확정 상수. HAL/RTOS에 의존하지 않는다.
  * @note    이 헤더는 순수 모듈과 STM32 adapter가 함께 쓴다. 여기에 HAL 헤더나
  *          main.h를 들이지 않는다 — 들어오는 순간 host 단위 테스트가 깨진다.
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

/* 길이 단위는 StackType_t **word**다 (1 word = 4 B). CMSIS osThreadAttr_t의
   stack_size는 **byte**라 값이 다르다 — 이 프로젝트는 task를 native
   xTaskCreateStatic으로만 만들므로 전부 word로 읽는다. */
#define TASK_STACK_WORDS_CONTROL   384U  /* 1,536 B */
#define TASK_STACK_WORDS_HEALTH    256U  /* 1,024 B */
#define TASK_STACK_WORDS_IO        512U  /* 2,048 B */
#define TASK_STACK_WORDS_IDLE      256U  /* 1,024 B */

#define HEALTH_PERIOD_MS           50U

/* LegacyIoTask는 개행 notification으로 깨어난다. 이 timeout은 telemetry/워치독
   통지/재무장을 위한 그물이며, 명령 지연이 아니라 주기 작업의 jitter 상한이다. */
#define LEGACY_IO_POLL_MS           2U

/* ControlTask가 명령을 반영하고 결과를 돌려줄 때까지 기다리는 **상한**.
   ControlTask는 최고 우선순위라 정상적으로는 게시 즉시 선점해 결과가 이미 와 있다.
   이 시간을 넘기면 적용 근거가 없으므로 ok가 아니라 err다. */
#define CMD_APPLY_TIMEOUT_MS       30U

/* HealthTask가 control heartbeat 정체로 보는 한계. 관측만 하고 IWDG는 켜지 않는다. */
#define HEALTH_HEARTBEAT_STALL_MS 100U

/* ---- 엔코더 ---- */

/* 로봇 전진 시 좌우 누적 틱이 모두 +가 되게 하는 보정. 2026-08-20 M2 실측:
   왼쪽 원시 부호 +, 오른쪽 원시 부호 -. 두 바퀴가 마주 보게 달려 있어 생기는
   차이이므로 배선이 아니라 여기서 잡는다. */
#define ENCODER_SIGN_LEFT         (+1)
#define ENCODER_SIGN_RIGHT        (-1)

/* ---- 모터 ---- */

/* TIM1 ARR. .ioc의 TIM1.Period와 반드시 같아야 하며 platform_init()에서 대조한다. */
#define MOTOR_PWM_ARR             4666U

/* duty 지령 단위는 per-mille 정수다. MCU에서 부동소수 파싱을 하지 않는다. */
#define MOTOR_DUTY_MAX_PM         1000

/* 양수 duty = 로봇 전진이 되게 하는 보정. 두 바퀴가 마주 보게 달려 있어 한쪽이
   -1이다. M3.md 5.3에서 실측 확정 (2026-08-21): 좌 +100 지령에 후진(-622 tps),
   우 +100 지령에 전진. 배선을 바꾸지 않고 여기서 잡는다. */
#define MOTOR_SIGN_LEFT           (-1)
#define MOTOR_SIGN_RIGHT          (+1)

/* ---- 속도 폐루프 (M4) ---- */

/* `vel` 지령의 클램프. limits.md 2절의 운용 상한 max_tps = 6000이며, PI 헤드룸
   20 %와 피드포워드 선형 회귀 구간(<= 750 ‰)의 상한을 겸한다. */
#define VEL_TARGET_MAX_TPS        6000

/* 피드포워드 계수. M4-B duty-tps 재측정(2026-08-21, Vbat 13.2 V, 무부하)의
   기울기 역수와 x절편이다. 좌우·정역 공통 회귀 구간은 <= 750 ‰이며,
   방향별 R^2 >= 0.9997이다.

     duty_ff = run_intercept * sign(target) + kff * target

   **deadband_*(약 50 ‰)는 이 식에 들어가지 않는다.** 정지마찰과 운동마찰은 다른
   물리량이고, 전자를 상시 피드포워드에 넣으면 약 30 ‰의 상수 바이어스가 전
   운전점에 걸려 적분기가 늘 그것을 상쇄하게 된다 (limits.md 2절). */
#define KFF_LEFT                  0.1187f   /* ‰/tps */
#define KFF_RIGHT                 0.1194f   /* ‰/tps */
#define RUN_INTERCEPT_LEFT        18.4f     /* ‰ */
#define RUN_INTERCEPT_RIGHT       23.8f     /* ‰ */

/* PI 게인. **추측값이 아니라 플랜트 시정수에서 산술로 나온 값이어야 한다.**
   M4.md 7.2:  Ti = tau,  Kp = kff * tau / tau_cl,  Ki_cycle = Kp * T / Ti.

   튜닝은 M4.md 7.3의 순서대로 한 번에 하나씩 바꾼다.
     1단계 FF 단독 : Kp = 0, Ki_cycle = 0
     2단계 P 추가  : Kp = kff / 2 (tau_cl = 2 tau), Ki_cycle = 0
     3단계 I 추가  : Ki_cycle = Kp * 0.01 / tau  <- 지금 여기

   M4-B 실측 tau는 좌 70 ms, 우 73 ms다. 초기 tau_cl은 각각 2 tau로 두며,
   FF-only와 P-only 기준 측정을 마치고 아래 초기 PI 게인을 적용했다. */
#define SPEED_KP_LEFT             0.05935f  /* ‰/tps — KFF_LEFT / 2 */
#define SPEED_KP_RIGHT            0.05970f  /* ‰/tps — KFF_RIGHT / 2 */
#define SPEED_KI_CYCLE_LEFT       0.008479f /* ‰/tps per cycle */
#define SPEED_KI_CYCLE_RIGHT      0.008178f /* ‰/tps per cycle */

/* 적분기 절댓값 상한. 운용 상한 6000 tps에서 FF가 731 ~ 740 ‰이므로 포화까지
   남은 여유가 약 260 ‰다. 적분기가 그 여유를 넘게 두면 anti-windup과 별개로
   클램프 자체가 의미를 잃는다. 반대로 적분기가 이 값을 지속적으로 쓰고 있다면
   게인 문제가 아니라 모델이나 하드웨어 문제이므로 튜닝을 멈추고 원인을 본다. */
#define SPEED_I_MAX_PM            250.0f    /* ‰ */

/* ---- 명령 워치독 ---- */

/* 마지막 유효 명령으로부터 이 시간이 지나면 duty 0. 호스트 툴이 죽거나 USB가
   빠져도 모터가 계속 돌지 않게 한다. limits.md 4절의 확정값이며 /cmd_vel 10 Hz
   발행에서 2주기를 놓칠 여유다. M3의 500 ms는 사람이 개입하는 수동 시험 기준이었고
   M4부터는 호스트 툴이 REFRESH_S = 0.05로 주기 갱신한다. */
#define CMD_WATCHDOG_MS           200U

/* ---- 통신 버퍼 ---- */
#define CMD_RX_RING_SIZE          64U  /* 2의 거듭제곱이어야 한다 */
#define CMD_LINE_CAP              32U
#define TX_LINE_CAP               64U

/* 한 번의 super-loop에서 소비할 RX 바이트/줄 상한. 호스트가 쓰레기를 최고속으로
   퍼부어도 command_poll이 워치독 평가를 굶기지 못하게 하는 유일한 장치다.
   230400 8N1에서 64바이트는 약 2.8 ms 분량이라 500 ms 워치독에 여유가 크다. */
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

/* `spd <ms> <tps_l> <duty_l> <tps_r> <duty_r>` — 값 4개다. 여기 실리는 값은 전부
   int32 범위 안이다: 측정 tps는 접힌 엔코더 델타(<= +-32767) x 100 Hz, duty는
   +-1000, 목표는 +-VEL_TARGET_MAX_TPS. 그래서 i64가 아니라 i32 폭으로 잡는다.
   값 5개(목표 추가)로 늘리면 87 B가 되어 TX_LINE_CAP 확장과 UART_TX_TIMEOUT_MS
   여유 재계산이 따라온다 — 목표는 ok가 이미 알려줬고 적분기와 포화 플래그는
   SWD로 g_rtos_metrics에서 본다. */
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
_Static_assert(CMD_APPLY_TIMEOUT_MS >= (3U * CONTROL_PERIOD_MS),
               "적용 대기 상한이 control cycle 몇 번보다 짧으면 정상 명령이 err가 된다");
_Static_assert(CMD_WATCHDOG_MS > CONTROL_PERIOD_MS,
               "워치독이 control 주기보다 짧으면 명령이 반영되기 전에 만료된다");
_Static_assert(TX_LINE_CAP >= TX_WORST_SPD_LINE_LEN,
               "TX_LINE_CAP이 최악의 spd 줄보다 작다");
_Static_assert(VEL_TARGET_MAX_TPS > 0 && VEL_TARGET_MAX_TPS <= 8000,
               "vel 목표 상한이 M3 실측 물리 상한(7591 ~ 7853 tps) 밖이다");

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

/* 8절 시험점이 데드밴드 위인지. 최저 시험점 1000 tps의 FF가 정지마찰
   (좌 50.8 / 우 49.2 ‰)을 넘지 못하면 기동 킥 없이는 출발하지 못한다. */
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
