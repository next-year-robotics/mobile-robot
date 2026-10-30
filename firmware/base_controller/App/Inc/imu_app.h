/**
  ******************************************************************************
  * @file    imu_app.h
  * @brief   ImuTask 본체와 그 계측값. 펌웨어 전용이다.
  * @note    rtos_app.c가 정적 메모리로 task를 띄운다(micro_ros_app.h와 같은 규칙).
  *          순수 규칙은 여기 없다. 레지스터는 icm42688.c, bias는 imu_bias.c에 있고
  *          둘 다 host에서 시험한다. 이 파일에 남은 것은 DRDY 대기, 재초기화,
  *          표본 게시의 배선뿐이다.
  *
  *          표본은 app_link_post_imu()로 길이 1 mailbox에 덮어쓴다. MicroRosTask가
  *          seq를 보고 새 것만 발행한다.
  ******************************************************************************
  */
#ifndef IMU_APP_H
#define IMU_APP_H

#include <stdint.h>

/** @brief SWD로 읽는 고정 크기 계측값. 물리량은 정수로 싣는다. */
typedef struct
{
  uint32_t ready;               /* 1 = 초기화 끝, 표본을 읽는 중 */
  uint32_t init_attempts;
  uint32_t init_status;         /* 마지막 icm42688_status_t. 0 = OK */
  uint32_t reinit_count;        /* 동작 중 IMU를 잃고 다시 초기화하러 간 횟수 */
  uint32_t who_am_i;            /* 0x47이어야 한다. 0x00/0xFF면 MISO */
  uint32_t fail_reg;            /* VERIFY 실패 레지스터와 쓴 값 / 읽은 값 */
  uint32_t fail_wrote;
  uint32_t fail_read;

  uint32_t sample_count;        /* 게시한 표본 */
  uint32_t invalid_count;       /* 0x8000 (첫 표본 전) */
  uint32_t read_fail_count;     /* 표본 버스트 SPI 실패 */
  uint32_t spi_error_count;     /* platform_imu 누적 (초기화 포함) */
  uint32_t drdy_count;          /* INT1 ISR 진입 */
  uint32_t drdy_timeout_count;  /* DRDY 없이 직접 읽은 횟수. 오르면 INT1 배선 */
  uint32_t drdy_missed_count;   /* 한 번 읽는 사이 지나간 DRDY. 오르면 ImuTask 지연 */
  uint32_t stale_read_count;    /* INT_STATUS.DATA_RDY가 0인 읽기 = 같은 표본 재독 */

  uint32_t bias_valid;
  uint32_t bias_update_count;
  uint32_t bias_reject_count;   /* 바퀴는 멈췄는데 자이로가 흔들려 버린 창 */
  int32_t  bias_mdps[3];        /* bias [m°/s] */
  int32_t  gyro_mdps[3];        /* 마지막 표본, bias 뺀 값 [m°/s] */
  int32_t  accel_mg[3];         /* 마지막 표본 [mg] */
  int32_t  temp_cdeg;           /* 마지막 표본 [0.01 °C] */
} imu_app_metrics_t;

/**
  * @brief  ImuTask 본체. SPI2의 유일한 소유자다.
  */
void imu_task(void *argument);

/**
  * @brief  INT1 상승 에지. platform_imu.c의 EXTI 콜백이 부른다.
  * @note   ISR이다. 시각을 남기고 ImuTask를 깨운다.
  */
void imu_app_on_drdy_isr(uint32_t cycles, uint32_t now_ms);

/** @brief 계측 snapshot. HealthTask가 읽어 g_rtos_metrics로 옮긴다. */
void imu_app_get_metrics(imu_app_metrics_t *out);

#endif /* IMU_APP_H */
