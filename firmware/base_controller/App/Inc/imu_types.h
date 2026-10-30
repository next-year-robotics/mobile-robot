/**
  ******************************************************************************
  * @file    imu_types.h
  * @brief   ImuTask → MicroRosTask로 넘어가는 IMU 표본. HAL/RTOS 비의존.
  ******************************************************************************
  */
#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint32_t seq;             /* 1부터 증가. 소비자는 같은 seq를 두 번 발행하지 않는다 */
  uint32_t sample_cycles;   /* 샘플 시각의 DWT cycle. DRDY 에지 또는 읽은 시각 */
  uint32_t sample_ms;       /* 같은 시각의 HAL tick. 나이 상한 판정용 */
  uint32_t group_delay_us;  /* 센서 필터 군지연. stamp = sample 시각 - 이 값 */
  float    gyro_radps[3];   /* bias를 뺀 값. 센서 축 */
  float    accel_mps2[3];   /* 센서 축. 중력 포함 */
  float    temp_c;
  bool     bias_valid;      /* 정지 창을 한 번이라도 통과했는가 */
  bool     drdy_stamped;    /* true면 sample_cycles가 DRDY 에지다 */
} imu_sample_t;

#endif /* IMU_TYPES_H */
