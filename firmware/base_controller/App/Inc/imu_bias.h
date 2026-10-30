/**
  ******************************************************************************
  * @file    imu_bias.h
  * @brief   정지 구간 자이로 bias 추정. 순수 모듈.
  * @note    HAL/RTOS 비의존. upgrade_plan.md 5.4절 3항.
  *
  *          "부팅 직후 1~2초 평균"만으로는 부족하다. 이 MCU는 주행 중에도 리셋된 적이
  *          있다(base_interface_revB.md 2a절). 그때 부팅 평균을 뜨면 회전 중 각속도가
  *          bias로 들어가 이후 내내 헤딩이 흐른다. 그래서 정지를 두 가지로 확인한다.
  *            1) 바퀴: duty 0이고 엔코더 누적값이 그대로다. ImuTask가 판정해 넘긴다.
  *            2) 자이로: 창 안에서 축마다 (최대 - 최소)가 작다. 바퀴는 멈췄는데
  *               사람이 로봇을 들어 돌리는 경우를 거른다.
  *          둘 다 만족한 창이 끝날 때마다 bias를 그 평균으로 바꾼다. 부팅 때 한 번이
  *          아니라 멈출 때마다 다시 잡으므로 온도 드리프트(±0.005 °/s/°C)도 따라간다.
  ******************************************************************************
  */
#ifndef IMU_BIAS_H
#define IMU_BIAS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  /* 진행 중인 창 */
  float    sum[3];
  float    min[3];
  float    max[3];
  uint32_t count;

  /* 결과 */
  float    bias[3];          /* [rad/s] 센서 축 */
  bool     valid;            /* 한 번이라도 창을 통과했는가 */

  /* 계측 */
  uint32_t update_count;     /* 통과한 창 수 */
  uint32_t spread_reject_count;  /* 바퀴는 멈췄는데 자이로가 흔들려 버린 창 수 */
} imu_bias_t;

void imu_bias_init(imu_bias_t *b);

/**
  * @brief  표본 하나를 넣는다.
  * @param  gyro_radps     bias를 빼기 전 자이로 [rad/s]
  * @param  wheels_still   이 표본 동안 바퀴가 멈춰 있었는가
  * @retval 이 호출로 bias가 갱신됐으면 true.
  */
bool imu_bias_update(imu_bias_t *b, const float gyro_radps[3], bool wheels_still);

/** @brief bias를 뺀다. 아직 valid가 아니면 그대로 복사한다. */
void imu_bias_apply(const imu_bias_t *b, const float in_radps[3],
                    float out_radps[3]);

#endif /* IMU_BIAS_H */
