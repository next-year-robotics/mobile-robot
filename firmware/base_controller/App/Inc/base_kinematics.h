/**
  ******************************************************************************
  * @file    base_kinematics.h
  * @brief   `/cmd_vel` <-> 바퀴 단위 변환. 순수 모듈이며 HAL/RTOS를 모른다.
  * @note    여기 있는 것은 차동구동 기하 하나뿐이다. 클램프 정책과 비정상 입력 거절이
  *          같이 들어 있는 이유는 그 둘이 "무엇을 바퀴 지령으로 삼을 것인가"라는 같은
  *          질문의 일부이기 때문이다.
  ******************************************************************************
  */
#ifndef BASE_KINEMATICS_H
#define BASE_KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  float   wheel_radius_m;
  float   wheel_separation_m;
  float   ticks_per_rev;
  int32_t max_tps;          /* 운용 상한. VEL_TARGET_MAX_TPS와 같다. */
} base_geometry_t;

/** @brief base_config.h + app_config.h의 확정값. */
extern const base_geometry_t g_base_geometry;

/**
  * @brief  body twist를 좌우 목표 tps로 바꾼다.
  *
  * @retval false면 지령을 쓸 수 없다는 뜻이며 출력은 0/0이다. 그런 경우는 둘뿐이다:
  *         NaN/Inf가 들어왔거나 기하 상수가 말이 안 될 때.
  *
  * @note   상한을 넘으면 좌우를 같은 비율로 줄인다. 바퀴마다 따로 잘라 내면 회전
  *         반경이 조용히 바뀐다. 직진 지령이 상한에 걸리는 순간 한쪽만 깎여서 로봇이
  *         휘어 버리는 식이다. 비율을 지키면 속도만 느려진다.
  *
  *         NaN 거절이 이 자리에 있는 이유: `/cmd_vel`은 네트워크 밖에서 온 값이다.
  *         NaN이 한 번 들어오면 PI 적분기까지 오염돼 전원을 껐다 켜야 낫는다. 숫자가
  *         아닌 것은 제어 경로에 들이지 않는다.
  */
bool base_kin_twist_to_tps(const base_geometry_t *g,
                           float linear_x_mps, float angular_z_radps,
                           int32_t *tps_left, int32_t *tps_right);

/**
  * @brief  누적 틱을 누적 바퀴 각도 [rad]로. `/joint_states.position`이다.
  * @note   double이다. `position`은 랩어라운드 없는 누적값이라 몇 시간만 굴러도 1e8
  *         rad를 넘는다. float32는 그 지점에서 분해능이 수십 rad로 떨어져
  *         오도메트리가 쓰는 차분이 통째로 사라진다.
  */
double base_kin_ticks_to_rad(const base_geometry_t *g, int64_t ticks);

/** @brief tps를 바퀴 각속도 [rad/s]로. `/joint_states.velocity`다. */
float base_kin_tps_to_radps(const base_geometry_t *g, int32_t tps);

#endif /* BASE_KINEMATICS_H */
