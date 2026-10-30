/**
  ******************************************************************************
  * @file    base_config.h
  * @brief   베이스 물리 상수. 인터페이스 계약이 정한 값을 옮겨 뒀다.
 * @note    `/cmd_vel`의 body twist를 지면에서 실현하려면 역기구학도 지면 보정된
 *          유효 눈금을 써야 한다. 공칭 치수는 CAD/URDF가 소유하고, 이 파일의
 *          반지름·축간거리는 MCU가 바퀴 목표를 만들 때 쓰는 지령 눈금이다.
  *
  *          자리가 `Core/`가 아니라 `App/Inc/`인 이유는 `Core/`가 CubeMX 재생성
  *          영역이기 때문이다. 손으로 쓴 헤더를 거기 두면 재생성 때 사라지거나
  *          diff가 더러워진다.
  ******************************************************************************
  */
#ifndef BASE_CONFIG_H
#define BASE_CONFIG_H

/* Scooter/Skate Wheel 100 x 24 mm 공칭. 무부하 값이며 실제 구름 반지름은 하중을
   받으면 이보다 작다. 그 보정은 호스트 오도메트리가 따로 한다. */
#define BASE_WHEEL_RADIUS_M        0.050f

/* M6 지면 회전으로 보정한 유효 축간거리. 물리 트레드 0.264 m와 구분한다.
   공칭값을 쓰면 지령 각속도가 0.264/0.30912배만 실행되어 정밀 폐루프가 수렴하지
   않았다(M12 바닥 시험, 2026-08-30). */
#define BASE_WHEEL_SEPARATION_M    0.30912f

/* 엔코더가 기어박스 출력축을 세므로 바퀴 지름이 바뀌어도 이 값은 그대로다. */
#define BASE_TICKS_PER_REV         2475.5f

/* `/joint_states`의 `name` 배열. 이 순서가 계약이다. */
#define BASE_JOINT_NAME_LEFT       "left_wheel_joint"
#define BASE_JOINT_NAME_RIGHT      "right_wheel_joint"

#endif /* BASE_CONFIG_H */
