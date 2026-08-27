/**
  ******************************************************************************
  * @file    base_config.h
  * @brief   베이스 물리 상수. 인터페이스 계약이 정한 값을 옮겨 뒀다.
  * @note    호스트(`mr_base`)와 값이 갈리는 것은 정상이다. 호스트 쪽 상수는
  *          오도메트리 캘리브레이션 대상이고, 여기 값은 `/cmd_vel` -> 바퀴 속도
  *          피드포워드 매핑에만 쓰인다. 몇 % 틀려도 누적되지 않고 상위 제어기가
  *          닫는 루프 안에서 사라진다.
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

/* 트레드 중심 간 거리. 기하학적 공칭값이다. 슬립이 섞인 유효값은 호스트가 따로
   보정한다. 그쪽과 갈리는 것이 정상이다. */
#define BASE_WHEEL_SEPARATION_M    0.264f

/* 엔코더가 기어박스 출력축을 세므로 바퀴 지름이 바뀌어도 이 값은 그대로다. */
#define BASE_TICKS_PER_REV         2475.5f

/* `/joint_states`의 `name` 배열. 이 순서가 계약이다. */
#define BASE_JOINT_NAME_LEFT       "left_wheel_joint"
#define BASE_JOINT_NAME_RIGHT      "right_wheel_joint"

#endif /* BASE_CONFIG_H */
