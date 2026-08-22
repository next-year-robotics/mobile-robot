/**
  ******************************************************************************
  * @file    base_config.h
  * @brief   `interface/base_contract.md`의 "물리 상수" 표를 그대로 옮긴 값.
  * @note    **이 파일은 원본이 아니다.** 원본은 `0_tmp/adocs/interface/base_contract.md`
  *          이고 여기 있는 것은 그 사본이다. 값을 고치려면 문서를 먼저 고친다.
  *
  *          계약 문서는 이 파일의 자리를 `Core/Inc/base_config.h`로 적었지만,
  *          `Core/`는 CubeMX 재생성 영역이라 손으로 쓴 헤더를 두면 재생성 때
  *          사라지거나 diff가 더러워진다. 그래서 애플리케이션 영역인 `App/Inc/`에
  *          둔다 — 자리만 다르고 역할은 계약 문서가 말한 그대로다.
  *
  *          **호스트(M6 `mr_base`)와 값이 갈리는 것은 정상이다.** 호스트 쪽 상수는
  *          오도메트리 캘리브레이션 대상이고, 여기 값은 `/cmd_vel` -> 바퀴 속도
  *          피드포워드 매핑에만 쓰인다. 2 % 틀려도 누적되지 않고 상위 제어기가
  *          닫는 루프 안에서 사라진다 (base_contract.md "MCU 측 역기구학과 상수
  *          중복 문제").
  ******************************************************************************
  */
#ifndef BASE_CONFIG_H
#define BASE_CONFIG_H

/* Scooter/Skate Wheel 100 x 24 mm 공칭 (2026-08-20 교체). 무부하 값이며 실제
   구름 반지름은 하중을 받으면 이보다 작다 — M6 직선 2 m 주행으로 보정한다. */
#define BASE_WHEEL_RADIUS_M        0.050f

/* M6 줄자 실측 (2026-08-22, 트레드 중심 간). 승계값 0.32는 바퀴 교체 후 트레드 폭이
   56 mm 좁아진 것을 반영하지 못한 값이라 폐기했다. 이 값은 기하학적 공칭값이며,
   슬립이 섞인 유효값은 호스트(M6 360도 x 4)가 따로 보정한다 — 그쪽과 갈리는 것이 정상이다. */
#define BASE_WHEEL_SEPARATION_M    0.264f

/* M2 실측 (2026-08-20). 좌 2476.5 / 우 2474.5의 평균이며 반복 RSD 0.035~0.122 %.
   엔코더가 기어박스 출력축을 세므로 바퀴 지름이 바뀌어도 이 값은 그대로다. */
#define BASE_TICKS_PER_REV         2475.5f

/* `/joint_states`의 `name` 배열. **이 순서가 계약이다** (base_contract.md 2절). */
#define BASE_JOINT_NAME_LEFT       "left_wheel_joint"
#define BASE_JOINT_NAME_RIGHT      "right_wheel_joint"

#endif /* BASE_CONFIG_H */
