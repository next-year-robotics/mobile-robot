# 필터를 거친 대상 마커 하나의 추적 상태. `marker_track` 노드가 발행한다.
#
# `aruco_opencv`의 원시 `/aruco_detections`와 다른 점은 두 가지다.
#   1. 대상 id 하나만 골라 나온다.
#   2. 점프 기각과 hold를 거친 뒤라, 마커가 잠깐 사라져도 값이 끊기지 않는다.
#
# 위치는 `header.frame_id`(= base_footprint) 기준 평면 좌표다. 카메라 광학
# 좌표계 → 본체 좌표계 변환은 `aruco_opencv`의 `output_frame`이 이미 끝냈다.

std_msgs/Header header

uint8 STATUS_LOST=0
uint8 STATUS_TRACKING=1
uint8 STATUS_HOLDING=2

uint8 status
bool locked
uint16 marker_id

# LOST일 때 아래 값들은 의미가 없다. status를 먼저 본다.

# 마커 중심 위치 [m]. x = 전방, y = 좌측 (REP-103).
float64 x
float64 y

# 마커 **바깥 법선**을 지면에 투영한 방향 [rad]. "마커가 바라보는 쪽"이다.
#
# 정렬 제어가 쓰는 축이므로 정의를 못박아 둔다. 로봇 정면에 있고 로봇을 마주보는
# 벽 마커라면 법선이 로봇 쪽(-x)을 향하므로 yaw = pi다. 로봇이 마커를 정면으로
# 보기까지 돌아야 할 각은 wrap(yaw + pi)이고, 위 예에서 0이 된다.
#
# 마커 자세 전체(쿼터니언)를 그대로 오일러로 풀지 않는 이유는, ArUco 마커 좌표계가
# 본체 좌표계에 대해 두 축이 90도 돌아가 있어 "yaw" 성분만 뽑으면 짐벌 부근에서
# 의미가 무너지기 때문이다. 법선 투영은 그런 문제가 없다.
float64 yaw

# yaw를 믿어도 되는가. 마커 법선이 거의 수직(마커가 눕혀 달림)이면 지면 투영이
# 무의미해지므로 거짓이 된다. 정렬 제어는 이 값이 거짓이면 goal을 거절한다.
# 추종 제어는 yaw를 쓰지 않으므로 무시한다.
bool yaw_valid

# HOLDING으로 버틴 시간 [s]. TRACKING이면 0.
float64 hold_duration

# 이번 표본이 점프로 기각됐는가. 기각되면 위 좌표는 직전 값이 유지된다.
bool rejected_jump
