# 베이스 상태. 10 Hz, BEST_EFFORT, KEEP_LAST(1).
#
# 필드를 늘리면 firmware/base_controller/micro_ros/build_libmicroros.sh 로
# libmicroros.a를 다시 뽑아야 펌웨어가 같은 정의를 쓴다.

std_msgs/Header header

# micro-ROS 에이전트 연결 상태
uint8 AGENT_WAITING=0
uint8 AGENT_AVAILABLE=1
uint8 AGENT_CONNECTED=2
uint8 AGENT_DISCONNECTED=3
uint8 agent_state

# 명령 워치독
bool    cmd_watchdog_ok      # false = 명령 만료, 모터 정지 중
float32 cmd_age_s            # now - cmd_vel.header.stamp

# 모터 출력 (-1.0 ~ +1.0, + = 전진)
float32 duty_left
float32 duty_right
bool    saturated_left       # PI 출력이 duty 한계에 걸림
bool    saturated_right

# 제어 루프 건전성
uint32 loop_count            # TIM6 틱 누적
uint32 loop_overruns         # 이전 루프가 안 끝난 상태로 다음 틱이 온 횟수
