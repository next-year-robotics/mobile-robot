# 앱과 대시보드에 보여 줄 상위 로봇 상태. mission 계층이 발행하고
# 브리지(robot.status)와 텔레메트리(/robots/{id}/status)가 그대로 옮긴다.
# QoS: Reliable, TransientLocal, KeepLast(1). 변경 즉시 + 2 s 주기.
std_msgs/Header header
string robot_id

uint8 MISSION_IDLE=0
uint8 MISSION_STARTING=1
uint8 MISSION_ACTIVE=2
uint8 MISSION_ARRIVED=3
uint8 MISSION_CANCELLING=4
uint8 MISSION_CANCELLED=5
uint8 MISSION_ERROR=6
uint8 MISSION_UNKNOWN=255
uint8 mission_state

uint8 TRACKING_UNAVAILABLE=0
uint8 TRACKING_SEARCHING=1
uint8 TRACKING_FOLLOWING=2
uint8 TRACKING_LOST=3
uint8 TRACKING_ERROR=4
uint8 TRACKING_UNKNOWN=255
uint8 tracking_state

string trip_id           # 빈 문자열 = null
string from_place_id
string to_place_id
int8   battery_percent   # 0..100, -1 = 알 수 없음(현재 인터페이스 없음)
string last_error        # 빈 문자열 = null. 사용자에게 보여도 되는 짧은 문장만
