# 앱 연결 상태. 세션이 열리고 닫힐 때, 그리고 heartbeat 타임아웃 시 발행한다.
# QoS: Reliable, TransientLocal, KeepLast(1)
std_msgs/Header header
bool   connected
string app_instance_id      # 연결 없음이면 빈 문자열
string session_id
builtin_interfaces/Time last_seen   # 마지막 유효 앱 메시지 시각
