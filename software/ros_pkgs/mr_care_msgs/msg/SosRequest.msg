# 안전/ArUco 계층이 SOS 조건을 확정했을 때 한 번 발행한다. 재시도는 브리지가 한다.
# QoS: Reliable, KeepLast(10)
std_msgs/Header header
string sos_id            # 발행자가 만든 UUID. sos.clear와 같은 id를 쓴다.

uint8 REASON_TRACKING_LOST=1
uint8 REASON_STUCK=2
uint8 REASON_LOW_BATTERY=3
uint8 REASON_MANUAL=4
uint8 REASON_UNKNOWN=0
uint8 reason

uint8 SEVERITY_CRITICAL=1
uint8 SEVERITY_WARNING=2
uint8 severity

string trip_id           # 빈 문자열 = null
