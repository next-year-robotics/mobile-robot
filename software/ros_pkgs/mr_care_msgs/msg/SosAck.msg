# 앱의 sos.ack 결과와 그 뒤에 도착한 위치 상태. 브리지 → 안전/ArUco/telemetry.
# ACK 3회 실패 시에도 app_received=false로 한 번 발행해 로컬 폴백을 유발한다.
# QoS: Reliable, KeepLast(10)
std_msgs/Header header
string sos_id
bool   app_received      # 앱이 sos.ack를 보냈는가
uint8  retries           # 재전송 횟수 (0..3)
bool   has_location      # cache 또는 fresh 위치가 한 번이라도 도착했는가
uint8  best_source       # PhoneLocation.SOURCE_* 중 가장 좋은 값
