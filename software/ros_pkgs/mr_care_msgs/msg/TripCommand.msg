# 앱의 trip.start / trip.cancel을 ROS로 옮긴 것. 브리지 → mission.
# QoS: Reliable, KeepLast(10)
std_msgs/Header header
string request_id        # 원 WebSocket 메시지 id (중복 제거·로그 대조용)
string trip_id           # start면 브리지가 발급한 새 UUID, cancel이면 대상 trip

uint8 COMMAND_START=1
uint8 COMMAND_CANCEL=2
uint8 command

string from_place_id     # start에서만 의미
string to_place_id       # start에서만 의미
string reason            # cancel에서만 의미. 예: user_request
