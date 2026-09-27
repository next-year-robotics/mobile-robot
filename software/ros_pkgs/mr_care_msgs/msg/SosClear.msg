# 활성 SOS 종료 통지. SosRequest와 같은 sos_id.
# QoS: Reliable, KeepLast(10)
std_msgs/Header header
string sos_id

uint8 RESOLUTION_TRACKING_RECOVERED=1
uint8 RESOLUTION_HANDLED=2
uint8 RESOLUTION_CANCELLED=3
uint8 RESOLUTION_UNKNOWN=0
uint8 resolution
