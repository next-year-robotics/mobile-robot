# 현재 프리셋 전체와 revision. 브리지가 places.yaml을 갱신할 때마다 발행한다.
# QoS: Reliable, TransientLocal, KeepLast(1)
std_msgs/Header header
string revision                  # ISO-8601 UTC. phone-bridge/1 catalog_revision과 같은 값
uint8  max_places
string default_origin_place_id   # 빈 문자열 = null (빈 카탈로그)
Place[] places
