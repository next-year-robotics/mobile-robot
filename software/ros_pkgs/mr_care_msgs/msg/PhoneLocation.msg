# 앱의 location.update. 브리지 → mission/safety/telemetry.
# 폰 GPS는 SOS 참고 정보다. 제어 루프나 로컬라이제이션 입력으로 쓰지 않는다.
# QoS: Reliable, KeepLast(10)
std_msgs/Header header
string message_id

uint8 REASON_PERIODIC=1
uint8 REASON_SOS=2
uint8 reason
string sos_id                    # reason==SOS일 때 대상 sos_id, 아니면 빈 문자열

bool    has_fix
float64 lat                      # has_fix=false면 NaN
float64 lon
float32 horizontal_accuracy_m    # 모르면 NaN
int64   captured_at_ms           # has_fix=false면 0
int64   age_ms                   # 모르면(has_fix=false) -1

uint8 SOURCE_NONE=0
uint8 SOURCE_CACHE=1
uint8 SOURCE_FRESH=2
uint8 source

uint8 FAILURE_NONE=0
uint8 FAILURE_PERMISSION_DENIED=1
uint8 FAILURE_PROVIDER_DISABLED=2
uint8 FAILURE_TIMEOUT=3
uint8 FAILURE_UNAVAILABLE=4
uint8 failure_reason

# age_ms > 300000 또는 accuracy > 50 m면 브리지가 true로 표시한다. 폐기하지는 않는다.
bool low_confidence
