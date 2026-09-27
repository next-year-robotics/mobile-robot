# 장소 프리셋 하나. phone-bridge/1의 place 객체와 필드가 1:1이다.
# 정본은 RPi5의 places.yaml이며 브리지가 소유한다.
string  id                 # ^[a-z0-9_-]{1,64}$
string  kind               # home|market|mart|senior_center|church|temple|hospital|pharmacy|other
string  label              # UTF-8 1~20자
float64 lat                # WGS84 십진도
float64 lon
float32 radius_m           # 5..500
bool    enabled
int32   sort_order
bool    is_default_origin
string  address            # 빈 문자열 = null
