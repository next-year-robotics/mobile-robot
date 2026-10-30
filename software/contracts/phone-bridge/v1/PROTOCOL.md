# phone-bridge/1 — RPi5 ↔ Android 앱 WebSocket 프로토콜

> 상태: v1 확정. 이 파일이 프로토콜의 **단일 정본**이다.
>
> 이력: `senior-care-android/docs/android-app-spec.md` v0.4(2026-08-28)의 7·8·9·10·15장을
> 2026-09-27에 이 저장소로 옮겼다. 앱 명세는 이 파일을 참조만 한다.
>
> 기계가 읽는 정본: `schema/messages.schema.json`(JSON Schema 2020-12), `fixtures/*.json`.
> 문서와 스키마가 어긋나면 스키마가 아니라 **문서를 고친 뒤 스키마·fixture를 같은 PR에서 맞춘다.**

## 1. 이 계약을 다루는 규칙

- 서버(`mr_phone_bridge`, 이 저장소)와 클라이언트(`senior-care-android`)는 이 디렉터리만 보고 독립적으로 구현한다.
- 앱·대시보드 저장소는 이 디렉터리의 **복사본**을 `contracts/`에 둔다. 복사는 각 저장소의 `scripts/sync-contracts.sh`로 하고, CI가 정본과의 차이를 검사한다. 복사본을 직접 고치지 않는다.
- v1에서 허용되는 변경은 **선택 필드 추가**와 enum 값 추가(수신 측은 모르는 enum을 `unknown`으로 처리)뿐이다. 필수 필드 삭제·의미 변경은 `/bridge/v2`다.
- 변경 절차: 이 문서 수정 → 스키마·fixture 수정 → `tools/validate_fixtures.py` 통과 → 서버·앱 양쪽 테스트 통과 → 병합. 한쪽만 바꾼 상태로 병합하지 않는다.
- fixture(현재 26개)는 문서 예시와 자동 테스트가 함께 읽는 단일 파일이다. 서버 테스트(`mr_phone_bridge/test/`)와 앱 테스트(`app/src/test/`)는 모두 이 파일을 파싱·검증한다.

### 1.1 관련 저장소

| 저장소 | 역할 |
|---|---|
| `mobile-robot` (이 저장소) | 서버 `software/ros_pkgs/mr_phone_bridge`, mock bridge, ROS 메시지 `mr_care_msgs`, 이 계약 |
| `senior-care-android` | Kotlin/Compose 클라이언트. 계약 복사본 `contracts/phone-bridge/v1` |
| `senior-care-dashboard` | 이 프로토콜을 쓰지 않는다. 텔레메트리는 `../care-telemetry/v1` |

### 1.2 참여자와 경계

```text
[Android 앱] ──phone-bridge/1 JSON over ws://──▶ [RPi5 mr_phone_bridge] ──mr_care_msgs──▶ [mission / safety / ArUco]
```

- 앱에는 ROS 라이브러리·메시지 타입·토픽 이름을 노출하지 않는다.
- 브리지와 앱은 `/cmd_vel`을 발행하지 않는다. 브리지 실패가 베이스 제어 루프를 멈추면 안 된다.
- 폰 GPS는 SOS 참고 정보다. 제어·로컬라이제이션에 넣지 않는다.
- 안전 정지·부저 같은 최종 폴백은 로봇이 소유한다.

---

## 2. 네트워크와 발견

### 7.1 기본 연결

| 항목 | 값 |
|---|---|
| 역할 | RPi5 = WebSocket server, Android = client |
| URL | `ws://<RPi5 IP>:8765/bridge/v1`. IP는 NSD 결과·저장한 endpoint·수동 입력에서 온다(7.2). 앱은 호스트 이름(`*.local`)을 직접 해석하지 않는다(7.3) |
| 서비스 타입 | `_mr-phone-bridge._tcp` |
| mDNS TXT | `api=1`, `robot_id=<stable-id>` |
| 문자 인코딩 | UTF-8 |
| payload | JSON text frame만 허용 |
| 단일 메시지 상한 | 16 KiB |
| 동시 앱 | MVP에서는 1대 |

휴대폰이 네트워크 게이트웨이라는 사실과 WebSocket의 client/server 역할은 별개다.

### 7.2 발견 순서

1. 저장된 마지막 endpoint가 있으면 2초 동안 먼저 시도한다.
2. 실패하면 Android `NsdManager`로 `_mr-phone-bridge._tcp`를 최대 10초 탐색한다.
3. 한 대면 자동 선택한다. 여러 대면 `robot_id` 목록을 보여 준다.
4. mDNS가 실패하면 설정 화면에서 IPv4 주소와 포트를 직접 입력할 수 있게 한다.
5. 서브넷 전체 포트 스캔은 구현하지 않는다.

일부 Android 핫스팟/OEM 조합은 multicast를 제한하므로 mDNS만을 유일한 진입 경로로 두지 않는다. 수동 endpoint는 디버그용이면서 현장 폴백이다.

### 7.3 보안 단계

MVP의 `ws://`는 인터넷에 공개되지 않은 개인 핫스팟 안에서만 허용한다. RPi5 방화벽은 핫스팟 인터페이스의 bridge 포트만 열고 다른 인터페이스에는 노출하지 않는다.

앱의 평문 허용 범위(2026-09-27 명시): 평문 `ws://`는 사설 IPv4(RFC 1918), 링크 로컬(`169.254.0.0/16`, `fe80::/10`), IPv6 ULA(`fc00::/7`)의 **IP 리터럴**에만 연다. 핫스팟이 DHCP로 준 RPi5 주소가 여기에 든다. 공인 IP나 호스트 이름으로는 평문 연결을 열지 않고 HTTP redirect를 따르지 않는다. debug 빌드만 loopback(`127.0.0.0/8`, `localhost`, `::1`)을 더 허용한다(개발 PC 데모·`adb reverse`).

다음 중 하나라도 충족하면 공개 시연 전 `wss://`와 기기 페어링 토큰을 적용해야 한다.

- 공용 Wi-Fi 사용
- 여러 로봇 또는 여러 앱이 같은 네트워크에 존재
- 실제 사용자 위치를 장기간 운용
- 외부 서버로 SOS 전달

`[OPEN-R1]` 현장 네트워크가 개인 핫스팟이 아닌 경우 인증 방식과 인증서 배포 방식을 확정해야 한다. 이 항목은 앱 UI 개발을 막지 않지만 현장 릴리스는 막는다.

---


---

## 3. WebSocket 프로토콜 `phone-bridge/1`

### 8.0 메시지 목록

| type | 방향 | 응답 | 용도 |
|---|---|---|---|
| `hello` | 앱 → RPi5 | `hello.ack` 또는 `error` | 세션 협상 |
| `hello.ack` | RPi5 → 앱 | 없음 | 세션 협상 성공 |
| `heartbeat` | 앱 → RPi5 | `heartbeat.ack` | 연결 생존 확인 |
| `heartbeat.ack` | RPi5 → 앱 | 없음 | heartbeat 응답 |
| `ack` | RPi5 → 앱 | 없음 | 요청 접수 성공 |
| `trip.start` | 앱 → RPi5 | `ack` 또는 `error` | 주행 시작 요청 |
| `trip.cancel` | 앱 → RPi5 | `ack` 또는 `error` | 주행 정지 요청 |
| `place.upsert` | 앱 → RPi5 | `ack` 또는 `error` | 프리셋 추가·수정 |
| `place.delete` | 앱 → RPi5 | `ack` 또는 `error` | 프리셋 삭제 |
| `location.update` | 앱 → RPi5 | `ack` 또는 `error` | periodic/SOS 위치 |
| `sos.ack` | 앱 → RPi5 | `ack` 또는 `error` | SOS 수신 확인 |
| `place.catalog` | RPi5 → 앱 | 없음 | 프리셋 전체 동기화 |
| `robot.status` | RPi5 → 앱 | 없음 | 앱용 로봇 상태 |
| `sos.request` | RPi5 → 앱 | `sos.ack` | SOS 처리 요청 |
| `sos.clear` | RPi5 → 앱 | 없음 | 활성 SOS 종료 통지 |
| `error` | 양방향 | 없음 | 요청 거절 또는 protocol 오류 |

서버는 `location.update`와 `sos.ack`에 일반 `ack`를 보낸다. 이 `ack`는 RPi5 브리지가 받았다는 뜻이지 SOS의 외부 전달 완료를 뜻하지 않는다.

### 8.1 공통 envelope

모든 메시지는 타입과 상관없이 같은 구조를 쓴다.

```json
{
  "version": 1,
  "type": "trip.start",
  "id": "e87864c5-3c72-42f8-9565-cf9219aa7cf2",
  "sent_at_ms": 1787792400000,
  "reply_to": null,
  "payload": {}
}
```

| 필드 | 타입 | 필수 | 규칙 |
|---|---|---:|---|
| `version` | integer | O | v1에서는 반드시 `1` |
| `type` | string | O | 위 표의 값만 발신 |
| `id` | string | O | canonical lowercase UUID v4 |
| `sent_at_ms` | integer | O | Unix epoch millisecond, 기록용 |
| `reply_to` | string/null | O | 응답이면 원 요청의 `id`, 아니면 `null` |
| `payload` | object | O | 타입별 스키마 |

- 타임아웃과 재시도 계산은 각 기기의 monotonic clock을 사용한다. `sent_at_ms`만으로 타임아웃을 계산하지 않는다.
- 알 수 없는 최상위 필드와 payload의 선택 필드는 무시한다.
- 필수 필드 누락, 잘못된 타입, 범위를 벗어난 숫자, `NaN`/`Infinity`는 `INVALID_MESSAGE`로 거절한다.
- 알 수 없는 `type`은 연결을 끊지 않고 `UNSUPPORTED_TYPE` 오류로 응답한다.
- 같은 `id`가 재수신되면 side effect를 반복하지 않고 10분 동안 보관한 기존 결과를 다시 응답한다.
- 메시지 상한을 넘기면 해당 frame을 거절하고 연결을 종료할 수 있다.
- v1에서 필드를 추가하는 변경은 선택 필드만 허용한다. 필수 필드 삭제·의미 변경은 `/bridge/v2`가 필요하다.

### 8.2 연결 handshake

WebSocket 연결 직후 앱이 3초 안에 `hello`를 보낸다.

#### 앱 → RPi5: `hello`

```json
{
  "version": 1,
  "type": "hello",
  "id": "c3c4a81d-f25b-487c-8f28-4ac47a9eaf70",
  "sent_at_ms": 1787792400000,
  "reply_to": null,
  "payload": {
    "app_instance_id": "9e889e5d-1c2d-4572-9513-9cd8283c917a",
    "app_version": "0.1.0",
    "platform": "android",
    "capabilities": ["location.cache", "location.fresh", "place.edit"]
  }
}
```

`app_instance_id`는 설치 시 생성해 DataStore에 저장한 임의 UUID다. IMEI, 전화번호, Android ID를 사용하지 않는다.

#### RPi5 → 앱: `hello.ack`

```json
{
  "version": 1,
  "type": "hello.ack",
  "id": "97fe12c8-5dd6-4de6-b638-a091db31b991",
  "sent_at_ms": 1787792400042,
  "reply_to": "c3c4a81d-f25b-487c-8f28-4ac47a9eaf70",
  "payload": {
    "session_id": "e2840b5b-f31f-4bba-a8ca-5b140c34cb1d",
    "robot_id": "mr-001",
    "server_version": "0.1.0",
    "heartbeat_interval_ms": 2000,
    "heartbeat_timeout_ms": 10000,
    "catalog_revision": "2026-08-27T12:00:00Z",
    "place_edit_enabled": true,
    "max_places": 12
  }
}
```

`place_edit_enabled=false`면 앱은 프리셋 편집 UI를 읽기 전용으로 만든다. 지원하지 않는 protocol version이면 서버는 `UNSUPPORTED_VERSION` 오류를 보내고 연결을 닫는다. handshake가 끝나기 전에는 `hello` 외의 메시지를 처리하지 않는다.

### 8.3 ACK와 오류

#### 성공: `ack`

```json
{
  "version": 1,
  "type": "ack",
  "id": "1bce692f-2f56-49bf-b02c-b6322c86d750",
  "sent_at_ms": 1787792400500,
  "reply_to": "e87864c5-3c72-42f8-9565-cf9219aa7cf2",
  "payload": {
    "status": "accepted",
    "trip_id": "bbef019a-dd3a-437e-8f5c-388913ae9eca"
  }
}
```

`accepted`는 검증을 통과해 RPi5 내부에 접수됐다는 뜻이지만 실제 로봇 상태 변경은 이후 `robot.status`가 기준이다. 서버는 command 수신 후 1초 안에 `ack` 또는 `error`를 보낸다.

`place.upsert`/`place.delete`의 `ack` payload는 `{"status":"accepted","place_id":"…","catalog_revision":"…"}`를 담는다.

#### 실패: `error`

```json
{
  "version": 1,
  "type": "error",
  "id": "f177b5a1-b7e5-4cba-89a2-014c66f553af",
  "sent_at_ms": 1787792400500,
  "reply_to": "e87864c5-3c72-42f8-9565-cf9219aa7cf2",
  "payload": {
    "code": "UNKNOWN_PLACE",
    "message": "Unknown destination place_id",
    "retryable": false
  }
}
```

오류 코드는 아래 11개로 고정한다.

| code | 의미 | 앱 동작 |
|---|---|---|
| `INVALID_MESSAGE` | 형식, 타입 또는 범위 오류 | 사용자 재시도 금지, 디버그 로그 |
| `UNSUPPORTED_TYPE` | 알 수 없는 `type` | 업데이트 필요 표시 |
| `UNSUPPORTED_VERSION` | protocol version 불일치 | 연결 중단, 업데이트 필요 표시 |
| `NOT_READY` | 로봇/브리지가 명령 받을 상태가 아님 | 상태 표시 후 재시도 허용 |
| `UNKNOWN_PLACE` | 카탈로그에 없는 장소 | 카탈로그 새로고침 |
| `INVALID_STATE` | 현재 상태에서 불가능한 명령 | 최신 `robot.status` 반영 |
| `CATALOG_CONFLICT` | `catalog_revision`이 최신이 아님 | 최신 catalog 반영 후 사용자에게 재확인 |
| `PLACE_LIMIT` | 프리셋 개수 상한 초과 | 삭제 후 추가하도록 안내 |
| `PLACE_IN_USE` | 진행 중 trip이 쓰는 장소 삭제 시도 | 주행 종료 후 삭제하도록 안내 |
| `PERMISSION_DENIED` | 페어링/인증 실패 | 연결 중단 |
| `INTERNAL_ERROR` | 서버 내부 오류 | `retryable`에 따름 |

내부 예외 stack trace, 파일 경로, 토큰은 `message`로 보내지 않는다.

### 8.4 heartbeat와 재연결

handshake 후 앱은 2초마다 `heartbeat`를 보내며 서버는 같은 요청에 `heartbeat.ack`로 응답한다.

```json
{
  "version": 1,
  "type": "heartbeat",
  "id": "1392a54d-c195-489f-88bb-a20022eef8b5",
  "sent_at_ms": 1787792402000,
  "reply_to": null,
  "payload": {"session_id": "e2840b5b-f31f-4bba-a8ca-5b140c34cb1d"}
}
```

`heartbeat.ack`는 공통 envelope를 사용하고 `reply_to`에 heartbeat의 `id`, payload에 현재 `session_id`를 넣는다.

- 서버는 유효한 앱 메시지가 10초 동안 없으면 세션을 끊고 ROS 쪽에 `connected=false`를 알린다.
- 앱은 heartbeat 응답이 10초 동안 없으면 소켓을 닫고 재연결한다.
- 재연결 간격은 `1, 2, 4, 8, 10, 10…초`에 ±20% jitter를 적용한다.
- 30초 이상 안정 연결되면 backoff를 초기화한다.
- 재연결 후 새 `session_id`를 받으며 place catalog와 robot status를 다시 동기화한다.
- 연결 끊김 중 command를 무기한 큐잉하지 않는다. 사용자의 `trip.start`/`trip.cancel`/`place.*`는 실패로 표시하고 재확인을 받는다.
- 로봇 쪽 동작(2026-09-27 결정): 주행 중(`mission_state`가 `starting`/`active`) 서버가 앱 연결 끊김을 알아챈 뒤 **30초 안에 다시 연결되지 않으면**
  로봇은 멈추고 trip을 `error`로 끝낸다(`last_error` "폰 연결이 끊겨 멈췄어요"). 30초 안에 재연결하면 주행은 그대로 이어진다. 재연결 뒤 자동으로
  다시 가지 않으며, 다시 가려면 앱에서 새로 출발한다. 응답 없이 사라진 경우(핫스팟·폰 꺼짐)는 위 10초 타임아웃이 앞에 붙어 약 40초 뒤에 선다.

---


---

## 4. 장소 프리셋과 주행

### 9.1 프리셋 모델

프리셋은 어르신이 홈 화면에서 고를 수 있는 **미리 지정해 둔 영역**이다. 집, 시장, 마트, 노인정, 교회, 절처럼 자주 가는 곳을 보호자가 미리 등록한다.

| 필드 | 타입 | 규칙 |
|---|---|---|
| `id` | string | 서버 생성. ASCII 소문자·숫자·`_`·`-`, 1~64자 |
| `kind` | string | `home`, `market`, `mart`, `senior_center`, `church`, `temple`, `hospital`, `pharmacy`, `other` |
| `label` | string | 화면 표시 이름. UTF-8 1~20자. 큰 글꼴에서 두 줄을 넘기지 않도록 짧게 |
| `lat` | number | `-90..90`, WGS84 |
| `lon` | number | `-180..180`, WGS84 |
| `radius_m` | number | `5..500`, 기본 30 |
| `enabled` | boolean | `false`면 홈 화면에서 선택 불가 |
| `sort_order` | integer | 홈 화면 타일 순서. 작은 값이 앞 |
| `is_default_origin` | boolean | 기본 출발지. 카탈로그가 비어 있지 않으면 전체에서 정확히 하나 |
| `address` | string/null | 등록 당시 역지오코딩 주소. 표시·확인용이며 판정에 쓰지 않는다 |

규칙:

- 프리셋 개수 상한은 `hello.ack`의 `max_places`(기본 12)다.
- `kind`가 같은 프리셋이 여러 개 있어도 된다(예: 마트 두 곳). `label`로 구분한다.
- 첫 프리셋을 추가할 때 기본 출발지가 지정되지 않았으면 서버가 그 프리셋을 기본 출발지로 지정한다.
- `is_default_origin=true`인 프리셋을 새로 지정하면 서버가 기존 기본 출발지를 자동으로 해제한다.
- 기본 출발지는 삭제할 수 없다. 다른 프리셋을 기본으로 지정한 뒤 삭제한다.
- 좌표의 단일 원본은 RPi5의 `places.yaml`이다. 앱은 목록을 하드코딩하지 않는다.

RPi5의 `places.yaml` 예시:

```yaml
version: 1
revision: "2026-08-27T12:00:00Z"
places:
  place_home:
    kind: home
    label: "집"
    lat: 36.1234
    lon: 128.5678
    radius_m: 20
    enabled: true
    sort_order: 10
    is_default_origin: true
    address: "경상북도 …"
  place_market:
    kind: market
    label: "시장"
    lat: 36.1251
    lon: 128.5702
    radius_m: 30
    enabled: true
    sort_order: 20
    is_default_origin: false
    address: "경상북도 …"
```

### 9.2 RPi5 → 앱: `place.catalog`

서버는 handshake 직후, 그리고 카탈로그가 바뀔 때마다 전체 목록을 보낸다. 부분 갱신은 없다.

```json
{
  "version": 1,
  "type": "place.catalog",
  "id": "da32d958-c86e-414a-b72f-edb9402948e8",
  "sent_at_ms": 1787792400100,
  "reply_to": null,
  "payload": {
    "revision": "2026-08-27T12:00:00Z",
    "max_places": 12,
    "default_origin_place_id": "place_home",
    "places": [
      {
        "id": "place_home",
        "kind": "home",
        "label": "집",
        "lat": 36.1234,
        "lon": 128.5678,
        "radius_m": 20,
        "enabled": true,
        "sort_order": 10,
        "is_default_origin": true,
        "address": "경상북도 …"
      },
      {
        "id": "place_market",
        "kind": "market",
        "label": "시장",
        "lat": 36.1251,
        "lon": 128.5702,
        "radius_m": 30,
        "enabled": true,
        "sort_order": 20,
        "is_default_origin": false,
        "address": null
      }
    ]
  }
}
```

- 좌표는 설정 화면의 지도 편집에 필요하므로 앱에 내려보낸다. 어르신 홈 화면에는 표시하지 않는다.
- 앱은 `enabled=false`인 장소를 홈 타일에 흐리게 표시하고 누르면 사용할 수 없는 이유를 알려 준다. 목록에서 감추지는 않는다(앱 명세 12.2 참조).
- 현재 선택이 새 catalog에서 사라지거나 비활성화되면 선택을 해제하고 안내한다.
- 마지막 정상 catalog를 로컬에 저장해 홈 화면을 즉시 그릴 수 있지만 연결 전에는 주행을 시작하지 않는다. 오프라인 표시가 필요하면 타일을 비활성 상태로 그린다.
- 빈 catalog에서는 `default_origin_place_id`를 `null`로 보낸다.

### 9.3 앱 → RPi5: `place.upsert`

프리셋을 추가하거나 수정할 때 보낸다. `place_id`가 `null`이면 추가, 값이 있으면 수정이다.

```json
{
  "version": 1,
  "type": "place.upsert",
  "id": "6d1f3f1e-5f6a-4a0e-9f6b-0a2f0a3f2b11",
  "sent_at_ms": 1787792500000,
  "reply_to": null,
  "payload": {
    "place_id": null,
    "catalog_revision": "2026-08-27T12:00:00Z",
    "kind": "senior_center",
    "label": "노인정",
    "lat": 36.1277,
    "lon": 128.5731,
    "radius_m": 25,
    "enabled": true,
    "sort_order": 40,
    "is_default_origin": false,
    "address": "경상북도 …",
    "source": "map_search"
  }
}
```

- `source`: `map_search`, `map_pin`, `manual_coords`, `current_location`. 어떤 경로로 지정했는지 기록용이다.
- `catalog_revision`이 서버의 최신 값과 다르면 서버는 `CATALOG_CONFLICT`를 반환한다. 앱은 최신 catalog를 받은 뒤 사용자에게 다시 확인받는다.
- 서버는 검증 후 `places.yaml`을 원자적으로 갱신하고 `ack`(새 `place_id`와 `catalog_revision` 포함)에 이어 새 `place.catalog`를 브로드캐스트한다.
- 주행 중(`mission_state`가 `starting`/`active`/`cancelling`)에는 현재 trip이 쓰는 장소의 좌표 수정을 거절한다(`INVALID_STATE`).

### 9.4 앱 → RPi5: `place.delete`

```json
{
  "version": 1,
  "type": "place.delete",
  "id": "9e7c9a20-3c4b-4c1e-9a2d-71b0f9a2c4d8",
  "sent_at_ms": 1787792510000,
  "reply_to": null,
  "payload": {
    "place_id": "place_temple",
    "catalog_revision": "2026-08-27T12:05:00Z"
  }
}
```

- 기본 출발지는 삭제할 수 없다(`INVALID_STATE`).
- 진행 중인 trip의 출발지/목적지는 삭제할 수 없다(`PLACE_IN_USE`).
- 삭제 확인 다이얼로그는 장소 이름을 그대로 보여 준다. "정말 삭제할까요?"만 띄우지 않는다.

### 9.5 앱 → RPi5: `trip.start`

```json
{
  "version": 1,
  "type": "trip.start",
  "id": "e87864c5-3c72-42f8-9565-cf9219aa7cf2",
  "sent_at_ms": 1787792410000,
  "reply_to": null,
  "payload": {
    "from_place_id": "place_home",
    "to_place_id": "place_market",
    "catalog_revision": "2026-08-27T12:00:00Z"
  }
}
```

- 출발지와 목적지는 같을 수 없다.
- 둘 다 현재 catalog에 있고 `enabled=true`여야 한다.
- 홈 화면에서 출발지를 따로 고르지 않았으면 `default_origin_place_id`를 `from_place_id`로 넣는다.
- 서버는 접수 시 `trip_id`가 포함된 `ack`를 반환한다.
- 앱은 ACK 전까지 시작 버튼을 비활성화하고 중복 탭을 막는다.
- 사용자가 시작을 확인하기 전에는 선택만으로 RPi5에 명령을 보내지 않는다.
- ACK 후 5초 동안 `실행 취소` action을 크게 표시한다.

### 9.6 앱 → RPi5: `trip.cancel`

```json
{
  "version": 1,
  "type": "trip.cancel",
  "id": "1a0ad887-2680-465d-af4a-c1956efe2a58",
  "sent_at_ms": 1787792413000,
  "reply_to": null,
  "payload": {
    "trip_id": "bbef019a-dd3a-437e-8f5c-388913ae9eca",
    "reason": "user_request"
  }
}
```

`trip.cancel`은 사용자가 누르는 `멈춤`에 대응하며 로봇의 실제 주행을 안전하게 멈추는 명령이다. 취소 ACK는 정지 명령을 접수했다는 뜻이므로 앱은 `멈추는 중입니다`를 표시하고, 실제 정지 완료는 `robot.status.mission_state=cancelled`로 확인한다. 감속 방식과 모터 정지 절차는 로봇 측 mission/safety 계층이 책임진다.

### 9.7 RPi5 → 앱: `robot.status`

```json
{
  "version": 1,
  "type": "robot.status",
  "id": "025832dc-f9ed-48bd-8354-602d314d27ad",
  "sent_at_ms": 1787792414000,
  "reply_to": null,
  "payload": {
    "robot_id": "mr-001",
    "bridge_state": "ready",
    "mission_state": "active",
    "tracking_state": "unavailable",
    "trip_id": "bbef019a-dd3a-437e-8f5c-388913ae9eca",
    "from_place_id": "place_home",
    "to_place_id": "place_market",
    "battery_percent": null,
    "last_error": null
  }
}
```

| 필드 | 허용값 |
|---|---|
| `bridge_state` | `ready`, `degraded`, `error` |
| `mission_state` | `idle`, `starting`, `active`, `arrived`, `cancelling`, `cancelled`, `error`, `unknown` |
| `tracking_state` | `unavailable`, `searching`, `following`, `lost`, `idle`, `error`, `unknown` |
| `battery_percent` | `0..100` 또는 `null` |

| `tracking_state` | 뜻 |
|---|---|
| `following` | 주행 중(`mission_state`가 `starting`/`active`) 대상 마커를 보고 있다 |
| `searching` | 주행을 시작했지만 이번 주행에서 아직 마커를 한 번도 잡지 못했다 |
| `lost` | 주행 중 잡았던 마커를 놓쳤다 |
| `idle` | 주행 중이 아니라 추적하지 않는다. 추적 입력은 살아 있다(마커가 보이든 안 보이든). **2026-09-27 추가** |
| `unavailable` | 추적 입력(`marker_track`)이 없다(ArUco 계층이 없거나 멈췄다) |
| `error` / `unknown` | 추적 계층 오류 / 알 수 없음 |

- 서버는 변경 즉시 보내고 변경이 없어도 2초마다 최신 상태를 보낸다.
- 앱은 마지막 수신 후 5초가 지나면 상태를 `stale`로 표시한다.
- 앱이 모르는 enum은 crash시키지 않고 `unknown`으로 표시한다.
- `following`·`searching`·`lost`는 주행 중에만 나온다. 2026-09-27 전에는 주행 밖에서도 마커가 보이면 `following`을 보냈다(로봇은 서 있는데 "따라가는 중"으로 보였다).
- 배터리 인터페이스가 없으므로 `battery_percent`는 `null`이다.

### 9.8 목적지 좌표의 역할

보호자는 위도·경도를 직접 입력하는 대신 지도 검색이나 핀 이동으로 목적지 좌표와 반경을 지정한다. 앱은 이 값을 검증해 `place.upsert`로 RPi5에 전달하는 데까지만 책임진다.

RPi5의 ROS 2 mission 계층은 `places.yaml`의 `lat`, `lon`, `radius_m`를 목적지 설정으로 사용하고 로봇 측 위치 추정과 주행 정책에 따라 도착 여부를 판단한다. 정확한 ROS 노드 구성과 판정 알고리즘은 Android 앱 계약 밖이며 로봇 담당자가 구현한다. 휴대폰의 실시간 GPS는 이 도착 판단이나 로봇 제어에 사용하지 않는다.

---


---

## 5. 위치와 SOS

### 10.1 위치 캐시

- 포그라운드 서비스가 실행 중일 때 30초 주기로 최신 위치를 갱신한다.
- 메모리에는 최신 정상 fix 1개만 둔다. 정확한 좌표는 디스크에 영구 저장하지 않는다.
- 활성 trip 동안 연결돼 있으면 60초마다 `reason=periodic` 위치를 RPi5로 보낸다.
- SOS가 오면 위치 API를 기다리지 않고 캐시 결과부터 보낸다.
- `age_ms`는 측정 시각부터 전송 시각까지를 monotonic time으로 계산한다.

프리셋 좌표(`places.yaml`)와 실시간 폰 위치는 서로 다른 데이터다. 프리셋 좌표는 저장되지만 실시간 위치는 저장하지 않는다.

### 10.2 앱 → RPi5: `location.update`

다음은 정상 fix 예시다.

```json
{
  "version": 1,
  "type": "location.update",
  "id": "ce249750-c80b-47e1-a2ed-29c50b4f1b31",
  "sent_at_ms": 1787792420000,
  "reply_to": null,
  "payload": {
    "reason": "sos",
    "sos_id": "835fcafe-60ac-4517-9f06-50bcd2486b6c",
    "has_fix": true,
    "lat": 36.1234,
    "lon": 128.5678,
    "horizontal_accuracy_m": 4.2,
    "captured_at_ms": 1787792419200,
    "age_ms": 800,
    "source": "cache",
    "failure_reason": null
  }
}
```

측위에 실패해도 필드를 생략하지 않는다.

```json
{
  "version": 1,
  "type": "location.update",
  "id": "c1e35838-37e2-4789-88d5-e228120b09b2",
  "sent_at_ms": 1787792420000,
  "reply_to": null,
  "payload": {
    "reason": "sos",
    "sos_id": "835fcafe-60ac-4517-9f06-50bcd2486b6c",
    "has_fix": false,
    "lat": null,
    "lon": null,
    "horizontal_accuracy_m": null,
    "captured_at_ms": null,
    "age_ms": null,
    "source": "none",
    "failure_reason": "provider_disabled"
  }
}
```

| 필드 | 규칙 |
|---|---|
| `reason` | `periodic` 또는 `sos` |
| `sos_id` | SOS 응답이면 해당 UUID, periodic이면 `null` |
| `lat` | `-90..90` 또는 `null` |
| `lon` | `-180..180` 또는 `null` |
| `horizontal_accuracy_m` | 0 이상의 유한 수 또는 `null` |
| `source` | `cache`, `fresh`, `none` |
| `failure_reason` | `permission_denied`, `provider_disabled`, `timeout`, `unavailable`, `null` |

RPi5는 `age_ms > 300000` 또는 `horizontal_accuracy_m > 50`인 fix를 낮은 신뢰도로 표시하지만 폐기하지는 않는다.

### 10.3 RPi5 → 앱: `sos.request`

```json
{
  "version": 1,
  "type": "sos.request",
  "id": "4ccb97ee-a8bf-40de-a211-5254059694ad",
  "sent_at_ms": 1787792420000,
  "reply_to": null,
  "payload": {
    "sos_id": "835fcafe-60ac-4517-9f06-50bcd2486b6c",
    "reason": "tracking_lost",
    "severity": "critical",
    "trip_id": "bbef019a-dd3a-437e-8f5c-388913ae9eca"
  }
}
```

| 필드 | 허용값 |
|---|---|
| `reason` | `tracking_lost`, `stuck`, `low_battery`, `manual`, `unknown` |
| `severity` | `critical`, `warning` |
| `trip_id` | 활성 trip UUID 또는 `null` |

### 10.4 앱 → RPi5: `sos.ack`

```json
{
  "version": 1,
  "type": "sos.ack",
  "id": "584bb007-a09d-4389-9e90-511490b2d82c",
  "sent_at_ms": 1787792420100,
  "reply_to": "4ccb97ee-a8bf-40de-a211-5254059694ad",
  "payload": {
    "sos_id": "835fcafe-60ac-4517-9f06-50bcd2486b6c",
    "received": true
  }
}
```

### 10.5 SOS 순서와 시간 제한

```text
RPi5 bridge                                      Android app
     │  sos.request(sos_id)                           │
     ├───────────────────────────────────────────────►│
     │                                                ├─ UI/알림/진동 갱신
     │  sos.ack                          ≤ 500 ms     │
     │◄───────────────────────────────────────────────┤
     │  location.update(cache 또는 none) ≤ 500 ms     │
     │◄───────────────────────────────────────────────┤
     │                                                ├─ 새 고정밀 측위, 최대 30초
     │  location.update(fresh 또는 none)              │
     │◄───────────────────────────────────────────────┤
```

- 앱은 `sos.ack`와 캐시 위치 전송을 새 측위보다 우선한다.
- 캐시가 없어도 500 ms 안에 `source=none` 위치 메시지를 보낸다.
- 새 측위는 최대 30초 시도하고 성공 시 `source=fresh`를 한 번 보낸다.
- 30초 안에 실패하면 최종 `source=none`, `failure_reason=timeout`을 보낸다.
- SOS 처리 중에만 partial WakeLock을 잡을 수 있으며 35초 hard timeout과 `finally` 해제가 있어야 한다.
- 같은 `sos_id`를 다시 받으면 알림/진동을 중복 생성하지 않되 ACK와 현재 최선 위치는 다시 전송한다.
- RPi5는 ACK가 2초 안에 없으면 같은 `sos_id`로 최대 3회 재전송한다.
- ACK 3회 실패 시 앱 연결 실패 상태를 ROS에 발행하고 로컬 폴백을 실행한다. 2026-09-27 결정: 주행 중이면 로봇을 세우고 trip을 `error`로 끝낸다(`last_error` "어르신 폰이 응답하지 않아 멈췄어요"). 마커가 다시 보여도 자동으로 다시 따라가지 않으며, 다시 가려면 앱에서 새로 출발한다.

### 10.6 RPi5 → 앱: `sos.clear`

SOS 원인이 해소되거나 로봇 측에서 사건을 종료하면 같은 `sos_id`로 종료를 통지한다.

```json
{
  "version": 1,
  "type": "sos.clear",
  "id": "18dcbcc2-ecc8-48b9-9312-727947f9cb52",
  "sent_at_ms": 1787792450000,
  "reply_to": null,
  "payload": {
    "sos_id": "835fcafe-60ac-4517-9f06-50bcd2486b6c",
    "resolution": "tracking_recovered"
  }
}
```

`resolution`은 `tracking_recovered`, `handled`, `cancelled`, `unknown` 중 하나다. 앱은 `sos.clear` 전까지 해당 SOS를 active로 유지한다. 사용자가 SOS 화면의 `확인`을 누르면 이전 화면으로 돌아갈 수 있지만 ongoing SOS 표시와 알림은 `sos.clear`까지 유지한다. 재연결 시 active SOS가 있으면 서버가 `sos.request`를 다시 보내 상태를 복원한다.

로봇 측 해제 사유(2026-09-27): 추적을 5초 연속 회복 → `tracking_recovered`, SOS 중 앱의 `trip.cancel` → `cancelled`, `trip.start` → `handled`, 로봇의 mission 계층 종료 → `unknown`.

`[OPEN-R3]` ACK 3회 실패 때의 로컬 폴백 중 **정지는 2026-09-27에 결정**했다(10.5). 부저·경광등은 하드웨어가 없어 미결이며 현장 릴리스 전에 결정한다.

MVP에서는 RPi5 브리지가 앱의 SOS ACK와 위치를 ROS 2 토픽으로 변환하고 로봇 측 노드가 이를 확인한다. 앱 개발자는 그 뒤의 ROS 처리 방식을 구현하거나 알 필요가 없으며, 브리지 ACK를 받으면 앱→RPi5 전달이 완료된 것으로 본다.

`[OPEN-P1]` 로봇 측에서 확인한 좌표를 실제 보호자에게 어떻게 전달할지는 별도 제품 요구사항이다. 자동 SMS/전화는 Android 권한과 배포 정책 검토가 필요하므로 Android MVP 인수 범위에 넣지 않는다.

---


---

## 6. RPi5 ↔ ROS 2 내부 계약

이 절은 로봇 담당자용이며 Android 개발자는 구현할 필요가 없다. 앱이 어떤 값을 받게 되는지 이해하는 용도로 읽으면 된다. 토픽과 메시지는 ArUco 노드가 생기기 전에 연결 지점을 고정하기 위한 제안이다.

### 15.1 패키지

- 새 패키지: `software/ros_pkgs/mr_phone_bridge`
- 메시지 확장: `software/ros_pkgs/mr_msgs`
- node name: `mr_phone_bridge`
- WebSocket 처리와 ROS executor가 서로를 blocking하지 않게 분리한다.
- 잘못된 앱 입력은 ROS에 발행하기 전에 검증한다.

### 15.2 제안 토픽

| 방향 | 토픽 | 제안 타입 | QoS | 의미 |
|---|---|---|---|---|
| bridge → ROS | `/phone/trip_command` | `mr_msgs/TripCommand` | Reliable, KeepLast 10 | start/cancel 요청 |
| bridge → ROS | `/phone/location` | `mr_msgs/PhoneLocation` | Reliable, KeepLast 10 | periodic/SOS 위치 |
| bridge → ROS | `/phone/link_status` | `mr_msgs/PhoneLinkStatus` | Reliable, TransientLocal, KeepLast 1 | 앱 연결 상태 |
| bridge → ROS | `/robot/place_catalog` | `mr_msgs/PlaceCatalog` | Reliable, TransientLocal, KeepLast 1 | 현재 프리셋 전체와 revision |
| ROS → bridge | `/robot/app_status` | `mr_msgs/RobotAppStatus` | Reliable, TransientLocal, KeepLast 1 | 앱용 상위 상태 |
| ROS → bridge | `/safety/sos_request` | `mr_msgs/SosRequest` | Reliable, KeepLast 10 | ArUco/안전 계층의 SOS |
| ROS → bridge | `/safety/sos_clear` | `mr_msgs/SosClear` | Reliable, KeepLast 10 | 활성 SOS 종료 |
| bridge → ROS | `/safety/sos_ack` | `mr_msgs/SosAck` | Reliable, KeepLast 10 | 앱 ACK/위치 결과 |

`/phone/trip_command`의 cancel 요청은 로봇의 실제 주행을 안전하게 멈춰야 한다. WebSocket ACK와 ROS 명령 접수는 정지 완료가 아니며, mission/safety 계층이 정지를 완료한 뒤 `/robot/app_status`의 mission 상태를 `cancelled`로 갱신한다.

새 메시지는 최소한 아래 정보를 보존해야 한다.

- `TripCommand`: request id, trip id, start/cancel enum, from/to place id, ROS stamp
- `PhoneLocation`: message id, optional sos id, has-fix, lat/lon, accuracy, age, source, failure reason
- `PhoneLinkStatus`: connected, app instance id, session id, last seen stamp
- `PlaceCatalog`: revision, place 배열(id, kind, label, lat, lon, radius_m, enabled, sort_order, is_default_origin)
- `RobotAppStatus`: mission/tracking 상태, trip id, from/to, optional battery, last error
- `SosRequest`: sos id, reason, severity, trip id, ROS stamp
- `SosClear`: sos id, resolution, ROS stamp
- `SosAck`: sos id, app received 여부, 위치 상태, ROS stamp

정확한 `.msg` 필드와 enum 숫자는 구현 PR에서 확정하되 WebSocket v1 의미를 바꾸면 안 된다.

### 15.3 프리셋 저장 규칙

- `places.yaml`의 소유자는 브리지다. 임시 파일에 쓰고 `rename`으로 교체하는 원자적 갱신을 사용한다.
- 갱신 성공 후에만 새 `revision`을 발급하고 `place.catalog` 브로드캐스트와 `/robot/place_catalog` 발행을 수행한다.
- 갱신 실패는 `INTERNAL_ERROR`로 회신하고 기존 파일을 유지한다.
- 최근 N개 이전 버전을 백업해 롤백할 수 있게 한다.
- 앱이 보낸 값은 좌표 범위, 라벨 길이, `kind` enum, 개수 상한, 기본 출발지 유일성까지 서버에서 다시 검증한다. 앱 검증을 신뢰하지 않는다.

### 15.4 현재 베이스 스택과의 관계

- `/base/status`를 읽을 경우 현재 계약대로 `BEST_EFFORT`, `KEEP_LAST(1)` 구독을 사용한다.
- `/base/status`에는 배터리 필드가 없으므로 `battery_percent=null`이 정상이다.
- 브리지는 `/joint_states`, `/odom`, TF를 앱으로 그대로 전달하지 않는다.
- 브리지는 `/cmd_vel`을 구독하거나 발행하지 않는다.
- 앱 heartbeat를 MCU 명령 워치독의 feed 조건으로 사용하지 않는다.

### 15.5 향후 ArUco 연결

ArUco 노드는 앱이나 WebSocket을 알 필요가 없다.

1. 추적 상태가 바뀌면 상위 상태 노드 또는 ArUco 노드가 `/robot/app_status.tracking_state`를 갱신한다.
2. 사용자 추적 상실 등 SOS 조건이 확정되면 `/safety/sos_request`를 한 번 발행한다. 2026-09-27 구현(`mr_mission`): 주행 중 이번 주행에서 마커를 잡은 뒤 놓친 상태가 10초 이어지면 `tracking_lost`/`critical`.
3. 재시도, 앱 ACK, GPS 회신은 `mr_phone_bridge`가 담당한다.
4. 추적 회복 등 종료 조건이 확정되면 같은 id로 `/safety/sos_clear`를 발행한다.
5. ArUco 노드가 결과를 알아야 하면 `/safety/sos_ack`를 구독한다.

이 구조에서는 ArUco 구현이 늦어져도 로봇 담당자가 CLI test publisher로 SOS를 발행해 앱 통합 시험을 할 수 있다.

---


---
## 7. 구현 노트 (서버)

### 7.1 패키지 구조와 mock bridge

`mr_phone_bridge`는 ament_python 패키지지만 **프로토콜 코어는 rclpy를 import하지 않는다.**

```text
mr_phone_bridge/
  protocol/     envelope 파싱·검증(JSON Schema), 오류 코드, 중복 id 캐시(10분), 세션 상태 머신
  places/       places.yaml 로드·검증·원자적 갱신(tmp + rename)·백업 N개·revision 발급
  server/       websockets 기반 WS 서버, heartbeat 타임아웃, 16 KiB 상한, rate limit
  ros/          rclpy 노드. mr_care_msgs 변환. 위 모듈과 executor를 분리한다(스레드 또는 asyncio 브리지)
  mock/         ROS 없이 server+protocol+places만 띄우는 진입점. 시나리오 스위치 제공
```

mock bridge는 앱 저장소가 아니라 **여기**에 있다. 개발 PC에서 `python3 -m mr_phone_bridge.mock --scenario default6`로 띄우고, 앱 저장소의 `tools/mock-bridge/run.sh`는 이 명령을 감싸기만 한다. 실제 서버와 mock이 같은 코드를 쓰므로 계약 드리프트가 생기지 않는다.

mock 시나리오(앱 명세 18장 요구사항):
`default6`(집·시장·마트·노인정·교회·절), `empty`, `conflict`(CATALOG_CONFLICT), `limit`(PLACE_LIMIT), `in-use`(PLACE_IN_USE), `slow-ack`(2 s 지연), `error-ack`, `flaky`(주기적 강제 종료), `sos`(tracking_lost 발행), `sos-x3`(같은 sos_id 3회), `sos-clear`, `garbage`(잘못된 JSON·알 수 없는 enum).

### 7.2 WebSocket ↔ mr_care_msgs 매핑

| WebSocket | ROS 토픽 | 타입 | 비고 |
|---|---|---|---|
| `trip.start` / `trip.cancel` | `/phone/trip_command` | `TripCommand` | start 시 브리지가 `trip_id` 발급 |
| `location.update` | `/phone/location` | `PhoneLocation` | `low_confidence` 계산은 브리지 |
| 세션 열림/닫힘/heartbeat 타임아웃 | `/phone/link_status` | `PhoneLinkStatus` | TransientLocal |
| `place.upsert` / `place.delete` 성공 후 | `/robot/place_catalog` | `PlaceCatalog` | TransientLocal, 시작 시 1회 |
| ← `robot.status` | `/robot/app_status` | `RobotAppStatus` | mission 계층이 발행. 없으면 브리지가 `mission_state=unknown`으로 채움 |
| ← `sos.request` | `/safety/sos_request` | `SosRequest` | 브리지가 2 s × 3회 재전송 |
| ← `sos.clear` | `/safety/sos_clear` | `SosClear` | |
| `sos.ack` + 위치 결과 → | `/safety/sos_ack` | `SosAck` | ACK 3회 실패도 `app_received=false`로 1회 발행 |

enum은 ROS에서 정수 상수, WebSocket에서 문자열이다. 변환표는 `mr_phone_bridge/ros/enums.py` 한 곳에만 둔다. 모르는 정수는 문자열 `unknown`으로, 모르는 문자열은 `INVALID_MESSAGE`로 처리한다.

### 7.3 `robot.status`를 채우는 방법 (mission 계층이 없을 때)

현재 `mobile-robot`에는 trip을 관리하는 mission 노드가 없다. 브리지는 다음 순서로 채운다.

1. `/robot/app_status`가 있으면 그대로 옮긴다.
2. 없으면 `mission_state=unknown`, `tracking_state`는 `marker_track`(`mr_aruco_msgs/MarkerTrack`)의 `status`로 채운다(TRACKING→`following`, HOLDING→`following`, LOST→`lost`, 토픽 없음→`unavailable`). 이때는 주행 여부를 모르므로 `idle`을 쓰지 않는다.
3. `battery_percent`는 인터페이스가 없으므로 `null`.
4. `bridge_state`는 브리지 자신의 상태다: ROS 연결 정상 `ready`, `/robot/app_status`·`marker_track` 모두 stale이면 `degraded`, 내부 예외면 `error`.

최소 mission 노드(`mr_mission`, 별도 태스크)는 `TripCommand`를 받아 `idle→starting→active`, cancel 시 `cancelling→cancelled`로 전이하고 follow 노드를 켜고 끈다. **도착(`arrived`) 판정 수단은 미결이다**(로봇에 GPS가 없음). 그때까지 `arrived`는 발행하지 않는다.

### 7.4 places.yaml

- 경로: `~/ros2_ws/config/places.yaml`(RPi5). launch 파라미터 `places_path`로 바꿀 수 있다.
- 형식은 4.1절 예시. `revision`은 갱신 시각 ISO-8601 UTC.
- 갱신: 임시 파일에 쓰고 `os.replace`. 성공 후에만 revision 발급·`place.catalog` 브로드캐스트·`/robot/place_catalog` 발행. 직전 5개 버전을 `places.yaml.bak.N`으로 남긴다.
- 서버는 앱 검증을 신뢰하지 않고 좌표 범위·라벨 길이·kind·개수 상한·기본 출발지 유일성을 다시 검사한다.

### 7.5 네트워크 배치

- RPi5는 어르신 폰의 핫스팟에 붙는다. 브리지 포트 8765는 핫스팟 인터페이스에만 연다(ufw).
- mDNS: `_mr-phone-bridge._tcp`, TXT `api=1`, `robot_id=<id>`. avahi 서비스 파일은 `mr_phone_bridge/config/avahi/`.
- 텔레메트리(`mr_care_telemetry`)도 같은 핫스팟으로 인터넷에 나간다. 폰 핫스팟이 꺼지면 둘 다 끊기는 것이 정상이며, 텔레메트리는 SQLite 큐에 이벤트를 쌓아 재전송한다.

## 8. 미결 항목

| ID | 내용 | 차단 대상 | 담당 |
|---|---|---|---|
| `OPEN-R1` | 공용망 사용 시 `wss://`·페어링 토큰 | 현장 릴리스 | 로봇 |
| `OPEN-R3` | SOS ACK 3회 실패 시 로컬 폴백. 정지는 2026-09-27 결정(10.5), 부저·경광등 미결 | 현장 릴리스 | 로봇 |
| `OPEN-N1` | 도착(`arrived`) 판정 수단. 로봇 GPS 모듈, 폰 GPS 참고, 또는 수동 종료 중 결정 | trip 완결 | 제품·로봇 |
| `OPEN-M1` | Kakao REST 키 보호(사용량 제한 또는 RPi5 프록시) | 현장 릴리스 | 로봇·제품 |

## 9. 변경 이력

| 날짜 | 변경 | 소비 측 할 일 |
|---|---|---|
| 2026-09-27 | 9.7 `tracking_state`에 `idle` 추가(v1이 허용하는 enum 추가)와 값별 뜻 표. `following`·`searching`·`lost`는 주행 중에만 | 앱: enum에 `IDLE`, 문구(예: "대기 중"). 그 전에는 `unknown`으로 보인다 |
| 2026-09-27 | 7.1 URL 행을 실제 연결 방식(`ws://<RPi5 IP>:8765/bridge/v1`)으로, 7.3에 앱의 평문 허용 범위 명시 | 없음(앱이 이미 그렇게 구현) |
| 2026-09-27 | 10.5·10.6·8: OPEN-R3 정지 결정, 로봇 측 SOS 해제 사유 | 없음(표시는 기존 `error`·`sos.clear` 그대로) |
| 2026-09-27 | 8.4: 주행 중 앱 연결이 30초 끊기면 로봇이 멈추고 trip `error`("폰 연결이 끊겨 멈췄어요"), 자동 재개 없음 | 없음(재연결 뒤 기존 `robot.status`의 `error`·`last_error` 표시 그대로). 문서 sync만 |
