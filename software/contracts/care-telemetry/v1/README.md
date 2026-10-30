# care-telemetry/1 — RPi5 → Firebase RTDB → 보호자 대시보드

> 상태: v1 초안 확정(2026-09-27). 이 디렉터리가 데이터 모델의 **단일 정본**이다.
> 기계가 읽는 정본: `schema/*.schema.json`, `fixtures/*.json`, `database.rules.json`.

## 1. 참여자

```text
[ROS 2 토픽] ──▶ [RPi5 mr_care_telemetry] ──HTTPS(Admin SDK)──▶ [Firebase RTDB senior-care-nyr] ──onValue──▶ [React 대시보드]
```

| 참여자 | 저장소 | 역할 |
|---|---|---|
| `mr_care_telemetry` | `mobile-robot` | ROS 구독 → JSON 요약 → RTDB 쓰기. 서비스 계정(Admin SDK)으로 쓰므로 보안 규칙을 우회한다. 단절 시 이벤트를 SQLite에 보관 후 재전송 |
| 대시보드 웹 | `senior-care-dashboard` | Google 로그인 후 `/robots/{robot_id}`의 자식 경로(`meta`·`status`·`events`·`sos`, `dev`는 `diag`도)를 각각 구독. 쓰기는 `/users/{uid}`만 |
| Firebase | 프로젝트 `senior-care-nyr`, RTDB 인스턴스 `senior-care-nyr-default-rtdb`(asia-southeast1) | |

원칙: **웹에서 로봇을 제어하지 않는다.** 영상·센서 원본·정밀 위치는 보내지 않는다. 유일한 예외는 SOS 활성 중의 어르신 폰 위치(§3.4)이며 SOS 종료 시 삭제한다.

## 2. 경로 개요

```text
/robots/{robot_id}/meta                 로봇 식별. 최초 1회 + 에이전트 시작 시
/robots/{robot_id}/status               최신 상태 1개(덮어쓰기). 변경 즉시 + 5 s 주기
/robots/{robot_id}/events/{event_id}    이벤트 로그(append). 즉시. 단절 시 큐 → 재전송
/robots/{robot_id}/sos/{sos_id}         활성 SOS(위치 포함). sos.clear 시 노드 삭제
/robots/{robot_id}/diag                 개발자 진단(P2). 5 s 주기. `dev` 역할만 읽는다
/access/{robot_id}/{uid}                열람 권한. 콘솔 또는 CLI로 사람이 넣는다
/users/{uid}                            사용자 프로필. 웹이 첫 로그인 때 스스로 쓴다
```

`robot_id`는 `hello.ack`의 값과 같다(`mr-001`). 시각은 모두 Unix epoch **milliseconds** 정수, 필드 이름은 `*_ms`.

**RTDB 키 규칙.** 이 계약의 모든 객체 키는 RTDB 키여야 한다: 비어 있지 않고, `.` `$` `#` `[` `]` `/`와 ASCII 제어 문자(0x00–0x1F, 0x7F)를 쓰지 않으며, UTF-8 768바이트 이하. 어기면 RTDB가 쓰기 전체를 `400`으로 거절한다. `tools/validate_fixtures.py`가 이 계약의 fixture 키를 검사한다. RTDB는 `null`과 빈 객체·빈 배열을 저장하지 않으므로, 읽는 쪽은 그런 필드가 없는 것을 `null`로 본다.

## 3. 문서 형식

### 3.1 `status` (schema/status.schema.json)

```json
{
  "updated_at_ms": 1790000000000,
  "agent": { "version": "0.1.0", "uptime_s": 3600, "queue_len": 0 },
  "bridge_state": "ready",
  "mission_state": "active",
  "tracking_state": "following",
  "trip_id": "bbef019a-dd3a-437e-8f5c-388913ae9eca",
  "from_place_id": "place_home",
  "to_place_id": "place_market",
  "battery_percent": null,
  "last_error": null,
  "phone_link": { "connected": true, "last_seen_ms": 1789999999000 },
  "base": { "agent_state": "connected", "cmd_watchdog_ok": true, "last_seen_ms": 1789999999900 }
}
```

- enum 값은 phone-bridge/1 `robot.status`와 **같은 문자열**을 쓴다. 웹은 모르는 값을 `unknown`으로 표시한다.
- `tracking_state`의 값별 뜻은 phone-bridge/1 PROTOCOL 9.7 표. 2026-09-27에 `idle`(주행 중이 아니라 추적하지 않음)이 추가됐고, `following`·`searching`·`lost`는 주행 중에만 나온다.
- 온라인 판정은 웹이 한다: `now - updated_at_ms < 15000`이면 online. 에이전트는 정상 종료 시 `mission_state`를 건드리지 않고 `agent.uptime_s`만 남긴다. `online` 필드는 두지 않는다(REST에는 onDisconnect가 없다).
- `base`는 `/base/status`(`mr_msgs/BaseStatus`) 요약이며 진단용이다. `agent_state`: `waiting|available|connected|disconnected`.

### 3.2 `events/{event_id}` (schema/event.schema.json)

`event_id`는 RTDB push id(시간순 정렬). 에이전트가 오프라인에서 만든 이벤트도 생성 시각 기준 push id를 붙여 순서를 지킨다.

```json
{
  "ts_ms": 1790000000000,
  "kind": "sos.request",
  "severity": "critical",
  "trip_id": "bbef019a-dd3a-437e-8f5c-388913ae9eca",
  "sos_id": "835fcafe-60ac-4517-9f06-50bcd2486b6c",
  "message": "로봇이 어르신을 놓쳤어요",
  "data": { "reason": "tracking_lost" }
}
```

| kind | severity | data |
|---|---|---|
| `agent.online` / `agent.offline` | info | `{ "version" }` |
| `link.connected` / `link.disconnected` | info / warning | `{ "app_instance_id" }` |
| `trip.start` | info | `{ "from_place_id", "to_place_id" }` |
| `trip.arrived` / `trip.cancelled` | info | `{ "reason" }` |
| `trip.error` | warning | `{ "error" }` |
| `sos.request` | critical / warning (`severity` 그대로) | `{ "reason" }` |
| `sos.ack` | info | `{ "app_received", "retries", "location": "none|cache|fresh", "accuracy_bucket": "good|fair|poor|none" }` |
| `sos.clear` | info | `{ "resolution" }` |
| `error` | warning | `{ "source", "error" }` |

이벤트에는 **좌표를 넣지 않는다.** 위치는 §3.4의 `sos` 노드에만 있고 SOS 종료와 함께 사라진다. `message`는 사용자에게 보여도 되는 한국어 한 문장이다.

보존: 에이전트가 하루 1회 오래된 이벤트를 지워 로봇당 최대 1,000개를 유지한다. 웹은 `limitToLast(200)`으로 읽는다.

### 3.3 `meta`

```json
{ "robot_id": "mr-001", "display_name": "돌봄 로봇 1호", "agent_version": "0.1.0", "first_seen_ms": 1789000000000 }
```

### 3.4 `sos/{sos_id}` (schema/sos.schema.json) — P2

```json
{
  "started_at_ms": 1790000000000,
  "updated_at_ms": 1790000012000,
  "reason": "tracking_lost",
  "severity": "critical",
  "trip_id": "bbef019a-dd3a-437e-8f5c-388913ae9eca",
  "app_received": true,
  "location": {
    "lat": 36.1234, "lon": 128.5678, "accuracy_m": 4.2,
    "captured_at_ms": 1789999999200, "source": "cache", "low_confidence": false
  }
}
```

- `sos.request` 수신 시 `location=null`로 생성, `PhoneLocation`이 올 때마다 `location` 갱신(cache → fresh 순서로 최대 3회), `sos.clear`에서 노드 삭제.
- 이 경로는 에이전트 설정 `publish_sos_location: true`일 때만 쓴다. 기본값 `true`. 대시보드 P1 구현에서는 이 노드가 없어도 동작해야 한다.

### 3.5 `diag` (schema/diag.schema.json) — P2

```json
{
  "updated_at_ms": 1790000000000,
  "topics": {
    "base": { "status": { "last_seen_ms": 1789999999900, "hz": 9.8 } },
    "joint_states": { "last_seen_ms": 1789999999950, "hz": 49.6 },
    "marker_track": { "last_seen_ms": 1789999990000, "hz": 0 },
    "robot": { "app_status": { "last_seen_ms": 0, "hz": 0 } }
  },
  "recent_errors": [ { "ts_ms": 1789999000000, "source": "telemetry", "error": "RTDB write timeout" } ]
}
```

- **`topics`는 ROS 토픽 이름을 RTDB 경로로 펼친 트리다**(2026-09-27 확정). 토픽 이름의 앞 `/`를 떼고 `/`마다 한 단계 내려간다: `/base/status` → `topics/base/status`. RTDB 키에 `/`를 쓸 수 없어서다(위 "RTDB 키 규칙").
- 잎(leaf)은 `last_seen_ms`를 가진 객체다: `{ "last_seen_ms", "hz" }`. 읽는 쪽은 트리를 내려가다 `last_seen_ms`가 있는 노드를 만나면 거기서 멈추고, 지나온 키를 `/`로 이어 `'/' + …`를 토픽 이름으로 쓴다.
- 한 토픽 이름이 다른 토픽 이름의 앞부분 경로가 되면 안 된다(`/robot`과 `/robot/app_status`를 함께 쓰지 않는다). 에이전트가 쓰는 토픽은 `/base/status`·`/joint_states`·`/marker_track`·`/robot/app_status` 넷이다.
- `last_seen_ms` = 0은 한 번도 받지 못했다는 뜻. `hz`는 직전 주기 동안의 평균 수신 빈도(≥ 0).
- `recent_errors`: 에이전트 쪽 최근 오류 최대 10개, 오래된 것부터. 오류가 없으면 필드가 없다(RTDB가 빈 배열을 저장하지 않는다). RTDB는 배열을 `"0"`, `"1"` … 키 객체로 저장하므로 읽는 쪽은 배열과 그 객체 모양을 모두 받아들인다.
- 읽기는 `access` 역할이 `dev`인 사용자만(§4). 보호자(`guardian`)는 읽을 수 없다. 오류 문구에 경로·호스트 같은 내부 정보가 들어갈 수 있어서다.

### 3.6 `access`, `users`

```json
"access": { "mr-001": { "<uid>": "guardian", "<uid2>": "dev" } }
"users":  { "<uid>": { "email": "…", "display_name": "…", "last_login_ms": 1790000000000 } }
```

`access`는 사람이 콘솔이나 `firebase database:set`으로 넣는다. 웹은 `/access/{robot_id}/{자기 uid}`를 읽어 역할을 알고, `dev`일 때만 진단 탭을 보인다.

## 4. 보안 규칙

`database.rules.json`이 정본이며 대시보드 저장소가 복사해 `firebase deploy --only database`로 배포한다.

- `/robots/{robot_id}/{meta,status,events,sos}`: `/access/{robot_id}/{auth.uid}`가 있는 로그인 사용자만 읽기.
- `/robots/{robot_id}/diag`: `/access/{robot_id}/{auth.uid}`가 `"dev"`인 사용자만 읽기(2026-09-27부터. 그 전에는 역할과 무관하게 읽을 수 있었다).
- `/robots/{robot_id}` 자체(자식 전체)를 한 번에 읽는 것은 누구에게도 허용하지 않는다. RTDB 규칙은 위에서 준 읽기 권한을 아래에서 되돌릴 수 없으므로, 읽기 권한은 자식마다 따로 준다. 웹은 자식 경로를 각각 구독한다.
- `/robots/**`: 클라이언트 쓰기 금지.
- `/access/{robot_id}/{uid}`: 본인 항목만 읽기. 쓰기 금지.
- `/users/{uid}`: 본인만 읽기·쓰기.
- 에이전트는 서비스 계정으로 쓰므로 규칙 대상이 아니다. 서비스 계정 JSON은 RPi5의 `~/.config/mr_care_telemetry/service-account.json`에만 두고 **어느 저장소에도 넣지 않는다.**

## 5. 에이전트 동작 규칙

- `status`: `RobotAppStatus`·`PhoneLinkStatus`·`/base/status` 중 하나라도 바뀌면 즉시, 변화 없어도 5 s마다 덮어쓴다. 오프라인이면 버린다(최신값만 의미 있음).
- `events`: 즉시 쓰기. 실패하면 SQLite(`~/.local/state/mr_care_telemetry/queue.db`)에 넣고 지수 백오프(1, 2, 4 … 60 s)로 재전송. 재전송 시 `ts_ms`는 원래 시각을 유지한다.
- `sos`: 즉시 쓰기·갱신·삭제. 오프라인 중 `sos.clear`가 오면 삭제 작업도 큐에 넣는다.
- `diag`(설정 `publish_diag: true`일 때만): 5 s마다 덮어쓴다. 실패하면 버린다(최신값만 의미 있음).
- RTDB가 `400` 같은 영구 오류로 거절한 쓰기는 다시 보내지 않고 **에이전트 로그에 ERROR로 남긴다**(같은 경로의 같은 오류는 묶어서 주기적으로). `diag.recent_errors`에만 넣으면 `diag` 자체가 거절될 때 오류가 보이지 않는다.
- Firebase 인증은 Admin SDK(`firebase-admin` 파이썬) 서비스 계정. REST로 직접 쓸 경우 서비스 계정 OAuth2 토큰을 붙인다.
- 어떤 경우에도 텔레메트리 실패가 브리지나 베이스 제어에 영향을 주면 안 된다. 별도 프로세스(systemd 유닛)로 돈다.

## 6. 변경 규칙

- 필드 추가는 선택 필드만. 웹은 없는 필드를 `null`/`unknown`으로 처리한다.
- enum 추가는 허용. 삭제·의미 변경은 `care-telemetry/2`와 새 경로 접두어(`/v2/robots/...`)가 필요하다.
- 변경 절차: 이 문서 → 스키마·fixture → `tools/validate_fixtures.py` → 에이전트·웹 양쪽 테스트 → 병합.

## 7. 변경 이력

| 날짜 | 변경 | v1 안에서 한 이유 |
|---|---|---|
| 2026-09-27 | §3.5 `diag.topics`를 토픽 이름 키(`"/base/status"`)에서 중첩 경로(`topics/base/status`)로 확정. `schema/diag.schema.json`·`fixtures/diag-*.json` 추가. §2에 RTDB 키 규칙 명시 | 이전 모양은 RTDB가 `400`으로 거절해 한 번도 저장된 적이 없다(읽는 쪽이 받아 본 적 없는 모양). 대시보드는 이미 중첩 경로를 읽는다 |
| 2026-09-27 | `status.tracking_state`에 `idle` 추가(phone-bridge/1과 같은 enum). `fixtures/status-idle-tracking-idle.json` | enum 추가는 v1 허용. 웹은 모르는 값을 `unknown`으로 보이므로 sync 전에도 깨지지 않는다 |
| 2026-09-27 | §4 `diag` 읽기를 `dev` 역할로 제한. 읽기 권한을 `/robots/{id}`에서 자식(`meta`·`status`·`events`·`sos`·`diag`)으로 내림 | 웹은 자식 경로만 구독하고 진단 탭은 이미 `dev`에게만 보인다. 보호자 화면 동작은 바뀌지 않는다 |
