# mr_care_telemetry

ROS 2 상태·이벤트를 요약해 Firebase RTDB에 쓰는 에이전트(`care-telemetry/1`). 데이터 모델 정본은
`software/contracts/care-telemetry/v1/README.md`이고 이 문서는 **에이전트 구현 선택**만 적는다.

```text
mr_care_telemetry/
  agent.py    코어: 입력(on_*) → status/events/sos 문서, 전송 스레드, 백오프, meta, 보존 정리   (rclpy 없음)
  rtdb.py     RTDB REST 클라이언트 + 서비스 계정 JWT(OAuth2) 토큰                              (rclpy 없음)
  outbox.py   SQLite 오프라인 큐(events·sos 쓰기를 순서대로)                                   (rclpy 없음)
  model.py    문서·한국어 문구·push id·sos_id 변환                                             (rclpy 없음)
  sim.py      ROS 없는 가짜 입력(--sim)
  node.py     rclpy 노드: 토픽 → agent.on_*
```

## 인증과 의존성

- **firebase-admin을 쓰지 않는다.** RPi5(Ubuntu 24.04)에서는 pip 설치가 필요해서다. 대신 계약 README 5장이 허용한
  "REST + 서비스 계정 OAuth2 토큰"을 쓴다. 서비스 계정 JSON으로 RS256 JWT를 서명해 토큰을 받고(`rtdb.py`),
  `Authorization: Bearer`로 붙인다. 필요한 것은 표준 라이브러리와 `python3-cryptography`(apt, 우분투 기본 설치)뿐이다.
- 서비스 계정 파일: RPi5 `~/.config/mr_care_telemetry/service-account.json`(`chmod 600`). **어느 저장소에도 넣지 않는다.**
  파일이 없어도 에이전트는 죽지 않는다. 오프라인으로 보고 이벤트를 큐에 쌓다가 파일이 생기면 재시작 없이 보낸다.
- 에뮬레이터: `FIREBASE_DATABASE_EMULATOR_HOST=host:port`(또는 파라미터 `emulator_host`)면
  `http://host:port/<path>.json?ns=senior-care-nyr-default-rtdb`에 `Bearer owner`로 쓴다.
- 실제 Firebase(`senior-care-nyr`)에서 JWT → 토큰 발급과 status·meta·events·sos 쓰기·삭제를 확인했다(2026-09-27, 개발 PC). 서명·클레임·토큰 캐시·401 재발급은 로컬 가짜 토큰 서버로 시험한다.

## 실행

```bash
# RPi5 (M2)
ros2 launch mr_care_telemetry care_telemetry.launch.py [params_file:=…]
sudo cp config/systemd/mr-care-telemetry.service /etc/systemd/system/ && sudo systemctl enable --now mr-care-telemetry

# ROS 없이 대시보드 개발용 시뮬(대시보드 저장소의 에뮬레이터 127.0.0.1:9000에 쓴다)
cd software/ros_pkgs/mr_care_telemetry
python3 -m mr_care_telemetry --sim --emulator 127.0.0.1:9000 [--fast] [--cycles 1] [--robot-id mr-001]
# 실제 DB에 시뮬 데이터(가짜 데이터가 실제 경로에 들어가므로 --real과 별도 robot_id를 쓴다)
python3 -m mr_care_telemetry --sim --real --robot-id mr-sim --service-account ~/.config/mr_care_telemetry/service-account.json
```

`--sim` 한 사이클(기본 약 60 s, `--fast`면 약 12 s): 앱 연결 → 집→시장 출발 → 추적 상실 SOS → 앱 ACK + 캐시 위치 →
새 위치 → 추적 회복 → 멈춤 → idle → 앱 연결 끊김. 실제 노드와 같은 `agent.on_*`를 부르므로 이벤트 규칙까지 그대로 시험된다.
대상(`--emulator` 또는 `--real`)을 주지 않으면 실행을 거부한다.

## 동작 규칙 (계약이 정하지 않은 부분)

| 항목 | 동작 |
|---|---|
| `status` 쓰기 | 입력이 바뀌면 곧바로(최소 0.5 s 간격), 아니면 5 s마다 PUT. 실패하면 버린다. 큐에 넣지 않는다 |
| `bridge_state` | `/phone/link_status` 발행자가 없으면(브리지 없음) `unknown`. 있으면 `/robot/app_status`(6 s 안) 또는 `/marker_track`(1 s 안)이 신선하면 `ready`, 아니면 `degraded` |
| `mission_state` 등 | `/robot/app_status`가 6 s 넘게 없으면 `unknown`·trip 필드 null. `tracking_state`는 marker_track 폴백 |
| `phone_link.connected` | 브리지가 없으면 false. `last_seen_ms`는 `PhoneLinkStatus.last_seen` |
| `base` | `/base/status`(BEST_EFFORT)를 한 번이라도 받았을 때만. `agent_state`·`cmd_watchdog_ok`가 바뀌면 곧바로 |
| null 필드 | RTDB는 null을 저장하지 않는다. 웹은 없는 필드를 null로 읽는다(계약 6장) |
| 이벤트 기준값 | 기동 직후 처음 받은 app_status·link_status는 기준값이다. 이미 진행 중인 trip·연결에 대해 이벤트를 다시 내지 않는다 |
| `trip.start` | app_status의 trip_id가 처음 `starting`/`active`가 될 때. 문구 "{출발지}에서 {목적지}(으)로 출발했어요"(장소 이름은 `/robot/place_catalog`, 받침에 따라 으로/로) |
| `trip.cancelled` | `cancelled`가 될 때. `data.reason`은 같은 trip의 `TripCommand` cancel 사유(없으면 `unknown`) |
| `trip.error` | `error`가 될 때 한 번. 문구는 mission의 `last_error` |
| `trip.arrived` | 규칙은 있지만 mission이 arrived를 내지 않는다(OPEN-N1) |
| `link.*` | `PhoneLinkStatus.connected`가 바뀔 때. 끊김은 warning |
| `sos.request` | `/safety/sos_request`. severity 그대로. 같은 sos_id 중복은 무시 |
| `sos.ack` | 성공 ACK는 1 s 모았다가 한 번(곧 오는 캐시 위치를 `location`·`accuracy_bucket`에 반영). 실패(app_received=false)는 즉시. 실패 뒤 늦은 성공은 한 번 더 |
| `accuracy_bucket` | fix 없음 `none`, ≤10 m `good`, ≤50 m `fair`, 그 이상·low_confidence `poor` |
| `sos.clear` | `/safety/sos_clear`. 모아 둔 ack가 있으면 먼저 낸다. 이후 같은 sos_id의 ack·위치는 무시 |
| sos_id | UUID v4가 아니면(CLI) 브리지와 같은 해시 규칙으로 바꾼 id를 RTDB 키·이벤트에 쓴다(앱 위치의 sos_id와 맞추려고) |
| `sos/{id}` | request에 PUT(location null), ACK 상태가 바뀌면 PATCH, 위치는 품질이 같거나 좋아질 때 최대 3회 PATCH, clear에 DELETE. 모두 큐를 거친다. `publish_sos_location=false`면 쓰지 않는다 |
| 위치 개인정보 | periodic 위치는 쓰지 않는다. 이벤트에 좌표를 넣지 않는다 |
| 큐 | SQLite `~/.local/state/mr_care_telemetry/queue.db`, 최대 5000개(넘으면 오래된 것부터 버림). id 순서로만 보낸다 |
| 재전송 | 일시 오류는 1, 2, 4 … 60 s 백오프로 맨 앞부터 다시. status가 다시 성공하면 백오프를 기다리지 않고 바로 보낸다 |
| 오류 분류 | **영구 오류는 RTDB의 400·413뿐**(키 규칙 위반·크기 초과: 다시 보내도 같다). 큐 작업이면 버리고 `ERROR` 로그. 그 밖의 4xx(404 = DB URL 오류, 401/403, 402/423 등)·5xx·네트워크·**토큰 발급 실패(키 오류·시계 어긋남 포함)**는 일시 오류라 큐에 남는다(2026-09-27: 전에는 404·토큰 400도 영구로 봐서 이벤트를 버릴 수 있었다) |
| 로그 | 노드로 띄우면 에이전트의 파이썬 로그(`agent`·`outbox`)를 ROS 로거 `mr_care_telemetry`로 옮긴다(launch 출력·`/rosout`·journald). 2026-09-27 전에는 옮기지 않아 `RTDB online`·이벤트 줄이 어디에도 안 나왔고 ERROR만 형식 없이 stderr로 갔다 |
| 영구 오류 로그 | status·diag·meta·보존 정리가 영구 오류로 거절되면 `ERROR RTDB rejected <대상>: …`를 대상마다 처음 한 번, 그 뒤로는 5분마다 "N more"로 묶어 남긴다. 다시 받아들여지면 `INFO RTDB accepts <대상> again`. status가 거절돼도 온라인으로 본다(RTDB는 닿았다). meta가 거절돼도 큐를 막지 않는다. 오류는 `diag.recent_errors`에도 들어가지만 거기에만 두면 diag 자체가 거절될 때 안 보인다(2026-09-27 diag 미기록 원인) |
| meta | 온라인이 되면 한 번. `first_seen_ms`는 이미 있으면 유지 |
| 보존 | 시작 시와 하루 1회 `events`를 shallow로 세어 1000개를 넘는 오래된 것을 지운다 |
| 종료 | `agent.offline`을 남기고 최대 3 s 동안 큐를 보낸다. 못 보낸 것은 다음 기동 때 보낸다 |
| `diag`(P2) | `publish_diag: true`면 5 s마다 `/base/status`·`/joint_states`·`/marker_track`·`/robot/app_status`의 마지막 수신 시각·Hz와 최근 오류 10개. 토픽 이름은 계약 3.5대로 **RTDB 경로로 펼친다**(`topics/base/status/{last_seen_ms,hz}`). 오류가 없으면 `recent_errors` 필드 없음. 실패하면 버린다(최신값만). 보호자(`guardian`)는 규칙상 읽지 못한다 |

## 시험

```bash
cd software/ros_pkgs/mr_care_telemetry
uv run --with pytest --with jsonschema --with cryptography python3 -m pytest test     # ROS 없이 (노드 시험 skip)
colcon test --packages-select mr_care_telemetry                                       # ROS 셸
```

- `test_units.py`: 문구·push id·outbox·REST URL·JWT 흐름(가짜 토큰 서버가 서명 검증)·오류 분류.
- `test_agent.py`: 메모리 RTDB + 가짜 시계로 이벤트 매핑, status 규칙, sos 노드, 오프라인 백오프·순서·`ts_ms` 보존, 보존 정리,
  diag 모양, 영구 오류 로그. 메모리 RTDB(`conftest.MemoryRtdb`)는 **실제 RTDB처럼 키 규칙(`. $ # [ ] /`·제어 문자·빈 키)을
  어긴 쓰기를 400으로 거절한다** — 제품 코드와 따로 구현해 같은 실수가 양쪽에 숨지 않게 했다.
- `test_emulator.py`: **실제 RTDB 에뮬레이터**(firebase-tools가 받아 둔 jar, 비어 있는 포트에 따로 띄움)에 `--sim` 한 사이클·
  SOS 노드·오프라인→온라인·diag 쓰기와 400 로그·계약 보안 규칙(역할별 읽기: `auth_variable_override`)을 시험하고 남은
  문서를 계약 스키마로 검사한다. jar가 없으면 skip
  (`firebase setup:emulators:database`로 받는다). 대시보드 에이전트가 쓰는 9000 포트 에뮬레이터와 섞이지 않는다.
- `test_node.py`: 격리 도메인(rclpy)에서 ROS 토픽 → 에이전트 → RTDB, diag 400이 `/rosout`에 ERROR로 뜨는지.
- 로그 검사는 `caplog` 대신 `conftest.agent_log`를 쓴다. ROS 셸의 pytest(launch_testing 플러그인)에서는 `caplog`가 아무것도 받지 못한다.
