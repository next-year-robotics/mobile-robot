# mr_phone_bridge

앱용 WebSocket 게이트웨이(`phone-bridge/1`)와 ROS 없이 도는 mock bridge. 계약 정본은
`software/contracts/phone-bridge/v1/PROTOCOL.md`이고 이 문서는 계약이 정하지 않은 **서버 구현 선택**만 적는다.

```text
mr_phone_bridge/
  protocol/   envelope 파싱·스키마 검증·오류 코드·중복 id 캐시·세션 상태     (rclpy 금지)
  places/     places.yaml 로드·검증·원자적 갱신·백업·revision, upsert/delete 규칙 (rclpy 금지)
  server/     코어(bridge.py), SOS 재시도(sos.py), 로봇 인터페이스(backend.py),
              websockets 전송(ws.py)                                          (rclpy 금지)
  mock/       `python3 -m mr_phone_bridge.mock` — 시나리오 12종, 콘솔 명령       (rclpy 금지)
  ros/        rclpy 노드, mr_care_msgs 변환(enums.py)
tools/devpc_demo/  개발 PC 데모 스택(demo.sh): 가짜 로봇 + 실제 mission·브리지·텔레메트리 + 폰(adb reverse)·대시보드
tools/rpi5_deploy/ RPi5 배포 스크립트(deploy.sh, 기본 dry-run, sudo 단계는 출력만)
```

계약 스키마·fixture는 저장소 경로(`software/contracts`)에서 읽는다. colcon 설치본은 `share/mr_phone_bridge/contracts/`에
사본이 들어간다(RPi5는 `software/contracts`를 `~/ros2_ws/src/contracts`로 같이 rsync해야 빌드된다). `MR_CONTRACTS_DIR`로 덮어쓸 수 있다.

## 개발·시험

```bash
# ROS 없이 (websockets 최신)
cd software/ros_pkgs/mr_phone_bridge
uv run --with pytest --with websockets --with jsonschema --with pyyaml python3 -m pytest test

# RPi5와 같은 라이브러리 버전(apt: websockets 10.4, jsonschema 4.10.3)
uv run --with pytest --with 'websockets==10.4' --with 'jsonschema==4.10.3' --with 'pyyaml==6.0.1' python3 -m pytest test

# ROS를 소싱한 셸에서 uv로 돌릴 때는 ROS의 launch_testing pytest 플러그인을 끈다
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run --with pytest --with websockets --with jsonschema --with pyyaml python3 -m pytest test
```

`colcon test`는 시스템 파이썬을 쓴다. `python3-websockets`가 없으면 websockets가 필요한 시험(서버·mock·노드 통합 38개)은 skip된다. 전부 돌리려면 AGENTS.md 4장의 PYTHONPATH 방법을 쓴다.

RPi5 없이 앱·대시보드를 실제 노드들에 붙여 보려면 `tools/devpc_demo/demo.sh`(같은 폴더 README).

## mock bridge

```bash
python3 -m mr_phone_bridge.mock --scenario default6          # ws://0.0.0.0:8765/bridge/v1
python3 -m mr_phone_bridge.mock --list                        # 시나리오 목록
~/dev/senior-care-android/tools/mock-bridge/run.sh --scenario sos   # 앱 저장소에서
```

첫 줄에 `mock bridge ready: ws://…` 를 찍는다(`--port 0`이면 실제 포트). 실기기 주소와 에뮬레이터 주소(`10.0.2.2`)도 같이 찍는다.

| 시나리오 | 동작 |
|---|---|
| `default6` | 집·시장·마트·노인정·교회·절(절은 `enabled=false`). trip.start → starting → 1 s → active, cancel → cancelling → 1 s → cancelled → 3 s → idle |
| `empty` | 빈 카탈로그. 첫 추가가 기본 출발지가 된다 |
| `conflict` | 세션마다 첫 `place.*`/`trip.start`를 `CATALOG_CONFLICT`로 거절하고 새 revision의 `place.catalog`를 보낸다. 다시 보내면 성공 |
| `limit` | 12곳으로 시작. 추가는 `PLACE_LIMIT` |
| `in-use` | 세션을 열 때 집→시장 주행 중(active). 집·시장 삭제 `PLACE_IN_USE`, 좌표 수정 `INVALID_STATE`, 새 trip.start `INVALID_STATE` |
| `slow-ack` | 모든 명령 응답을 2 s 늦게(`--ack-delay`) |
| `error-ack` | trip.start `NOT_READY`, trip.cancel `INTERNAL_ERROR`(retryable), place.* `INTERNAL_ERROR`. `--error-code`로 고정 가능. location/sos.ack는 정상 |
| `flaky` | 세션 20 s(`--flaky-period`)마다 끊는다: close 1011 → TCP abort(1006) → 무응답(서버가 heartbeat 타임아웃 4001로 닫음) 순환 |
| `sos` | hello 5 s(`--sos-delay`) 뒤 `tracking_lost` SOS. ACK 없으면 2 s × 3 재전송. 재연결하면 같은 SOS 복원. 종료는 콘솔 `clear` |
| `sos-x3` | 같은 sos_id의 `sos.request`를 ACK와 무관하게 2 s 간격 3번 |
| `sos-clear` | SOS 10 s(`--sos-clear-after`) 뒤 `sos.clear(tracking_recovered)`. 앱이 끊겨 있으면 다음 세션에 전달 |
| `garbage` | 3 s마다 하나씩: 깨진 JSON, JSON 배열, 모르는 type, 모르는 enum, 모르는 필드, version 2, binary frame, 필수 필드 누락 status, 모르는 kind가 든 catalog(곧바로 정상 catalog), 모르는 error code, 모르는 요청에 대한 ack |

기타 옵션: `--places FILE`(없으면 시나리오 초기값으로 만든다), `--read-only`(`place_edit_enabled=false`), `--arrive-after S`
(mock에서만 `arrived`를 볼 수 있다. 실제 로봇은 OPEN-N1 전까지 내지 않는다), `--heartbeat-timeout-ms`.

터미널에서 띄우면 콘솔 명령을 받는다(`help`): `sos [reason] [severity]`, `clear`, `mission <state>`, `tracking <state>`,
`bridge <state>`, `battery <n|null>`, `error <문장|null>`, `drop [close|abort|silent]`, `status`, `places`, `quit`.

## 서버 동작 (계약이 정하지 않은 부분)

| 항목 | 동작 |
|---|---|
| 경로 | `/bridge/v1`만. 다른 경로는 연결 직후 1008로 닫는다 |
| hello 대기 | 연결 후 5 s 안에 hello가 없으면 1008 (앱은 3 s 안에 보낸다) |
| handshake 전 명령 | `INVALID_STATE` "send hello first" |
| 같은 연결에서 두 번째 hello | `INVALID_STATE` |
| 동시 앱 | 새 연결의 hello가 성공하면 이전 세션을 4000으로 닫는다. 링크 상태는 끊김 없이 새 세션으로 넘어간다 |
| handshake 순서 | `hello.ack` → `place.catalog` → `robot.status` → (전달 못 한 `sos.clear`) → (활성 `sos.request`) |
| 서버 전용 type을 앱이 보냄 | `UNSUPPORTED_TYPE` |
| 앱의 `error` | 로그만 남기고 응답하지 않는다 |
| 중복 id | 서버 전체에서 10분. **성공(ack)만** 저장해 같은 frame을 다시 보낸다. error는 저장하지 않는다(재시도하면 다시 판정) |
| `CATALOG_CONFLICT` 뒤 | 최신 `place.catalog`를 바로 이어서 보낸다 |
| trip.start 판정 순서 | revision(`CATALOG_CONFLICT`) → 출발=도착(`INVALID_MESSAGE`) → 없는 장소(`UNKNOWN_PLACE`) → 비활성 장소(`INVALID_STATE`) → mission 모름(`NOT_READY`) → 주행 중(`INVALID_STATE`) |
| trip.cancel | trip_id가 현재 trip 또는 방금 발급한 trip이면 전달·ack. mission 상태를 모르면(unknown) 무조건 전달·ack(정지는 거절하지 않는다). 그 외 `INVALID_STATE` |
| 방금 발급한 trip | mission이 반영하기 전(최대 5 s)에도 starting으로 보고 두 번째 start 거절, 그 장소 삭제 금지, 취소 허용 |
| place id | `place_<kind>`, 겹치면 `place_<kind>_2`… |
| revision | ISO-8601 UTC 밀리초(`2026-09-27T12:00:00.123Z`). 직전 값보다 항상 크다(시계가 뒤로 가도) |
| 기본 출발지 | upsert로 `is_default_origin=false`로 바꾸기는 `INVALID_STATE`(다른 곳을 기본으로 지정해야 한다). 마지막 한 곳은 기본 출발지여도 삭제할 수 있다 |
| 주행 중 장소 수정 | 출발지·목적지의 `lat`/`lon`/`radius_m` 변경만 거절. 이름·enabled 등은 허용 |
| `place_edit_enabled=false` | place.* 는 `INVALID_STATE` |
| places.yaml이 깨짐 | 가장 최근의 유효한 `.bak.N`으로 기동하고 오류 로그. 백업도 없으면 기동 실패(파일을 덮어쓰지 않는다) |
| SOS 재시도 | 앱이 끊겨 있어도 시도 횟수를 센다 → 약 8 s 뒤 `SosAck(app_received=false, retries=3)` 1회. 이후 ACK가 오면 `true`로 다시 발행 |
| SOS 위치 | ACK 이후 위치 품질이 좋아질 때마다(none→cache→fresh) `SosAck` 재발행. ACK 전 위치는 기록만 |
| low_confidence | `age_ms > 300000` 또는 `horizontal_accuracy_m > 50` |
| rate limit | 연결당 20 msg/s, burst 40. 넘으면 1008 |
| 닫기 코드 | 1001 서버 종료, 1008 정책(경로·hello 타임아웃·버전·rate limit), 1009 16 KiB 초과, 1011 mock flaky, 4000 새 세션, 4001 heartbeat 타임아웃 |

## ROS 노드 (`ros/`)

```bash
ros2 launch mr_phone_bridge phone_bridge.launch.py [params_file:=…]
```

| 방향 | 토픽 | 타입 | QoS |
|---|---|---|---|
| 발행 | `/phone/trip_command` | `TripCommand` | Reliable, KeepLast 10 |
| 발행 | `/phone/location` | `PhoneLocation` | Reliable, KeepLast 10 |
| 발행 | `/phone/link_status` | `PhoneLinkStatus` | Reliable, TransientLocal, KeepLast 1 (연결 중 5 s마다 재발행) |
| 발행 | `/robot/place_catalog` | `PlaceCatalog` | Reliable, TransientLocal, KeepLast 1 (기동 시 + 변경 시) |
| 발행 | `/safety/sos_ack` | `SosAck` | Reliable, KeepLast 10 |
| 구독 | `/robot/app_status` | `RobotAppStatus` | Reliable, TransientLocal, KeepLast 1 |
| 구독 | `/safety/sos_request`, `/safety/sos_clear` | `SosRequest`, `SosClear` | Reliable, KeepLast 10 |
| 구독 | `/marker_track` | `mr_aruco_msgs/MarkerTrack` | 기본 10. `mr_aruco_msgs`가 설치돼 있을 때만 |

- WebSocket 서버는 별도 스레드의 asyncio 루프에서 돈다. ROS 콜백 → 코어는 `call_soon_threadsafe`, 코어 → ROS는 큐 + 20 ms 타이머.
- `robot.status` 채우기: `/robot/app_status`가 6 s(`app_status_stale_s`) 안에 왔으면 그대로. 아니면 `mission_state=unknown`,
  `tracking_state`는 `/marker_track`(1 s 안, TRACKING/HOLDING→following, LOST→lost) 또는 `unavailable`.
  `bridge_state`는 둘 다 낡았으면 `degraded`, 발행 변환 오류가 있었으면 `error`. `battery_percent`는 app_status의 -1 → null.
- mission 노드가 없으면(`unknown`) trip.start는 `NOT_READY`다. trip.cancel은 그래도 전달한다.
- `/safety/sos_request`의 `sos_id`가 UUID v4가 아니면(CLI 시험 등) 해시로 고정된 v4 모양 id를 앱에 보내고, `SosAck`·`sos_clear`는
  원래 id로 대응한다. CLI 예:
  `ros2 topic pub --once /safety/sos_request mr_care_msgs/msg/SosRequest "{sos_id: test1, reason: 1, severity: 1}"`
- `PhoneLocation` 변환: 값이 없으면 `lat/lon/horizontal_accuracy_m = NaN`, `captured_at_ms = 0`, `age_ms = -1`.
- 이 노드는 `/cmd_vel`을 발행·구독하지 않는다. 앱 heartbeat를 MCU 워치독에 쓰지 않는다.

## RPi5 배포 (M2, 사용자 확인 후)

**스크립트: `tools/rpi5_deploy/deploy.sh`**(기본 dry-run, sudo 단계는 출력만. 같은 폴더 README). 아래는 손으로 할 때의 요지다.

```bash
# 개발 PC에서. contracts도 같이 보내야 setup.py가 스키마를 share/에 넣는다.
rsync -a software/ros_pkgs/{mr_care_msgs,mr_phone_bridge,mr_mission,mr_care_telemetry} software/contracts rpi5:~/ros2_ws/src/
ssh rpi5 'sudo apt install -y python3-websockets'      # jsonschema·yaml은 이미 있다
ssh rpi5 'source /opt/ros/jazzy/setup.bash && cd ~/ros2_ws && colcon build --packages-select mr_care_msgs mr_phone_bridge'
# systemd
ssh rpi5 'sudo cp ~/ros2_ws/src/mr_phone_bridge/config/systemd/mr-phone-bridge.service /etc/systemd/system/ \
          && sudo systemctl daemon-reload && sudo systemctl enable --now mr-phone-bridge'
```

`~/ros2_ws/src/contracts`는 패키지가 아니지만(package.xml 없음) colcon이 무시하므로 괜찮다.

### 방화벽 (ufw)

브리지 포트는 핫스팟 인터페이스에만 연다. 인터페이스 이름은 `ip -br addr`로 확인한다(예: `wlan0`).

```bash
sudo ufw default deny incoming
sudo ufw allow in on wlan0 to any port 8765 proto tcp     # phone-bridge
sudo ufw allow in on wlan0 to any port 5353 proto udp     # mDNS
sudo ufw allow 22/tcp                                     # ssh (필요하면 인터페이스 제한)
sudo ufw enable && sudo ufw status verbose
```

## RPi5 mDNS

`config/avahi/mr-phone-bridge.service`를 `/etc/avahi/services/`에 복사한다. TXT `api=1`, `robot_id=mr-001`, 포트 8765.
`robot_id`·포트를 바꾸면 이 파일과 `config/phone_bridge.yaml`을 같이 고친다.
