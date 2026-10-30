# 개발 PC 데모 스택 (`demo.sh`)

RPi5 없이 **실제** 브리지·mission·텔레메트리를 개발 PC에 띄우고, 폰 앱(USB)과 대시보드를 붙여 보는 도구.
mock bridge(`python3 -m mr_phone_bridge.mock`)는 브리지 혼자 흉내 내지만, 이것은 ROS 노드들이 실제로 돌고
로봇 쪽만 가짜다. 앱 ↔ 브리지 ↔ mission ↔ 텔레메트리 ↔ Firebase ↔ 대시보드 전체 경로를 본다.

```text
S26U 앱 ──USB(adb reverse 8765)──▶ mr_phone_bridge(127.0.0.1:8765) ─┐
                                                                     ├─ ROS 격리 도메인 151 (localhost 발견)
fake_robot.py(/follow/enable, /marker_track, /odom) ◀─ mr_mission ──┤
                                                                     └─ mr_care_telemetry ──▶ Firebase RTDB ──▶ 대시보드(localhost:5173)
```

## 안전

- ROS는 `ROS_DOMAIN_ID=151`(100 미만 거부) + `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`로만 돈다. 같은 LAN의 로봇(도메인 0)에
  명령이 나가지 않는다. **RPi5에서 돌리지 않는다**(가짜 `/follow/enable`·`/odom`이 실제 노드와 겹친다).
- 브리지는 `127.0.0.1`에만 연다. 폰은 `adb reverse`로만 붙는다.
- 기본 텔레메트리 대상은 **실제 Firebase**(`robots/mr-001`)다. 앱만 시험할 때는 `--no-telemetry`, 쓰레기 데이터를 피하려면
  `--emulator HOST:PORT`. 큐 파일은 대상별(`queue-real.db`, `queue-emulator.db`)이라 서로 섞이지 않는다.

## 쓰는 법

```bash
T=~/dev/mobile-robot-care/software/ros_pkgs/mr_phone_bridge/tools/devpc_demo
$T/demo.sh build                       # 처음 한 번, 그리고 패키지 코드를 바꾼 뒤(symlink-install이라 파이썬만 바꾸면 불필요)
$T/demo.sh start --dashboard           # 실제 Firebase + 대시보드. 폰이 adb에 잡혀 있으면 adb reverse까지
$T/demo.sh ctl sos                     # SOS(기본 tracking_lost). 이유: tracking_lost|stuck|low_battery|manual
$T/demo.sh ctl clear                   # 마지막 SOS 해제(기본 tracking_recovered). handled|cancelled
$T/demo.sh ctl lost / found            # 가짜 마커 상실·회복 → tracking_state lost/following. 주행 중 lost 10 s → mission이 SOS(M3)
$T/demo.sh status
$T/demo.sh stop                        # 브리지 → 텔레메트리 → mission → 가짜 로봇 → 대시보드 순, adb reverse 해제
$T/demo.sh access list                 # 실제 Firebase Auth 사용자와 /access/mr-001 역할
$T/demo.sh access grant <uid> dev      # 대시보드 열람 권한(dev면 진단 탭). revoke <uid>
```

- 앱: debug 빌드를 설치하고 홈 상단 설정을 **길게 눌러** 로봇 주소 `127.0.0.1`, 포트 `8765` → 로봇 연결 시작.
  release 빌드는 평문 허용 목록 때문에 `127.0.0.1`에 붙지 못한다(앱의 `app/src/debug/res/xml/network_security_config.xml`).
- 대시보드: **`http://localhost:5173`**으로 연다. Firebase Auth 허용 도메인이 `localhost`라서 `127.0.0.1`이면 Google 로그인이 막힌다.
  처음 로그인한 계정은 "접근 권한이 없어요"가 뜬다 → `access list`로 uid 확인 → `access grant`.
- 가짜 로봇: follow가 켜지면 odom 0.5 m/s, 꺼지면 0 → mission이 정지를 확인한다. 마커는 기본 TRACKING.
  `mr_aruco_msgs`는 `~/dev/mobile-robot`(ArUco 브랜치 체크아웃)에서 **읽기만** 해서 `$MR_DEMO_DIR/ws`에 빌드한다. 없으면 마커 없이 돈다.
- SOS는 두 길이다: `ctl sos`/`clear`(가짜 로봇이 직접 발행, 주행과 무관) 또는 주행 중 `ctl lost`(mission이 10 s 뒤 SOS, `found` 5 s 뒤 해제). 앱이 SOS에 응답하지 않으면 약 8 s 뒤 mission이 멈추고 trip이 `error`가 된다(M3 OPEN-R3 결정).
- 주행 중 앱(폰·가짜 앱) 연결이 30 s 끊기면 mission이 멈추고 trip이 `error`가 된다(D6). 폰으로 데모하다 USB·앱을 끊어도 같다.

## 가짜 앱 (`fake_app.py`, 폰 없이)

phone-bridge/1 클라이언트. 폰 대신 브리지에 붙어 출발·멈춤·SOS 응답·연결 끊김을 흉내 낸다. 받은 메시지는 모두 계약
스키마로 검사하고(위반은 로그 `CONTRACT VIOLATION`), 송수신은 `$MR_DEMO_DIR/fake_app.jsonl`에 남는다.

**기본 포트 8765에서는 쓰지 않는다**(`demo.sh`와 `fake_app.py`가 거부한다). 브리지는 앱 세션을 하나만 두므로 붙는 순간
진짜 폰(다른 사람의 데모)의 세션을 밀어낸다. 폰 없이 돌릴 때는 전부 바꾼다:

```bash
export MR_DEMO_DIR=/tmp/mr-fake-demo MR_DEMO_PORT=18865 MR_DEMO_DOMAIN=152 ADB=/nonexistent
$T/demo.sh build
$T/demo.sh start --fake-app --no-telemetry         # 또는 --emulator 127.0.0.1:<비공개 포트>
$T/demo.sh ctl trip start 집 마트                   # 장소: id(place_home)·짧은 id(home)·이름(집)
$T/demo.sh ctl app wait mission_state=active 5
$T/demo.sh ctl lost                                 # 10 s 뒤 SOS → 가짜 앱이 곧바로 ACK + 위치(cache → 2 s 뒤 fresh)
$T/demo.sh ctl app ack no                           # 다음 SOS는 무응답 → 약 8 s 뒤 mission 정지·error
$T/demo.sh ctl app location none                    # SOS 위치: cache+fresh(기본)|cache|fresh|none|off
$T/demo.sh ctl app drop abort 5                     # 연결 끊기(close|abort|silent), 5 s 뒤 다시 연결
$T/demo.sh ctl trip cancel
$T/demo.sh ctl app status                           # 연결·robot.status·활성 SOS·정책
$T/demo.sh stop
```

`ctl app wait <조건> [초]`는 조건이 맞을 때까지 기다린다(`mission_state=error`, `tracking_state!=lost`, `last_error~폰`,
`connected=false`, 또는 메시지 type `sos.request`·`sos.clear` — type은 `ctl app mark` 뒤에 온 것만). 명령 전체는
`fake_app.py` 머리말.

## 반복 e2e (`e2e.py`)

스택을 스스로 띄우고(가짜 앱 + 비공개 RTDB 에뮬레이터) 시나리오를 돌린 뒤 끈다. 실제 Firebase에는 쓰지 않는다.

```bash
MR_DEMO_DIR=/tmp/mr-fake-demo MR_DEMO_PORT=18865 MR_DEMO_DOMAIN=152 ADB=/nonexistent python3 $T/e2e.py
python3 $T/e2e.py --emulator off sos-no-ack          # 텔레메트리 없이 시나리오 하나만
```

| 시나리오 | 확인하는 것 |
|---|---|
| `basic` | 출발 → active·follow on → 멈춤 → 정지 확인 → cancelled → idle, RTDB `trip.start`·`trip.cancelled` |
| `sos-no-ack` | M3 D4: LOST 10 s → SOS 4번(재시도 3) → 무응답 → follow off·`error`("어르신 폰이 응답하지 않아 멈췄어요") → 회복 5 s → `sos.clear(tracking_recovered)` → 자동 재개 없음. RTDB 이벤트 순서, `sos.ack(app_received=false, retries=3)`, sos 노드 삭제 |
| `sos-ack` | 앱이 ACK·위치를 보내면 멈추지 않음(active·follow on 유지, 재전송 없음), RTDB `sos/{id}`의 `app_received`·위치(fresh 4 m), `sos.ack` 이벤트(cache·fair), 회복 시 노드 삭제 → 즉시 재무장·두 번째 SOS(위치 none → 버킷 none) → SOS 중 멈춤 → `sos.clear(cancelled)` |
| `link-drop` | D6(주행 중 앱 끊김, 기본값 30 s 그대로): abort 뒤 5 s 재연결 → 계속 / close → 약 30 s에 follow off·`error`("폰 연결이 끊겨 멈췄어요"), 재연결 뒤에도 `error`·재개 없음 / silent → 약 40 s(heartbeat 10 s + 30 s). RTDB `link.disconnected → trip.error → link.connected` |

전부 약 3분 30초(2026-09-27, 75개 확인).

## 설정(환경 변수)

| 변수 | 기본값 |
|---|---|
| `MR_DEMO_DIR` | `~/.local/state/mr-devpc-demo` (빌드·로그·places.yaml·큐·PGID) |
| `MR_DEMO_DOMAIN` | `151` (100 이상만) |
| `MR_DEMO_PORT` | `8765` |
| `MR_SERVICE_ACCOUNT` | `~/.config/mr_care_telemetry/service-account.json` |
| `MR_ARUCO_MSGS` | `~/dev/mobile-robot/software/ros_pkgs/mr_aruco_msgs` |
| `MR_DASHBOARD_DIR` | `~/dev/senior-care-dashboard/web` |
| `ADB` | `adb` 또는 `~/Android/Sdk/platform-tools/adb` (`--fake-app`이면 쓰지 않는다) |

places는 처음 시작할 때 `default6`으로 시드되고, 앱에서 고친 내용은 `$MR_DEMO_DIR/places.yaml`에 남는다(지우면 다시 시드).
`--emulator`일 때 대시보드는 에뮬레이터 모드(`VITE_USE_EMULATORS=1`, Auth 9099·RTDB 9000)로 뜬다. 그 경우 대시보드 저장소에서
`firebase emulators:start`를 띄우고 `--emulator 127.0.0.1:9000`을 쓴다.
