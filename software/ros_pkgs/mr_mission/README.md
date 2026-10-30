# mr_mission

시니어케어 **최소** mission 노드(TASKS.md M1-C). `/phone/trip_command`를 받아 follow 노드(`mr_aruco`)를
켜고 끄며 `/robot/app_status`를 발행한다. 브리지(`mr_phone_bridge`)가 이 상태를 앱의 `robot.status`로 옮긴다.

> **이 노드는 follow를 켜서 로봇을 움직인다.** 실기 시험은 사용자가 지켜볼 때만 한다(AGENTS.md 5장).

```text
idle ─start→ starting ─/follow/enable(true) 성공→ active
                 └ 3 s 안에 실패·무응답 → error ("추종을 시작하지 못했어요") + follow 끄기
starting/active/error ─cancel→ cancelling ─/follow/enable(false) 성공 + 정지 확인→ cancelled ─3 s→ idle
                                   └ 5 s 안에 확인 못 함 → error ("정지를 확인하지 못했어요")
```

- `arrived`는 **내지 않는다**. 도착 판정 수단이 미결이다(OPEN-N1). 도착 후에는 사용자가 앱에서 `멈춤`을 누른다.
- 정지는 거절하지 않는다. idle에서 cancel이 와도 follow disable을 보낸다. trip_id가 달라도 현재 trip을 멈춘다.
- 정지 확인: `/odom`의 |v| < 0.02 m/s, |w| < 0.05 rad/s가 0.3 s 유지. `/odom`이 없으면 disable 성공 1 s 뒤 시간 기준으로 본다
  (follow 노드는 비활성 전이 때 0을 한 번 보내고 MCU 워치독 200 ms가 backstop).
- disable 호출이 실패하면 0.5 s마다 다시 보낸다. 늦게 도착한 enable 성공(이미 취소됨)에는 즉시 disable을 다시 보낸다.
- `tracking_state`: `/marker_track`(있을 때만) TRACKING/HOLDING → `following`, LOST → starting 중이거나 이번 trip에서
  아직 못 잡았으면 `searching`, 아니면 `lost`, 1 s 넘게 없으면 `unavailable`.
- 기동 시 follow를 한 번 끈다. 주행 중 종료되면 follow disable을 최선 노력으로 보낸다.
- `battery_percent`는 -1(인터페이스 없음).

```bash
ros2 launch mr_mission mission.launch.py
```

## SOS 판단(M3) — 2026-09-27 사용자 결정

규칙과 선택지는 `docs/senior-care/m3-safety-design.md`. 상태 기계는 `mr_mission/safety.py`(rclpy 없음), 배선은 `node.py`.

| 규칙 | 동작 | 파라미터 |
|---|---|---|
| SOS 발생 | trip이 `active`이고 이번 trip에서 마커를 한 번이라도 잡은 뒤, LOST(또는 `/marker_track` 끊김 1 s 이상)가 10 s → `/safety/sos_request`(`tracking_lost`, `critical`, 새 UUID, 현재 trip_id). `searching`·주행 밖에서는 내지 않는다 | `sos_lost_s: 10.0` |
| 해제 | 마커를 5 s 연속 다시 잡으면 `/safety/sos_clear`(`tracking_recovered`). trip 상태와 무관 | `sos_recover_s: 5.0` |
| 앱 조작 해제 | SOS 중 `/phone/trip_command` cancel → `cancelled`, start → `handled`. 그 trip의 놓침은 끝난 것으로 보고 다시 잡은 뒤에만 새 SOS | |
| 재무장 | 해제 즉시. 다음 SOS도 다시 10 s 놓침이 필요 | |
| 폰 무응답(OPEN-R3) | `/safety/sos_ack`에 `app_received=false` → 멈춤과 같은 경로(follow disable + 정지 확인)로 세우고 trip은 `error`("어르신 폰이 응답하지 않아 멈췄어요"). 마커가 다시 보여도 **자동 재개 없음**. 앱에서 멈춤을 누르면 `cancelled`로 끝난다 | `sos_stop_on_no_ack: true` |
| SOS 중 trip | `active` 그대로(tracking_state `lost`). follow_node가 마커를 놓치면 스스로 서고, 다시 보이면 따라간다 | |
| 종료 | 활성 SOS가 있으면 `sos_clear(unknown)`을 내고 끈다(아무도 clear를 못 내서 앱·대시보드에 SOS가 남지 않게) | |
| 끄기 | `sos_enabled: false`면 SOS 토픽을 만들지 않는다(CLI로 SOS를 시험할 때) | `sos_enabled: true` |

`/marker_track`은 `mr_aruco_msgs`가 있을 때만 구독한다. 없으면 SOS도 나지 않는다(마커를 잡은 적이 없으므로).

## 앱 연결 끊김(D6) — 2026-09-27 사용자 결정

설계·재현 기록은 `docs/senior-care/m3-safety-design.md` §5. 상태 기계는 `safety.LinkWatch`.

| 규칙 | 동작 | 파라미터 |
|---|---|---|
| 정지 | trip이 `starting`/`active`인데 `/phone/link_status`가 30 s 연속 끊김 → 멈춤과 같은 경로(follow disable + 정지 확인)로 세우고 trip `error`("폰 연결이 끊겨 멈췄어요"). trip마다 한 번 | `link_lost_stop_s: 30.0`(`0.0`이면 끔) |
| 끊김의 뜻 | 마지막 값이 `connected=false`, 또는 `connected=true`가 12 s 넘게 재발행되지 않음(브리지는 연결 중 5 s마다 재발행. 브리지 프로세스가 죽으면 latched `true`가 남는다), 또는 받은 적이 없음 | `link_stale_s: 12.0` |
| 세는 구간 | trip 안에서만: max(끊긴 때, trip 시작). 30 s 안에 다시 붙으면 아무것도 안 한다(로그만) | |
| 재연결 뒤 | 자동 재개 없음. 앱은 `error`와 `last_error`를 본다. 다시 가려면 새로 출발(앱 멈춤이면 `cancelled`) | |
| 기준 시각 | 브리지가 끊김을 알아챈 때. 정상 닫기·앱 종료는 즉시, 응답 없이 사라지면(핫스팟·폰 꺼짐) heartbeat 10 s 뒤 → 합쳐 약 30 s / 40 s | |

앱 없이 CLI로 출발시키면(`/phone/trip_command` 직접 발행) 링크가 없으므로 30 s 뒤 선다. 그런 시험(체크리스트 M3-4)은
`link_lost_stop_s:=0.0`으로 띄운다.

시험(rclpy 없이 상태 기계만 — `test_machine.py`, `test_safety.py`):

```bash
cd software/ros_pkgs/mr_mission && uv run --with pytest python3 -m pytest test
```

노드 통합 시험(`test/test_node.py`)은 ROS를 소싱한 셸에서 격리 도메인 + 가짜 follow 서비스 + 가짜 `/odom`으로 돈다.
