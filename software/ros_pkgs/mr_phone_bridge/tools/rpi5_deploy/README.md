# rpi5_deploy — RPi5 배포 스크립트 (M2)

RPi5가 돌아오면 시니어케어 패키지 4개(`mr_care_msgs`·`mr_phone_bridge`·`mr_mission`·`mr_care_telemetry`)와
`software/contracts`를 RPi5 `~/ros2_ws`로 보내고 빌드한다. **기본은 dry-run**(명령만 출력, RPi5에 접속하지 않는다).

```bash
T=software/ros_pkgs/mr_phone_bridge/tools/rpi5_deploy
$T/deploy.sh                          # dry-run: 전체 순서와 명령을 본다
$T/deploy.sh --apply                  # sudo가 필요 없는 단계를 실제로 한다
$T/deploy.sh --apply --only sync,build   # 코드만 다시 보낼 때
$T/deploy.sh --apply --seed-places    # 빌드 뒤 places.yaml이 없으면 default6 장소를 넣는다
```

| 단계 | 하는 일 | `--apply`에서 |
|---|---|---|
| preflight | 로컬 패키지 확인, 커밋·미커밋 수 표시, ssh 접속, RPi5 HOME 확인(유닛은 `/home/emjdpi` 고정), 실행 중인 관련 프로세스 표시 | 실행 |
| sync | `rsync -a`(**`--delete` 없음**, `__pycache__`·`mr_phone_bridge/tools/` 제외) → `~/ros2_ws/src/`, 배포 기록 `~/ros2_ws/src/.senior-care-deploy` | 실행 |
| build | `colcon build --packages-select` 4개 (+ `--seed-places`) | 실행. `apt install python3-websockets`는 출력만 |
| units | systemd 유닛 3개·avahi 서비스 파일 설치, 브리지·텔레메트리 enable + restart | **출력만(sudo)**. mr-mission은 설치만 하고 enable·start하지 않는다 |
| ufw | ssh 22 먼저 → 핫스팟 인터페이스(`--iface`, 기본 `wlan0`)에만 8765/tcp·5353/udp → default deny → enable | **출력만(sudo)**. RPi5 인터페이스 목록은 보여 준다 |
| sa | 서비스 계정을 `~/.config/mr_care_telemetry/service-account.json`로(폴더 700, 파일 600, 임시 파일 → mv). 같은 파일이면 건너뜀, 다른 파일이 있으면 `--replace-sa` 없이는 건너뜀 | 실행 |
| check | `~/.bashrc`·`/etc/environment`에 0이 아닌 `ROS_DOMAIN_ID`가 있는지, 설치된 유닛의 도메인, 패키지 설치, `/joint_states` 발행자 수, 서비스 계정 권한, 유닛 상태 | 실행(읽기만) |

- sudo 단계는 끝에 모아 다시 출력한다. 사용자가 확인하고 위에서부터 직접 실행한다(`ssh -t rpi5 'sudo …'`).
- **mr-mission**은 follow를 켜서 로봇을 움직인다. 실주행 시험 때 사용자가 지켜보며 `sudo systemctl start mr-mission`, 끝나면 `stop`.
- ufw `default deny incoming`을 켜면 개발 PC에서 RPi5 ROS(도메인 0)를 보던 DDS도 막힌다. 필요하면 LAN 쪽에 `7400:7700/udp`를 연다(스크립트가 안내만 한다).
- 브리지·텔레메트리 로그: `ssh -t rpi5 'journalctl -u mr-phone-bridge -u mr-care-telemetry -n 50'` — `RTDB online`, `RTDB rejected …`(영구 오류)를 본다.

## 시험 (RPi5 없이)

`MR_DEPLOY_SSH`로 ssh를 바꿀 수 있다. 2026-09-27에 "옵션·호스트를 버리고 가짜 HOME에서 `bash -c`로 실행하는" 가짜 ssh로
`--apply` 전체(rsync 제외 규칙·`--delete` 없음, 실제 colcon 빌드, places 시드, 서비스 계정 700/600·같은 파일 건너뜀·
`--replace-sa`, 도메인 경고)를 확인했다. 서비스 계정은 가짜 파일로 했다.

```bash
cat > /tmp/fakessh <<'EOF'
#!/usr/bin/env bash
while [ $# -gt 0 ]; do case $1 in -o) shift 2 ;; -*) shift ;; *) break ;; esac; done
shift; export HOME=$FAKE_HOME; cd "$HOME" && exec bash -c "$*"
EOF
chmod +x /tmp/fakessh; mkdir -p /tmp/fakehome/ros2_ws/src
FAKE_HOME=/tmp/fakehome MR_DEPLOY_SSH=/tmp/fakessh $T/deploy.sh --apply --sa <가짜 서비스 계정>
```
