#!/usr/bin/env bash
# 개발 PC 데모 스택: 가짜 로봇 + mr_mission + mr_phone_bridge + mr_care_telemetry (+ 대시보드, 폰 adb reverse).
# RPi5 없이 앱·대시보드를 실제 브리지·mission·텔레메트리에 붙여 본다. 자세한 것은 같은 폴더 README.md.
#
#   demo.sh build                                   스크래치 워크스페이스에 빌드(처음 한 번, 코드 바꾼 뒤)
#   demo.sh start [--emulator HOST:PORT | --no-telemetry] [--dashboard]
#   demo.sh ctl lost|found|sos [reason]|clear [resolution]
#   demo.sh status | stop
#   demo.sh access list | grant <uid> [dev|guardian] | revoke <uid>
#
# ROS는 격리 도메인(기본 151, localhost 발견)에서만 돈다. 로봇 도메인 0에는 아무것도 나가지 않는다.
set -o pipefail   # -u·-e는 ROS setup 스크립트와 맞지 않는다

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKGS=$(cd "$HERE/../../.." && pwd)                                  # software/ros_pkgs
STATE=${MR_DEMO_DIR:-$HOME/.local/state/mr-devpc-demo}
DOMAIN=${MR_DEMO_DOMAIN:-151}
PORT=${MR_DEMO_PORT:-8765}
SA=${MR_SERVICE_ACCOUNT:-$HOME/.config/mr_care_telemetry/service-account.json}
ARUCO_MSGS=${MR_ARUCO_MSGS:-$HOME/dev/mobile-robot/software/ros_pkgs/mr_aruco_msgs}
DASHBOARD=${MR_DASHBOARD_DIR:-$HOME/dev/senior-care-dashboard/web}
ADB=${ADB:-$(command -v adb || echo "$HOME/Android/Sdk/platform-tools/adb")}
NAMES="bridge telemetry mission fake_robot dashboard"                # 끄는 순서

die() { echo "demo: $*" >&2; exit 1; }

ros_env() {
  [ "$DOMAIN" -ge 100 ] 2>/dev/null || die "MR_DEMO_DOMAIN($DOMAIN)은 100 이상이어야 한다(로봇 도메인과 격리)"
  [ -f "$STATE/ws/install/setup.bash" ] || die "빌드가 없다. 먼저: $0 build"
  source /opt/ros/jazzy/setup.bash
  source "$STATE/ws/install/setup.bash"
  export ROS_DOMAIN_ID=$DOMAIN ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  unset FIREBASE_DATABASE_EMULATOR_HOST
}

alive() { [ -f "$STATE/$1.pgid" ] && pgrep -g "$(cat "$STATE/$1.pgid")" > /dev/null; }

# 새 세션(= 프로세스 그룹)으로 띄우고 PGID를 적는다. 끌 때 그룹 전체에 SIGINT를 보낸다.
launch() {
  local name=$1; shift
  rm -f "$STATE/$name.pgid"
  setsid bash -c 'echo $$ > "$0"; exec "$@"' "$STATE/$name.pgid" "$@" > "$STATE/$name.log" 2>&1 < /dev/null &
  for _ in $(seq 50); do [ -s "$STATE/$name.pgid" ] && return; sleep 0.1; done
  die "$name 기동 실패 (로그: $STATE/$name.log)"
}

phone() { [ -x "$ADB" ] && "$ADB" get-state 2>/dev/null | grep -q '^device$'; }

cmd_build() {
  mkdir -p "$STATE"
  source /opt/ros/jazzy/setup.bash
  local paths=("$PKGS/mr_msgs" "$PKGS/mr_care_msgs" "$PKGS/mr_phone_bridge" "$PKGS/mr_mission" "$PKGS/mr_care_telemetry")
  if [ -f "$ARUCO_MSGS/package.xml" ]; then
    paths+=("$ARUCO_MSGS")          # 읽기만 한다. 산출물은 $STATE/ws 아래
  else
    echo "demo: mr_aruco_msgs 없음($ARUCO_MSGS) → 가짜 마커 없이 tracking_state=unavailable"
  fi
  colcon --log-base "$STATE/ws/log" build --build-base "$STATE/ws/build" --install-base "$STATE/ws/install" \
    --symlink-install --paths "${paths[@]}"
}

cmd_start() {
  local telemetry=real emulator="" dashboard=0
  while [ $# -gt 0 ]; do
    case $1 in
      --emulator) telemetry=emulator; emulator=$2; shift ;;
      --no-telemetry) telemetry=off ;;
      --dashboard) dashboard=1 ;;
      *) die "모르는 옵션: $1" ;;
    esac
    shift
  done
  for n in $NAMES; do alive "$n" && die "이미 실행 중($n). 먼저: $0 stop"; done
  ss -ltn | grep -q ":$PORT " && die "포트 $PORT 사용 중(mock bridge가 떠 있나?)"
  ros_env
  mkdir -p "$STATE"
  cd "$STATE" || die "$STATE"

  [ -f places.yaml ] || python3 -c "
from pathlib import Path
from mr_phone_bridge.mock.scenarios import seed_places, write_seed
write_seed(Path('$STATE/places.yaml'), *seed_places('default6'))" || die "places 시드 실패"
  sed -e "s#bind_host: .*#bind_host: \"127.0.0.1\"#" \
      -e "s#port: 8765#port: $PORT#" -e "s#places_path: .*#places_path: \"$STATE/places.yaml\"#" \
      "$PKGS/mr_phone_bridge/config/phone_bridge.yaml" > bridge.yaml
  sed -e "s#service_account_path: .*#service_account_path: \"$SA\"#" \
      -e "s#emulator_host: .*#emulator_host: \"$emulator\"#" \
      -e "s#queue_db_path: .*#queue_db_path: \"$STATE/queue-$telemetry.db\"#" \
      -e "s#publish_diag: false#publish_diag: true#" \
      "$PKGS/mr_care_telemetry/config/care_telemetry.yaml" > telemetry.yaml

  launch fake_robot python3 "$HERE/fake_robot.py"
  launch mission ros2 launch mr_mission mission.launch.py
  if [ "$telemetry" != off ]; then
    [ "$telemetry" = real ] && [ ! -f "$SA" ] && echo "demo: 서비스 계정 없음($SA) → 텔레메트리는 큐에만 쌓는다"
    launch telemetry ros2 launch mr_care_telemetry care_telemetry.launch.py params_file:="$STATE/telemetry.yaml"
  fi
  launch bridge ros2 launch mr_phone_bridge phone_bridge.launch.py params_file:="$STATE/bridge.yaml"
  for _ in $(seq 150); do ss -ltn | grep -q "127.0.0.1:$PORT " && break; sleep 0.1; done
  ss -ltn | grep -q "127.0.0.1:$PORT " || die "브리지가 뜨지 않았다(로그: $STATE/bridge.log)"

  if [ $dashboard = 1 ]; then
    [ -d "$DASHBOARD/node_modules" ] || die "대시보드 없음 또는 npm ci 전: $DASHBOARD"
    local emu=0; [ "$telemetry" = emulator ] && emu=1
    launch dashboard bash -c "cd '$DASHBOARD' && VITE_USE_EMULATORS=$emu exec npm run dev"
  fi
  if phone; then "$ADB" reverse "tcp:$PORT" "tcp:$PORT" > /dev/null && echo "폰: adb reverse tcp:$PORT 설정"; fi

  echo "데모 스택 실행 중 (ROS_DOMAIN_ID=$DOMAIN, 상태·로그: $STATE)"
  echo "  브리지     ws://127.0.0.1:$PORT/bridge/v1  (앱: 설정 길게 누르기 → 로봇 주소 127.0.0.1, 포트 $PORT)"
  case $telemetry in
    real) echo "  텔레메트리 실제 Firebase robots/mr-001" ;;
    emulator) echo "  텔레메트리 에뮬레이터 $emulator" ;;
    off) echo "  텔레메트리 끔" ;;
  esac
  [ $dashboard = 1 ] && echo "  대시보드   http://localhost:5173  (127.0.0.1이 아니라 localhost: Firebase 허용 도메인)"
  echo "  조작       $0 ctl sos | clear | lost | found      끄기: $0 stop"
}

cmd_stop() {
  local any=0
  for n in $NAMES; do
    alive "$n" || continue
    any=1; kill -INT -- "-$(cat "$STATE/$n.pgid")" 2>/dev/null && echo "stop $n"
    [ "$n" = bridge ] && sleep 1        # 텔레메트리가 link.disconnected를 남기게
  done
  for _ in $(seq 80); do
    local left=0; for n in $NAMES; do alive "$n" && left=1; done
    [ $left = 0 ] && break; sleep 0.1
  done
  for n in $NAMES; do
    alive "$n" && { echo "force $n"; kill -TERM -- "-$(cat "$STATE/$n.pgid")"; }
    rm -f "$STATE/$n.pgid"
  done
  if phone; then "$ADB" reverse --remove "tcp:$PORT" 2>/dev/null && echo "폰: adb reverse 해제"; fi
  [ $any = 0 ] && echo "실행 중인 것 없음"
  return 0
}

cmd_status() {
  for n in $NAMES; do
    if alive "$n"; then echo "$n: 실행 중 (pgid $(cat "$STATE/$n.pgid"), 로그 $STATE/$n.log)"; else echo "$n: -"; fi
  done
  ss -ltn | grep -E ":($PORT|5173) " || true
  phone && "$ADB" reverse --list
  return 0
}

cmd_ctl() {
  [ $# -ge 1 ] || die "사용: $0 ctl lost|found|sos [tracking_lost|stuck|low_battery|manual]|clear [tracking_recovered|handled|cancelled]"
  alive fake_robot || die "데모가 실행 중이 아니다"
  ros_env
  timeout 15 ros2 topic pub --once -w 1 /demo/cmd std_msgs/msg/String "{data: '$*'}" > /dev/null \
    && echo "sent: $*" || die "전송 실패"
}

case ${1:-} in
  build) shift; cmd_build "$@" ;;
  start) shift; cmd_start "$@" ;;
  stop) cmd_stop ;;
  status) cmd_status ;;
  ctl) shift; cmd_ctl "$@" ;;
  access) shift; python3 "$HERE/access.py" "$@" ;;
  *) sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
