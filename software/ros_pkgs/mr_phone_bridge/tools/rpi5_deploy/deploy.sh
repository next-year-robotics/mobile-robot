#!/usr/bin/env bash
# RPi5 배포(M2): 시니어케어 패키지 4개와 계약을 RPi5 ~/ros2_ws로 보내고 빌드한다. 자세한 것은 같은 폴더 README.md.
#
#   deploy.sh                    dry-run(기본). 모든 단계의 명령을 출력만 한다. RPi5에 접속하지 않는다
#   deploy.sh --apply            sudo가 필요 없는 단계(rsync·빌드·서비스 계정 복사·점검)를 실제로 한다
#   deploy.sh --apply --only sync,build
#
#   --host HOST        ssh 대상(기본 rpi5)            --iface IF    핫스팟 인터페이스(기본 wlan0, ufw 규칙에 쓴다)
#   --sa PATH          서비스 계정 JSON(기본 ~/.config/mr_care_telemetry/service-account.json)
#   --replace-sa       RPi5에 다른 서비스 계정이 이미 있어도 바꾼다(기본은 건너뜀)
#   --seed-places      (빌드 뒤) RPi5에 places.yaml이 없을 때만 default6 장소를 넣는다
#   --only LIST        preflight,sync,build,units,ufw,sa,check 중 일부(쉼표)
#
# 순서: preflight → sync(rsync, --delete 없음) → build(colcon) → units(systemd·avahi) → ufw → sa(chmod 600) → check(도메인 0)
# sudo가 필요한 단계(apt·systemd·avahi·ufw)는 --apply여도 **명령만 출력한다.** 사용자가 확인하고 직접 실행한다.
# 하지 않는 것: sudo 실행, rsync --delete, mr-mission 기동·enable(follow를 켜서 로봇을 움직이는 노드 — 사용자 입회 때만).
set -o pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../../../../.." && pwd)                          # 저장소 루트
PKGS="mr_care_msgs mr_phone_bridge mr_mission mr_care_telemetry"
STEPS_ALL="preflight sync build units ufw sa check"
HOST=rpi5
IFACE=wlan0
SA=$HOME/.config/mr_care_telemetry/service-account.json
REMOTE_USER_HOME=/home/emjdpi                                    # systemd 유닛 파일에 고정된 경로
SSH=${MR_DEPLOY_SSH:-ssh}                                        # 시험용으로 바꿀 수 있다(README)
APPLY=0 REPLACE_SA=0 SEED=0 ONLY=""

die() { echo "deploy: $*" >&2; exit 1; }
while [ $# -gt 0 ]; do
  case $1 in
    --apply) APPLY=1 ;;
    --host) HOST=$2; shift ;;
    --iface) IFACE=$2; shift ;;
    --sa) SA=$2; shift ;;
    --replace-sa) REPLACE_SA=1 ;;
    --seed-places) SEED=1 ;;
    --only) ONLY=$2; shift ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    *) die "모르는 옵션: $1 (--help)" ;;
  esac
  shift
done
STEPS=${ONLY//,/ }
STEPS=${STEPS:-$STEPS_ALL}
for s in $STEPS; do case " $STEPS_ALL " in *" $s "*) ;; *) die "모르는 단계: $s" ;; esac; done
want() { case " $STEPS " in *" $1 "*) return 0 ;; esac; return 1; }

ROS_ENV='source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0'
WS_ENV="$ROS_ENV && source ~/ros2_ws/install/setup.bash"
NOTES=()
note() { NOTES+=("$*"); }

step() { echo; echo "=== $1"; }
sq() { local s=${1//\'/\'\\\'\'}; printf "'%s'" "$s"; }     # 셸에 그대로 붙여 넣을 수 있게 작은따옴표로(한글 유지)
# 원격 명령(sudo 없음): dry-run이면 출력, --apply면 실행
remote() {
  echo "  + $SSH $HOST $(sq "$1")"
  [ $APPLY = 1 ] || return 0
  "$SSH" "$HOST" "$1"
}
# 로컬 명령(sudo 없음)
local_run() {
  printf '  +'; printf ' %q' "$@"; echo
  [ $APPLY = 1 ] || return 0
  "$@"
}
# sudo 명령: 언제나 출력만 한다. $2는 설명(명령에 넣지 않는다)
SUDO_CMDS=()
sudo_cmd() {                          # 한 명령만. '&&'로 잇지 않는다(뒤쪽은 sudo 없이 돈다)
  case $1 in *'&&'*|*';'*) die "sudo_cmd에는 명령 하나만: $1" ;; esac
  SUDO_CMDS+=("$1")
  echo "  [sudo — 직접 실행] $SSH -t $HOST $(sq "sudo $1")${2:+    # $2}"
}

if [ $APPLY = 1 ]; then
  echo "RPi5 배포 → $HOST (--apply) 단계: $STEPS"
  echo "  sudo가 필요한 단계는 출력만 한다. 끝에 모아 다시 보여 준다."
else
  echo "RPi5 배포 → $HOST (dry-run: 접속도 변경도 하지 않는다) 단계: $STEPS"
fi

# ------------------------------------------------------------------ preflight
if want preflight; then
  step "preflight: 로컬 소스·RPi5 접속"
  for p in $PKGS; do [ -f "$REPO/software/ros_pkgs/$p/package.xml" ] || die "패키지 없음: $p"; done
  [ -d "$REPO/software/contracts" ] || die "software/contracts 없음"
  rev=$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)
  dirty=$(git -C "$REPO" status --porcelain -- software/ros_pkgs software/contracts 2>/dev/null | wc -l)
  echo "  소스 $REPO @ $rev (미커밋 변경 $dirty개 — 그대로 보낸다)"
  if [ $APPLY = 1 ]; then
    "$SSH" -o BatchMode=yes -o ConnectTimeout=5 "$HOST" true 2>/dev/null || die "$HOST 에 ssh 접속 실패"
    rhome=$("$SSH" "$HOST" 'echo $HOME')
    [ "$rhome" = "$REMOTE_USER_HOME" ] || note "RPi5 HOME이 $rhome 이다. systemd 유닛은 $REMOTE_USER_HOME 을 가정한다(유닛 파일 User=·경로 확인)"
    # [x] 꼴: 이 명령을 실행하는 셸 자신의 명령 줄에는 걸리지 않는다
    "$SSH" "$HOST" 'pgrep -fa "ros2 launc[h]|phone_bridg[e]|care_telemetr[y]|mr_missio[n]" || true' | sed 's/^/  실행 중: /'
  else
    echo "  + $SSH -o BatchMode=yes -o ConnectTimeout=5 $HOST true   (접속 확인)"
  fi
fi

# ------------------------------------------------------------------ sync
if want sync; then
  step "sync: rsync (--delete 없음. RPi5에만 있는 파일은 지우지 않는다)"
  srcs=()
  for p in $PKGS; do srcs+=("$REPO/software/ros_pkgs/$p"); done
  srcs+=("$REPO/software/contracts")
  local_run rsync -a -e "$SSH" --exclude __pycache__ --exclude '*.pyc' --exclude .pytest_cache --exclude '*.egg-info' \
    --exclude '/mr_phone_bridge/tools/' "${srcs[@]}" "$HOST:~/ros2_ws/src/" || die "rsync 실패"
  stamp="rev=$(git -C "$REPO" rev-parse HEAD 2>/dev/null) dirty=$(git -C "$REPO" status --porcelain -- software 2>/dev/null | wc -l) at=$(date -Iseconds) from=$(hostname)"
  remote "echo '$stamp' > ~/ros2_ws/src/.senior-care-deploy" || die "배포 기록 실패"
fi

# ------------------------------------------------------------------ build
if want build; then
  step "build: colcon (RPi5에서)"
  sudo_cmd "apt install -y python3-websockets" "처음 한 번. 브리지 실행에 필요(빌드에는 불필요)"
  remote "$ROS_ENV && cd ~/ros2_ws && colcon build --packages-select $PKGS 2>&1 | tail -5" || die "colcon build 실패"
  if [ $SEED = 1 ]; then                 # 빌드된 mr_phone_bridge의 시드 함수를 쓴다. 파일이 있으면 건드리지 않는다
    remote "test -f ~/ros2_ws/config/places.yaml && echo '  places.yaml 있음: 그대로 둔다' || ($WS_ENV && python3 -c 'from pathlib import Path; from mr_phone_bridge.mock.scenarios import seed_places, write_seed; write_seed(Path.home() / \"ros2_ws/config/places.yaml\", *seed_places(\"default6\"))' && echo '  places.yaml default6 시드')" \
      || note "places 시드 실패"
  fi
fi

# ------------------------------------------------------------------ units
if want units; then
  step "units: systemd·avahi (sudo — 출력만)"
  src='~/ros2_ws/src'
  sudo_cmd "install -m 644 $src/mr_phone_bridge/config/systemd/mr-phone-bridge.service $src/mr_care_telemetry/config/systemd/mr-care-telemetry.service $src/mr_mission/config/systemd/mr-mission.service /etc/systemd/system/"
  sudo_cmd "install -m 644 $src/mr_phone_bridge/config/avahi/mr-phone-bridge.service /etc/avahi/services/"
  sudo_cmd "systemctl daemon-reload"
  sudo_cmd "systemctl enable mr-phone-bridge mr-care-telemetry"
  sudo_cmd "systemctl restart mr-phone-bridge mr-care-telemetry" "새로 빌드한 코드로"
  echo "  mr-mission은 설치만 한다. enable·start하지 않는다. 실주행 시험 때 사용자가 지켜보며:"
  echo "    $SSH -t $HOST 'sudo systemctl start mr-mission'   (끝나면 stop)"
fi

# ------------------------------------------------------------------ ufw
if want ufw; then
  step "ufw: 브리지 포트는 핫스팟 인터페이스($IFACE)에만 (sudo — 출력만)"
  if [ $APPLY = 1 ]; then
    "$SSH" "$HOST" 'ip -br addr' | sed 's/^/  인터페이스: /'
    "$SSH" "$HOST" "ip -br addr show $IFACE" > /dev/null 2>&1 || note "RPi5에 인터페이스 $IFACE 가 없다. --iface로 핫스팟 쪽 이름을 준다"
  fi
  echo "  ssh를 먼저 허용한다(순서가 바뀌면 ssh가 끊긴다)."
  sudo_cmd "ufw allow 22/tcp"
  sudo_cmd "ufw allow in on $IFACE to any port 8765 proto tcp" "phone-bridge"
  sudo_cmd "ufw allow in on $IFACE to any port 5353 proto udp" "mDNS"
  echo "  개발 PC에서 ros2 CLI·rviz로 RPi5(도메인 0)를 볼 거면 DDS도 연다(안 쓰면 생략):"
  echo "    $SSH -t $HOST 'sudo ufw allow in on <LAN 인터페이스> proto udp to any port 7400:7700'"
  sudo_cmd "ufw default deny incoming"
  sudo_cmd "ufw enable" "ssh 끊김 경고에 y"
  sudo_cmd "ufw status verbose"
fi

# ------------------------------------------------------------------ sa
if want sa; then
  step "sa: 서비스 계정 → RPi5 ~/.config/mr_care_telemetry/service-account.json (600, 폴더 700)"
  if [ ! -f "$SA" ]; then
    note "서비스 계정 파일 없음: $SA (텔레메트리는 파일이 생길 때까지 큐에 쌓는다)"
    echo "  건너뜀: $SA 없음"
  else
    perm=$(stat -c %a "$SA")
    [ "$perm" = 600 ] || note "로컬 서비스 계정 권한이 $perm 이다(600 권장): chmod 600 $SA"
    lsum=$(sha256sum < "$SA" | cut -c1-64)
    dest='~/.config/mr_care_telemetry/service-account.json'
    echo "  + $SSH $HOST $(sq "umask 077 && mkdir -p ~/.config/mr_care_telemetry && chmod 700 ~/.config/mr_care_telemetry && cat > $dest.tmp && chmod 600 $dest.tmp && mv -f $dest.tmp $dest") < $SA"
    if [ $APPLY = 1 ]; then
      rsum=$("$SSH" "$HOST" "test -f $dest && sha256sum < $dest | cut -c1-64")
      if [ "$rsum" = "$lsum" ]; then
        echo "  같은 파일이 이미 있다: 건너뜀"
        "$SSH" "$HOST" "chmod 700 ~/.config/mr_care_telemetry && chmod 600 $dest"
      elif [ -n "$rsum" ] && [ $REPLACE_SA = 0 ]; then
        note "RPi5에 다른 서비스 계정이 있다. 바꾸려면 --replace-sa"
        echo "  다른 파일이 이미 있다: 건너뜀(--replace-sa)"
      else
        "$SSH" "$HOST" "umask 077 && mkdir -p ~/.config/mr_care_telemetry && chmod 700 ~/.config/mr_care_telemetry && cat > $dest.tmp && chmod 600 $dest.tmp && mv -f $dest.tmp $dest" < "$SA" \
          || die "서비스 계정 복사 실패"
        echo "  복사함(sha256 앞 12자 ${lsum:0:12})"
      fi
    fi
  fi
fi

# ------------------------------------------------------------------ check
if want check; then
  step "check: 도메인 0·설치·권한 (읽기만)"
  remote "grep -n 'ROS_DOMAIN_ID' ~/.bashrc /etc/environment 2>/dev/null | grep -v 'ROS_DOMAIN_ID=0\\b' && echo '  !! 위 줄: ROS_DOMAIN_ID가 0이 아니다(유닛은 0으로 고정하지만 수동 셸이 갈린다)' || echo '  bashrc: 다른 ROS_DOMAIN_ID 없음'"
  remote "grep -h 'ROS_DOMAIN_ID' /etc/systemd/system/mr-*.service 2>/dev/null | sort -u || echo '  유닛 미설치'"
  remote "$WS_ENV && for p in $PKGS; do ros2 pkg prefix \$p > /dev/null && echo \"  설치됨 \$p\" || echo \"  !! 없음 \$p\"; done"
  remote "$WS_ENV && timeout 10 ros2 topic info /joint_states -v 2>/dev/null | grep -m1 'Publisher count' || echo '  /joint_states 없음(베이스 스택이 꺼져 있으면 정상)'"
  remote "stat -c '  %a %n' ~/.config/mr_care_telemetry ~/.config/mr_care_telemetry/service-account.json 2>/dev/null || echo '  서비스 계정 없음'"
  remote "systemctl is-active mr-phone-bridge mr-care-telemetry mr-mission 2>/dev/null | paste -sd' ' | sed 's/^/  bridge telemetry mission: /'"
  echo "  로그: $SSH -t $HOST 'journalctl -u mr-phone-bridge -u mr-care-telemetry -n 50'  ('RTDB online'·'RTDB rejected' 확인)"
fi

# ------------------------------------------------------------------ 요약
echo
if [ ${#SUDO_CMDS[@]} -gt 0 ]; then
  echo "=== sudo 단계(직접 실행 — 확인 후 위에서부터 차례로)"
  for c in "${SUDO_CMDS[@]}"; do echo "  $SSH -t $HOST $(sq "sudo $c")"; done
fi
for n in "${NOTES[@]}"; do echo "!! $n"; done
[ $APPLY = 1 ] || echo "(dry-run: 아무것도 바꾸지 않았다. 실제로 하려면 --apply)"
exit 0
