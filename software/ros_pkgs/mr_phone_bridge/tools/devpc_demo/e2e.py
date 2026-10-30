#!/usr/bin/env python3
"""데모 스택 e2e: 가짜 앱(fake_app.py)·가짜 로봇(fake_robot.py)으로 주행·M3 SOS·앱 연결 끊김 흐름을 반복 시험한다.

    MR_DEMO_DIR=<스크래치> MR_DEMO_PORT=18865 MR_DEMO_DOMAIN=152 ADB=/nonexistent \\
        python3 e2e.py [--emulator auto|off|HOST:PORT] [시나리오 …]

- 스택을 스스로 띄우고(`demo.sh start --fake-app`) 끝나면 끈다(`demo.sh stop`). 이미 떠 있으면 실패한다.
- `--emulator auto`(기본): firebase-tools가 받아 둔 RTDB 에뮬레이터 jar를 빈 포트에 띄워 텔레메트리 결과(events·sos)까지
  검사하고 끝나면 끈다. `off`: 텔레메트리 없이 앱·mission만. 실제 Firebase에는 쓰지 않는다.
- 포트 8765(브리지 기본값)는 거부한다(가짜 앱이 진짜 폰 세션을 밀어낸다). 도메인은 100 이상(demo.sh가 검사).
- 시나리오(기본: 전부, 순서대로): basic, sos-no-ack, sos-ack, link-drop. 전부 약 4분.
- 빌드는 하지 않는다. 패키지 코드를 바꿨으면 먼저 `demo.sh build`(파이썬만 바꿨으면 symlink-install이라 불필요).
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from fake_app import DEFAULT_BRIDGE_PORT, control  # noqa: E402

DEMO = HERE / 'demo.sh'
STATE = Path(os.environ.get('MR_DEMO_DIR', Path.home() / '.local/state/mr-devpc-demo'))
PORT = int(os.environ.get('MR_DEMO_PORT', DEFAULT_BRIDGE_PORT))
NS = 'senior-care-nyr-default-rtdb'
ROBOT = 'mr-001'
MSG_NO_ACK_STOP = '어르신 폰이 응답하지 않아 멈췄어요'
MSG_LINK_STOP = '폰 연결이 끊겨 멈췄어요'
LINK_STOP_S = 30.0                    # mr_mission link_lost_stop_s(기본값 그대로 시험)
SCENARIOS = ('basic', 'sos-no-ack', 'sos-ack', 'link-drop')


class Fail(Exception):
    pass


class Run:

    def __init__(self, emulator: str | None):
        self.emu = emulator
        self.results: list[tuple[str, str, bool, str]] = []
        self.scenario = ''

    # ------------------------------------------------------------ 도구

    def check(self, name: str, ok: bool, detail: str = '') -> bool:
        self.results.append((self.scenario, name, bool(ok), detail))
        print(f'  {"PASS" if ok else "FAIL"}  {name}{"  — " + detail if detail else ""}', flush=True)
        return bool(ok)

    def require(self, name: str, ok: bool, detail: str = '') -> None:
        if not self.check(name, ok, detail):
            raise Fail(name)

    def app(self, line: str, timeout: float = 30.0) -> dict:
        return control(STATE, line, timeout)

    def wait(self, cond: str, timeout: float) -> dict:
        return self.app(f'wait {cond} {timeout:g}', timeout + 5)

    def robot(self, cmd: str) -> None:
        subprocess.run([str(DEMO), 'ctl', *cmd.split()], check=True, stdout=subprocess.DEVNULL)

    def status(self) -> dict:
        return self.app('status').get('robot_status') or {}

    def follow(self) -> str | None:
        """가짜 로봇 로그의 마지막 follow 상태('on'/'off')."""
        text = (STATE / 'fake_robot.log').read_text(errors='replace')
        hits = re.findall(r'fake follow (on|off)', text)
        return hits[-1] if hits else None

    def history(self, since_seq: int, direction: str, mtype: str) -> list[dict]:
        out = []
        with open(STATE / 'fake_app.jsonl', encoding='utf-8') as f:
            for line in f:
                e = json.loads(line)
                if e['seq'] > since_seq and e['dir'] == direction and e['type'] == mtype:
                    out.append(e)
        return out

    def rtdb(self, path: str):
        if not self.emu:
            return None
        url = f'http://{self.emu}/{path.strip("/")}.json?ns={NS}'
        req = urllib.request.Request(url, headers={'Authorization': 'Bearer owner'})
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read() or b'null')

    def events(self, *, trip_id: str | None = None, sos_id: str | None = None, settle_s: float = 1.5) -> list[dict]:
        """RTDB events 중 trip_id·sos_id가 맞는 것(push id 순서). 에이전트 큐가 비도록 조금 기다린다."""
        time.sleep(settle_s)
        evs = self.rtdb(f'robots/{ROBOT}/events') or {}
        out = [evs[k] for k in sorted(evs)]
        if trip_id:
            out = [e for e in out if e.get('trip_id') == trip_id]
        if sos_id:
            out = [e for e in out if e.get('sos_id') == sos_id]
        return out

    # ------------------------------------------------------------ 공통 단계

    def begin_trip(self) -> str:
        """마커를 보이게 하고, 남은 주행을 멈춘 뒤 집 → 마트 출발. trip_id를 돌려준다."""
        self.robot('found')
        st = self.status()
        if st.get('mission_state') in ('starting', 'active', 'cancelling', 'error'):
            self.app('trip cancel')
            self.wait('mission_state=cancelled', 8)
        res = self.app('trip start 집 마트')
        self.require('trip.start ack', res.get('ok'), str(res.get('payload') or res.get('error')))
        trip_id = res['payload']['trip_id']
        w = self.wait('mission_state=active', 5)
        self.require('mission active', w['ok'], f'{w.get("elapsed_s")} s')
        time.sleep(0.3)
        self.require('follow on', self.follow() == 'on')
        return trip_id

    def end_trip(self) -> None:
        self.app('trip cancel')
        self.wait('mission_state=cancelled', 8)

    # ------------------------------------------------------------ 시나리오

    def basic(self) -> None:
        trip = self.begin_trip()
        t0 = time.monotonic()
        res = self.app('trip cancel')
        self.require('trip.cancel ack', res.get('ok'), str(res.get('error') or ''))
        w = self.wait('mission_state=cancelled', 6)
        self.check('cancelled (정지 확인)', w['ok'], f'{time.monotonic() - t0:.1f} s')
        self.check('follow off', self.follow() == 'off')
        w = self.wait('mission_state=idle', 5)
        self.check('cancelled → idle', w['ok'])
        if self.emu:
            kinds = [e['kind'] for e in self.events(trip_id=trip)]
            self.check('RTDB trip.start → trip.cancelled', kinds == ['trip.start', 'trip.cancelled'], str(kinds))

    def sos_no_ack(self) -> None:
        """M3 D4: 주행 중 LOST 10 s → SOS → 앱 무응답(ACK 없음) → 정지·error → 회복 5 s → 해제, 자동 재개 없음."""
        self.app('ack no')
        try:
            trip = self.begin_trip()
            mark = self.app('mark')['mark']
            t0 = time.monotonic()
            self.robot('lost')
            w = self.wait('sos.request', 16)
            t_sos = time.monotonic() - t0
            self.require('sos.request', w['ok'], f'LOST 뒤 {t_sos:.1f} s')
            self.check('SOS까지 약 10 s', 9.0 <= t_sos <= 12.5, f'{t_sos:.1f} s')
            sos_id = w['match']['payload']['sos_id']
            self.check('sos.request trip_id·reason', w['match']['payload']['trip_id'] == trip
                       and w['match']['payload']['reason'] == 'tracking_lost', str(w['match']['payload']))
            w = self.wait('mission_state=error', 20)
            t_err = time.monotonic() - t0
            self.require('mission error', w['ok'], f'LOST 뒤 {t_err:.1f} s, SOS 뒤 {t_err - t_sos:.1f} s')
            self.check('last_error 문구', w['status']['last_error'] == MSG_NO_ACK_STOP, repr(w['status']['last_error']))
            n_req = len([e for e in self.history(mark, 'in', 'sos.request') if e['payload']['sos_id'] == sos_id])
            self.check('sos.request 4번(첫 전송 + 재시도 3)', n_req == 4, f'{n_req}')
            self.check('follow off', self.follow() == 'off')
            t1 = time.monotonic()
            self.robot('found')
            w = self.wait('sos.clear', 9)
            self.require('sos.clear', w['ok'], f'회복 뒤 {time.monotonic() - t1:.1f} s')
            self.check('해제 사유 tracking_recovered', w['match']['payload']['resolution'] == 'tracking_recovered')
            time.sleep(2.0)
            st = self.status()
            self.check('자동 재개 없음(error 유지, follow off)', st.get('mission_state') == 'error' and self.follow() == 'off',
                       f'{st.get("mission_state")}/{self.follow()}')
            if self.emu:
                evs = self.events(trip_id=trip)
                kinds = [e['kind'] for e in evs]
                self.check('RTDB 이벤트 순서', kinds == ['trip.start', 'sos.request', 'sos.ack', 'trip.error', 'sos.clear'],
                           str(kinds))
                ack = next((e for e in evs if e['kind'] == 'sos.ack'), {})
                self.check('RTDB sos.ack app_received=false retries=3',
                           ack.get('data', {}).get('app_received') is False and ack['data'].get('retries') == 3,
                           str(ack.get('data')))
                self.check('RTDB sos 노드 삭제', self.rtdb(f'robots/{ROBOT}/sos/{sos_id}') is None)
        finally:
            self.app('ack yes')

    def sos_ack(self) -> None:
        """앱 SOS 구현 대응: ACK·위치를 보내면 mission은 멈추지 않는다. RTDB sos.ack·sos 노드 위치, 해제 시 삭제."""
        self.app('ack yes')
        self.app('location cache+fresh')
        self.app('accuracy 18 4')
        trip = self.begin_trip()
        # --- 1) cache(18 m) → fresh(4 m)
        mark = self.app('mark')['mark']
        t0 = time.monotonic()
        self.robot('lost')
        w = self.wait('sos.request', 16)
        self.require('sos.request', w['ok'], f'LOST 뒤 {time.monotonic() - t0:.1f} s')
        req = w['match']
        sos_id = req['payload']['sos_id']
        acks = [e for e in self.history(mark, 'out', 'sos.ack') if e['payload']['sos_id'] == sos_id]
        self.require('앱 sos.ack 보냄', acks, '')
        self.check('sos.ack reply_to = sos.request id', acks[0]['reply_to'] == req['id'])
        lat_ms = acks[0]['wall_ms'] - req['wall_ms']
        self.check('가짜 앱 ACK ≤ 500 ms(앱 쪽 SLA 흉내)', lat_ms <= 500, f'{lat_ms} ms')
        time.sleep(3.0)                      # fresh(2 s 뒤)까지
        locs = [e['payload']['source'] for e in self.history(mark, 'out', 'location.update')
                if e['payload']['sos_id'] == sos_id]
        self.check('위치 cache → fresh', locs == ['cache', 'fresh'], str(locs))
        if self.emu:
            node = self.rtdb(f'robots/{ROBOT}/sos/{sos_id}') or {}
            loc = node.get('location') or {}
            self.check('RTDB sos 노드 app_received=true', node.get('app_received') is True, str(node.get('app_received')))
            self.check('RTDB sos 노드 위치 fresh 4 m', loc.get('source') == 'fresh' and loc.get('accuracy_m') == 4.0
                       and loc.get('low_confidence') is False, str(loc))
            ack = [e for e in self.events(sos_id=sos_id, settle_s=0) if e['kind'] == 'sos.ack']
            data = ack[0]['data'] if ack else {}
            self.check('RTDB sos.ack 이벤트 1건(app_received=true, retries=0, cache·fair)',
                       len(ack) == 1 and data == {'app_received': True, 'retries': 0, 'location': 'cache',
                                                  'accuracy_bucket': 'fair'}, str([e['data'] for e in ack]))
        # 무응답 정지 기한(SOS 뒤 약 8 s)을 넉넉히 넘겨도 멈추지 않아야 한다
        w = self.wait('mission_state!=active', 8)
        n_req = len([e for e in self.history(mark, 'in', 'sos.request') if e['payload']['sos_id'] == sos_id])
        self.check('ACK 뒤 mission 정지 없음(active 유지)', not w['ok'], str(w.get('match') or 'active'))
        self.check('ACK 뒤 재전송 없음', n_req == 1, f'sos.request {n_req}번')
        self.check('follow on 유지', self.follow() == 'on')
        self.check('SOS 중 tracking_state lost', self.status().get('tracking_state') == 'lost')
        t1 = time.monotonic()
        self.robot('found')
        w = self.wait('sos.clear', 9)
        self.require('sos.clear', w['ok'], f'회복 뒤 {time.monotonic() - t1:.1f} s')
        self.check('해제 사유 tracking_recovered', w['match']['payload']['resolution'] == 'tracking_recovered')
        if self.emu:
            time.sleep(1.5)
            self.check('RTDB sos 노드 삭제', self.rtdb(f'robots/{ROBOT}/sos/{sos_id}') is None)
        # --- 2) 같은 trip에서 다시 놓침(즉시 재무장), 이번엔 위치 없음(권한·GPS 꺼짐)
        self.app('location none')
        mark = self.app('mark')['mark']
        self.robot('lost')
        w = self.wait('sos.request', 16)
        self.require('재무장 뒤 두 번째 SOS', w['ok'])
        sos2 = w['match']['payload']['sos_id']
        self.check('새 sos_id', sos2 != sos_id)
        time.sleep(2.0)
        locs = [(e['payload']['source'], e['payload']['failure_reason']) for e in self.history(mark, 'out', 'location.update')
                if e['payload']['sos_id'] == sos2]
        self.check('위치 none(provider_disabled) 1번', locs == [('none', 'provider_disabled')], str(locs))
        if self.emu:
            node = self.rtdb(f'robots/{ROBOT}/sos/{sos2}') or {}
            self.check('RTDB sos 노드 위치 없음', node.get('app_received') is True and 'location' not in node, str(node))
            ack = [e for e in self.events(sos_id=sos2, settle_s=0) if e['kind'] == 'sos.ack']
            self.check('RTDB sos.ack location none·bucket none',
                       len(ack) == 1 and ack[0]['data'].get('location') == 'none'
                       and ack[0]['data'].get('accuracy_bucket') == 'none', str([e['data'] for e in ack]))
        # --- 3) SOS 중 앱 멈춤 → sos.clear(cancelled)
        res = self.app('trip cancel')
        self.check('SOS 중 trip.cancel ack', res.get('ok'))
        w = self.wait('sos.clear', 5)
        self.check('SOS 중 멈춤 → sos.clear(cancelled)', w['ok'] and w['match']['payload']['resolution'] == 'cancelled',
                   str(w.get('match', {}).get('payload')))
        w = self.wait('mission_state=cancelled', 6)
        self.check('cancelled', w['ok'])
        self.robot('found')
        self.app('location cache+fresh')
        if self.emu:
            kinds = [e['kind'] for e in self.events(trip_id=trip)]
            want = ['trip.start', 'sos.request', 'sos.ack', 'sos.clear', 'sos.request', 'sos.ack', 'sos.clear',
                    'trip.cancelled']
            self.check('RTDB 이벤트 순서', kinds == want, str(kinds))
            self.check('RTDB sos 노드 모두 삭제', not self.rtdb(f'robots/{ROBOT}/sos'))

    def link_drop(self) -> None:
        """D6(2026-09-27 결정): 주행 중 앱 링크가 30 s(브리지가 알아챈 때부터) 끊기면 정지·error, 자동 재개 없음.
        30 s 안에 다시 붙으면 계속."""
        # 1) abort 뒤 5 s에 재연결 → 유예 안이므로 계속
        trip = self.begin_trip()
        t0 = time.monotonic()
        self.app('drop abort 5')
        down = self.bridge_link_down(after=time.time() - 0.5, timeout=5.0)
        self.check('[abort] 브리지가 즉시 끊김 감지', down is not None and down - t0 < 1.0,
                   f'{down - t0:.1f} s' if down is not None else 'not detected')
        w = self.wait('connected=true', 10)
        self.check('[abort] 5 s 뒤 재연결', w['ok'])
        time.sleep(max(0.0, LINK_STOP_S + 3.0 - (time.monotonic() - t0)))
        st, fol = self.status(), self.follow()
        self.check(f'[abort] 끊긴 뒤 {LINK_STOP_S + 3:g} s: 유예 안에 돌아와 계속(active·follow on)',
                   st.get('mission_state') == 'active' and st.get('trip_id') == trip and fol == 'on',
                   f'{st.get("mission_state")}/{fol}')
        self.check('[abort] mission 로그: 끊김 경고 → 회복', 'phone link restored during trip ' + trip in
                   (STATE / 'mission.log').read_text(errors='replace'))
        # 2) close 뒤 재연결 없음 → 30 s에 정지·error, 재연결해도 재개 없음
        for mode, detect, lo, hi in (('close', 0.0, 29.5, 32.0), ('silent', 10.0, 38.5, 42.5)):
            if mode != 'close':
                trip = self.begin_trip()
            since_wall = time.time()
            t0 = time.monotonic()
            self.app(f'drop {mode}')
            t_err = self.wait_mission('error', hi + 3.0)
            elapsed = None if t_err is None else t_err - t0
            self.check(f'[{mode}] 끊긴 뒤 약 {detect + LINK_STOP_S:g} s에 mission error',
                       elapsed is not None and lo <= elapsed <= hi, f'{elapsed:.1f} s' if elapsed else 'no error')
            time.sleep(0.5)
            self.check(f'[{mode}] follow off', self.follow() == 'off')
            self.app('connect')
            w = self.wait('mission_state=error', 10)       # 재연결 handshake의 robot.status
            st = w.get('status') or {}
            self.check(f'[{mode}] 재연결 뒤 error·last_error·같은 trip',
                       st.get('mission_state') == 'error' and st.get('last_error') == MSG_LINK_STOP
                       and st.get('trip_id') == trip, f'{st.get("mission_state")} {st.get("last_error")!r}')
            time.sleep(3.0)
            self.check(f'[{mode}] 자동 재개 없음', self.status().get('mission_state') == 'error' and self.follow() == 'off')
            if self.emu:
                evs = [e for e in self.events(settle_s=1.0) if e['ts_ms'] >= since_wall * 1000
                       and (e['kind'].startswith('link.') or e.get('trip_id') == trip)]
                kinds = [e['kind'] for e in evs]
                err = next((e for e in evs if e['kind'] == 'trip.error'), {})
                self.check(f'[{mode}] RTDB link.disconnected → trip.error → link.connected',
                           kinds == ['link.disconnected', 'trip.error', 'link.connected'], str(kinds))
                self.check(f'[{mode}] RTDB trip.error 문구', err.get('message') == MSG_LINK_STOP
                           and err.get('data', {}).get('error') == MSG_LINK_STOP, str(err.get('message')))
        self.end_trip()

    def wait_mission(self, state: str, timeout: float) -> float | None:
        """앱이 끊겨 있어도 보이게 mission 로그로 기다린다. 도달한 monotonic 시각."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.mission_now() == state:
                return time.monotonic()
            time.sleep(0.1)
        return None

    def mission_now(self) -> str | None:
        """앱이 끊겨 있어도 보이게 mission 로그의 마지막 전이로 현재 상태를 읽는다."""
        text = (STATE / 'mission.log').read_text(errors='replace')
        hits = re.findall(r'-> (\w+) \(trip', text)
        return hits[-1] if hits else None

    def bridge_link_down(self, after: float, timeout: float) -> float | None:
        """브리지 로그에서 after(epoch s) 뒤 세션이 닫힌 시각을 찾는다. 돌려주는 값은 monotonic 기준."""
        end = time.monotonic() + timeout
        pat = re.compile(r'\[(\d+\.\d+)\] \[mr_phone_bridge\]: session \S+ closed: ')
        while time.monotonic() < end:
            text = (STATE / 'bridge.log').read_text(errors='replace')
            for m in pat.finditer(text):
                ts = float(m.group(1))
                if ts >= after:
                    return time.monotonic() - (time.time() - ts)
            time.sleep(0.2)
        return None


# ============================================================ 진입점

def free_port() -> int:
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def start_emulator() -> tuple[subprocess.Popen, str]:
    jars = sorted(glob.glob(os.path.expanduser('~/.cache/firebase/emulators/firebase-database-emulator-*.jar')))
    if not jars or shutil.which('java') is None:
        sys.exit('e2e: java 또는 RTDB 에뮬레이터 jar가 없다(firebase setup:emulators:database). --emulator off로 돌린다')
    port = free_port()
    host = f'127.0.0.1:{port}'
    proc = subprocess.Popen(['java', '-jar', jars[-1], '--host', '127.0.0.1', '--port', str(port)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    for _ in range(150):
        try:
            urllib.request.urlopen(f'http://{host}/.json?ns=probe', timeout=1).read()
            return proc, host
        except OSError:
            if proc.poll() is not None:
                sys.exit('e2e: RTDB 에뮬레이터가 바로 끝났다')
            time.sleep(0.2)
    proc.terminate()
    sys.exit('e2e: RTDB 에뮬레이터가 뜨지 않았다')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--emulator', default='auto', help='auto(기본) | off | HOST:PORT(이미 떠 있는 비공개 에뮬레이터)')
    ap.add_argument('scenarios', nargs='*', metavar='시나리오', help=' | '.join(SCENARIOS))
    args = ap.parse_args()
    args.scenarios = args.scenarios or list(SCENARIOS)
    bad = [s for s in args.scenarios if s not in SCENARIOS]
    if bad:
        ap.error(f'모르는 시나리오 {bad}: {", ".join(SCENARIOS)}')
    if PORT == DEFAULT_BRIDGE_PORT:
        sys.exit(f'e2e: MR_DEMO_PORT가 {DEFAULT_BRIDGE_PORT}(기본)이다. 가짜 앱이 진짜 폰 세션을 밀어내므로 다른 포트로(예: 18865)')
    emu_proc, emu = None, None
    if args.emulator == 'auto':
        emu_proc, emu = start_emulator()
    elif args.emulator != 'off':
        emu = args.emulator
    start = [str(DEMO), 'start', '--fake-app'] + (['--emulator', emu] if emu else ['--no-telemetry'])
    run = Run(emu)
    print(f'e2e: 상태 {STATE}, 포트 {PORT}, 텔레메트리 {emu or "끔"}', flush=True)
    started = False
    try:
        if subprocess.run(start).returncode != 0:
            print('e2e: demo.sh start 실패', file=sys.stderr)
            return 2
        started = True
        for name in args.scenarios:
            run.scenario = name
            print(f'\n== {name}', flush=True)
            try:
                getattr(run, name.replace('-', '_'))()
            except Fail as e:
                print(f'  (중단: {e})', flush=True)
            except Exception as e:
                run.check('예외 없음', False, repr(e))
        violations = run.app('status').get('violations')
        run.scenario = 'all'
        run.check('가짜 앱이 받은 메시지 전부 계약 스키마 통과', violations == 0, f'violations={violations}')
    finally:
        if started:
            subprocess.run([str(DEMO), 'stop'])
        if emu_proc is not None:
            emu_proc.terminate()
            try:
                emu_proc.wait(10)
            except subprocess.TimeoutExpired:
                emu_proc.kill()
    failed = [r for r in run.results if not r[2]]
    print(f'\ne2e: {len(run.results) - len(failed)}/{len(run.results)} PASS')
    for sc, name, _, detail in failed:
        print(f'  FAIL [{sc}] {name} {detail}')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
