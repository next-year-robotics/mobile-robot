"""ROS 없이 도는 mock bridge. 실제 브리지와 같은 서버 코어를 쓴다.

    python3 -m mr_phone_bridge.mock --scenario default6 [--port 8765] [--places /tmp/places.yaml]
    python3 -m mr_phone_bridge.mock --list

앱 저장소의 `tools/mock-bridge/run.sh`가 이 명령을 감싼다. 시나리오는 PROTOCOL.md 7.1.
터미널에서 띄우면 콘솔 명령(`help`)으로 SOS·상태를 직접 바꿀 수 있다.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import socket
import sys
import tempfile
import threading
from pathlib import Path

from ..places.store import PlaceStore
from ..protocol.envelope import new_id
from ..protocol.errors import ALL_CODES
from ..server.backend import BRIDGE_STATES, MISSION_STATES, TRACKING_STATES
from ..server.bridge import BridgeConfig, BridgeCore
from .backend import MockBackend
from .scenarios import SCENARIOS, ScenarioHooks, ScenarioOptions, seed_places, write_seed

log = logging.getLogger('mock')

CONSOLE_HELP = """\
명령:
  sos [reason] [severity]   SOS 발행 (reason: tracking_lost|stuck|low_battery|manual, severity: critical|warning)
  clear [resolution]        활성 SOS 모두 종료 (tracking_recovered|handled|cancelled)
  mission <state>           mission_state 강제 (idle|starting|active|arrived|cancelling|cancelled|error|unknown)
  tracking <state>          tracking_state 강제 (unavailable|searching|following|lost|idle|error)
  bridge <state>            bridge_state 강제 (ready|degraded|error)
  battery <0-100|null>      battery_percent
  error <문장|null>         last_error
  drop [close|abort|silent] 앱 연결 끊기
  status                    현재 robot.status payload
  places                    현재 카탈로그
  quit                      종료"""


def lan_addresses() -> list[str]:
    addrs = []
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('192.0.2.1', 9))    # TEST-NET, 패킷은 나가지 않는다
            addrs.append(s.getsockname()[0])
    except OSError:
        pass
    return addrs


def build(args) -> tuple[BridgeCore, MockBackend, Path]:
    if args.places:
        places_path = Path(args.places).expanduser()
        if not places_path.exists():
            write_seed(places_path, *seed_places(args.scenario))
    else:
        tmpdir = Path(tempfile.mkdtemp(prefix='mr-mock-bridge-'))
        places_path = tmpdir / 'places.yaml'
        write_seed(places_path, *seed_places(args.scenario))
    store = PlaceStore(places_path, max_places=args.max_places)
    backend = MockBackend(arrive_after_s=args.arrive_after)
    opts = ScenarioOptions(ack_delay_s=args.ack_delay, error_code=args.error_code,
                           flaky_period_s=args.flaky_period, sos_delay_s=args.sos_delay,
                           sos_clear_after_s=args.sos_clear_after, garbage_period_s=args.garbage_period)
    cfg = BridgeConfig(robot_id=args.robot_id, heartbeat_timeout_ms=args.heartbeat_timeout_ms,
                       place_edit_enabled=not args.read_only)
    core = BridgeCore(cfg, store, backend, ScenarioHooks(args.scenario, opts, backend))
    return core, backend, places_path


class Console:
    """stdin 한 줄 = 명령 하나. 입력은 스레드에서 읽고 처리는 루프에서 한다."""

    def __init__(self, core: BridgeCore, backend: MockBackend, stop: asyncio.Event):
        self.core = core
        self.backend = backend
        self.stop = stop

    def start(self, loop: asyncio.AbstractEventLoop) -> None:
        def reader():
            for line in sys.stdin:
                loop.call_soon_threadsafe(lambda ln=line: asyncio.ensure_future(self.run(ln)))
            loop.call_soon_threadsafe(lambda: None)
        threading.Thread(target=reader, daemon=True, name='mock-console').start()

    async def run(self, line: str) -> None:
        parts = line.split()
        if not parts:
            return
        cmd, args = parts[0].lower(), parts[1:]
        b, core = self.backend, self.core
        try:
            if cmd in ('help', '?'):
                print(CONSOLE_HELP, flush=True)
            elif cmd == 'sos':
                reason = args[0] if args else 'tracking_lost'
                severity = args[1] if len(args) > 1 else 'critical'
                b.set(tracking_state='lost')
                await core.request_sos(new_id(), reason, severity, b.state.trip_id)
            elif cmd == 'clear':
                for sos_id in list(core.sos.active):
                    await core.clear_sos(sos_id, args[0] if args else 'tracking_recovered')
                b.set(tracking_state='following' if b.state.mission_state in ('starting', 'active')
                      else 'unavailable')
            elif cmd == 'mission' and args and args[0] in MISSION_STATES:
                b.set(mission_state=args[0])
            elif cmd == 'tracking' and args and args[0] in TRACKING_STATES:
                b.set(tracking_state=args[0])
            elif cmd == 'bridge' and args and args[0] in BRIDGE_STATES:
                b.set(bridge_state=args[0])
            elif cmd == 'battery' and args:
                b.set(battery_percent=None if args[0] == 'null' else float(args[0]))
            elif cmd == 'error' and args:
                b.set(last_error=None if args == ['null'] else ' '.join(args))
            elif cmd == 'drop':
                peer = core.active
                mode = args[0] if args else 'close'
                if peer is None:
                    print('연결된 앱이 없다', flush=True)
                elif mode == 'abort':
                    core.abort_peer(peer)
                elif mode == 'silent':
                    peer.silent = True
                else:
                    core.close_peer(peer, 1011, 'mock console drop')
            elif cmd == 'status':
                print(core.status_payload(), flush=True)
            elif cmd == 'places':
                cat = core.store.catalog
                print(f'revision {cat.revision}', flush=True)
                for p in cat.sorted_places():
                    flags = ('기본 ' if p['is_default_origin'] else '') + ('' if p['enabled'] else '비활성')
                    print(f"  {p['id']:<24} {p['label']:<8} {p['kind']:<14} {flags}", flush=True)
            elif cmd in ('quit', 'exit'):
                self.stop.set()
            else:
                print(f'모르는 명령: {line.strip()}  (help)', flush=True)
        except Exception as e:     # 콘솔 오타로 mock이 죽지 않게
            print(f'오류: {e}', flush=True)


async def run(args) -> int:
    from ..server.ws import BridgeServer   # websockets가 없으면 여기서 실패한다
    core, backend, places_path = build(args)
    server = BridgeServer(core, args.host, args.port)
    try:
        await server.start()
    except OSError as e:
        print(f'mock bridge: cannot listen on {args.host}:{args.port}: {e.strerror or e}', file=sys.stderr)
        return 1
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop.set)
        except (NotImplementedError, RuntimeError):
            pass
    url = f'ws://{args.host}:{server.port}{core.cfg.path}'
    print(f'mock bridge ready: {url} scenario={args.scenario} places={places_path}', flush=True)
    print(f'  {SCENARIOS[args.scenario]}', flush=True)
    if args.host in ('0.0.0.0', '::'):
        for ip in lan_addresses():
            print(f'  실기기: ws://{ip}:{server.port}{core.cfg.path}', flush=True)
        print(f'  에뮬레이터: ws://10.0.2.2:{server.port}{core.cfg.path}', flush=True)
    console = args.console if args.console is not None else sys.stdin.isatty()
    if console:
        print("  콘솔 명령은 'help'", flush=True)
        Console(core, backend, stop).start(loop)
    try:
        await stop.wait()
    finally:
        await server.stop()
    return 0


def parse_args(argv=None):
    ap = argparse.ArgumentParser(prog='mock_bridge', description='phone-bridge/1 mock (ROS 없이)')
    ap.add_argument('--scenario', choices=list(SCENARIOS), default='default6')
    ap.add_argument('--list', action='store_true', help='시나리오 목록')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--places', default=None, help='places.yaml 경로. 없으면 시나리오 초기값으로 만든다. '
                                                   '생략하면 임시 파일')
    ap.add_argument('--robot-id', default='mr-001')
    ap.add_argument('--max-places', type=int, default=12)
    ap.add_argument('--read-only', action='store_true', help='place_edit_enabled=false')
    ap.add_argument('--heartbeat-timeout-ms', type=int, default=10000)
    ap.add_argument('--ack-delay', type=float, default=2.0, help='slow-ack 지연 [s]')
    ap.add_argument('--error-code', choices=sorted(ALL_CODES), default=None, help='error-ack 코드 고정')
    ap.add_argument('--flaky-period', type=float, default=20.0, help='flaky 끊김 주기 [s]')
    ap.add_argument('--sos-delay', type=float, default=5.0, help='hello 후 SOS까지 [s]')
    ap.add_argument('--sos-clear-after', type=float, default=10.0, help='sos-clear에서 clear까지 [s]')
    ap.add_argument('--garbage-period', type=float, default=3.0)
    ap.add_argument('--arrive-after', type=float, default=None,
                    help='active 후 arrived로 바꿀 시간 [s]. 실제 로봇은 아직 arrived를 내지 않는다(OPEN-N1)')
    ap.add_argument('--console', dest='console', action='store_true', default=None, help='콘솔 명령 강제 사용')
    ap.add_argument('--no-console', dest='console', action='store_false')
    ap.add_argument('--log-level', default='INFO', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'])
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.list:
        for name, desc in SCENARIOS.items():
            print(f'{name:<10} {desc}')
        return 0
    logging.basicConfig(level=args.log_level, stream=sys.stderr,
                        format='%(asctime)s.%(msecs)03d %(levelname).1s %(name)s: %(message)s',
                        datefmt='%H:%M:%S')
    logging.getLogger('websockets').setLevel(logging.WARNING)
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
