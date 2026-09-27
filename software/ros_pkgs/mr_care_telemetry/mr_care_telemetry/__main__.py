"""ROS 없이 도는 텔레메트리 시뮬(`--sim`). 대시보드 개발용.

    # 에뮬레이터(대시보드 저장소의 `firebase emulators:start`가 127.0.0.1:9000)
    python3 -m mr_care_telemetry --sim --emulator 127.0.0.1:9000 [--fast] [--cycles 1]
    FIREBASE_DATABASE_EMULATOR_HOST=127.0.0.1:9000 python3 -m mr_care_telemetry --sim

    # 실제 DB(서비스 계정 필요). 가짜 데이터가 실제 로봇 경로에 들어가므로 --real을 명시해야 한다.
    python3 -m mr_care_telemetry --sim --real --robot-id mr-sim --service-account ~/.config/mr_care_telemetry/service-account.json
"""
from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import tempfile
import threading

from .agent import AgentConfig, TelemetryAgent
from .outbox import Outbox
from .rtdb import RtdbClient
from .sim import Simulator

DEFAULT_DB = 'https://senior-care-nyr-default-rtdb.asia-southeast1.firebasedatabase.app'


def parse_args(argv=None):
    ap = argparse.ArgumentParser(prog='mr_care_telemetry', description='care-telemetry/1 agent (ROS 없는 시뮬)')
    ap.add_argument('--sim', action='store_true', help='가짜 로봇 입력으로 돈다(현재 유일한 ROS 없는 모드)')
    ap.add_argument('--emulator', default=os.environ.get('FIREBASE_DATABASE_EMULATOR_HOST'),
                    help='RTDB 에뮬레이터 host:port (기본: FIREBASE_DATABASE_EMULATOR_HOST)')
    ap.add_argument('--real', action='store_true', help='에뮬레이터가 아닌 실제 DB에 쓴다')
    ap.add_argument('--database-url', default=DEFAULT_DB)
    ap.add_argument('--namespace', default=None, help='에뮬레이터 ns (기본: database-url 첫 라벨)')
    ap.add_argument('--service-account', default='~/.config/mr_care_telemetry/service-account.json')
    ap.add_argument('--robot-id', default='mr-001')
    ap.add_argument('--display-name', default='돌봄 로봇 1호')
    ap.add_argument('--queue', default=None, help='SQLite 큐 경로(기본: 임시 파일)')
    ap.add_argument('--fast', action='store_true', help='시간을 1/5로')
    ap.add_argument('--cycles', type=int, default=None, help='사이클 수(기본 무한)')
    ap.add_argument('--no-sos-location', action='store_true')
    ap.add_argument('--diag', action='store_true')
    ap.add_argument('--log-level', default='INFO', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'])
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=args.log_level, stream=sys.stderr,
                        format='%(asctime)s.%(msecs)03d %(levelname).1s %(name)s: %(message)s', datefmt='%H:%M:%S')
    if not args.sim:
        print('ROS 없이 돌리려면 --sim. 실제 로봇에서는 `ros2 launch mr_care_telemetry care_telemetry.launch.py`',
              file=sys.stderr)
        return 2
    if not args.emulator and not args.real:
        print('대상이 없다: --emulator host:port(또는 FIREBASE_DATABASE_EMULATOR_HOST), 실제 DB면 --real',
              file=sys.stderr)
        return 2
    emulator = None if args.real else args.emulator
    if emulator:
        client = RtdbClient(args.database_url, emulator_host=emulator, namespace=args.namespace)
    else:
        client = RtdbClient.from_settings(args.database_url, args.service_account, emulator_host=None)
    queue_path = args.queue or os.path.join(tempfile.mkdtemp(prefix='mr-telemetry-sim-'), 'queue.db')
    cfg = AgentConfig(robot_id=args.robot_id, display_name=args.display_name, version='sim',
                      publish_sos_location=not args.no_sos_location, publish_diag=args.diag)
    agent = TelemetryAgent(cfg, client, Outbox(queue_path))
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    print(f'telemetry sim → {client.target} robot={args.robot_id} queue={queue_path}', flush=True)
    agent.start()
    try:
        Simulator(agent, speed=5.0 if args.fast else 1.0, stop=stop).run(args.cycles)
    finally:
        agent.stop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
