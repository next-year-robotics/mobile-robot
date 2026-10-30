"""공용 시험 도구: 계약 스키마, 메모리 RTDB, 가짜 시계, RTDB 에뮬레이터(별도 포트)."""
from __future__ import annotations

import glob
import json
import logging
import os
import re
import shutil
import socket
import subprocess
import sys
import time
import urllib.parse
import urllib.request
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from mr_care_telemetry.rtdb import RtdbError  # noqa: E402


def contracts_dir() -> Path:
    env = os.environ.get('MR_CONTRACTS_DIR')
    if env:
        return Path(env) / 'care-telemetry' / 'v1'
    for parent in PKG.parents:
        cand = parent / 'contracts' / 'care-telemetry' / 'v1'
        if cand.is_dir():
            return cand
    raise FileNotFoundError('care-telemetry/v1 contracts not found')


def load_json(path: Path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


class Schemas:
    def __init__(self):
        from jsonschema import Draft202012Validator
        d = contracts_dir() / 'schema'
        self.v = {name: Draft202012Validator(load_json(d / f'{name}.schema.json'))
                  for name in ('status', 'event', 'sos', 'meta', 'diag')}

    def check(self, name: str, doc) -> None:
        errs = [f'{"/".join(map(str, e.absolute_path))}: {e.message}' for e in self.v[name].iter_errors(doc)]
        assert not errs, f'{name} violates contract: {errs}'


@pytest.fixture(scope='session')
def schemas():
    pytest.importorskip('jsonschema')
    return Schemas()


class LogRecords(logging.Handler):
    def __init__(self):
        super().__init__(logging.DEBUG)
        self.records: list[logging.LogRecord] = []

    def emit(self, record):
        self.records.append(record)

    def messages(self, level=logging.DEBUG, contains: str = '') -> list[str]:
        return [r.getMessage() for r in self.records if r.levelno >= level and contains in r.getMessage()]


@pytest.fixture
def agent_log():
    """에이전트 로그 기록. caplog 대신 쓴다: colcon test(ROS 셸)에서는 ROS pytest 플러그인 때문에 caplog가
    아무것도 받지 못한다. 로거에 직접 handler를 단다."""
    lg = logging.getLogger('mr_care_telemetry.agent')
    h, old = LogRecords(), lg.level
    lg.addHandler(h)
    lg.setLevel(logging.DEBUG)
    yield h
    lg.removeHandler(h)
    lg.setLevel(old)


class FakeClock:
    def __init__(self, t: float = 1000.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


# 실제 RTDB의 키 규칙(에뮬레이터 400 문구: "Key value can't be empty or contain $ # [ ] / or .").
# 제품 코드(model.KEY_BAD)와 따로 적는다: 같은 실수가 양쪽에 있으면 시험이 못 잡는다.
RTDB_KEY_FORBIDDEN = re.compile(r'[.$#\[\]/\x00-\x1f\x7f]')
RTDB_KEY_MAX_BYTES = 768
RTDB_MAX_DEPTH = 32


def rtdb_key_violations(value, path=()) -> list[str]:
    """값 안의 객체 키 중 RTDB가 거절할 것. 배열은 RTDB가 "0", "1" … 키로 저장하므로 안으로만 들어간다."""
    bad = []
    if len(path) > RTDB_MAX_DEPTH:
        return [f'{"/".join(map(str, path))}: deeper than {RTDB_MAX_DEPTH}']
    if isinstance(value, dict):
        for k, v in value.items():
            if not isinstance(k, str) or not k or RTDB_KEY_FORBIDDEN.search(k) \
                    or len(k.encode('utf-8')) > RTDB_KEY_MAX_BYTES:
                bad.append(f'{"/".join(map(str, path)) or "<root>"}: {k!r}')
            else:
                bad += rtdb_key_violations(v, path + (k,))
    elif isinstance(value, list):
        for i, v in enumerate(value):
            bad += rtdb_key_violations(v, path + (str(i),))
    return bad


class MemoryRtdb:
    """RtdbClient 대역. RTDB의 PUT/PATCH/DELETE/GET(shallow) 의미를 흉내 낸다. offline이면 일시 오류.
    RTDB처럼 키 규칙을 어긴 쓰기는 아무것도 바꾸지 않고 400(영구 오류)으로 거절한다."""

    target = 'memory'

    def __init__(self):
        self.tree: dict = {}
        self.offline = False
        self.calls: list[tuple] = []
        self.fail_status: dict[str, int] = {}     # path 접두어 → 영구 오류 코드

    def _parts(self, path):
        return [p for p in path.strip('/').split('/') if p]

    def _node(self, parts, create=False):
        node = self.tree
        for p in parts:
            if not isinstance(node, dict) or p not in node:
                if not create:
                    return None
                node[p] = {}
            node = node[p]
        return node

    def _set(self, parts, value):
        if not parts:
            self.tree = value if isinstance(value, dict) else {}
            return
        parent = self._node(parts[:-1], create=True)
        if value is None or value == {}:
            parent.pop(parts[-1], None)
        else:
            parent[parts[-1]] = json.loads(json.dumps(value, allow_nan=False))
        self._prune(self.tree)

    def _prune(self, node):
        if isinstance(node, dict):
            for k in list(node):
                self._prune(node[k])
                if node[k] is None or node[k] == {}:
                    del node[k]

    def _reject_bad_keys(self, method, path, value):
        parts = self._parts(path)
        bad = [f'path segment {p!r}' for p in parts if RTDB_KEY_FORBIDDEN.search(p)]
        if method == 'PUT':
            bad += rtdb_key_violations(value)
        elif method == 'PATCH':
            for k, v in (value or {}).items():         # PATCH 키는 하위 경로('a/b')일 수 있다
                segs = self._parts(k)
                if not segs or any(RTDB_KEY_FORBIDDEN.search(seg) for seg in segs):
                    bad.append(f'patch key {k!r}')
                bad += rtdb_key_violations(v, tuple(segs))
        if bad:
            raise RtdbError(f"{method} {path} -> 400 Invalid data; Key value can't be empty or contain "
                            f"$ # [ ] / or . ({bad[0]})", status=400, transient=False)

    def request(self, method, path, value=None, params=None):
        self.calls.append((method, path, value))
        if self.offline:
            raise RtdbError('offline (fake)')
        for prefix, code in self.fail_status.items():
            if path.startswith(prefix):
                raise RtdbError(f'{method} {path} -> {code}', status=code, transient=False)
        self._reject_bad_keys(method, path, value)
        parts = self._parts(path)
        if method == 'PUT':
            self._set(parts, value)
        elif method == 'PATCH':
            for k, v in value.items():
                self._set(parts + self._parts(k), v)
        elif method == 'DELETE':
            self._set(parts, None)
        elif method == 'GET':
            node = self._node(parts)
            if node is not None and params and params.get('shallow') == 'true' and isinstance(node, dict):
                return {k: True for k in node}
            return json.loads(json.dumps(node)) if node is not None else None
        return None

    def put(self, path, value):
        self.request('PUT', path, value)

    def patch(self, path, value):
        self.request('PATCH', path, value)

    def delete(self, path):
        self.request('DELETE', path)

    def get(self, path, **params):
        return self.request('GET', path, params=params)

    def at(self, path):
        return self._node(self._parts(path))


# ------------------------------------------------------------------ RTDB 에뮬레이터

def free_port() -> int:
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def emulator_jar() -> str | None:
    env = os.environ.get('RTDB_EMULATOR_JAR')
    if env and os.path.isfile(env):
        return env
    jars = sorted(glob.glob(os.path.expanduser('~/.cache/firebase/emulators/firebase-database-emulator-*.jar')))
    return jars[-1] if jars else None


class Emulator:
    """firebase-tools가 받아 둔 RTDB 에뮬레이터 jar를 사용하지 않는 포트에 띄운다.
    대시보드 에이전트가 쓰는 9000 포트 에뮬레이터와 섞이지 않는다."""

    def __init__(self, port: int | None = None):
        self.jar = emulator_jar()
        self.port = port or free_port()
        self.host = f'127.0.0.1:{self.port}'
        self.proc = None

    def start(self, timeout: float = 30.0) -> 'Emulator':
        self.proc = subprocess.Popen(['java', '-jar', self.jar, '--host', '127.0.0.1', '--port', str(self.port)],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                urllib.request.urlopen(f'http://{self.host}/.json?ns=probe', timeout=1).read()
                return self
            except OSError:
                if self.proc.poll() is not None:
                    raise RuntimeError('RTDB emulator exited')
                time.sleep(0.2)
        self.stop()
        raise RuntimeError('RTDB emulator did not start')

    def stop(self) -> None:
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()

    def rest(self, method: str, path: str, value=None, ns='senior-care-nyr-default-rtdb', auth='owner',
             as_uid: str | None = None):
        """as_uid: 관리자 토큰 + auth_variable_override로 그 uid의 로그인 사용자처럼 규칙을 적용받는다.
        (이 에뮬레이터는 서명 없는 JWT나 아무 Bearer 문자열을 관리자로 본다 — 규칙 시험에는 이것만 믿을 수 있다.)"""
        data = None if value is None else json.dumps(value).encode()
        headers = {'Content-Type': 'application/json'}
        if auth:
            headers['Authorization'] = f'Bearer {auth}'
        query = {'ns': ns}
        if as_uid is not None:
            query['auth_variable_override'] = json.dumps({'uid': as_uid})
        req = urllib.request.Request(f'http://{self.host}/{path.strip("/")}.json?{urllib.parse.urlencode(query)}',
                                     data=data, method=method, headers=headers)
        with urllib.request.urlopen(req, timeout=5) as r:
            body = r.read()
        return json.loads(body) if body else None


@pytest.fixture(scope='session')
def emulator():
    if shutil.which('java') is None or emulator_jar() is None:
        pytest.skip('java or firebase RTDB emulator jar not available (firebase setup:emulators:database)')
    emu = Emulator().start()
    yield emu
    emu.stop()
