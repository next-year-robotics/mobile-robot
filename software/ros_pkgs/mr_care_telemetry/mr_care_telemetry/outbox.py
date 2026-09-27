"""SQLite 오프라인 큐. events·sos 쓰기를 순서대로 보관한다(README 5장).

- 한 줄 = RTDB 요청 하나(method, path, JSON body). id 순서로만 보낸다(sos 생성→갱신→삭제 순서 보존).
- 이벤트의 `ts_ms`와 push id는 만들 때 정해지므로 늦게 보내도 원래 시각·순서가 유지된다.
- `max_ops`를 넘으면 가장 오래된 것부터 버린다(로봇이 며칠 오프라인일 때 디스크를 지킨다).
여러 스레드에서 부른다(ROS 콜백이 넣고 전송 스레드가 뺀다). 연결 하나 + 잠금.
"""
from __future__ import annotations

import json
import logging
import os
import sqlite3
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

log = logging.getLogger(__name__)

SCHEMA = """
CREATE TABLE IF NOT EXISTS ops (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    method TEXT NOT NULL,
    path TEXT NOT NULL,
    body TEXT,
    created_ms INTEGER NOT NULL,
    attempts INTEGER NOT NULL DEFAULT 0
)
"""


@dataclass(frozen=True)
class Op:
    id: int
    method: str
    path: str
    body: Any
    created_ms: int
    attempts: int


class Outbox:

    def __init__(self, path: str | os.PathLike = ':memory:', *, max_ops: int = 5000):
        path = os.fspath(path)
        if path != ':memory:':
            path = os.path.expanduser(path)
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.max_ops = max_ops
        self._lock = threading.Lock()
        self._db = sqlite3.connect(path, check_same_thread=False, isolation_level=None, timeout=5.0)
        if path != ':memory:':
            self._db.execute('PRAGMA journal_mode=WAL')
            self._db.execute('PRAGMA synchronous=NORMAL')
        self._db.execute(SCHEMA)

    def put(self, method: str, path: str, body: Any = None) -> int:
        text = None if body is None else json.dumps(body, ensure_ascii=False, allow_nan=False)
        with self._lock:
            cur = self._db.execute('INSERT INTO ops(method, path, body, created_ms) VALUES (?, ?, ?, ?)',
                                   (method, path, text, time.time_ns() // 1_000_000))
            n = self._db.execute('SELECT COUNT(*) FROM ops').fetchone()[0]
            if n > self.max_ops:
                drop = n - self.max_ops
                self._db.execute('DELETE FROM ops WHERE id IN (SELECT id FROM ops ORDER BY id LIMIT ?)', (drop,))
                log.warning('outbox full: dropped %d oldest op(s)', drop)
            return cur.lastrowid

    def peek(self, limit: int = 20) -> list[Op]:
        with self._lock:
            rows = self._db.execute('SELECT id, method, path, body, created_ms, attempts FROM ops '
                                    'ORDER BY id LIMIT ?', (limit,)).fetchall()
        return [Op(r[0], r[1], r[2], None if r[3] is None else json.loads(r[3]), r[4], r[5]) for r in rows]

    def done(self, op_id: int) -> None:
        with self._lock:
            self._db.execute('DELETE FROM ops WHERE id = ?', (op_id,))

    def failed(self, op_id: int) -> None:
        with self._lock:
            self._db.execute('UPDATE ops SET attempts = attempts + 1 WHERE id = ?', (op_id,))

    def __len__(self) -> int:
        with self._lock:
            return self._db.execute('SELECT COUNT(*) FROM ops').fetchone()[0]

    def close(self) -> None:
        with self._lock:
            self._db.close()
