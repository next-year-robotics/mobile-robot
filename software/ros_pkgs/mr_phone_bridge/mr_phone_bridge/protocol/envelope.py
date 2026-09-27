"""phone-bridge/1 envelope 파싱·검증·생성(PROTOCOL.md 8.1).

검증은 계약의 JSON Schema(`schema/messages.schema.json`)를 그대로 쓴다. 스키마의 루트 `oneOf`는
오류 메시지가 뭉개지므로 쓰지 않고, envelope를 먼저 검사한 뒤 `$defs/<type>` 하나로 검사한다.
"""
from __future__ import annotations

import json
import re
import time
import uuid
from dataclasses import dataclass
from functools import lru_cache
from typing import Any

from jsonschema import Draft202012Validator
from jsonschema.exceptions import best_match

from . import contracts_path
from .errors import INVALID_MESSAGE, UNSUPPORTED_TYPE, UNSUPPORTED_VERSION, ProtocolError

PROTOCOL_VERSION = 1
MAX_FRAME_BYTES = 16 * 1024

# 앱이 보내는 type. `error`는 양방향이다.
CLIENT_TYPES = frozenset({
    'hello', 'heartbeat', 'trip.start', 'trip.cancel', 'place.upsert', 'place.delete',
    'location.update', 'sos.ack', 'error',
})
# 서버가 보내는 type.
SERVER_TYPES = frozenset({
    'hello.ack', 'heartbeat.ack', 'ack', 'error', 'place.catalog', 'robot.status',
    'sos.request', 'sos.clear',
})

UUID_RE = re.compile(r'^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$')


@dataclass(frozen=True)
class Message:
    """검증을 통과한 수신 메시지."""
    type: str
    id: str
    sent_at_ms: int
    reply_to: str | None
    payload: dict


def new_id() -> str:
    """canonical lowercase UUID v4."""
    return str(uuid.uuid4())


def now_ms() -> int:
    return time.time_ns() // 1_000_000


def is_uuid(value: Any) -> bool:
    return isinstance(value, str) and UUID_RE.match(value) is not None


class Schemas:
    """계약 스키마의 `$defs/<name>` 단위 검증기 모음."""

    def __init__(self, schema: dict):
        self._schema = schema
        self._validators: dict[str, Draft202012Validator] = {}

    def has(self, name: str) -> bool:
        return name in self._schema['$defs']

    def enum(self, name: str) -> tuple[str, ...]:
        return tuple(self._schema['$defs'][name]['enum'])

    def validator(self, name: str) -> Draft202012Validator:
        v = self._validators.get(name)
        if v is None:
            sub = {
                '$schema': self._schema['$schema'],
                '$defs': self._schema['$defs'],
                '$ref': f'#/$defs/{name}',
            }
            v = Draft202012Validator(sub)
            self._validators[name] = v
        return v

    def first_error(self, name: str, obj: Any) -> str | None:
        """오류가 없으면 None, 있으면 `경로: 설명` 한 줄."""
        err = best_match(self.validator(name).iter_errors(obj))
        if err is None:
            return None
        path = '/'.join(str(p) for p in err.absolute_path) or '<root>'
        return f'{path}: {err.message}'


@lru_cache(maxsize=1)
def schemas() -> Schemas:
    path = contracts_path() / 'schema' / 'messages.schema.json'
    with open(path, encoding='utf-8') as f:
        return Schemas(json.load(f))


def _reject_constant(name: str):
    raise ValueError(f'{name} is not allowed')


def parse_frame(data: str | bytes, max_bytes: int = MAX_FRAME_BYTES) -> Message:
    """앱이 보낸 frame 하나를 검증한다. 실패하면 ProtocolError.

    순서: text frame → 크기 → JSON(NaN/Infinity 거절) → object → version → envelope → type → payload.
    """
    if isinstance(data, (bytes, bytearray, memoryview)):
        raise ProtocolError(INVALID_MESSAGE, 'binary frames are not accepted; send JSON text frames')
    try:
        size = len(data.encode('utf-8'))
    except UnicodeEncodeError:
        raise ProtocolError(INVALID_MESSAGE, 'frame is not valid UTF-8') from None
    if size > max_bytes:
        raise ProtocolError(INVALID_MESSAGE, f'frame exceeds {max_bytes} bytes', close=True)
    try:
        obj = json.loads(data, parse_constant=_reject_constant)
    except (ValueError, RecursionError):
        raise ProtocolError(INVALID_MESSAGE, 'malformed JSON (NaN/Infinity are not allowed)') from None
    if not isinstance(obj, dict):
        raise ProtocolError(INVALID_MESSAGE, 'message must be a JSON object')

    reply_to = obj.get('id') if is_uuid(obj.get('id')) else None
    version = obj.get('version')
    if isinstance(version, int) and not isinstance(version, bool) and version != PROTOCOL_VERSION:
        raise ProtocolError(UNSUPPORTED_VERSION, f'protocol version {version} is not supported; use 1',
                            reply_to=reply_to, close=True)

    s = schemas()
    err = s.first_error('envelope', obj)
    if err:
        raise ProtocolError(INVALID_MESSAGE, err, reply_to=reply_to)
    mtype = obj['type']
    if mtype not in CLIENT_TYPES:
        raise ProtocolError(UNSUPPORTED_TYPE, f'type "{mtype[:40]}" is not accepted by the server',
                            reply_to=reply_to)
    err = s.first_error(mtype, obj)
    if err:
        raise ProtocolError(INVALID_MESSAGE, err, reply_to=reply_to)
    try:
        json.dumps(obj, ensure_ascii=False).encode('utf-8')
    except UnicodeEncodeError:
        raise ProtocolError(INVALID_MESSAGE, 'string contains invalid unicode', reply_to=reply_to) from None
    return Message(type=mtype, id=obj['id'], sent_at_ms=int(obj['sent_at_ms']),
                   reply_to=obj['reply_to'], payload=obj['payload'])


def make(mtype: str, payload: dict, reply_to: str | None = None) -> dict:
    """서버가 보낼 envelope."""
    return {
        'version': PROTOCOL_VERSION,
        'type': mtype,
        'id': new_id(),
        'sent_at_ms': now_ms(),
        'reply_to': reply_to,
        'payload': payload,
    }


def make_error(err: ProtocolError, reply_to: str | None = None) -> dict:
    return make('error', err.payload(), reply_to if reply_to is not None else err.reply_to)


def encode(msg: dict) -> str:
    return json.dumps(msg, ensure_ascii=False, separators=(',', ':'), allow_nan=False)


def check_outgoing(msg: dict) -> str | None:
    """보내는 메시지를 계약 스키마로 검사한다. 문제가 없으면 None."""
    s = schemas()
    mtype = msg.get('type')
    if not isinstance(mtype, str) or mtype not in SERVER_TYPES:
        return f'unknown outgoing type {mtype!r}'
    return s.first_error(mtype, msg)
