"""places.yaml 로드·검증·원자적 갱신·백업·revision(PROTOCOL.md 4.1, 6 15.3, 7.4).

- 좌표의 단일 원본은 이 파일이고 소유자는 브리지다.
- 갱신: 임시 파일에 쓰고 fsync → 직전 파일을 `.bak.1`로 밀어 두고(최대 5개) → `os.replace`.
  성공한 뒤에만 새 revision이 생긴다. 실패하면 예외를 올리고 기존 파일·메모리 상태를 그대로 둔다.
- revision은 ISO-8601 UTC 밀리초 문자열이다. 같은 초에 두 번 고쳐도 겹치지 않게 직전 값보다 항상 크다.
"""
from __future__ import annotations

import copy
import math
import os
import re
import shutil
import tempfile
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import yaml

from ..protocol.envelope import schemas

FILE_VERSION = 1
DEFAULT_MAX_PLACES = 12
DEFAULT_BACKUPS = 5
PLACE_ID_RE = re.compile(r'^[a-z0-9_-]{1,64}$')
FIELDS = ('kind', 'label', 'lat', 'lon', 'radius_m', 'enabled', 'sort_order', 'is_default_origin', 'address')


class PlaceStoreError(Exception):
    """places.yaml을 읽거나 쓸 수 없다."""


class PlaceValidationError(ValueError):
    """카탈로그 내용이 규칙에 맞지 않는다."""


@dataclass
class Catalog:
    revision: str
    places: dict[str, dict] = field(default_factory=dict)    # id → FIELDS
    max_places: int = DEFAULT_MAX_PLACES

    @property
    def default_origin_place_id(self) -> str | None:
        for pid, p in self.places.items():
            if p['is_default_origin']:
                return pid
        return None

    def sorted_places(self) -> list[dict]:
        items = sorted(self.places.items(), key=lambda kv: (kv[1]['sort_order'], kv[0]))
        return [{'id': pid, **{k: p[k] for k in FIELDS}} for pid, p in items]

    def to_payload(self) -> dict:
        """`place.catalog` payload."""
        return {
            'revision': self.revision,
            'max_places': self.max_places,
            'default_origin_place_id': self.default_origin_place_id,
            'places': self.sorted_places(),
        }


def normalize_place(raw: dict) -> dict:
    """FIELDS만 남기고 address 누락은 None으로 채운다."""
    p = {k: raw.get(k) for k in FIELDS}
    return p


def validate_places(places: dict[str, dict], *, max_places: int | None = None) -> None:
    """좌표 범위·라벨·kind·개수·기본 출발지 유일성. 앱 검증을 믿지 않는다."""
    s = schemas()
    if not isinstance(places, dict):
        raise PlaceValidationError('places must be a mapping of id to place')
    for pid, p in places.items():
        if not isinstance(pid, str) or not PLACE_ID_RE.match(pid):
            raise PlaceValidationError(f'invalid place id {pid!r}')
        if not isinstance(p, dict):
            raise PlaceValidationError(f'{pid}: place must be a mapping')
        err = s.first_error('place', {'id': pid, **p})
        if err:
            raise PlaceValidationError(f'{pid}: {err}')
        for k in ('lat', 'lon', 'radius_m'):
            if not math.isfinite(float(p[k])):
                raise PlaceValidationError(f'{pid}: {k} must be finite')
        if not p['label'].strip():
            raise PlaceValidationError(f'{pid}: label must not be blank')
    if max_places is not None and len(places) > max_places:
        raise PlaceValidationError(f'{len(places)} places exceed max_places {max_places}')
    defaults = [pid for pid, p in places.items() if p['is_default_origin']]
    if places and len(defaults) != 1:
        raise PlaceValidationError(f'exactly one default origin required, found {len(defaults)}')


def revision_to_ms(revision: str) -> int | None:
    try:
        dt = datetime.fromisoformat(revision.replace('Z', '+00:00'))
    except (ValueError, AttributeError):
        return None
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return int(dt.timestamp() * 1000)


def ms_to_revision(ms: int) -> str:
    dt = datetime.fromtimestamp(ms / 1000.0, tz=timezone.utc)
    return dt.strftime('%Y-%m-%dT%H:%M:%S.') + f'{ms % 1000:03d}Z'


def next_revision(prev: str | None, now_ms: int | None = None) -> str:
    now = time.time_ns() // 1_000_000 if now_ms is None else now_ms
    prev_ms = revision_to_ms(prev) if prev else None
    if prev_ms is not None and now <= prev_ms:
        now = prev_ms + 1
    rev = ms_to_revision(now)
    if rev == prev:
        rev = ms_to_revision(now + 1)
    return rev


class PlaceStore:
    """places.yaml 하나를 소유한다. 한 스레드(asyncio 루프)에서만 호출한다."""

    def __init__(self, path: str | os.PathLike, *, max_places: int = DEFAULT_MAX_PLACES,
                 backups: int = DEFAULT_BACKUPS):
        self.path = Path(os.path.expanduser(os.fspath(path)))
        self.max_places = max_places
        self.backups = backups
        self._catalog: Catalog | None = None
        self.load_warning: str | None = None

    @property
    def catalog(self) -> Catalog:
        if self._catalog is None:
            self.load()
        return self._catalog

    def backup_path(self, n: int) -> Path:
        return self.path.with_name(f'{self.path.name}.bak.{n}')

    # ------------------------------------------------------------------ load

    def load(self) -> Catalog:
        """파일을 읽는다. 없으면 빈 카탈로그를 만들어 쓴다.

        파일이 깨졌으면 가장 최근의 유효한 백업을 쓰고 `load_warning`에 남긴다.
        백업도 없으면 PlaceStoreError. 사람이 고친 파일을 덮어쓰지 않으려고 자동 복구는 하지 않는다.
        """
        if not self.path.exists():
            cat = Catalog(revision=next_revision(None), places={}, max_places=self.max_places)
            self._write(cat)
            self._catalog = cat
            return cat
        try:
            self._catalog = self._read(self.path)
            return self._catalog
        except PlaceStoreError as first:
            for n in range(1, self.backups + 1):
                bak = self.backup_path(n)
                if not bak.exists():
                    continue
                try:
                    self._catalog = self._read(bak)
                except PlaceStoreError:
                    continue
                self.load_warning = f'{self.path.name} is invalid ({first}); using {bak.name}'
                return self._catalog
            raise

    def _read(self, path: Path) -> Catalog:
        try:
            with open(path, encoding='utf-8') as f:
                doc = yaml.safe_load(f)
        except (OSError, yaml.YAMLError) as e:
            raise PlaceStoreError(f'cannot read {path.name}: {e}') from None
        if not isinstance(doc, dict):
            raise PlaceStoreError(f'{path.name}: top level must be a mapping')
        if doc.get('version') != FILE_VERSION:
            raise PlaceStoreError(f'{path.name}: version must be {FILE_VERSION}')
        revision = doc.get('revision')
        if not isinstance(revision, str) or not revision:
            raise PlaceStoreError(f'{path.name}: revision must be a non-empty string')
        raw = doc.get('places') or {}
        if not isinstance(raw, dict):
            raise PlaceStoreError(f'{path.name}: places must be a mapping')
        places = {pid: normalize_place(p) if isinstance(p, dict) else p for pid, p in raw.items()}
        try:
            validate_places(places)
        except PlaceValidationError as e:
            raise PlaceStoreError(f'{path.name}: {e}') from None
        return Catalog(revision=revision, places=places, max_places=self.max_places)

    # ----------------------------------------------------------------- write

    def replace(self, places: dict[str, dict]) -> Catalog:
        """새 places 전체로 교체한다. 검증 실패는 PlaceValidationError, 쓰기 실패는 PlaceStoreError."""
        places = {pid: normalize_place(p) for pid, p in copy.deepcopy(places).items()}
        validate_places(places, max_places=self.max_places)
        cat = Catalog(revision=next_revision(self.catalog.revision), places=places,
                      max_places=self.max_places)
        self._write(cat)
        self._catalog = cat
        return cat

    def bump_revision(self) -> Catalog:
        """내용은 그대로 두고 revision만 새로 발급한다(mock conflict 시나리오용)."""
        return self.replace(self.catalog.places)

    def _write(self, cat: Catalog) -> None:
        doc = {
            'version': FILE_VERSION,
            'revision': cat.revision,
            'places': {pid: {k: p[k] for k in FIELDS} for pid, p in
                       sorted(cat.places.items(), key=lambda kv: (kv[1]['sort_order'], kv[0]))},
        }
        text = yaml.safe_dump(doc, allow_unicode=True, sort_keys=False, default_flow_style=False)
        tmp = None
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(prefix=f'.{self.path.name}.', suffix='.tmp', dir=self.path.parent)
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                f.write(text)
                f.flush()
                os.fsync(f.fileno())
            self._rotate_backups()
            os.replace(tmp, self.path)
            tmp = None
            self._fsync_dir()
        except OSError as e:
            raise PlaceStoreError(f'cannot write {self.path.name}: {e.strerror or e}') from None
        finally:
            if tmp is not None:
                try:
                    os.unlink(tmp)
                except OSError:
                    pass

    def _rotate_backups(self) -> None:
        if self.backups <= 0 or not self.path.exists():
            return
        oldest = self.backup_path(self.backups)
        if oldest.exists():
            oldest.unlink()
        for n in range(self.backups - 1, 0, -1):
            src = self.backup_path(n)
            if src.exists():
                os.replace(src, self.backup_path(n + 1))
        shutil.copy2(self.path, self.backup_path(1))

    def _fsync_dir(self) -> None:
        try:
            fd = os.open(self.path.parent, os.O_RDONLY)
        except OSError:
            return
        try:
            os.fsync(fd)
        except OSError:
            pass
        finally:
            os.close(fd)
