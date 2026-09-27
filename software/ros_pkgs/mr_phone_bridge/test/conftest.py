"""공용 테스트 도구. websockets가 필요한 도구는 그 모듈에서 importorskip 뒤에 쓴다."""
from __future__ import annotations

import asyncio
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mr_phone_bridge.protocol import contracts_path  # noqa: E402


def load_fixture(name: str) -> dict:
    with open(contracts_path() / 'fixtures' / f'{name}.json', encoding='utf-8') as f:
        return json.load(f)


def run(coro, timeout: float = 30.0):
    """pytest-asyncio 없이 코루틴 테스트를 돈다."""
    return asyncio.run(asyncio.wait_for(coro, timeout))


class FakeClock:

    def __init__(self, t: float = 1000.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


@pytest.fixture
def clock():
    return FakeClock()
