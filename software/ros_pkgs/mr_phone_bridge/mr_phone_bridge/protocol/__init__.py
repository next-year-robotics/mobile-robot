"""phone-bridge/1 프로토콜 코어. rclpy를 import하지 않는다.

정본: software/contracts/phone-bridge/v1/PROTOCOL.md, schema/messages.schema.json.
개발 PC에서는 저장소 경로에서 읽고, 설치본(RPi5)은 setup.py가 share/에 복사한 사본을 읽는다.
"""
import os
from pathlib import Path

_REL = Path('phone-bridge') / 'v1'


def contracts_path() -> Path:
    """phone-bridge/v1 계약 디렉터리.

    찾는 순서: 환경변수 MR_CONTRACTS_DIR → 이 파일의 상위 경로 중
    `software/contracts`, `contracts`, `share/mr_phone_bridge/contracts`.
    """
    env = os.environ.get('MR_CONTRACTS_DIR')
    if env:
        return Path(env) / _REL
    here = Path(__file__).resolve()
    for parent in here.parents:
        for cand in (parent / 'software' / 'contracts' / _REL,
                     parent / 'contracts' / _REL,
                     parent / 'share' / 'mr_phone_bridge' / 'contracts' / _REL):
            if (cand / 'schema' / 'messages.schema.json').is_file():
                return cand
    raise FileNotFoundError('phone-bridge/v1 contracts not found; set MR_CONTRACTS_DIR')
