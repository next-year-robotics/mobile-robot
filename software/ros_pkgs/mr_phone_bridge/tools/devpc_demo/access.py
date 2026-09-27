"""실제 Firebase(senior-care-nyr) 관리 보조: Auth 사용자 목록, /access/{robot_id}/{uid} 부여·회수.

    python3 access.py list
    python3 access.py grant <uid> [dev|guardian] [--robot mr-001]
    python3 access.py revoke <uid> [--robot mr-001]

서비스 계정: MR_SERVICE_ACCOUNT 또는 ~/.config/mr_care_telemetry/service-account.json.
계약(care-telemetry README 3.6)상 access는 사람이 넣는다. 이 스크립트는 사람이 직접 돌리는 CLI다.
"""
import argparse
import json
import os
import sys
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'mr_care_telemetry'))
from mr_care_telemetry import rtdb  # noqa: E402
from mr_care_telemetry.rtdb import RtdbClient, ServiceAccountToken  # noqa: E402

PROJECT = 'senior-care-nyr'
DB = 'https://senior-care-nyr-default-rtdb.asia-southeast1.firebasedatabase.app'
SA = os.environ.get('MR_SERVICE_ACCOUNT', str(Path.home() / '.config/mr_care_telemetry/service-account.json'))


def list_users():
    rtdb.SCOPES = 'https://www.googleapis.com/auth/cloud-platform'      # Auth 관리 API 범위
    token = ServiceAccountToken(SA).token()
    req = urllib.request.Request(
        f'https://identitytoolkit.googleapis.com/v1/projects/{PROJECT}/accounts:batchGet?maxResults=100',
        headers={'Authorization': f'Bearer {token}'})
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.load(r).get('users', [])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['list', 'grant', 'revoke'])
    ap.add_argument('uid', nargs='?')
    ap.add_argument('role', nargs='?', default='dev', choices=['dev', 'guardian'])
    ap.add_argument('--robot', default='mr-001')
    a = ap.parse_args()
    if not Path(SA).is_file():
        sys.exit(f'service account not found: {SA}')
    if a.cmd == 'list':
        access = RtdbClient(DB, token=ServiceAccountToken(SA)).get(f'access/{a.robot}') or {}
        for u in list_users():
            print(u['localId'], u.get('email', '-'), access.get(u['localId'], '(no access)'))
        return
    if not a.uid:
        ap.error('uid required')
    client = RtdbClient(DB, token=ServiceAccountToken(SA))
    if a.cmd == 'grant':
        client.put(f'access/{a.robot}/{a.uid}', a.role)
    else:
        client.delete(f'access/{a.robot}/{a.uid}')
    print(f'access/{a.robot} =', client.get(f'access/{a.robot}'))


if __name__ == '__main__':
    main()
