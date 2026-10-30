# contracts — 시니어케어 외부 인터페이스 정본

이 디렉터리는 RPi5가 바깥(안드로이드 앱, 보호자 대시보드)과 주고받는 데이터 형식의 **단일 정본**이다.
서버·클라이언트 어느 쪽도 이 디렉터리 밖의 문서를 근거로 구현하지 않는다.

| 계약 | 참여자 | 문서 | 기계 정본 |
|---|---|---|---|
| `phone-bridge/v1` | `mr_phone_bridge`(여기) ↔ `senior-care-android` | `phone-bridge/v1/PROTOCOL.md` | `schema/messages.schema.json`, `fixtures/*.json` |
| `care-telemetry/v1` | `mr_care_telemetry`(여기) → Firebase RTDB → `senior-care-dashboard` | `care-telemetry/v1/README.md` | `schema/*.schema.json`, `fixtures/*.json`, `database.rules.json` |

## 검증

```bash
cd software/contracts/tools
uv run --with jsonschema python3 validate_fixtures.py          # 전체
uv run --with jsonschema python3 validate_fixtures.py ../phone-bridge/v1
```

## 다른 저장소로 복사

앱·대시보드 저장소는 `scripts/sync-contracts.sh`로 이 디렉터리의 해당 계약을 `contracts/`에 복사한다.
복사본은 편집하지 않는다. 복사본 아래 `SOURCE` 파일에 원본 커밋 해시가 남는다.
CI는 원본(`mobile-robot` `main`, 또는 `CONTRACTS_REF`로 지정한 ref)과 diff해 어긋나면 실패시킨다.

## 변경 규칙

1. 문서(PROTOCOL.md / README.md)를 먼저 고친다.
2. 스키마와 fixture를 같은 PR에서 맞추고 `validate_fixtures.py`를 통과시킨다.
3. 영향받는 저장소(앱·대시보드)의 sync PR을 함께 낸다. 서버만 먼저 병합하지 않는다.
4. v1 안에서는 선택 필드·enum 값 추가만 허용한다. 그 외는 v2 디렉터리를 새로 만든다.
