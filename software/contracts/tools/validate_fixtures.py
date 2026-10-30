#!/usr/bin/env python3
"""계약 fixture를 JSON Schema로 검증한다.

사용:  uv run --with jsonschema python3 validate_fixtures.py [contract-dir ...]
       (인자가 없으면 software/contracts/*/v*/ 전체)

각 계약 디렉터리는 schema/*.schema.json 과 fixtures/*.json 을 가진다.
phone-bridge는 fixtures/*.json 을 schema/messages.schema.json 으로,
care-telemetry는 fixtures/<name>.json 을 schema/<name 앞부분>.schema.json 으로 검증한다.
database.rules.json 이 있는 계약(= RTDB에 쓰는 계약)은 fixture의 모든 객체 키가 RTDB 키 규칙을 지키는지도 본다
(비어 있지 않음, . $ # [ ] / 와 제어 문자 금지, UTF-8 768바이트 이하). 어기면 RTDB가 쓰기 전체를 400으로 거절한다.
"""
import json, re, sys, pathlib
from jsonschema import Draft202012Validator

RTDB_KEY_BAD = re.compile(r'[.$#\[\]/\x00-\x1f\x7f]')

def rtdb_key_errors(node, path=()):
    if isinstance(node, dict):
        for k, v in node.items():
            here = path + (k,)
            if not k or RTDB_KEY_BAD.search(k) or len(k.encode('utf-8')) > 768:
                yield f"{'/'.join(map(str, path)) or '<root>'}: invalid RTDB key {k!r}"
            yield from rtdb_key_errors(v, here)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from rtdb_key_errors(v, path + (i,))

ROOT = pathlib.Path(__file__).resolve().parents[1]

def rel(p: pathlib.Path):
    p = p.resolve()
    return p.relative_to(ROOT) if p.is_relative_to(ROOT) else p

def load(p):
    with open(p, encoding="utf-8") as f:
        return json.load(f)

def validate_dir(cdir: pathlib.Path) -> int:
    schemas = {p.name[: -len(".schema.json")]: load(p) for p in (cdir / "schema").glob("*.schema.json")}
    fixtures = sorted((cdir / "fixtures").glob("*.json"))
    if not schemas or not fixtures:
        print(f"skip {cdir} (schema {len(schemas)}, fixtures {len(fixtures)})")
        return 0
    failures = 0
    rtdb = (cdir / "database.rules.json").is_file()
    for fx in fixtures:
        if "messages" in schemas:
            schema = schemas["messages"]
        else:
            key = fx.stem.split("-")[0]
            schema = schemas.get(key)
            if schema is None:
                print(f"  ?? {rel(fx)}: no schema for prefix '{key}'")
                failures += 1
                continue
        v = Draft202012Validator(schema)
        doc = load(fx)
        errs = [f"{'/'.join(map(str, e.absolute_path)) or '<root>'}: {e.message[:160]}"
                for e in sorted(v.iter_errors(doc), key=lambda e: e.path)]
        if rtdb:
            errs += list(rtdb_key_errors(doc))
        if errs:
            failures += 1
            print(f"  FAIL {rel(fx)}")
            for e in errs[:3]:
                print(f"       - {e}")
        else:
            print(f"  ok   {rel(fx)}")
    return failures

def main(argv):
    dirs = [pathlib.Path(a).resolve() for a in argv] or sorted(p for p in ROOT.glob("*/v*") if p.is_dir())
    total = 0
    for d in dirs:
        print(f"== {rel(d)}")
        total += validate_dir(d)
    print("ALL OK" if total == 0 else f"{total} fixture(s) failed")
    return 1 if total else 0

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
