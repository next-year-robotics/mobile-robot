#!/usr/bin/env python3
"""계약 fixture를 JSON Schema로 검증한다.

사용:  uv run --with jsonschema python3 validate_fixtures.py [contract-dir ...]
       (인자가 없으면 software/contracts/*/v*/ 전체)

각 계약 디렉터리는 schema/*.schema.json 과 fixtures/*.json 을 가진다.
phone-bridge는 fixtures/*.json 을 schema/messages.schema.json 으로,
care-telemetry는 fixtures/<name>.json 을 schema/<name 앞부분>.schema.json 으로 검증한다.
"""
import json, sys, pathlib
from jsonschema import Draft202012Validator

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
        errs = sorted(v.iter_errors(load(fx)), key=lambda e: e.path)
        if errs:
            failures += 1
            print(f"  FAIL {rel(fx)}")
            for e in errs[:3]:
                print(f"       - {'/'.join(map(str, e.absolute_path)) or '<root>'}: {e.message[:160]}")
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
