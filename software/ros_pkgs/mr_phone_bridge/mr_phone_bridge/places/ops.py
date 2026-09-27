"""place.upsert / place.delete 규칙(PROTOCOL.md 4.1, 9.3, 9.4).

두 함수 모두 저장하지 않는다. 새 places 전체와 대상 id를 돌려주고, 저장은 호출자가 PlaceStore.replace로 한다.
검사 순서: revision → 대상 존재 → 개수 상한 → 진행 중 trip → 기본 출발지 규칙.
"""
from __future__ import annotations

import copy

from ..protocol.errors import (CATALOG_CONFLICT, INVALID_MESSAGE, INVALID_STATE, PLACE_IN_USE,
                               PLACE_LIMIT, UNKNOWN_PLACE, ProtocolError)
from .store import FIELDS, Catalog

COORD_FIELDS = ('lat', 'lon', 'radius_m')


def check_revision(catalog: Catalog, revision: str) -> None:
    if revision != catalog.revision:
        raise ProtocolError(CATALOG_CONFLICT,
                            f'catalog_revision is stale (current {catalog.revision})')


def new_place_id(kind: str, existing) -> str:
    """`place_<kind>`, 겹치면 `place_<kind>_2`, `_3` …"""
    base = f'place_{kind}'
    if base not in existing:
        return base
    n = 2
    while f'{base}_{n}' in existing:
        n += 1
    return f'{base}_{n}'


def plan_upsert(catalog: Catalog, payload: dict, in_use: set[str]) -> tuple[dict[str, dict], str]:
    """추가(place_id=null) 또는 수정. 돌려준 places는 검증 전이다(PlaceStore.replace가 다시 검증)."""
    check_revision(catalog, payload['catalog_revision'])
    places = copy.deepcopy(catalog.places)
    pid = payload['place_id']
    new = {k: payload[k] for k in FIELDS}
    if not new['label'].strip():
        raise ProtocolError(INVALID_MESSAGE, 'payload/label: must not be blank')

    if pid is None:
        if len(places) >= catalog.max_places:
            raise ProtocolError(PLACE_LIMIT, f'place limit {catalog.max_places} reached')
        pid = new_place_id(new['kind'], places)
        if not places:
            # 첫 프리셋은 기본 출발지가 된다.
            new['is_default_origin'] = True
    else:
        old = places.get(pid)
        if old is None:
            raise ProtocolError(UNKNOWN_PLACE, f'unknown place_id {pid}')
        if pid in in_use and any(old[k] != new[k] for k in COORD_FIELDS):
            raise ProtocolError(INVALID_STATE, 'cannot move a place used by the current trip')
        if old['is_default_origin'] and not new['is_default_origin']:
            raise ProtocolError(INVALID_STATE,
                                'cannot unset the default origin; mark another place as default instead')

    if new['is_default_origin']:
        for other_id, other in places.items():
            if other_id != pid:
                other['is_default_origin'] = False
    places[pid] = new
    return places, pid


def plan_delete(catalog: Catalog, payload: dict, in_use: set[str]) -> tuple[dict[str, dict], str]:
    check_revision(catalog, payload['catalog_revision'])
    places = copy.deepcopy(catalog.places)
    pid = payload['place_id']
    old = places.get(pid)
    if old is None:
        raise ProtocolError(UNKNOWN_PLACE, f'unknown place_id {pid}')
    if pid in in_use:
        raise ProtocolError(PLACE_IN_USE, 'place is used by the current trip')
    if old['is_default_origin'] and len(places) > 1:
        raise ProtocolError(INVALID_STATE,
                            'cannot delete the default origin; mark another place as default first')
    del places[pid]
    return places, pid
