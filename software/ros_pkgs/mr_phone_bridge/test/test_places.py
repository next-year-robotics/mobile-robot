"""places.yaml 저장소와 upsert/delete 규칙. websockets 없이 돈다."""
import os

import pytest
import yaml
from conftest import load_fixture

from mr_phone_bridge.mock.scenarios import default6_places, write_seed
from mr_phone_bridge.places.ops import new_place_id, plan_delete, plan_upsert
from mr_phone_bridge.places.store import (PlaceStore, PlaceStoreError, PlaceValidationError, next_revision,
                                          revision_to_ms)
from mr_phone_bridge.protocol import errors as E
from mr_phone_bridge.protocol.envelope import check_outgoing, make
from mr_phone_bridge.protocol.errors import ProtocolError

REV = '2026-08-27T12:00:00Z'


@pytest.fixture
def store(tmp_path):
    write_seed(tmp_path / 'places.yaml', default6_places(), REV)
    return PlaceStore(tmp_path / 'places.yaml')


def upsert(rev, **kw):
    p = {k: v for k, v in load_fixture('place-upsert')['payload'].items()}
    p['catalog_revision'] = rev
    p.update(kw)
    return p


def test_seed_matches_fixture(store):
    payload = store.catalog.to_payload()
    assert payload == load_fixture('place-catalog-default6')['payload']
    assert check_outgoing(make('place.catalog', payload)) is None


def test_missing_file_creates_empty(tmp_path):
    s = PlaceStore(tmp_path / 'sub' / 'places.yaml')
    cat = s.catalog
    assert cat.places == {} and cat.default_origin_place_id is None
    assert (tmp_path / 'sub' / 'places.yaml').exists()
    assert check_outgoing(make('place.catalog', cat.to_payload())) is None


def test_replace_atomic_backups_and_revision(store):
    path = store.path
    revs = [store.catalog.revision]
    for i in range(7):
        places = dict(store.catalog.places)
        places['place_home'] = {**places['place_home'], 'label': f'집{i}'}
        revs.append(store.replace(places).revision)
    assert len(set(revs)) == len(revs)
    assert [revision_to_ms(r) for r in revs] == sorted(revision_to_ms(r) for r in revs)
    assert yaml.safe_load(path.read_text(encoding='utf-8'))['places']['place_home']['label'] == '집6'
    assert '집6' in path.read_text(encoding='utf-8')          # allow_unicode
    baks = sorted(p.name for p in path.parent.glob('places.yaml.bak.*'))
    assert baks == [f'places.yaml.bak.{n}' for n in range(1, 6)]
    assert yaml.safe_load(store.backup_path(1).read_text(encoding='utf-8'))['places']['place_home']['label'] == '집5'
    assert not [p for p in path.parent.iterdir() if p.name.endswith('.tmp')]
    # 다시 읽어도 같다
    again = PlaceStore(path).catalog
    assert again.revision == revs[-1] and again.places == store.catalog.places


def test_next_revision_monotonic():
    r1 = next_revision(None, now_ms=1_790_000_000_000)
    r2 = next_revision(r1, now_ms=1_790_000_000_000)       # 같은 ms
    r3 = next_revision(r2, now_ms=1_789_000_000_000)       # 시계가 뒤로
    assert revision_to_ms(r1) < revision_to_ms(r2) < revision_to_ms(r3)
    assert r1.endswith('Z') and r1 == '2026-09-21T14:13:20.000Z'


def test_failed_write_keeps_old(store, monkeypatch):
    before = store.path.read_text(encoding='utf-8')
    rev = store.catalog.revision

    def boom(*a, **k):
        raise OSError(28, 'No space left on device')
    monkeypatch.setattr(os, 'replace', boom)
    with pytest.raises(PlaceStoreError):
        store.replace({**store.catalog.places})
    assert store.path.read_text(encoding='utf-8') == before
    assert store.catalog.revision == rev
    assert not [p for p in store.path.parent.iterdir() if p.name.endswith('.tmp')]


def test_corrupt_file_falls_back_to_backup(store):
    store.replace({**store.catalog.places})
    good_rev = PlaceStore(store.backup_path(1)).catalog.revision
    store.path.write_text('version: 1\nrevision: x\nplaces: [not, a, map]\n', encoding='utf-8')
    s2 = PlaceStore(store.path)
    assert s2.catalog.revision == good_rev
    assert 'using places.yaml.bak.1' in s2.load_warning


def test_corrupt_file_without_backup_raises(tmp_path):
    p = tmp_path / 'places.yaml'
    p.write_text(':: not yaml ::\n  - [', encoding='utf-8')
    with pytest.raises(PlaceStoreError):
        PlaceStore(p).load()
    assert p.read_text(encoding='utf-8').startswith('::')       # 덮어쓰지 않는다


@pytest.mark.parametrize('field,value', [
    ('lat', 91), ('lon', -181), ('radius_m', 4), ('radius_m', 501), ('label', ''), ('label', '   '),
    ('label', '가' * 21), ('kind', 'castle'), ('enabled', 'yes'), ('sort_order', 1.5), ('lat', float('nan')),
])
def test_validation_rejects(store, field, value):
    places = dict(store.catalog.places)
    places['place_market'] = {**places['place_market'], field: value}
    with pytest.raises(PlaceValidationError):
        store.replace(places)


def test_validation_default_origin_unique(store):
    places = dict(store.catalog.places)
    places['place_market'] = {**places['place_market'], 'is_default_origin': True}
    with pytest.raises(PlaceValidationError):
        store.replace(places)
    places = {k: {**v, 'is_default_origin': False} for k, v in store.catalog.places.items()}
    with pytest.raises(PlaceValidationError):
        store.replace(places)


def test_validation_bad_id(store):
    with pytest.raises(PlaceValidationError):
        store.replace({**store.catalog.places, 'Bad Id': store.catalog.places['place_mart']})


def test_limit_enforced_on_replace(tmp_path):
    write_seed(tmp_path / 'p.yaml', default6_places(), REV)
    s = PlaceStore(tmp_path / 'p.yaml', max_places=5)
    s.catalog    # 로드는 상한을 넘어도 된다(설정이 줄어든 경우)
    with pytest.raises(PlaceValidationError):
        s.replace(dict(s.catalog.places))


# ------------------------------------------------------------------ ops

def test_new_place_id():
    assert new_place_id('mart', {}) == 'place_mart'
    assert new_place_id('mart', {'place_mart': 1, 'place_mart_2': 1}) == 'place_mart_3'


def test_upsert_add(store):
    places, pid = plan_upsert(store.catalog, upsert(REV, kind='hospital', label='병원'), set())
    assert pid == 'place_hospital' and not places[pid]['is_default_origin']
    cat = store.replace(places)
    assert len(cat.places) == 7 and cat.default_origin_place_id == 'place_home'


def test_upsert_first_place_becomes_default(tmp_path):
    s = PlaceStore(tmp_path / 'p.yaml')
    places, pid = plan_upsert(s.catalog, upsert(s.catalog.revision, is_default_origin=False), set())
    assert places[pid]['is_default_origin'] is True
    assert s.replace(places).default_origin_place_id == pid


def test_upsert_new_default_clears_old(store):
    places, pid = plan_upsert(store.catalog, upsert(REV, place_id='place_market', kind='market', label='시장',
                                                    lat=36.1251, lon=128.5702, radius_m=30,
                                                    is_default_origin=True), set())
    cat = store.replace(places)
    assert cat.default_origin_place_id == 'place_market'
    assert not cat.places['place_home']['is_default_origin']


def test_upsert_cannot_unset_default(store):
    home = store.catalog.places['place_home']
    with pytest.raises(ProtocolError) as ei:
        plan_upsert(store.catalog, upsert(REV, place_id='place_home', **{**home, 'is_default_origin': False}), set())
    assert ei.value.code == E.INVALID_STATE


def test_upsert_conflict_unknown_limit(store, tmp_path):
    with pytest.raises(ProtocolError) as ei:
        plan_upsert(store.catalog, upsert('2026-01-01T00:00:00Z'), set())
    assert ei.value.code == E.CATALOG_CONFLICT and ei.value.retryable
    with pytest.raises(ProtocolError) as ei:
        plan_upsert(store.catalog, upsert(REV, place_id='place_nowhere'), set())
    assert ei.value.code == E.UNKNOWN_PLACE
    s = PlaceStore(store.path, max_places=6)
    with pytest.raises(ProtocolError) as ei:
        plan_upsert(s.catalog, upsert(REV), set())
    assert ei.value.code == E.PLACE_LIMIT


def test_upsert_in_use_coordinates(store):
    market = store.catalog.places['place_market']
    base = upsert(REV, place_id='place_market', **market)
    with pytest.raises(ProtocolError) as ei:
        plan_upsert(store.catalog, {**base, 'lat': 36.2}, {'place_market'})
    assert ei.value.code == E.INVALID_STATE
    places, _ = plan_upsert(store.catalog, {**base, 'label': '전통시장'}, {'place_market'})   # 이름은 된다
    assert places['place_market']['label'] == '전통시장'


def test_upsert_blank_label(store):
    with pytest.raises(ProtocolError) as ei:
        plan_upsert(store.catalog, upsert(REV, label='  '), set())
    assert ei.value.code == E.INVALID_MESSAGE


def test_delete_rules(store):
    places, pid = plan_delete(store.catalog, {'place_id': 'place_temple', 'catalog_revision': REV}, set())
    assert pid == 'place_temple' and 'place_temple' not in places
    for payload, in_use, code in [
        ({'place_id': 'place_temple', 'catalog_revision': 'old'}, set(), E.CATALOG_CONFLICT),
        ({'place_id': 'place_nowhere', 'catalog_revision': REV}, set(), E.UNKNOWN_PLACE),
        ({'place_id': 'place_market', 'catalog_revision': REV}, {'place_market'}, E.PLACE_IN_USE),
        ({'place_id': 'place_home', 'catalog_revision': REV}, set(), E.INVALID_STATE),
    ]:
        with pytest.raises(ProtocolError) as ei:
            plan_delete(store.catalog, payload, in_use)
        assert ei.value.code == code


def test_delete_last_place_allowed(tmp_path):
    s = PlaceStore(tmp_path / 'p.yaml')
    places, pid = plan_upsert(s.catalog, upsert(s.catalog.revision), set())
    s.replace(places)
    places, _ = plan_delete(s.catalog, {'place_id': pid, 'catalog_revision': s.catalog.revision}, set())
    assert s.replace(places).to_payload()['default_origin_place_id'] is None
