"""Unit tests for cac_sim.material_model: sand accounting for excavation and dumping."""
import math
import os

import pytest

from cac_sim import arena_layout
from cac_sim import material_model as mm

P = arena_layout.load_params(os.path.join(os.path.dirname(__file__), '..', 'config', 'arena_params.yaml'))
DIG = (3.30, 0.0, 0.017)        # dig-pose lip from arm_params.yaml (0.94 m ahead of x = 2.35)
POUR = (4.06, 0.0, 0.58)        # pour-pose lip 0.91 m ahead of a stop at x = 3.15 (arm_params.yaml)
SCOOP = mm.MaterialParams().bucket_capacity_m3 * mm.MaterialParams().fill_factor


def conserved(m):
    return math.isclose(m.collected_m3, m.deposited_in_target_m3 + m.deposited_outside_target_m3 + m.in_bucket_m3,
                        abs_tol=1e-12) and min(m.collected_m3, m.in_bucket_m3, m.deposited_in_target_m3,
                                               m.deposited_outside_target_m3) >= 0.0


def test_defaults_match_arena_config():
    # the defaults (used by these tests) are the configured values
    p = mm.MaterialParams.from_params(P)
    d = mm.MaterialParams()
    for k in ('fill_factor', 'bulk_density_kg_m3', 'max_cut_height_m', 'dig_cell_m', 'max_dig_depth_m',
              'excavation_x', 'construction_x', 'arena_half_width', 'berm_target_xy', 'berm_target_radius_m'):
        assert getattr(p, k) == getattr(d, k), k
    assert math.isclose(p.bucket_capacity_m3, d.bucket_capacity_m3, rel_tol=1e-3)


def test_bucket_capacity_matches_urdf_bucket():
    import re
    src = open(os.path.join(os.path.dirname(__file__), '..', 'urdf', 'arm.xacro')).read()
    m = re.search(r'<collision>\s*<origin[^>]*/>\s*<geometry><box size="([\d.]+) ([\d.]+) ([\d.]+)"/>',
                  src[src.index('name="bucket_link"'):])
    lx, wy, hz = (float(v) for v in m.groups())
    plate = 0.01
    interior = (lx - plate) * (wy - 2 * plate) * (hz - plate / 2)
    assert math.isclose(P['material']['bucket_capacity_l'] / 1000.0, interior, rel_tol=0.01)


def test_scoop_then_dump_in_target():
    m = mm.MaterialModel()
    ev = m.scoop('c1', DIG, True)
    assert ev.accepted and math.isclose(ev.volume_m3, SCOOP)
    assert math.isclose(m.in_bucket_m3, SCOOP)
    ev = m.dump('c1', POUR, True)
    assert ev.accepted and ev.detail['in_target']
    assert math.isclose(m.deposited_in_target_m3, SCOOP) and m.in_bucket_m3 == 0.0
    assert conserved(m)
    s = m.summary()
    assert s['scoops_ok'] == 1 and s['dumps_ok'] == 1 and s['rejected'] == 0
    assert math.isclose(s['deposited_in_target_cm3'], round(SCOOP * 1e6, 1))


def test_payload_capacity_is_enforced():
    m = mm.MaterialModel(mm.MaterialParams(fill_factor=0.8))
    assert m.scoop('a', DIG, True).accepted
    ev = m.scoop('b', (3.30, 0.5, 0.017), True)        # second pass: only the remaining room
    assert ev.accepted and math.isclose(m.in_bucket_m3, m.p.bucket_capacity_m3)
    ev = m.scoop('c', (3.30, -0.5, 0.017), True)
    assert not ev.accepted and 'full' in ev.reason
    assert conserved(m)


def test_partial_dump():
    m = mm.MaterialModel()
    m.scoop('a', DIG, True)
    assert m.dump('d1', POUR, True, fraction=0.25).accepted
    assert math.isclose(m.in_bucket_m3, 0.75 * SCOOP)
    assert m.dump('d2', POUR, True).accepted
    assert m.in_bucket_m3 == 0.0 and math.isclose(m.deposited_in_target_m3, SCOOP)
    assert not m.dump('d3', POUR, True, fraction=0.0).accepted
    assert conserved(m)


@pytest.mark.parametrize('lip,why', [
    ((1.80, 0.0, 0.017), 'outside the excavation zone'),     # obstacle zone
    ((-3.5, 0.0, 0.017), 'outside the excavation zone'),     # start zone
    ((3.80, 0.0, 0.017), 'outside the excavation zone'),     # construction zone
    ((3.30, 2.70, 0.017), 'outside the excavation zone'),    # beyond the wall
    ((3.30, 0.0, 0.20), 'above the sand'),                   # bucket in the air
])
def test_scoop_rejected_at_invalid_location(lip, why):
    m = mm.MaterialModel()
    ev = m.scoop('x', lip, True)
    assert not ev.accepted and why in ev.reason
    assert m.collected_m3 == 0.0 and conserved(m)


def test_scoop_and_dump_need_their_arm_pose():
    m = mm.MaterialModel()
    assert not m.scoop('x', DIG, False).accepted
    m.scoop('y', DIG, True)
    ev = m.dump('z', POUR, False)
    assert not ev.accepted and 'pose' in ev.reason and math.isclose(m.in_bucket_m3, SCOOP)


def test_dump_outside_construction_zone_keeps_the_load():
    m = mm.MaterialModel()
    m.scoop('a', DIG, True)
    ev = m.dump('b', (3.2, 0.0, 0.58), True)               # still over the excavation zone
    assert not ev.accepted and math.isclose(m.in_bucket_m3, SCOOP)
    assert conserved(m)


def test_dump_outside_target_counted_separately():
    m = mm.MaterialModel()
    m.scoop('a', DIG, True)
    ev = m.dump('b', (4.06, 1.5, 0.58), True)              # construction zone, far from the berm target
    assert ev.accepted and not ev.detail['in_target']
    assert m.deposited_in_target_m3 == 0.0 and math.isclose(m.deposited_outside_target_m3, SCOOP)
    assert m.summary()['deposited_in_target_cm3'] == 0.0
    assert conserved(m)


def test_empty_bucket_dump_rejected():
    m = mm.MaterialModel()
    ev = m.dump('a', POUR, True)
    assert not ev.accepted and 'empty' in ev.reason


def test_dig_site_is_exhausted():
    p = mm.MaterialParams()
    m = mm.MaterialModel(p)
    n = 0
    while True:
        ev = m.scoop(f's{n}', DIG, True)
        if not ev.accepted:
            break
        m.dump(f'd{n}', POUR, True)
        n += 1
        assert n < 100
    assert 'exhausted' in ev.reason
    assert math.isclose(m.collected_m3, m.site_capacity_m3())
    assert math.isclose(n, math.ceil(m.site_capacity_m3() / SCOOP))
    # a different site still has sand
    assert m.scoop('other', (3.30, 0.80, 0.017), True).accepted
    assert conserved(m)


def test_repeated_action_ids_are_not_credited_twice():
    m = mm.MaterialModel()
    assert m.scoop('cycle1', DIG, True).accepted
    ev = m.scoop('cycle1', DIG, True)
    assert not ev.accepted and 'duplicate' in ev.reason
    assert m.dump('cycle1', POUR, True).accepted
    assert not m.dump('cycle1', POUR, True).accepted
    assert math.isclose(m.collected_m3, SCOOP) and math.isclose(m.deposited_in_target_m3, SCOOP)
    # a rejected id may be retried (nothing was credited)
    m2 = mm.MaterialModel()
    assert not m2.scoop('r', DIG, False).accepted
    assert m2.scoop('r', DIG, True).accepted


def test_reset_clears_everything():
    m = mm.MaterialModel()
    m.scoop('a', DIG, True)
    m.dump('a', POUR, True)
    m.reset()
    assert (m.collected_m3, m.in_bucket_m3, m.deposited_in_target_m3, m.events) == (0.0, 0.0, 0.0, [])
    assert m.scoop('a', DIG, True).accepted          # ids are forgotten too


def test_multi_cycle_accounting():
    m = mm.MaterialModel()
    for k in range(3):
        assert m.scoop(f'c{k}', (3.30, 0.05 * k, 0.017), True).accepted
        assert m.dump(f'c{k}', POUR, True).accepted
        assert conserved(m)
    s = m.summary()
    assert s['scoops_ok'] == 3 and s['dumps_ok'] == 3
    assert math.isclose(m.deposited_in_target_m3, 3 * SCOOP)
    assert math.isclose(s['collected_kg'], round(3 * SCOOP * m.p.bulk_density_kg_m3, 3))


def test_bcp_mass_formula():
    # rulebook: volume within target (cm^3) / (run minutes * robot kg)
    assert math.isclose(mm.bcp_mass(1000.0, 10.0, 50.0), 2.0)
    with pytest.raises(ValueError):
        mm.bcp_mass(1000.0, 0.0, 50.0)


def test_pile_shape_volume_grows_monotonically():
    prev = (0.0, 0.0)
    for v in (1e-4, 1e-3, 5e-3, 2e-2):
        r, h = mm.pile_shape(v)
        assert math.isclose(math.pi * r * r * h / 3.0, v, rel_tol=1e-6)
        assert r > prev[0] and h > prev[1]
        prev = (r, h)
    assert mm.pile_shape(0.0) == (0.0, 0.0)
