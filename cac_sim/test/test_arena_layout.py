"""Unit tests for cac_sim.arena_layout (no ROS / Gazebo needed)."""
import math
import os
import random
import struct

import numpy as np
import pytest

from cac_sim import arena_layout as A

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = A.load_params(os.path.join(HERE, '..', 'config', 'arena_params.yaml'))
SEEDS = range(0, 60)


@pytest.fixture(scope='module')
def layouts():
    return {s: A.generate_layout(PARAMS, s) for s in SEEDS}


def test_layout_is_deterministic():
    a, b = A.generate_layout(PARAMS, 17), A.generate_layout(PARAMS, 17)
    assert a.start == b.start
    assert [(o.name, o.x, o.y, o.diameter) for o in a.obstacles] == [(o.name, o.x, o.y, o.diameter) for o in b.obstacles]


def test_required_obstacles_and_sizes(layouts):
    o = PARAMS['obstacles']
    for lay in layouts.values():
        boulders = [b for b in lay.obstacles if b.kind == 'boulder' and b.zone == 'obstacle']
        craters = [c for c in lay.obstacles if c.kind == 'crater']
        assert len(boulders) == o['num_boulders'] >= 3
        assert len(craters) == o['num_craters'] >= 3
        for b in boulders:
            assert o['boulder_diameter_min'] <= b.diameter <= o['boulder_diameter_max']
            assert b.z <= b.diameter / 2.0 + 1e-9          # never floating
        for c in craters:
            assert 0.40 <= c.diameter <= 0.50 + 1e-9       # rulebook: up to 40-50 cm wide
            assert o['crater_depth_min'] <= c.depth <= o['crater_depth_max']


def test_obstacles_in_their_zones_and_spaced(layouts):
    zx = PARAMS['zones']
    half_w = PARAMS['arena']['width'] / 2
    m = PARAMS['obstacles']['zone_edge_margin']
    for lay in layouts.values():
        for ob in lay.obstacles:
            x0, x1 = zx['obstacle_zone'] if ob.zone == 'obstacle' else zx['excavation_zone']
            assert x0 + m - 1e-9 <= ob.x <= x1 - m + 1e-9
            assert abs(ob.y) <= half_w - m + 1e-9
        for i, a in enumerate(lay.obstacles):
            for b in lay.obstacles[i + 1:]:
                assert math.hypot(a.x - b.x, a.y - b.y) >= PARAMS['obstacles']['min_obstacle_clearance'] - 1e-9


def test_start_pose_rules(layouts):
    fp = A.Footprint.from_params(PARAMS)
    sz = PARAMS['zones']['starting_zone']
    half_l, half_w = PARAMS['arena']['length'] / 2, PARAMS['arena']['width'] / 2
    spin = fp.spin_radius + PARAMS['robot']['spin_margin']
    for lay in layouts.values():
        x, y, yaw = lay.start
        assert any(abs(math.atan2(math.sin(yaw - c), math.cos(yaw - c))) < 1e-9 for c in A.YAW_CHOICES)
        for cx, cy in fp.corners(x, y, yaw):
            assert sz[0] + PARAMS['robot']['start_margin'] - 1e-9 <= cx <= sz[1] - PARAMS['robot']['start_margin'] + 1e-9
            assert abs(cy) <= half_w - PARAMS['robot']['start_margin'] + 1e-9
        # room to turn in place: walls and obstacles
        assert min(x + half_l, half_l - x, y + half_w, half_w - y) >= spin - 1e-9
        for ob in lay.obstacles:
            assert math.hypot(ob.x - x, ob.y - y) >= spin + ob.radius - 1e-9


def test_every_emitted_layout_is_reachable(layouts):
    for s, lay in layouts.items():
        assert lay.validated
        r = A.check_reachability(PARAMS, lay.start, lay.obstacles)
        assert r.ok, (s, r.reached)


def test_reachability_rejects_a_blocked_arena():
    start = A.sample_start_pose(PARAMS, None)
    wall = [A.Obstacle('boulder', f'b{k}', 0.0, y, 0.3, 0.6) for k, y in enumerate(np.arange(-2.5, 2.6, 0.4))]
    assert not A.check_reachability(PARAMS, start, wall).ok
    assert A.check_reachability(PARAMS, start, []).ok


def test_reachability_respects_footprint_not_just_width():
    # Two boulders leaving a 0.80 m gap: wider than the rover (0.74 m) but
    # narrower than footprint + 2 * validation_margin -> must be rejected.
    start = A.sample_start_pose(PARAMS, None)
    gap = 0.80
    obs = []
    for y in np.arange(gap / 2 + 0.15, 2.6, 0.25):
        obs += [A.Obstacle('boulder', 'n', 0.0, y, 0.15, 0.3), A.Obstacle('boulder', 's', 0.0, -y, 0.15, 0.3)]
    assert not A.check_reachability(PARAMS, start, obs).ok


def test_crater_mesh_is_closed_and_deep(tmp_path):
    path = str(tmp_path / 'c.stl')
    n = A.write_crater_stl(path, 0.45, 0.12, 0.55, -0.30)
    with open(path, 'rb') as f:
        f.read(80)
        count = struct.unpack('<I', f.read(4))[0]
        tris = [struct.unpack('<12fH', f.read(50)) for _ in range(count)]
    assert count == n > 0
    zs = [t[k] for t in tris for k in (5, 8, 11)]
    assert min(z for z in zs if z > -0.29) == pytest.approx(-0.12, abs=0.005)   # bowl bottom
    assert max(zs) == pytest.approx(0.0, abs=1e-6)                              # flush with sand
    # closed solid: every edge is shared by exactly two triangles
    edges = {}
    for t in tris:
        v = [tuple(round(c, 5) for c in t[3 + 3 * k:6 + 3 * k]) for k in range(3)]
        for a, b in ((v[0], v[1]), (v[1], v[2]), (v[2], v[0])):
            e = tuple(sorted((a, b)))
            edges[e] = edges.get(e, 0) + 1
    assert set(edges.values()) == {2}


def test_floor_tiles_cover_floor_minus_holes():
    holes = [(-1.0, -0.4, 0.2, 0.8), (1.0, 1.6, -2.5, -1.9), (2.0, 2.6, 1.0, 1.6)]
    tiles = A.floor_tiles(4.5, 2.5, holes)
    area = sum((x1 - x0) * (y1 - y0) for x0, x1, y0, y1 in tiles)
    assert area == pytest.approx(9.0 * 5.0 - sum((h[1] - h[0]) * (h[3] - h[2]) for h in holes))
    rng = random.Random(0)
    for _ in range(2000):   # sample points: covered exactly once unless in a hole
        x, y = rng.uniform(-4.5, 4.5), rng.uniform(-2.5, 2.5)
        n = sum(t[0] <= x < t[1] and t[2] <= y < t[3] for t in tiles)
        in_hole = any(h[0] <= x < h[1] and h[2] <= y < h[3] for h in holes)
        assert n == (0 if in_hole else 1)


def test_generated_world(tmp_path):
    lay = A.generate_layout(PARAMS, 6)
    base = os.path.join(HERE, '..', 'worlds', 'cac_arena.sdf')
    out = A.generate_world(PARAMS, base, str(tmp_path), lay)
    sdf = open(out).read()
    assert sdf.count('<model name="sand_bed">') == 1
    for ob in lay.obstacles:
        assert f'<model name="{ob.name}">' in sdf
    for mid in A.marker_poses(PARAMS):
        assert os.path.exists(tmp_path / f'aruco_{mid}.png')
    assert 'arena_wall_east' in sdf and 'berm_target_marker' in sdf


def test_markers_follow_rulebook():
    mk = PARAMS['markers']
    poses = A.marker_poses(PARAMS)
    assert len(poses) <= 5                       # at most 5 markers
    assert mk['board_size'] <= 0.25 + 1e-9       # <= 25 x 25 cm
    sz = PARAMS['zones']['starting_zone']
    for x, y, z, yaw in poses.values():
        assert z + mk['board_size'] / 2 <= 0.60  # within 60 cm of height
        assert sz[0] - 0.01 <= x <= sz[1]        # starting-zone perimeter only
