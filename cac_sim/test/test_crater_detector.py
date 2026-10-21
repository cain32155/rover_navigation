"""crater_detector: hits seen before the map frame exists are kept (in odom) and
moved into the map at the first marker fix, with the usual min_hits rule."""
import math

import numpy as np
import pytest
import rclpy

from cac_sim.crater_detector import CraterDetector


@pytest.fixture
def det():
    rclpy.init()
    node = CraterDetector()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _patch(cx, cy, n=6, size=0.10):
    g = np.linspace(-size / 2, size / 2, n)
    X, Y = np.meshgrid(cx + g, cy + g)
    return np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, -0.05)])


def _T(x, y, yaw):
    T = np.eye(4)
    T[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    T[:2, 3] = [x, y]
    return T


def _centres(det):
    cs = det.get_parameter('cell_size').value
    return np.array([[(i + 0.5) * cs, (j + 0.5) * cs] for i, j in det._confirmed])


def test_pre_fix_hits_land_in_map_after_fix(det):
    for _ in range(2):                                   # two frames -> min_hits
        det._count_hits(det._odom_hits, _patch(1.0, 0.5))
    assert not det._confirmed                            # nothing published before the fix
    T_map_odom = _T(-3.3, 1.2, math.pi / 2)
    det._merge_odom_hits(T_map_odom)
    assert det._confirmed and not det._odom_hits
    expect = (T_map_odom @ [1.0, 0.5, 0.0, 1.0])[:2]     # (-3.8, 2.2)
    d = np.hypot(*(_centres(det) - expect).T)
    assert d.max() < 0.10 + 0.05                         # patch half-diagonal + one cell


def test_single_pre_fix_frame_is_not_confirmed(det):
    det._count_hits(det._odom_hits, _patch(1.0, 0.5))
    det._merge_odom_hits(_T(0.0, 0.0, 0.0))
    assert not det._confirmed and det._hits


def test_pre_fix_and_post_fix_frames_add_up(det):
    det._count_hits(det._odom_hits, _patch(1.0, 0.5))
    det._merge_odom_hits(_T(0.0, 0.0, 0.0))
    det._count_hits(det._hits, _patch(1.0, 0.5))
    det._confirmed |= {k for k, n in det._hits.items() if n >= det.get_parameter('min_hits').value}
    assert det._confirmed


# --- occlusion shadow: a ray-cast depth camera over a cosine bowl (arena_layout.crater_profile) ---

from cac_sim.arena_layout import crater_profile          # noqa: E402
from cac_sim.crater_detector import fit_ground, frame_cells   # noqa: E402

SEED12_CRATER_2 = (1.31, 0.04, 0.203, 0.059)              # x, y, rim radius, depth (5.9 cm: the seed-12 crater)
_BOX = (-0.75, 1.30, -0.50, 0.50)


def _observable(x, y):
    return 0.3 < math.hypot(x, y) < 2.2 and not (_BOX[0] < x < _BOX[1] and _BOX[2] < y < _BOX[3])


def _render(pose, crater, w=160, h=120, hfov=1.15, pitch=0.26, cam=(0.47, 0.0, 0.33)):
    """Points (base_footprint) of the urdf RGB-D camera (640x480 at the detector's stride 4)."""
    x, y, yaw = pose
    cx, cy, r, dep = crater
    vfov = 2 * math.atan(math.tan(hfov / 2) * h / w)
    U, V = np.meshgrid((np.arange(w) + 0.5) / w * 2 - 1, (np.arange(h) + 0.5) / h * 2 - 1)
    d = np.stack([np.ones_like(U), -U * math.tan(hfov / 2), -V * math.tan(vfov / 2)], -1).reshape(-1, 3)
    cp, sp = math.cos(pitch), math.sin(pitch)
    d = d @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]).T
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    P = np.array(cam)[None, None, :] + np.arange(0.1, 2.6, 0.005)[None, :, None] * d[:, None, :]
    c, s = math.cos(yaw), math.sin(yaw)
    terr = crater_profile(np.hypot(x + c * P[..., 0] - s * P[..., 1] - cx, y + s * P[..., 0] + c * P[..., 1] - cy),
                          r, dep)
    below = P[..., 2] <= terr
    pts = P[np.arange(len(d)), below.argmax(1)][below.any(1)]
    keep = np.array([_observable(a, b) for a, b in pts[:, :2]])
    return pts[keep]


def _approach(crater, bearing_deg, shadow, steps=(2.1, 1.9, 1.7, 1.5, 1.3)):
    hits, b = {}, math.radians(bearing_deg)
    for dist in steps:
        pose = (crater[0] - dist * math.cos(b), crater[1] - dist * math.sin(b), 0.0)
        pb = _render(pose, crater)
        low, sh = frame_cells(pb, fit_ground(pb), _T(*pose), 0.015, 0.05, 2, shadow, _observable)
        for key in low | sh:
            hits[key] = hits.get(key, 0) + 1
    return {k for k, n in hits.items() if n >= 2}


def _core_and_false(cells, crater, cs=0.05):
    cx, cy, r, _ = crater
    core = {(i, j) for i in range(int(cx / cs) - 6, int(cx / cs) + 7) for j in range(int(cy / cs) - 6, int(cy / cs) + 7)
            if math.hypot((i + .5) * cs - cx, (j + .5) * cs - cy) < 0.66 * r}     # deeper than 1.5 cm
    false = sum(1 for i, j in cells if math.hypot((i + .5) * cs - cx, (j + .5) * cs - cy) > r + 0.05)
    return len(cells & core) / len(core), false


@pytest.mark.parametrize('bearing', [0, 25])
def test_occlusion_shadow_maps_the_hidden_near_slope(bearing):
    without, f0 = _core_and_false(_approach(SEED12_CRATER_2, bearing, 0.0), SEED12_CRATER_2)
    with_sh, f1 = _core_and_false(_approach(SEED12_CRATER_2, bearing, 0.45), SEED12_CRATER_2)
    assert f0 == 0 and f1 == 0                       # nothing marked outside the crater
    # measured: 0 deg 0.73 -> 1.00, 25 deg 0.55 -> 0.82 of the core deeper than the threshold
    assert with_sh >= 0.8 and with_sh >= without + 0.25


def test_flat_ground_marks_nothing():
    flat = (1.31, 0.04, 0.203, 0.0)
    assert _approach(flat, 0, 0.45) == set()
