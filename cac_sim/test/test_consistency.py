"""Cross-file consistency: arena_params footprint == Nav2 footprint == URDF envelope."""
import ast
import math
import os
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from cac_sim import arena_layout as A

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
PARAMS = A.load_params(os.path.join(ROOT, 'config', 'arena_params.yaml'))
FP = A.Footprint.from_params(PARAMS)


def _nav2():
    with open(os.path.join(ROOT, 'config', 'nav2_params.yaml')) as f:
        return yaml.safe_load(f)


def test_nav2_footprint_matches_arena_params():
    nav = _nav2()
    expected = sorted([(FP.front, FP.half_width), (FP.front, -FP.half_width),
                       (-FP.rear, -FP.half_width), (-FP.rear, FP.half_width)])
    for cm in ('local_costmap', 'global_costmap'):
        p = nav[cm][cm]['ros__parameters']
        assert 'robot_radius' not in p
        poly = sorted(tuple(v) for v in ast.literal_eval(p['footprint']))
        assert poly == pytest.approx(expected)
        # inflation must reach past the turn-in-place circle
        assert p['inflation_layer']['inflation_radius'] > FP.spin_radius


def test_mission_tolerances_cover_nav2_goal_checker():
    gc = _nav2()['controller_server']['ros__parameters']['goal_checker']
    m = PARAMS['mission']
    assert gc['xy_goal_tolerance'] < m['arrival_xy_tolerance']
    assert gc['yaw_goal_tolerance'] < m['arrival_yaw_tolerance']


def test_mission_footprint_padding_matches_nav2():
    for cm in ('local_costmap', 'global_costmap'):
        assert _nav2()[cm][cm]['ros__parameters']['footprint_padding'] == PARAMS['mission']['footprint_padding']


def _rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin(el):
    o = el.find('origin') if el is not None else None
    xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    T = np.eye(4)
    T[:3, :3] = _rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def _geometry_points(geom):
    """Sample points on a collision geometry in its own frame."""
    if geom.find('box') is not None:
        sx, sy, sz = (float(v) / 2 for v in geom.find('box').get('size').split())
        return np.array([[a, b, c] for a in (-sx, sx) for b in (-sy, sy) for c in (-sz, sz)])
    if geom.find('cylinder') is not None:
        r, h = float(geom.find('cylinder').get('radius')), float(geom.find('cylinder').get('length'))
        ang = np.linspace(0, 2 * np.pi, 48, endpoint=False)
        return np.array([[r * math.cos(a), r * math.sin(a), z] for a in ang for z in (-h / 2, h / 2)])
    if geom.find('sphere') is not None:
        r = float(geom.find('sphere').get('radius'))
        return np.array([[r * a, r * b, r * c] for a, b, c in
                         ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))])
    return np.zeros((0, 3))


@pytest.fixture(scope='module')
def urdf():
    out = subprocess.run(['xacro', os.path.join(ROOT, 'urdf', 'rover.urdf.xacro')],
                         capture_output=True, text=True, check=True).stdout
    return ET.fromstring(out)


def _collision_points_in_base_footprint(urdf):
    joints = {j.find('child').get('link'): j for j in urdf.findall('joint')}
    cache = {}

    def T_of(link):            # base_footprint <- link, all joints at zero
        if link == 'base_footprint':
            return np.eye(4)
        if link not in cache:
            j = joints[link]
            cache[link] = T_of(j.find('parent').get('link')) @ _origin(j)
        return cache[link]
    pts = []
    for link in urdf.findall('link'):
        for col in link.findall('collision'):
            local = _geometry_points(col.find('geometry'))
            if len(local):
                T = T_of(link.get('name')) @ _origin(col)
                pts.append(local @ T[:3, :3].T + T[:3, 3])
    return np.vstack(pts)


def test_urdf_envelope_matches_footprint(urdf):
    pts = _collision_points_in_base_footprint(urdf)
    above = pts[pts[:, 2] > 0.005]          # parts that can touch obstacles
    xmax, xmin, ymax = above[:, 0].max(), above[:, 0].min(), np.abs(above[:, 1]).max()
    # footprint must contain the collision envelope, and not be grossly larger
    assert xmax <= FP.front + 1e-3 and -xmin <= FP.rear + 1e-3 and ymax <= FP.half_width + 1e-3
    assert FP.front - xmax < 0.05 and FP.rear + xmin < 0.05 and FP.half_width - ymax < 0.05


def test_urdf_within_competition_envelope(urdf):
    pts = _collision_points_in_base_footprint(urdf)
    assert pts[:, 0].max() - pts[:, 0].min() <= 1.5      # stowed length <= 1.5 m
    assert 2 * np.abs(pts[:, 1]).max() <= 0.75           # width <= 0.75 m
    assert pts[:, 2].max() - min(0.0, pts[:, 2].min()) <= 0.75   # height <= 0.75 m


def test_low_visuals_are_hidden_from_the_lidar(urdf, tmp_path):
    """Every rover visual that can reach the 0.10 m scan plane must carry the
    self-filter visibility flag after URDF->SDF conversion."""
    import shutil
    if shutil.which('gz') is None:
        pytest.skip('gz CLI not available')
    path = tmp_path / 'rover.urdf'
    path.write_text(ET.tostring(urdf, encoding='unicode'))
    sdf = subprocess.run(['gz', 'sdf', '-p', str(path)], capture_output=True, text=True, check=True).stdout
    root = ET.fromstring(sdf)
    # link poses in the model frame (SDF 1.x from gz sdf -p: link <pose> relative to the model)
    low_links = set()
    for link_name in ('front_left_wheel_link', 'rear_left_wheel_link', 'front_right_wheel_link',
                      'rear_right_wheel_link', 'left_rocker_link', 'right_rocker_link', 'lidar_link'):
        low_links.add(link_name)
    for link in root.iter('link'):
        if link.get('name') in low_links:
            vis = link.findall('visual')
            assert vis, link.get('name')
            for v in vis:
                flags = v.find('visibility_flags')
                assert flags is not None and int(flags.text) & 1 == 0, (link.get('name'), v.get('name'))


def test_spin_recovery_no_faster_than_measured_no_slip_rate():
    # Faster in-place turns slid the rover sideways unseen by odometry (seed 10).
    b = _nav2()['behavior_server']['ros__parameters']
    assert b['max_rotational_vel'] <= 0.35 and b['min_rotational_vel'] < b['max_rotational_vel']


def test_wall_margin_and_start_margin():
    nav = _nav2()['global_costmap']['global_costmap']['ros__parameters']
    band = PARAMS['localization']['keepout_band']
    assert band + nav['footprint_padding'] >= 0.17 - 1e-9          # wall margin kept
    # the start footprint, padded, must stay out of the lethal band by a cell
    assert PARAMS['robot']['start_margin'] >= band + nav['footprint_padding'] + 0.025


def test_wall_filter_lidar_height_matches_urdf(urdf):
    # wall_filter.lidar_height (ground-return rejection) must be the scan plane height
    joints = {j.find('child').get('link'): j for j in urdf.findall('joint')}
    z = 0.0
    link = 'lidar_link'
    while link != 'base_footprint':
        j = joints[link]
        z += _origin(j)[2, 3]
        link = j.find('parent').get('link')
    import inspect
    from cac_sim import wall_filter
    assert "'lidar_height', 0.10" in inspect.getsource(wall_filter)
    assert abs(z - 0.10) < 1e-6


def test_robot_mass_param_matches_urdf(urdf):
    # robot.mass_kg feeds the BCP-mass estimate; rulebook limit 80 kg
    total = sum(float(m.get('value')) for m in urdf.iter('mass'))
    assert abs(PARAMS['robot']['mass_kg'] - total) < 0.01
    assert total <= 80.0
