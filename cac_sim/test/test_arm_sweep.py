"""Excavator arm sweep: forward kinematics of the generated URDF along the mission's
arm sequences (config/arm_params.yaml), interpolated linearly in joint space
(each gz JointPositionController drives its joint independently, so the real
path is close to this), checked against the rover's other collision geometry,
the ground and the camera's view.

Gazebo does not self-collide links of one model by default, so an intersection
would not be stopped by physics -- it would just be a bucket passing through
the chassis. This test is what keeps the poses honest.
"""
import math
import os
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
ARM = ('boom_link', 'stick_link', 'bucket_link')
JOINTS = ('boom_joint', 'stick_joint', 'bucket_joint')
# links that legitimately touch: the boom pivots on top of its mount, each arm
# link hinges on the previous one
ADJACENT = {('boom_link', 'arm_mount_link'), ('stick_link', 'boom_link'), ('bucket_link', 'stick_link')}
CLEARANCE = 0.01          # m kept between non-adjacent parts
BUCKET_DIG_DEPTH = 0.03   # m the bucket may reach below the sand surface while digging


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin(el):
    o = el.find('origin') if el is not None else None
    T = np.eye(4)
    if o is not None:
        T[:3, :3] = _rpy(*[float(v) for v in o.get('rpy', '0 0 0').split()])
        T[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
    return T


def _axis_rot(axis, q):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    return T


class Model:
    def __init__(self, root):
        self.joints = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.shapes = {}          # link -> [(T_link_geom, kind, dims)]
        for link in root.findall('link'):
            for col in link.findall('collision'):
                g = col.find('geometry')
                if g.find('box') is not None:
                    dims = [float(v) for v in g.find('box').get('size').split()]
                    self.shapes.setdefault(link.get('name'), []).append((_origin(col), 'box', dims))
                elif g.find('cylinder') is not None:
                    dims = [float(g.find('cylinder').get('radius')), float(g.find('cylinder').get('length'))]
                    self.shapes.setdefault(link.get('name'), []).append((_origin(col), 'cyl', dims))

    def T(self, link, q):
        if link == 'base_footprint':
            return np.eye(4)
        j = self.joints[link]
        T = self.T(j.find('parent').get('link'), q) @ _origin(j)
        if j.get('type') in ('revolute', 'continuous'):
            ax = [float(v) for v in j.find('axis').get('xyz').split()]
            T = T @ _axis_rot(ax, q.get(j.get('name'), 0.0))
        return T

    def arm_points(self, q, step=0.01):
        """Volume samples of every arm collision box, base_footprint frame: {link: Nx3}."""
        out = {}
        for link in ARM:
            pts = []
            for Tg, kind, d in self.shapes[link]:
                axes = [np.linspace(-s / 2, s / 2, max(2, int(round(s / step)) + 1)) for s in d]
                local = np.array(np.meshgrid(*axes, indexing='ij')).reshape(3, -1).T
                T = self.T(link, q) @ Tg
                pts.append(local @ T[:3, :3].T + T[:3, 3])
            out[link] = np.vstack(pts)
        return out

    def inside(self, link, pts, q, margin):
        """Which points lie within `margin` of any collision shape of `link`."""
        hit = np.zeros(len(pts), bool)
        for Tg, kind, d in self.shapes.get(link, []):
            T = self.T(link, q) @ Tg
            loc = (pts - T[:3, 3]) @ T[:3, :3]
            if kind == 'box':
                hit |= np.all(np.abs(loc) <= np.array(d) / 2 + margin, axis=1)
            else:
                r, h = d
                hit |= (np.hypot(loc[:, 0], loc[:, 1]) <= r + margin) & (np.abs(loc[:, 2]) <= h / 2 + margin)
        return hit


@pytest.fixture(scope='module')
def model():
    out = subprocess.run(['xacro', os.path.join(ROOT, 'urdf', 'rover.urdf.xacro')],
                         capture_output=True, text=True, check=True).stdout
    return Model(ET.fromstring(out))


def _sequences():
    with open(os.path.join(ROOT, 'config', 'arm_params.yaml')) as f:
        arm = yaml.safe_load(f)['/**']['ros__parameters']['arm']
    n = len(arm['joints'])
    poses = {k: [arm[k][i:i + n] for i in range(0, len(arm[k]), n)] for k in ('dig_sequence', 'dump_sequence')}
    return arm['stow'], poses


def _path(stow, seq, steps=40):
    """(q, segment-end-pose index) along stow -> seq[0] -> ... linearly in joint space."""
    out, prev = [], list(stow)
    for k, target in enumerate(seq):
        for s in range(1, steps + 1):
            out.append(({j: a + (b - a) * s / steps for j, a, b in zip(JOINTS, prev, target)}, k))
        prev = list(target)
    return out


def test_arm_sequences_clear_rover_and_ground(model):
    stow, seqs = _sequences()
    others = [l for l in model.shapes if l not in ARM]
    problems = []
    for name, seq in seqs.items():
        for q, k in _path(stow, seq):
            pts = model.arm_points(q)
            for link, p in pts.items():
                for other in others + [a for a in ARM if a != link]:
                    if (link, other) in ADJACENT or (other, link) in ADJACENT:
                        continue
                    if model.inside(other, p, q, CLEARANCE).any():
                        problems.append((name, k, link, other, [round(v, 2) for v in q.values()]))
                zmin = p[:, 2].min()
                allowed = -BUCKET_DIG_DEPTH if (link == 'bucket_link' and name == 'dig_sequence') else 0.02
                if zmin < allowed:
                    problems.append((name, k, link, f'ground z={zmin:.3f}', [round(v, 2) for v in q.values()]))
    assert not problems, problems[:10]


def test_stowed_arm_outside_camera_view(model):
    q = dict(zip(JOINTS, _sequences()[0]))
    T_cam = model.T('camera_link', q)
    hfov = 1.15
    vfov = 2 * math.atan(math.tan(hfov / 2) * 480 / 640)
    for link, p in model.arm_points(q).items():
        loc = (p - T_cam[:3, 3]) @ T_cam[:3, :3]          # camera_link: x forward, y left, z up
        ahead = loc[:, 0] > 0.05
        in_view = ahead & (np.abs(np.arctan2(loc[:, 1], loc[:, 0])) < hfov / 2) \
            & (np.abs(np.arctan2(loc[:, 2], loc[:, 0])) < vfov / 2)
        assert not in_view.any(), f'{link} occludes the camera when stowed'


def test_arm_limits_cover_mission_poses():
    stow, seqs = _sequences()
    out = subprocess.run(['xacro', os.path.join(ROOT, 'urdf', 'rover.urdf.xacro')],
                         capture_output=True, text=True, check=True).stdout
    lim = {j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))
           for j in ET.fromstring(out).findall('joint') if j.get('name') in JOINTS}
    for seq in [[stow]] + list(seqs.values()):
        for pose in seq:
            for jn, v in zip(JOINTS, pose):
                assert lim[jn][0] <= v <= lim[jn][1], (jn, v, lim[jn])
