"""Arena layout generation for the CAC simulation (pure Python + numpy, no ROS).

Everything that decides WHAT the arena contains lives here so it can be unit
tested without Gazebo: the randomized start pose, boulder/crater placement, the
footprint-reachability check that rejects unnavigable layouts, the crater
depression meshes, the floor tiling around them, the ArUco localization markers
and the final world SDF.

The launch file calls `generate_world()` once per run, BEFORE Gazebo starts, and
hands Gazebo the generated world. Baking obstacles into the world (instead of
spawning them into a running server, as the old obstacle_spawner did) is what
makes real crater depressions possible: the sand bed has to be built with holes
where the craters go, and that has to exist before the rover is spawned onto it.

Coordinates are the arena/world frame described in worlds/cac_arena.sdf:
X along the 9 m length (start zone at -X), Y across the 5 m width, Z=0 = sand top.
"""
import math
import os
import random
import re
import struct
from dataclasses import dataclass, field

import numpy as np
import yaml


# --------------------------------------------------------------------------- params

def load_params(path: str) -> dict:
    """The `ros__parameters` block of config/arena_params.yaml as a nested dict."""
    with open(path) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


@dataclass(frozen=True)
class Footprint:
    """Rover footprint rectangle relative to base_footprint (x forward, y left)."""
    front: float
    rear: float
    half_width: float

    @classmethod
    def from_params(cls, p: dict) -> 'Footprint':
        r = p['robot']
        return cls(float(r['footprint_front']), float(r['footprint_rear']), float(r['footprint_half_width']))

    @property
    def spin_radius(self) -> float:
        """Radius swept by the farthest corner when turning in place about base_footprint."""
        return math.hypot(max(self.front, self.rear), self.half_width)

    def corners(self, x: float, y: float, yaw: float) -> list:
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + c * bx - s * by, y + s * bx + c * by)
                for bx, by in ((self.front, self.half_width), (self.front, -self.half_width),
                               (-self.rear, -self.half_width), (-self.rear, self.half_width))]

    def x_extent(self, x: float, y: float, yaw: float) -> tuple:
        xs = [p[0] for p in self.corners(x, y, yaw)]
        return min(xs), max(xs)


@dataclass
class Obstacle:
    kind: str            # 'boulder' or 'crater'
    name: str
    x: float
    y: float
    radius: float        # ground-plane radius used for clearance/reachability
    diameter: float      # boulder: sphere diameter; crater: total rim-to-rim diameter
    z: float = 0.0       # boulder sphere centre height (partially buried when < radius)
    depth: float = 0.0   # crater depth below the sand surface
    zone: str = 'obstacle'


@dataclass
class Layout:
    seed: int
    start: tuple                       # (x, y, yaw) of base_footprint
    obstacles: list = field(default_factory=list)
    attempts: int = 1                  # placement attempts needed to pass validation
    validated: bool = False


# ------------------------------------------------------------------- start pose

YAW_CHOICES = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)  # rulebook: N/S/E/W


def _start_region(p: dict, fp: Footprint, yaw: float):
    """Allowed (x_min, x_max, y_min, y_max) for base_footprint at `yaw`, or None.

    Two constraints: (1) the whole footprint lies inside the starting zone with
    robot.start_margin to every boundary; (2) the rover can turn in place without
    its swept circle (fp.spin_radius + robot.spin_margin) reaching a wall.
    (2) stands in for the rulebook's allowed remote-control repositioning inside
    the start zone "to lock in the localization solution": the localization
    routine has to rotate to find the start-zone markers.
    """
    zx0, zx1 = p['zones']['starting_zone']
    half_w = p['arena']['width'] / 2.0
    half_l = p['arena']['length'] / 2.0
    margin = p['robot']['start_margin']
    spin = fp.spin_radius + p['robot']['spin_margin']
    xs = [c[0] for c in fp.corners(0.0, 0.0, yaw)]
    ys = [c[1] for c in fp.corners(0.0, 0.0, yaw)]
    x_lo = max(zx0 + margin - min(xs), -half_l + spin)
    x_hi = min(zx1 - margin - max(xs), half_l - spin)
    y_lo = max(-half_w + margin - min(ys), -half_w + spin)
    y_hi = min(half_w - margin - max(ys), half_w - spin)
    if x_lo > x_hi or y_lo > y_hi:
        return None
    return x_lo, x_hi, y_lo, y_hi


def sample_start_pose(p: dict, rng) -> tuple:
    """Random (x, y, yaw); rng=None gives the fixed default (zone centre region, yaw 0)."""
    fp = Footprint.from_params(p)
    if rng is None:
        region = _start_region(p, fp, 0.0)
        if region is None:
            raise RuntimeError('rover does not fit the starting zone at yaw 0 -- check arena_params.yaml')
        return (region[0] + region[1]) / 2.0, (region[2] + region[3]) / 2.0, 0.0
    choices = [y for y in YAW_CHOICES if _start_region(p, fp, y) is not None]
    if not choices:
        raise RuntimeError('rover does not fit the starting zone at any N/S/E/W heading')
    yaw = rng.choice(choices)
    x0, x1, y0, y1 = _start_region(p, fp, yaw)
    return rng.uniform(x0, x1), rng.uniform(y0, y1), yaw


# ------------------------------------------------------------- obstacle placement

def _place_obstacles(p: dict, rng, start_xy: tuple, fp: Footprint) -> list:
    """One placement draw. The draw ORDER matches the original obstacle_spawner
    (boulders: diameter, position, bury; craters: diameter, position, depth) so a
    seed whose first draw validates reproduces the original obstacle positions."""
    o = p['obstacles']
    zx0, zx1 = p['zones']['obstacle_zone']
    half_w = p['arena']['width'] / 2.0
    margin = o['zone_edge_margin']
    min_gap = o['min_obstacle_clearance']
    start_clear = fp.spin_radius + p['robot']['spin_margin']
    placed = []

    def try_place(radius, x0, x1):
        for _ in range(200):
            x = rng.uniform(x0 + margin, x1 - margin)
            y = rng.uniform(-half_w + margin, half_w - margin)
            if math.hypot(x - start_xy[0], y - start_xy[1]) < start_clear + radius:
                continue
            if all(math.hypot(x - q.x, y - q.y) >= min_gap for q in placed):
                return x, y
        return None

    for i in range(int(o['num_boulders'])):
        d = rng.uniform(o['boulder_diameter_min'], o['boulder_diameter_max'])
        pos = try_place(d / 2.0, zx0, zx1)
        if pos is None:
            return None
        bury = rng.uniform(o['boulder_bury_fraction_min'], o['boulder_bury_fraction_max'])
        placed.append(Obstacle('boulder', f'boulder_{i}', pos[0], pos[1], d / 2.0, d,
                               z=d / 2.0 * (1.0 - 2.0 * bury)))
    for i in range(int(o['num_craters'])):
        d = rng.uniform(o['crater_diameter_min'], o['crater_diameter_max'])
        pos = try_place(d / 2.0, zx0, zx1)
        if pos is None:
            return None
        depth = rng.uniform(o['crater_depth_min'], o['crater_depth_max'])
        placed.append(Obstacle('crater', f'crater_{i}', pos[0], pos[1], d / 2.0, d, depth=depth))
    # Rulebook: "Similar or smaller boulders may also be present in the excavation zone."
    ex0, ex1 = p['zones']['excavation_zone']
    for i in range(int(o.get('num_excavation_boulders', 0))):
        d = rng.uniform(o['excavation_boulder_diameter_min'], o['excavation_boulder_diameter_max'])
        pos = try_place(d / 2.0, ex0, ex1)
        if pos is None:
            return None
        bury = rng.uniform(o['boulder_bury_fraction_min'], o['boulder_bury_fraction_max'])
        placed.append(Obstacle('boulder', f'excavation_boulder_{i}', pos[0], pos[1], d / 2.0, d,
                               z=d / 2.0 * (1.0 - 2.0 * bury), zone='excavation'))
    return placed


def generate_layout(p: dict, seed: int, randomize: bool = True) -> Layout:
    """Start pose + obstacles for `seed`, resampled until the reachability check passes.

    Attempt 0 uses random.Random(seed) for obstacles (the original stream);
    attempt k>0 uses random.Random(seed * 1000003 + k). The start pose always
    uses random.Random(seed + 1), as before, so it never depends on attempts.
    """
    fp = Footprint.from_params(p)
    if not randomize:
        return Layout(seed, sample_start_pose(p, None), [], 1, True)
    start = sample_start_pose(p, random.Random(seed + 1))
    max_attempts = int(p['obstacles']['max_layout_attempts'])
    for attempt in range(max_attempts):
        rng = random.Random(seed if attempt == 0 else seed * 1000003 + attempt)
        obstacles = _place_obstacles(p, rng, start[:2], fp)
        if obstacles is None:
            continue
        if check_reachability(p, start, obstacles).ok:
            return Layout(seed, start, obstacles, attempt + 1, True)
    raise RuntimeError(f'seed {seed}: no navigable layout after {max_attempts} attempts')


# ------------------------------------------------------------ reachability check

@dataclass
class Reachability:
    ok: bool
    reached: dict          # goal name -> bool


def mission_goals(p: dict) -> dict:
    """Navigation goals the mission needs, as {name: (x, y, yaw)} in the arena frame."""
    m = p['mission']
    return {name: tuple(float(v) for v in m[name]) for name in
            ('excavation_approach', 'construction_approach', 'return_approach')}


def check_reachability(p: dict, start: tuple, obstacles: list, res: float = 0.05,
                       n_headings: int = 16) -> Reachability:
    """Can the real footprint drive from `start` to every mission goal?

    Search over (x, y, heading) on a `res` grid with `n_headings` headings.
    Motions: drive forward/backward ~3 cells along the heading, or rotate in place
    by one heading step; a state is free when the footprint, grown by
    obstacles.validation_margin, overlaps no obstacle and stays wall_margin inside
    the walls. Rotation sweeps between 22.5 deg steps are approximated by the two
    end states (corner arc deviates < 2 cm from the chord). Reachability is
    symmetric (every motion is reversible), so one flood fill from the start
    answers start->goal and goal->start (the return leg) together.
    """
    fp = Footprint.from_params(p)
    m = float(p['obstacles']['validation_margin'])
    wall_m = float(p['obstacles']['validation_wall_margin'])
    half_l, half_w = p['arena']['length'] / 2.0, p['arena']['width'] / 2.0
    nx, ny = int(round(2 * half_l / res)), int(round(2 * half_w / res))
    xs = -half_l + res * (np.arange(nx) + 0.5)
    ys = -half_w + res * (np.arange(ny) + 0.5)
    X, Y = np.meshgrid(xs, ys)
    thetas = np.arange(n_headings) * 2.0 * math.pi / n_headings
    cx_off = (fp.front - fp.rear) / 2.0          # rectangle centre ahead of base_footprint
    hl, hw = (fp.front + fp.rear) / 2.0 + m, fp.half_width + m
    free = np.zeros((n_headings, ny, nx), dtype=bool)
    for t, th in enumerate(thetas):
        ct, st = math.cos(th), math.sin(th)
        ok = np.ones_like(X, dtype=bool)
        for bx, by in fp.corners(0.0, 0.0, th):
            ok &= (np.abs(X + bx) <= half_l - wall_m) & (np.abs(Y + by) <= half_w - wall_m)
        rcx, rcy = X + ct * cx_off, Y + st * cx_off
        for ob in obstacles:
            dx, dy = ob.x - rcx, ob.y - rcy
            bx, by = ct * dx + st * dy, -st * dx + ct * dy
            qx = np.clip(np.abs(bx) - hl, 0.0, None)
            qy = np.clip(np.abs(by) - hw, 0.0, None)
            ok &= np.hypot(qx, qy) >= ob.radius
        free[t] = ok
    steps = [(int(round(3 * math.cos(th))), int(round(3 * math.sin(th)))) for th in thetas]

    def shift(a, dx, dy):
        out = np.zeros_like(a)
        out[max(dy, 0):ny + min(dy, 0), max(dx, 0):nx + min(dx, 0)] = \
            a[max(-dy, 0):ny + min(-dy, 0), max(-dx, 0):nx + min(-dx, 0)]
        return out

    def cell(x, y):
        return (min(ny - 1, max(0, int((y + half_w) / res))),
                min(nx - 1, max(0, int((x + half_l) / res))))

    def heading(yaw):
        return int(round((yaw % (2 * math.pi)) / (2 * math.pi / n_headings))) % n_headings

    goals = mission_goals(p)
    R = np.zeros_like(free)
    j, i = cell(start[0], start[1])
    t0 = heading(start[2])
    if free[t0, j, i]:
        R[t0, j, i] = True
        while True:
            N = R.copy()
            for t in range(n_headings):
                dx, dy = steps[t]
                N[t] |= (shift(R[t], dx, dy) | shift(R[t], -dx, -dy)) & free[t]
                N[t] |= (R[(t + 1) % n_headings] | R[(t - 1) % n_headings]) & free[t]
            if np.array_equal(N, R):
                break
            R = N
    reached = {}
    for name, (gx, gy, gyaw) in goals.items():
        gj, gi = cell(gx, gy)
        gt = heading(gyaw)
        # A goal counts when its own heading or a neighbouring one is reached
        # (Nav2's yaw tolerance is far wider than one 22.5 deg step).
        reached[name] = bool(R[gt, gj, gi] or R[(gt + 1) % n_headings, gj, gi]
                             or R[(gt - 1) % n_headings, gj, gi])
    return Reachability(all(reached.values()), reached)


# ---------------------------------------------------------------- crater meshes

def crater_profile(r: np.ndarray, radius: float, depth: float) -> np.ndarray:
    """Cosine bowl: -depth at the centre, 0 with zero slope at the rim."""
    return np.where(r < radius, -depth * 0.5 * (1.0 + np.cos(np.pi * r / radius)), 0.0)


def write_crater_stl(path: str, diameter: float, depth: float, patch: float,
                     floor_z: float, res: float = 0.02):
    """Closed binary-STL solid: a `patch` x `patch` square of sand whose top is
    flat at z=0 except for a bowl of `diameter`/`depth` in the middle; sides and
    bottom (at floor_z) close the volume so collision is robust. Model-local
    coordinates, centred on the crater."""
    n = max(4, int(math.ceil(patch / res)))
    g = np.linspace(-patch / 2.0, patch / 2.0, n + 1)
    GX, GY = np.meshgrid(g, g)
    Z = crater_profile(np.hypot(GX, GY), diameter / 2.0, depth)
    tris = []

    def quad(a, b, c, d):          # counter-clockwise seen from outside
        tris.append((a, b, c))
        tris.append((a, c, d))
    P = lambda i, j: (GX[j, i], GY[j, i], Z[j, i])  # noqa: E731
    for j in range(n):
        for i in range(n):
            quad(P(i, j), P(i + 1, j), P(i + 1, j + 1), P(i, j + 1))
    h, b = patch / 2.0, floor_z
    # Bottom: a fan from the centre over the same boundary vertices the sides
    # use, so the solid is watertight (no T-junctions). Faces -z.
    ring = ([(x, -h) for x in g[:-1]] + [(h, y) for y in g[:-1]] +
            [(x, h) for x in g[::-1][:-1]] + [(-h, y) for y in g[::-1][:-1]])
    for k in range(len(ring)):
        (x0, y0), (x1, y1) = ring[k], ring[(k + 1) % len(ring)]
        tris.append(((0.0, 0.0, b), (x1, y1, b), (x0, y0, b)))
    for k in range(n):                                             # four sides
        x0, x1 = g[k], g[k + 1]
        quad((x0, -h, b), (x1, -h, b), (x1, -h, 0.0), (x0, -h, 0.0))
        quad((x1, h, b), (x0, h, b), (x0, h, 0.0), (x1, h, 0.0))
        quad((-h, x1, b), (-h, x0, b), (-h, x0, 0.0), (-h, x1, 0.0))
        quad((h, x0, b), (h, x1, b), (h, x1, 0.0), (h, x0, 0.0))
    with open(path, 'wb') as f:
        f.write(b'cac_sim crater'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        for a, b2, c in tris:
            a, b2, c = np.array(a), np.array(b2), np.array(c)
            nrm = np.cross(b2 - a, c - a)
            ln = np.linalg.norm(nrm)
            nrm = nrm / ln if ln > 0 else nrm
            f.write(struct.pack('<12fH', *nrm, *a, *b2, *c, 0))
    return len(tris)


# ------------------------------------------------------------------ floor tiles

def floor_tiles(half_l: float, half_w: float, holes: list) -> list:
    """Axis-aligned rectangles (x0, x1, y0, y1) covering the floor minus `holes`
    (each also (x0, x1, y0, y1)); holes are assumed not to overlap each other."""
    xs = sorted({-half_l, half_l} | {v for h in holes for v in (h[0], h[1]) if -half_l < v < half_l})
    tiles = []
    for xa, xb in zip(xs[:-1], xs[1:]):
        xm = (xa + xb) / 2.0
        cut = sorted((h[2], h[3]) for h in holes if h[0] < xm < h[1])
        y = -half_w
        for ya, yb in cut:
            if ya > y:
                tiles.append((xa, xb, y, ya))
            y = max(y, yb)
        if y < half_w:
            tiles.append((xa, xb, y, half_w))
    return tiles


# ---------------------------------------------------------------- aruco markers

def write_marker_png(path: str, marker_id: int, dictionary: str, px: int = 400, border_frac: float = 0.1):
    """ArUco marker with a white quiet zone; needs OpenCV (cv2.aruco)."""
    import cv2
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    inner = int(round(px * (1.0 - 2 * border_frac)))
    img = cv2.aruco.drawMarker(d, marker_id, inner) if hasattr(cv2.aruco, 'drawMarker') \
        else cv2.aruco.generateImageMarker(d, marker_id, inner)
    pad = (px - inner) // 2
    img = cv2.copyMakeBorder(img, pad, px - inner - pad, pad, px - inner - pad, cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(path, img)


# ------------------------------------------------------------------ world SDF

_SAND = ('<material><ambient>0.76 0.65 0.45 1</ambient><diffuse>0.76 0.65 0.45 1</diffuse>'
         '<specular>0.05 0.05 0.05 1</specular></material>')
_SAND_FRICTION = '<surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>'


def _floor_model(tiles: list, thickness: float) -> str:
    links = []
    for k, (x0, x1, y0, y1) in enumerate(tiles):
        geo = f'<geometry><box><size>{x1 - x0:.4f} {y1 - y0:.4f} {thickness}</size></box></geometry>'
        links.append(
            f'<collision name="tile_{k}"><pose>{(x0 + x1) / 2:.4f} {(y0 + y1) / 2:.4f} {-thickness / 2} 0 0 0</pose>'
            f'{geo}{_SAND_FRICTION}</collision>'
            f'<visual name="tile_{k}"><pose>{(x0 + x1) / 2:.4f} {(y0 + y1) / 2:.4f} {-thickness / 2} 0 0 0</pose>'
            f'{geo}{_SAND}</visual>')
    return ('<model name="sand_bed"><static>true</static><link name="link">'
            + ''.join(links) + '</link></model>')


def _crater_model(ob: Obstacle, mesh_path: str) -> str:
    uri = 'file://' + os.path.abspath(mesh_path)
    mesh = f'<geometry><mesh><uri>{uri}</uri></mesh></geometry>'
    return (f'<model name="{ob.name}"><static>true</static><pose>{ob.x:.4f} {ob.y:.4f} 0 0 0 0</pose>'
            f'<link name="link"><collision name="collision">{mesh}{_SAND_FRICTION}</collision>'
            f'<visual name="visual">{mesh}{_SAND}</visual></link></model>')


def _boulder_model(ob: Obstacle) -> str:
    r = ob.diameter / 2.0
    geo = f'<geometry><sphere><radius>{r:.4f}</radius></sphere></geometry>'
    return (f'<model name="{ob.name}"><static>true</static><pose>{ob.x:.4f} {ob.y:.4f} {ob.z:.4f} 0 0 0</pose>'
            f'<link name="link"><collision name="collision">{geo}</collision>'
            f'<visual name="visual">{geo}<material><ambient>0.45 0.42 0.38 1</ambient>'
            f'<diffuse>0.45 0.42 0.38 1</diffuse></material></visual></link></model>')


def marker_poses(p: dict) -> dict:
    """{marker_id: (x, y, z, yaw)} from the flattened markers.ids / markers.poses params."""
    mk = p['markers']
    flat = [float(v) for v in mk['poses']]
    if len(flat) != 4 * len(mk['ids']):
        raise ValueError('markers.poses must hold 4 numbers per id in markers.ids')
    return {int(i): tuple(flat[4 * k:4 * k + 4]) for k, i in enumerate(mk['ids'])}


def _marker_model(marker_id: int, pose: tuple, png_path: str, board: float) -> str:
    x, y, z, yaw = pose
    uri = os.path.abspath(png_path)
    # A thin board whose +X face carries the marker texture. The plane visual
    # sits 1 mm in front of the board so the texture is never z-fighting. Its
    # rpy (90, 0, 90 deg) maps plane normal -> +X and texture "up" -> world +Z
    # (verified live: with a plain 90 deg pitch the marker rendered rotated 90 deg
    # in-plane, which ArUco IDs don't reveal but every pose estimate does).
    return (f'<model name="marker_{marker_id}"><static>true</static><pose>{x} {y} {z} 0 0 {yaw}</pose>'
            f'<link name="link">'
            f'<collision name="collision"><pose>-0.005 0 0 0 0 0</pose><geometry><box><size>0.01 {board} {board}</size>'
            f'</box></geometry></collision>'
            f'<visual name="board"><pose>-0.005 0 0 0 0 0</pose><geometry><box><size>0.01 {board} {board}</size>'
            f'</box></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual>'
            f'<visual name="tag"><pose>0.001 0 0 1.5708 0 1.5708</pose><geometry><plane><normal>0 0 1</normal>'
            f'<size>{board} {board}</size></plane></geometry><material><ambient>1 1 1 1</ambient>'
            f'<diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{uri}</albedo_map></metal></pbr></material>'
            f'</visual></link></model>')


def generate_world(p: dict, base_world: str, out_dir: str, layout: Layout) -> str:
    """Write the run's world SDF (+ crater meshes, marker textures) into out_dir; returns its path."""
    os.makedirs(out_dir, exist_ok=True)
    with open(base_world) as f:
        sdf = f.read()
    half_l, half_w = p['arena']['length'] / 2.0, p['arena']['width'] / 2.0
    thickness = float(p['arena']['sand_depth'])
    patch_margin = float(p['obstacles']['crater_patch_margin'])
    models, holes = [], []
    for ob in layout.obstacles:
        if ob.kind == 'crater':
            patch = ob.diameter + 2.0 * patch_margin
            mesh = os.path.join(out_dir, f'{ob.name}.stl')
            write_crater_stl(mesh, ob.diameter, ob.depth, patch, -thickness)
            holes.append((ob.x - patch / 2, ob.x + patch / 2, ob.y - patch / 2, ob.y + patch / 2))
            models.append(_crater_model(ob, mesh))
        else:
            models.append(_boulder_model(ob))
    mk = p['markers']
    for marker_id, pose in marker_poses(p).items():
        png = os.path.join(out_dir, f'aruco_{marker_id}.png')
        write_marker_png(png, marker_id, mk['dictionary'],
                         border_frac=(float(mk['board_size']) - float(mk['size'])) / 2.0 / float(mk['board_size']))
        models.append(_marker_model(marker_id, pose, png, float(mk['board_size'])))
    floor = _floor_model(floor_tiles(half_l, half_w, holes), thickness)
    new_sdf, n = re.subn(r'<model name="sand_bed">.*?</model>', floor, sdf, count=1, flags=re.S)
    if n != 1:
        raise RuntimeError(f'{base_world}: no <model name="sand_bed"> block to replace')
    new_sdf = new_sdf.replace('</world>', '\n'.join(models) + '\n</world>', 1)
    out = os.path.join(out_dir, 'cac_arena_generated.sdf')
    with open(out, 'w') as f:
        f.write(new_sdf)
    return out


def describe(layout: Layout) -> str:
    lines = [f'seed={layout.seed} start=({layout.start[0]:.2f}, {layout.start[1]:.2f}, '
             f'yaw={math.degrees(layout.start[2]):.0f} deg) attempts={layout.attempts}']
    for ob in layout.obstacles:
        extra = f'depth={ob.depth:.3f}' if ob.kind == 'crater' else f'centre_z={ob.z:.3f}'
        lines.append(f'  {ob.name:22s} ({ob.x:+.2f}, {ob.y:+.2f}) d={ob.diameter:.3f} {extra}')
    return '\n'.join(lines)
