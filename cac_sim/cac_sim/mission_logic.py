"""Mission decision logic, kept free of ROS so it can be unit tested.

mission_state_machine.py owns the ROS plumbing (action client, TF, timers) and
asks this module every question whose answer decides a state transition:
  - did the rover really arrive?            -> arrival_ok() + zone_ok()
  - what to do with a navigation result?    -> decide_nav_result()
  - which goal to try next on this leg?     -> candidate_goals()
  - how to get off a lethal cell?          -> escape_motion()
  - start another excavate/dump cycle?      -> another_cycle()
  - back up to the dig site between cycles? -> reverse_distance() + straight_drive_safe()
  - will the bucket dig inside the zone?    -> dig_correction()
  - time to stop and look sideways?         -> scan_due()
"""
import math
from dataclasses import dataclass

# Nav2 error codes (nav2_msgs ComputePathToPose / FollowPath). NavigateToPose's
# default behaviour tree passes the failing server's code through.
NONE = 0
PLANNER_CODES = {200: 'planner UNKNOWN', 201: 'INVALID_PLANNER', 202: 'planner TF_ERROR',
                 203: 'START_OUTSIDE_MAP', 204: 'GOAL_OUTSIDE_MAP', 205: 'START_OCCUPIED',
                 206: 'GOAL_OCCUPIED', 207: 'planner TIMEOUT', 208: 'NO_VALID_PATH'}
CONTROLLER_CODES = {100: 'controller UNKNOWN', 101: 'INVALID_CONTROLLER', 102: 'controller TF_ERROR',
                    103: 'INVALID_PATH', 104: 'PATIENCE_EXCEEDED', 105: 'FAILED_TO_MAKE_PROGRESS',
                    106: 'NO_VALID_CONTROL', 107: 'CONTROLLER_TIMED_OUT'}
START_OCCUPIED, GOAL_OCCUPIED, NO_VALID_PATH = 205, 206, 208


def error_name(code: int) -> str:
    if code == NONE:
        return 'NONE (no code -- e.g. goal rejected, canceled or BT-level failure)'
    return PLANNER_CODES.get(code) or CONTROLLER_CODES.get(code) or f'code {code}'


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


@dataclass(frozen=True)
class Footprint:
    front: float
    rear: float
    half_width: float

    def corners(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + c * bx - s * by, y + s * bx + c * by)
                for bx, by in ((self.front, self.half_width), (self.front, -self.half_width),
                               (-self.rear, -self.half_width), (-self.rear, self.half_width))]


@dataclass(frozen=True)
class Zones:
    starting: tuple
    excavation: tuple
    construction: tuple
    half_length: float          # inner wall faces at x = +-half_length
    half_width: float


def arrival_ok(goal: tuple, pose: tuple, xy_tol: float, yaw_tol: float) -> bool:
    """Localized pose (x, y, yaw) within tolerance of goal (x, y, yaw)."""
    return (math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= xy_tol
            and abs(angle_diff(pose[2], goal[2])) <= yaw_tol)


def zone_ok(leg: str, pose: tuple, fp: Footprint, z: Zones, wall_clearance: float = 0.10,
            cross_margin: float = 0.0) -> bool:
    """Leg-specific zone condition on the localized footprint.

    excavation: some part of the rover has crossed into the excavation zone
      (rulebook: operations may start "as soon as any part of it crosses").
    construction: some part has crossed into the construction zone AND every
      corner stays wall_clearance inside the east wall.
    cross_margin: how far past the zone line "crossed" must be (the mission uses
      0.10 m against localization error; the rule itself is 0).
    return: the rover centre is back inside the starting zone.
    Every leg also requires all corners to be inside the arena.
    """
    xs = [c[0] for c in fp.corners(*pose)]
    ys = [c[1] for c in fp.corners(*pose)]
    inside = max(abs(v) for v in xs) <= z.half_length and max(abs(v) for v in ys) <= z.half_width
    if not inside:
        return False
    if leg == 'excavation':
        return max(xs) > z.excavation[0] + cross_margin
    if leg == 'construction':
        return max(xs) > z.construction[0] + cross_margin and max(xs) <= z.half_length - wall_clearance
    if leg == 'return':
        return z.starting[0] <= pose[0] <= z.starting[1]
    raise ValueError(f'unknown leg {leg!r}')


@dataclass
class NavDecision:
    action: str      # 'success' | 'retry' | 'retry_alternate' | 'fail'
    reason: str


def decide_nav_result(succeeded: bool, error_code: int, arrived: bool, in_zone: bool,
                      attempt: int, max_attempts: int, timed_out: bool = False) -> NavDecision:
    """Turn one NavigateToPose outcome into the next step for the leg.

    A leg only succeeds when Nav2 reported SUCCEEDED *and* the localized pose
    confirms arrival and the zone condition. Everything else is a failed attempt;
    attempts are bounded, after which the leg (and the mission) fails.
    GOAL_OCCUPIED / NO_VALID_PATH switch to an alternate goal on the next try.
    """
    if succeeded and arrived and in_zone:
        return NavDecision('success', 'arrival verified')
    if succeeded:
        reason = ('Nav2 reported success but the localized pose is not at the goal'
                  if not arrived else 'at the goal but the zone condition failed')
    elif timed_out:
        reason = 'leg attempt timed out'
    else:
        reason = f'navigation failed: {error_name(error_code)}'
    if attempt >= max_attempts:
        return NavDecision('fail', f'{reason} (attempt {attempt}/{max_attempts}, giving up)')
    if error_code in (GOAL_OCCUPIED, NO_VALID_PATH) or (succeeded and not in_zone):
        return NavDecision('retry_alternate', f'{reason} (attempt {attempt}/{max_attempts})')
    return NavDecision('retry', f'{reason} (attempt {attempt}/{max_attempts})')


def candidate_goals(leg: str, goal: tuple, z: Zones, fp: Footprint) -> list:
    """Primary goal first, then alternates that still satisfy the leg's zone
    condition (used after GOAL_OCCUPIED / NO_VALID_PATH, e.g. a boulder or a
    detected crater sitting on the primary goal). Shifts are lateral (y) and
    small along x, so the alternates stay where the leg needs the rover."""
    x, y, yaw = goal
    out = [goal]
    for dy in (0.5, -0.5, 1.0, -1.0, 1.5, -1.5):
        for dx in (0.0, 0.1, -0.1):
            cand = (x + dx, y + dy, yaw)
            if zone_ok(leg, cand, fp, z) and _keeps_off_walls(cand, fp, z):
                out.append(cand)
    return out


def _keeps_off_walls(pose: tuple, fp: Footprint, z: Zones, margin: float = 0.25) -> bool:
    return all(abs(cx) <= z.half_length - margin and abs(cy) <= z.half_width - margin
               for cx, cy in fp.corners(*pose))


def footprint_cost_ok(grid, pose: tuple, fp: Footprint, lethal: int = 99, step: float = 0.05) -> bool:
    """True when no cell under the footprint is lethal/inscribed in `grid`
    (nav_msgs/OccupancyGrid-like: .info.resolution/width/height/origin, .data);
    unknown (-1) counts as free, cells outside the grid as blocked."""
    info = grid.info
    res, w, h = info.resolution, info.width, info.height
    ox, oy = info.origin.position.x, info.origin.position.y
    c, s = math.cos(pose[2]), math.sin(pose[2])
    bx = -fp.rear
    while bx <= fp.front + 1e-9:
        by = -fp.half_width
        while by <= fp.half_width + 1e-9:
            wx, wy = pose[0] + c * bx - s * by, pose[1] + s * bx + c * by
            i, j = int((wx - ox) / res), int((wy - oy) / res)
            if not (0 <= i < w and 0 <= j < h):
                return False
            if grid.data[j * w + i] >= lethal:
                return False
            by += step
        bx += step
    return True


def lethal_cells(grid, pose: tuple, fp: Footprint, pad: float = 0.0, lethal: int = 100,
                 step: float = 0.02) -> set:
    """Grid cells (i, j) at or above `lethal` under the footprint grown by `pad`
    (edges included, sampled every `step` m); cells outside the grid count as lethal."""
    info = grid.info
    res, w, h = info.resolution, info.width, info.height
    ox, oy = info.origin.position.x, info.origin.position.y
    c, s = math.cos(pose[2]), math.sin(pose[2])
    x0, x1, hw = -fp.rear - pad, fp.front + pad, fp.half_width + pad
    nx, ny = int(math.ceil((x1 - x0) / step)), int(math.ceil(2 * hw / step))
    out = set()
    for a in range(nx + 1):
        bx = x0 + (x1 - x0) * a / nx
        for b in range(ny + 1):
            by = -hw + 2 * hw * b / ny
            i = int(math.floor((pose[0] + c * bx - s * by - ox) / res))
            j = int(math.floor((pose[1] + s * bx + c * by - oy) / res))
            if not (0 <= i < w and 0 <= j < h) or grid.data[j * w + i] >= lethal:
                out.add((i, j))
    return out


def footprint_max_cost(grid, pose: tuple, fp: Footprint, pad: float = 0.0, step: float = 0.05) -> int:
    """Highest cell value under the padded footprint (cells outside the grid count as 100)."""
    info = grid.info
    res, w, h = info.resolution, info.width, info.height
    ox, oy = info.origin.position.x, info.origin.position.y
    c, s = math.cos(pose[2]), math.sin(pose[2])
    x0, x1, hw = -fp.rear - pad, fp.front + pad, fp.half_width + pad
    nx, ny = int(math.ceil((x1 - x0) / step)), int(math.ceil(2 * hw / step))
    best = 0
    for a in range(nx + 1):
        bx = x0 + (x1 - x0) * a / nx
        for b in range(ny + 1):
            by = -hw + 2 * hw * b / ny
            i = int(math.floor((pose[0] + c * bx - s * by - ox) / res))
            j = int(math.floor((pose[1] + s * bx + c * by - oy) / res))
            best = max(best, grid.data[j * w + i] if (0 <= i < w and 0 <= j < h) else 100)
    return best


def escape_motion(grid, pose: tuple, fp: Footprint, pad: float = 0.02, margin: float = 0.03,
                  max_turn: float = math.radians(60), max_drive: float = 0.25, inscribed: int = 99,
                  objective: str = 'smallest'):
    """Smallest motion that takes the (padded) footprint off every lethal cell.

    The lattice planner refuses to plan from a pose whose padded footprint covers
    a lethal cell (START_OCCUPIED), and Nav2's default behaviour tree runs no
    recovery for that code, so a rover that has turned a corner 1-2 cm into the
    virtual wall keep-out band stays stuck. Candidates, in order: turns in place
    of 5..max_turn deg (the rover stays where it is), then straight drives of
    0.05..max_drive m forward, then backward. A candidate is accepted when no
    intermediate pose covers a lethal cell the start pose does not already cover
    (no new contact), the end pose is clear even with `pad + margin`, and the
    end pose's centre cell is below `inscribed` (a planner also rejects a start
    whose centre is within the inscribed radius of an obstacle; a turn in place
    cannot fix that, a drive can).
    objective 'smallest' returns the first accepted candidate in the order above;
    'clearest' returns the accepted candidate whose end pose has the lowest
    footprint_max_cost (ties: the smaller motion) -- for a start the planner
    refuses although no lethal cell is visible under it.
    Returns ('turn', dyaw_rad), ('drive', metres) or None.
    """
    start = lethal_cells(grid, pose, fp, pad)
    x, y, yaw = pose

    def centre_free(p):
        info = grid.info
        i = int(math.floor((p[0] - info.origin.position.x) / info.resolution))
        j = int(math.floor((p[1] - info.origin.position.y) / info.resolution))
        return 0 <= i < info.width and 0 <= j < info.height and grid.data[j * info.width + i] < inscribed

    def ok(poses):
        if any(not lethal_cells(grid, p, fp, pad) <= start for p in poses):
            return False
        return not lethal_cells(grid, poses[-1], fp, pad + margin) and centre_free(poses[-1])

    cands = []
    k = 1
    while math.radians(5 * k) <= max_turn + 1e-9:
        for sgn in (1, -1):
            a = sgn * math.radians(5 * k)
            n = max(2, int(abs(a) / math.radians(2.5)))
            cands.append((('turn', a), [(x, y, yaw + a * m / n) for m in range(1, n + 1)]))
        k += 1
    for sgn in (1, -1):
        d = 0.05
        while d <= max_drive + 1e-9:
            n = max(2, int(d / 0.025))
            cands.append((('drive', sgn * d), [(x + sgn * d * m / n * math.cos(yaw),
                                                y + sgn * d * m / n * math.sin(yaw), yaw) for m in range(1, n + 1)]))
            d += 0.05
    best = None
    for rank, (motion, poses) in enumerate(cands):
        if not ok(poses):
            continue
        if objective == 'smallest':
            return motion
        score = (footprint_max_cost(grid, poses[-1], fp, pad), rank)
        if best is None or score < best[0]:
            best = (score, motion)
    return None if best is None else best[1]


def turn_in_place_safe(grid, pose: tuple, target_yaw: float, fp: Footprint, pad: float = 0.02) -> bool:
    """True if turning in place from pose to target_yaw never puts the padded
    footprint on a lethal cell the start pose does not already cover."""
    start = lethal_cells(grid, pose, fp, pad)
    d = angle_diff(target_yaw, pose[2])
    n = max(2, int(abs(d) / math.radians(2.5)))
    return all(lethal_cells(grid, (pose[0], pose[1], pose[2] + d * k / n), fp, pad) <= start
               for k in range(1, n + 1))


def straight_drive_safe(grid, pose: tuple, dist: float, fp: Footprint, pad: float = 0.02,
                        step: float = 0.025) -> bool:
    """True if driving straight (dist > 0 forward, < 0 backward) from pose never puts
    the padded footprint on a lethal cell the start pose does not already cover."""
    start = lethal_cells(grid, pose, fp, pad)
    n = max(2, int(abs(dist) / step))
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return all(lethal_cells(grid, (pose[0] + c * dist * k / n, pose[1] + s * dist * k / n, pose[2]), fp, pad)
               <= start for k in range(1, n + 1))


def reverse_distance(pose: tuple, goal: tuple, max_lateral: float = 0.25, max_yaw: float = 0.35,
                     max_dist: float = 1.2):
    """Metres to back up straight from pose to reach goal, or None.

    Between cycles the rover stands at the construction goal facing the berm and
    the excavation goal lies straight behind it with the same heading. The lattice
    planner (no reverse primitives) would turn around and back, sweeping the 1.13 m
    rover through a full circle twice; backing up is the excavator's own move.
    Only offered when the goal is behind (along-track), close to the rover's line
    (lateral) and at about the same heading."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = goal[0] - pose[0], goal[1] - pose[1]
    along, lateral = c * dx + s * dy, -s * dx + c * dy
    if along >= -0.05 or -along > max_dist or abs(lateral) > max_lateral:
        return None
    if abs(angle_diff(goal[2], pose[2])) > max_yaw:
        return None
    return along


def another_cycle(cycles_done: int, max_cycles: int, mission_elapsed: float, budget: float,
                  cycle_estimate: float, return_reserve: float, drift_now: float = 0.0,
                  drift_per_cycle: float = 0.0, drift_max: float = float('inf')):
    """(bool, reason): start another excavate -> dump cycle?

    Bounded three ways: at most max_cycles; the elapsed mission time plus one more
    cycle (estimated from the cycles so far) plus the time kept for the return leg
    must fit the run-time budget (rulebook prelims: 10 min run); and the estimated
    dead-reckoning error after one more cycle must stay within drift_max. Markers are
    seen only from the start zone, so every cycle adds unseen drift (seed 10, E3:
    0.14 -> 0.34 m over 6 cycles; the 4th pour landed 0.15 m outside the berm target)."""
    if cycles_done >= max_cycles:
        return False, f'{cycles_done}/{max_cycles} cycles done'
    if drift_now + drift_per_cycle > drift_max:
        return False, (f'localization drift estimate: {drift_now:.2f} m now + {drift_per_cycle:.2f} m per cycle '
                       f'> {drift_max:.2f} m')
    need = mission_elapsed + cycle_estimate + return_reserve
    if need > budget:
        return False, (f'time budget: {mission_elapsed:.0f} s elapsed + {cycle_estimate:.0f} s per cycle + '
                       f'{return_reserve:.0f} s return reserve > {budget:.0f} s')
    return True, f'{mission_elapsed:.0f} s elapsed, next cycle ends by ~{need - return_reserve:.0f} s'


def dig_correction(pose: tuple, lip_forward: float, zone_x: tuple, target_lip_x: float,
                   near_margin: float = 0.05, far_margin: float = 0.10, max_drive: float = 0.5):
    """Straight drive (m, + forward) that puts the dig-pose bucket lip back inside the
    excavation zone, 0.0 if it already lands there, None if no short drive can.

    The lip lands lip_forward ahead of base_footprint (arm FK). Arrival tolerance
    (0.30 m) lets the rover stop where the lip would cut across x = 3.5, in the
    construction zone (seed 10, E1: lip at x = 3.50, both scoops rightly refused).
    """
    c = math.cos(pose[2])
    lip_x = pose[0] + lip_forward * c
    if zone_x[0] + near_margin <= lip_x <= zone_x[1] - far_margin:
        return 0.0
    if abs(c) < 0.5:
        return None
    d = (target_lip_x - lip_x) / c
    return d if abs(d) <= max_drive else None


def scan_due(pose: tuple, last_scan_xy, spacing: float, zone_x: tuple, edge: float = 0.3) -> bool:
    """True when the rover is inside the obstacle zone (edge m in from its x-limits) and
    has moved `spacing` m since the last scan (or never scanned on this crossing).

    The camera sees +-33 deg ahead; a route that runs past a crater and then turns
    toward it crosses ground seen only at the image edge (seed 12, E2: crater_2 at
    29-33 deg for 9.5 s, never mapped, a wheel in it). A scan stop turns the camera
    across +-50 deg so the ground beside the route is seen before the route turns."""
    if not zone_x[0] + edge <= pose[0] <= zone_x[1] - edge:
        return False
    return last_scan_xy is None or math.hypot(pose[0] - last_scan_xy[0], pose[1] - last_scan_xy[1]) >= spacing
