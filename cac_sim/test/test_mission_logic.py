"""Unit tests for cac_sim.mission_logic: the rules that gate state transitions."""
import math
import os
from types import SimpleNamespace

from cac_sim import arena_layout
from cac_sim import mission_logic as ml

P = arena_layout.load_params(os.path.join(os.path.dirname(__file__), '..', 'config', 'arena_params.yaml'))
FP = ml.Footprint(P['robot']['footprint_front'], P['robot']['footprint_rear'], P['robot']['footprint_half_width'])
Z = ml.Zones(tuple(P['zones']['starting_zone']), tuple(P['zones']['excavation_zone']),
             tuple(P['zones']['construction_zone']), P['arena']['length'] / 2, P['arena']['width'] / 2)
GOALS = {k: tuple(P['mission'][k]) for k in ('excavation_approach', 'construction_approach', 'return_approach')}


def test_arrival_requires_position_and_heading():
    goal = (3.05, 0.0, 0.0)
    assert ml.arrival_ok(goal, (3.10, 0.1, 0.2), 0.3, 0.6)
    assert not ml.arrival_ok(goal, (3.50, 0.0, 0.0), 0.3, 0.6)
    assert not ml.arrival_ok(goal, (3.05, 0.0, 1.0), 0.3, 0.6)
    assert ml.arrival_ok((0, 0, math.pi), (0, 0, -math.pi + 0.1), 0.3, 0.6)   # wraps


def test_mission_goals_satisfy_their_zone_conditions():
    assert ml.zone_ok('construction', GOALS['construction_approach'], FP, Z)
    assert ml.zone_ok('excavation', GOALS['excavation_approach'], FP, Z)
    assert ml.zone_ok('return', GOALS['return_approach'], FP, Z)


def test_construction_zone_condition():
    assert not ml.zone_ok('construction', (2.80, 0.0, 0.0), FP, Z)    # front short of x=3.5
    assert not ml.zone_ok('construction', (4.00, 0.0, 0.0), FP, Z)    # old berm goal: through the wall


def test_old_berm_goal_only_fits_sideways():
    # The audit found the old goal (berm centre, x=4.0) clears the east wall only
    # when the rover is turned ~90 deg; the mission used the start heading instead.
    for deg in range(0, 360, 15):
        sideways = min(abs(deg - 90), abs(deg - 270)) < 10
        assert ml.zone_ok('construction', (4.0, 0.0, math.radians(deg)), FP, Z) == sideways


def test_excavation_and_return_conditions():
    assert not ml.zone_ok('excavation', (1.0, 0.0, 0.0), FP, Z)
    assert not ml.zone_ok('return', (-2.0, 0.0, math.pi), FP, Z)


def test_success_needs_nav_success_and_verified_arrival():
    assert ml.decide_nav_result(True, 0, True, True, 1, 3).action == 'success'
    # Nav2 says SUCCEEDED but the pose is elsewhere -> not success
    assert ml.decide_nav_result(True, 0, False, False, 1, 3).action != 'success'
    # error_code 0 failure (e.g. rejected / BT ack timeout) -> plain retry
    assert ml.decide_nav_result(False, 0, False, False, 1, 3).action == 'retry'


def test_retries_are_bounded():
    d = ml.decide_nav_result(False, ml.NO_VALID_PATH, False, False, 3, 3)
    assert d.action == 'fail' and 'NO_VALID_PATH' in d.reason
    assert ml.decide_nav_result(False, ml.NO_VALID_PATH, False, False, 1, 3).action == 'retry_alternate'
    assert ml.decide_nav_result(False, 105, False, False, 3, 3).action == 'fail'
    assert ml.decide_nav_result(False, 0, False, False, 2, 3, timed_out=True).reason.startswith('leg attempt timed out')


def test_candidate_goals_keep_zone_condition():
    for leg, goal in (('construction', GOALS['construction_approach']), ('excavation', GOALS['excavation_approach']),
                      ('return', GOALS['return_approach'])):
        cands = ml.candidate_goals(leg, goal, Z, FP)
        assert cands[0] == goal and len(cands) > 1
        for c in cands:
            assert ml.zone_ok(leg, c, FP, Z)
            for cx, cy in FP.corners(*c):
                assert abs(cx) <= 4.5 - 0.25 + 1e-9 or c == goal


def _grid(blocked):
    w, h, res = 100, 60, 0.1
    data = [0] * (w * h)
    for (x, y) in blocked:
        data[int((y + 3.0) / res) * w + int((x + 5.0) / res)] = 100
    info = SimpleNamespace(resolution=res, width=w, height=h,
                           origin=SimpleNamespace(position=SimpleNamespace(x=-5.0, y=-3.0)))
    return SimpleNamespace(info=info, data=data)


def test_footprint_cost_check():
    assert ml.footprint_cost_ok(_grid([]), (0.0, 0.0, 0.0), FP)
    assert not ml.footprint_cost_ok(_grid([(0.3, 0.1)]), (0.0, 0.0, 0.0), FP)
    assert ml.footprint_cost_ok(_grid([(1.5, 0.0)]), (0.0, 0.0, 0.0), FP)
    assert not ml.footprint_cost_ok(_grid([]), (4.9, 0.0, 0.0), FP)   # leaves the grid


def _boundary_grid(boulders=()):
    """Arena keep-out band (as arena_boundary_publisher builds it) + lethal discs, 5 cm cells."""
    res, pad, band = 0.05, 0.30, 0.15
    nx, ny = int(round((9.0 + 2 * pad) / res)), int(round((5.0 + 2 * pad) / res))
    ox, oy = -4.5 - pad, -2.5 - pad
    data = []
    for j in range(ny):
        for i in range(nx):
            x, y = ox + res * (i + 0.5), oy + res * (j + 0.5)
            hit = abs(x) > 4.5 - band or abs(y) > 2.5 - band or any(
                math.hypot(x - bx, y - by) < r for bx, by, r in boulders)
            data.append(100 if hit else 0)
    info = SimpleNamespace(resolution=res, width=nx, height=ny,
                           origin=SimpleNamespace(position=SimpleNamespace(x=ox, y=oy)))
    return SimpleNamespace(info=info, data=data)


# Seed 10 (suite3): believed pose when Nav2 returned START_OCCUPIED, next to boulder_1.
SEED10_POSE = (0.506, 1.585, math.radians(55.1))
SEED10_BOULDER = (0.94, 0.66, 0.19)


def test_seed10_start_pose_covers_the_keepout_band():
    g = _boundary_grid([SEED10_BOULDER])
    assert ml.lethal_cells(g, SEED10_POSE, FP, pad=0.02)          # what the planner refuses
    assert not ml.lethal_cells(g, SEED10_POSE, FP, pad=0.0)       # 2 cm padding tips it


def test_escape_motion_clears_footprint_without_new_contact():
    g = _boundary_grid([SEED10_BOULDER])
    kind, amount = ml.escape_motion(g, SEED10_POSE, FP, pad=0.02)
    x, y, yaw = SEED10_POSE
    end = (x + amount * math.cos(yaw), y + amount * math.sin(yaw), yaw) if kind == 'drive' else (x, y, yaw + amount)
    assert not ml.lethal_cells(g, end, FP, pad=0.05)
    assert (kind == 'drive' and abs(amount) <= 0.25) or (kind == 'turn' and abs(amount) <= math.radians(60))


def test_escape_motion_prefers_turning_in_place():
    # Corner just inside the north band, nothing else nearby: a small turn clears it.
    g = _boundary_grid()
    pose = (0.0, 2.35 - 0.37 - 0.02 - 0.66 * math.sin(math.radians(20)) + 0.03, math.radians(20))
    assert ml.lethal_cells(g, pose, FP, pad=0.02)
    kind, amount = ml.escape_motion(g, pose, FP, pad=0.02)
    assert kind == 'turn'


def test_escape_motion_none_when_boxed_in():
    # Lethal discs on every side: no motion may add contact, so nothing is returned.
    g = _boundary_grid([(0.0, 0.0, 0.05), (0.75, 0.0, 0.1), (-0.6, 0.0, 0.1), (0.0, 0.45, 0.1), (0.0, -0.45, 0.1),
                        (0.6, 0.5, 0.1), (0.6, -0.5, 0.1), (-0.5, 0.5, 0.1), (-0.5, -0.5, 0.1)])
    assert ml.escape_motion(g, (0.0, 0.0, 0.0), FP, pad=0.02) is None


def test_start_occupied_is_a_plain_retry():
    d = ml.decide_nav_result(False, ml.START_OCCUPIED, False, False, 1, 3)
    assert d.action == 'retry' and 'START_OCCUPIED' in d.reason


def test_escape_motion_requires_free_centre():
    # Footprint clear but the centre cell inscribed (99): turning cannot help,
    # only a drive that takes the centre out of the inscribed band.
    g = _boundary_grid()
    w = g.info.width
    for j in range(g.info.height):
        for i in range(w):
            x = g.info.origin.position.x + 0.05 * (i + 0.5)
            if -0.1 < x < 0.1 and g.data[j * w + i] < 100:
                g.data[j * w + i] = 99
    pose = (0.0, 0.0, 0.0)
    m = ml.escape_motion(g, pose, FP, pad=0.02)
    assert m is not None and m[0] == 'drive' and abs(m[1]) >= 0.1


def test_escape_clearest_moves_away_from_cost():
    # Free start, but a wall of high (non-lethal) cost just ahead: 'clearest' must
    # back away or turn away rather than make the smallest token motion.
    g = _boundary_grid()
    w = g.info.width
    for j in range(g.info.height):
        for i in range(w):
            x = g.info.origin.position.x + 0.05 * (i + 0.5)
            if 0.55 < x < 0.80 and g.data[j * w + i] < 100:
                g.data[j * w + i] = 90
    pose = (0.0, 0.0, 0.0)
    assert ml.escape_motion(g, pose, FP, pad=0.06, objective='smallest') == ('turn', math.radians(5))
    kind, amount = ml.escape_motion(g, pose, FP, pad=0.06, objective='clearest')
    end = (amount * 1.0, 0.0, 0.0) if kind == 'drive' else (0.0, 0.0, amount)
    assert ml.footprint_max_cost(g, end, FP, 0.06) < 90


def test_zone_cross_margin_and_goals():
    # every mission goal, and a stop 0.20 m short of it (Nav2 goal tolerance),
    # still crosses its zone line by the mission's margin
    m = P['mission']['zone_cross_margin']
    for leg, key in (('construction', 'construction_approach'), ('excavation', 'excavation_approach')):
        x, y, yaw = GOALS[key]
        assert ml.zone_ok(leg, (x - 0.20, y, yaw), FP, Z, cross_margin=m)
    # 2 cm across (seed 10 r4_3) is not enough with the margin
    x_front = Z.construction[0] + 0.02
    assert ml.zone_ok('construction', (x_front - FP.front, 0.5, 0.0), FP, Z)
    assert not ml.zone_ok('construction', (x_front - FP.front, 0.5, 0.0), FP, Z, cross_margin=m)


def test_turn_in_place_safe():
    g = _boundary_grid([(0.0, 0.85, 0.05)])          # lethal disc just off the left side
    assert ml.turn_in_place_safe(g, (0.0, 0.0, 0.0), 0.0, FP, 0.06)
    # turning 90 deg swings the front over the disc
    assert not ml.turn_in_place_safe(g, (0.0, 0.0, 0.0), math.pi / 2, FP, 0.06)
    assert ml.turn_in_place_safe(g, (0.0, 0.0, 0.0), -0.3, FP, 0.06)


def test_reverse_distance_between_cycles():
    exc, con = GOALS['excavation_approach'], GOALS['construction_approach']
    d = ml.reverse_distance(con, exc)
    assert d is not None and math.isclose(d, exc[0] - con[0])          # straight back, negative
    # typical arrival errors are still accepted
    assert ml.reverse_distance((con[0] - 0.15, 0.12, 0.25), exc) is not None
    # goal ahead, too far sideways, or heading too different: leave it to Nav2
    assert ml.reverse_distance(exc, con) is None
    assert ml.reverse_distance((con[0], 0.5, 0.0), exc) is None
    assert ml.reverse_distance((con[0], 0.0, 0.8), exc) is None
    assert ml.reverse_distance((exc[0] + 0.02, 0.0, 0.0), exc) is None   # already there


def test_straight_drive_safe():
    g = _boundary_grid([(1.0, 0.0, 0.05)])           # lethal disc 1 m ahead of the origin
    assert ml.straight_drive_safe(g, (0.0, 0.0, 0.0), -0.8, FP, 0.06)       # backing away
    assert not ml.straight_drive_safe(g, (0.0, 0.0, 0.0), 0.5, FP, 0.06)    # front reaches the disc
    # the real between-cycle move: construction goal -> excavation goal, clear arena
    con, exc = GOALS['construction_approach'], GOALS['excavation_approach']
    assert ml.straight_drive_safe(_boundary_grid(), con, exc[0] - con[0], FP, 0.06)
    # an excavation-zone boulder behind the rover blocks it
    assert not ml.straight_drive_safe(_boundary_grid([(2.35 - 0.6, 0.0, 0.15)]), con, exc[0] - con[0], FP, 0.06)


def test_another_cycle_is_bounded_by_count_and_time():
    ok, _ = ml.another_cycle(1, 10, 200.0, 600.0, 90.0, 150.0)
    assert ok                                           # 200 + 90 + 150 = 440 <= 600
    ok, why = ml.another_cycle(1, 10, 400.0, 600.0, 90.0, 150.0)
    assert not ok and 'budget' in why                   # 640 > 600
    ok, why = ml.another_cycle(3, 3, 100.0, 600.0, 60.0, 150.0)
    assert not ok and '3/3' in why
    # exactly at the budget is allowed
    assert ml.another_cycle(1, 5, 360.0, 600.0, 90.0, 150.0)[0]


def test_dig_correction_keeps_the_lip_in_the_excavation_zone():
    zone, lip, target = (2.0, 3.5), 0.94, 2.35 + 0.94
    assert ml.dig_correction((2.35, 0.0, 0.0), lip, zone, target) == 0.0          # nominal
    d = ml.dig_correction((2.56, 0.04, math.radians(2)), lip, zone, target)      # seed 10 E1: lip at 3.50
    assert d is not None and d < 0 and math.isclose(2.56 + d * math.cos(math.radians(2)) + lip *
                                                    math.cos(math.radians(2)), target, abs_tol=1e-6)
    assert ml.dig_correction((1.0, 0.0, 0.0), lip, zone, target) is None          # too far to fix by a nudge
    assert ml.dig_correction((2.5, 0.0, math.pi / 2), lip, zone, target) == 0.0   # sideways: lip x = 2.5, inside
    assert ml.dig_correction((3.45, 0.0, math.radians(80)), lip, zone, target) is None  # lip at 3.61, nearly sideways


def test_escape_long_drive_backs_out_of_a_closing_corridor():
    # Lethal cells along both sides of the front half (a corridor that narrowed in the
    # costmap): no turn or short drive ends clear, backing 0.65 m along the track does.
    g = _boundary_grid([(x / 100, s * 0.46, 0.04) for x in range(20, 65, 5) for s in (1, -1)])
    pose = (0.0, 0.0, 0.0)
    assert ml.escape_motion(g, pose, FP, pad=0.06, objective='clearest') is None
    kind, amount = ml.escape_motion(g, pose, FP, pad=0.06, objective='clearest', max_drive=0.8)
    assert kind == 'drive' and -0.8 <= amount < -0.25
    assert not ml.lethal_cells(g, (amount, 0.0, 0.0), FP, 0.06 + 0.03)


def test_scan_due_only_in_the_obstacle_zone_every_spacing():
    zone = (-2.7, 2.0)
    assert ml.scan_due((-1.0, 0.0, 0.0), None, 1.2, zone)                 # first time in the zone
    assert not ml.scan_due((-3.5, 0.0, 0.0), None, 1.2, zone)             # start zone
    assert not ml.scan_due((2.5, 0.0, 0.0), None, 1.2, zone)              # excavation zone
    assert not ml.scan_due((1.85, 0.0, 0.0), None, 1.2, zone)             # within 0.3 m of the edge
    assert not ml.scan_due((-0.2, 0.0, 0.0), (-1.0, 0.0), 1.2, zone)      # 0.8 m since the last scan
    assert ml.scan_due((0.3, 0.0, 0.0), (-1.0, 0.0), 1.2, zone)


def test_another_cycle_stops_on_the_drift_estimate():
    ok, why = ml.another_cycle(3, 10, 200.0, 600.0, 60.0, 150.0, drift_now=0.20, drift_per_cycle=0.08, drift_max=0.25)
    assert not ok and 'drift' in why
    assert ml.another_cycle(3, 10, 200.0, 600.0, 60.0, 150.0, drift_now=0.10, drift_per_cycle=0.08,
                            drift_max=0.25)[0]


def test_construction_goal_margin_survives_nav2_tolerance_and_localization_error():
    # Nav2 stops up to 0.20 m short; localization x error reached 0.14 m (E4 seed 9: the
    # front was believed 0.14 m across x = 3.5 but was 0.3 cm short). The goal must leave
    # the zone crossing true with that error, and the pour (lip 0.91 m ahead) on the berm target.
    gx = GOALS['construction_approach'][0]
    berm, r = P['zones']['berm_target'][0], 0.4
    for stop in (gx, gx - 0.20):
        assert ml.zone_ok('construction', (stop - 0.14, 0.0, 0.0), FP, Z, cross_margin=0.10)
        assert abs(stop + 0.91 - berm) <= r - 0.15            # pour inside the target with 0.15 m to spare
        assert ml.zone_ok('construction', (stop, 0.0, 0.0), FP, Z)   # and off the east wall
