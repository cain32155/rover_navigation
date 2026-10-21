"""mission_state_machine: excavation/dump crediting and the cycle loop, driven with stubbed arm, TF and clock.

No Gazebo: the node's handlers are called directly; _arm_step, _lip, _mission_elapsed
and _start_leg are replaced so each test controls what the arm reached, where the
bucket lip was and how much mission time has passed.
"""
import pytest
import rclpy
from rclpy.parameter import Parameter

from cac_sim.mission_state_machine import MissionStateMachine, State

DIG_LIP = ((3.29, 0.0, 0.017), (0.94, 0.0, 0.017))      # (map, base_footprint)
POUR_LIP = ((4.06, 0.0, 0.58), (0.91, 0.0, 0.58))


@pytest.fixture
def node():
    rclpy.init()
    n = MissionStateMachine()
    n.set_parameters([Parameter('autonomy', value=False), Parameter('mission.num_cycles', value=5),
                      Parameter('mission.time_budget_sec', value=600.0),
                      Parameter('mission.return_reserve_sec', value=150.0),
                      Parameter('mission.first_cycle_estimate_sec', value=90.0)])
    n._max_cycles = 5
    n._t = 100.0
    n._mission_elapsed = lambda: n._t
    n._legs = []
    n._start_leg = lambda state, reverse_first=False: n._legs.append((state, reverse_first))
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _arm(node, poses_reached, done):
    """Stub the arm: `poses_reached` poses of the sequence reached, sequence finished or not."""
    node._arm_reached = poses_reached
    node._arm_step = lambda: 'done' if done else 'running'


def _dig(node, lip=DIG_LIP):
    node._lip = lambda frame: lip[0] if frame == 'map' else lip[1]
    _arm(node, 2, False)            # cut pose reached -> lip recorded
    node._handle_excavation()
    _arm(node, 3, False)            # curl reached -> scoop evaluated
    node._handle_excavation()
    _arm(node, 5, True)
    node._handle_excavation()


def _dump(node, lip=POUR_LIP):
    node._lip = lambda frame: lip[0] if frame == 'map' else lip[1]
    node._pour_ev = None
    _arm(node, 2, False)            # pour pose reached -> dump evaluated
    node._handle_dump()
    _arm(node, 4, True)
    node._handle_dump()


def _arrive_excavation(node):
    n = len(node._cycles) + 1
    node._cycle = {'cycle': n, 't_start': node._t, 'dig_attempts': 0, 'scoop_l': 0.0, 'dump_l': 0.0,
                   'in_target': None, 'ok': False, 'result': 'digging', 'rot_start': node._rot_since_fix}
    node._cycles.append(node._cycle)
    node._start_dig()


def test_one_full_cycle_credits_scoop_and_dump(node):
    _arrive_excavation(node)
    _dig(node)
    assert node._legs[-1] == (State.TRAVEL_TO_CONSTRUCTION, False)
    assert node._material.in_bucket_m3 > 0
    node._t = 160.0
    _dump(node)
    c = node._cycles[0]
    assert c['ok'] and c['in_target'] and c['dump_l'] == c['scoop_l'] > 0
    assert node._material.in_bucket_m3 == 0.0
    # 160 s elapsed + 90 s first-cycle estimate + 150 s reserve <= 600 -> another cycle, backing up first
    assert node._legs[-1] == (State.TRAVEL_TO_EXCAVATION, True)
    s = node._run_summary()
    assert s['cycles_ok'] == 1 and s['material']['dumps_ok'] == 1 and s['bcp_mass_estimate'] > 0
    assert s['material_events'][0]['lip_base'] == [0.94, 0.0, 0.017]


def test_cycles_stop_at_the_time_budget(node):
    node._max_cycles = 10
    for k in range(10):
        node._t += 20.0                 # back up + re-approach
        _arrive_excavation(node)
        _dig(node)
        node._t += 80.0
        _dump(node)
        if node._legs[-1][0] == State.RETURN:
            break
    s = node._run_summary()
    assert node._legs[-1][0] == State.RETURN
    assert s['cycles_ok'] == len(node._cycles) >= 2
    assert node._t + 150.0 <= 600.0             # the return reserve is still available
    assert all(c['duration_sec'] == 100.0 for c in node._cycles[1:])   # timed dump to dump
    assert any(h[0] == 'cycle' and h[1] == 'stop' and 'budget' in h[2] for h in node._history)


def test_scoop_outside_excavation_zone_is_retried_then_cycle_fails(node):
    _arrive_excavation(node)
    bad = ((1.9, 0.0, 0.017), (0.94, 0.0, 0.017))      # localization says the lip is in the obstacle zone
    _dig(node, bad)
    assert node._state == State.EXCAVATION and node._cycle['dig_attempts'] == 2   # second pass started
    _dig(node, bad)
    c = node._cycles[0]
    assert not c['ok'] and c['result'].startswith('failed')
    assert node._legs[-1] == (State.RETURN, False)
    assert node._material.collected_m3 == 0.0
    assert len(node._failed_actions) == 2


def test_dump_outside_construction_zone_keeps_load_and_ends_cycles(node):
    _arrive_excavation(node)
    _dig(node)
    _dump(node, ((3.3, 0.0, 0.58), (0.91, 0.0, 0.58)))
    c = node._cycles[0]
    assert not c['ok'] and 'not credited' in c['result']
    assert node._material.in_bucket_m3 > 0
    assert node._legs[-1] == (State.RETURN, False)
    assert node._run_summary()['material']['deposited_in_target_l'] == 0.0


def test_missing_tf_at_cut_pose_is_not_credited(node):
    _arrive_excavation(node)
    node._lip = lambda frame: None
    _arm(node, 2, False)
    node._handle_excavation()
    _arm(node, 3, False)
    node._handle_excavation()
    assert node._scoop_ev is not None and not node._scoop_ev.accepted
    assert 'TF' in node._scoop_ev.reason


def test_dig_repositions_when_the_lip_would_leave_the_zone(node):
    import math
    from types import SimpleNamespace
    res, w, h = 0.05, 200, 120
    info = SimpleNamespace(resolution=res, width=w, height=h,
                           origin=SimpleNamespace(position=SimpleNamespace(x=-0.5, y=-3.0)))
    node._costmap = SimpleNamespace(info=info, data=[0] * (w * h))
    node._pose = lambda: (2.56, 0.04, math.radians(2))        # seed 10, E1: lip would land at x = 3.50
    node._arm_queue = []
    _arrive_excavation(node)
    assert node._dig_move is not None and node._dig_move['dist'] < -0.15
    assert any(hh[1] == 'reposition' for hh in node._history)
    node._drive_straight_step = lambda r: True                  # drive finished
    node._arm_present = lambda: True
    node._handle_excavation()
    assert node._dig_move is None and node._arm_queue           # dig sequence started after the nudge


def _fail_leg(node, name):
    """Run the leg logic once with Nav2's last allowed attempt aborted (PATIENCE_EXCEEDED)."""
    from action_msgs.msg import GoalStatus
    node._leg = {'name': name, 'candidates': [(2.35, 0.0, 0.0)], 'cand': 0, 'attempt': 3, 'phase': 'navigating',
                 'sent_at': node._now_s(), 'next_block_check': node._now_s() + 100.0, 'blocked_checks': 0,
                 'final_turn_done': True}
    node._pose = lambda: (1.0, 0.0, 0.0)
    node._nav_result = (GoalStatus.STATUS_ABORTED, 104)
    node._run_leg(lambda: None)


def test_later_cycle_leg_failure_returns_instead_of_failing(node):
    _arrive_excavation(node)
    _dig(node)
    _dump(node)
    assert node._cycles[0]['ok']
    _fail_leg(node, 'excavation')                       # cycle 2 never reaches the excavation zone
    assert node._state != State.FAILED
    assert node._legs[-1] == (State.RETURN, False)
    assert node._run_summary()['cycles_ok'] == 1        # the deposited cycle still counts
    assert any(h[:2] == ['cycle', 'failed'] for h in node._history)
    assert node._failed_actions and 'PATIENCE_EXCEEDED' in node._failed_actions[-1][2]


def test_first_excavation_leg_failure_still_fails_the_mission(node):
    _fail_leg(node, 'excavation')
    assert node._state == State.FAILED


def test_scan_stop_turns_both_ways_and_resumes_without_using_an_attempt(node):
    import math
    from types import SimpleNamespace
    res, w, h = 0.05, 200, 120
    info = SimpleNamespace(resolution=res, width=w, height=h,
                           origin=SimpleNamespace(position=SimpleNamespace(x=-5.0, y=-3.0)))
    node._costmap = SimpleNamespace(info=info, data=[0] * (w * h))
    yaw = [0.0]
    node._pose = lambda: (-0.5, 0.0, yaw[0])
    node._leg = {'name': 'excavation', 'candidates': [(2.35, 0.0, 0.0)], 'cand': 0, 'attempt': 1,
                 'phase': 'navigating', 'sent_at': node._now_s(), 'next_block_check': node._now_s() + 100.0,
                 'blocked_checks': 0, 'final_turn_done': True}
    node._nav_result = None
    node._run_leg(lambda: None)
    assert node._leg['phase'] == 'scan'
    visited = []
    for _ in range(200):                                 # simulate the turns: jump to each target heading
        sc = node._leg.get('scan')
        if node._leg['phase'] != 'scan':
            break
        if sc['k'] < len(sc['heads']):
            yaw[0] = sc['heads'][sc['k']]
            if not visited or abs(visited[-1] - yaw[0]) > 1e-6:
                visited.append(yaw[0])
            sc['hold_since'] = (sc['hold_since'] or 0.0) - 10.0 if sc['hold_since'] is not None else None
        node._scan_step(node._leg)
    assert node._leg['phase'] == 'send' and node._leg['attempt'] == 0       # next send makes it attempt 1 again
    assert [round(math.degrees(v)) for v in visited] == [-50, 50, 0]
    node._leg['phase'] = 'navigating'
    assert not node._start_scan(node._leg)               # same place: no second scan


def _run_cycles(node, rot_per_cycle):
    node._max_cycles = 10
    node._rot_since_fix = 13.0                     # after crossing the obstacle zone with scan stops
    for k in range(10):
        node._t += 20.0
        _arrive_excavation(node)
        if node._rot_base is None:
            node._rot_base = node._rot_since_fix
        node._rot_since_fix += rot_per_cycle / 2
        _dig(node)
        node._t += 10.0
        _dump(node)
        if node._legs[-1][0] == State.RETURN:
            break
        node._rot_since_fix += rot_per_cycle / 2


def test_drift_budget_stops_looping_cycles(node):
    _run_cycles(node, 7.5)                         # seed 10: Nav2 loops every cycle
    stop = [h for h in node._history if h[:2] == ['cycle', 'stop']]
    assert stop and 'drift' in stop[-1][2]
    # cycle 1 turned 3.75 rad (+1 allowance -> 0.115 m predicted): cycle 2 allowed; it really
    # turned 7.5 rad (0.153 m) and the next would add 0.10 m: stop. A prediction, not a bound.
    assert len(node._cycles) == 2
    assert node._cycles[-1]['drift_estimate_m'] < 0.16


def test_drift_budget_allows_straight_cycles(node):
    _run_cycles(node, 0.8)                         # typical: align + back up, ~0-1.4 rad
    assert len(node._cycles) == 10
    assert not any(h[:2] == ['cycle', 'stop'] and 'drift' in h[2] for h in node._history)


def test_marker_fix_resets_the_drift_estimate(node):
    from std_msgs.msg import String
    import json
    node._rot_since_fix = 20.0
    node._on_loc_status(String(data=json.dumps({'initialized': True, 'fixes': 3})))
    assert node._rot_since_fix == 0.0
