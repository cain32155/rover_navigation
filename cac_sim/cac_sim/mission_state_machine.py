#!/usr/bin/env python3
"""Top-level mission sequencing for the CAC rover.

State flow (excavate -> carry -> dump cycles, bounded by mission.num_cycles and
mission.time_budget_sec):

  START -> LOCALIZE -> SURVEY -> TRAVEL_TO_EXCAVATION -> EXCAVATION -> TRAVEL_TO_CONSTRUCTION
        -> DUMP -> (back up, TRAVEL_TO_EXCAVATION ... while another_cycle()) -> RETURN -> COMPLETE
  any state -> FAILED (terminal, with a logged reason)

How each transition is earned:
  LOCALIZE   turns in place in steps (rotate, pause) until marker_localizer has a
             fix from the start-zone ArUco markers (status "initialized").
             The start pose is generated with turn-in-place clearance.
  SURVEY     before the first leg, turns in place to face each of
             mission.survey_headings (map frame, toward the obstacle zone) and
             holds still there, so the depth camera sweeps the ground next to
             the start zone and crater_detector confirms craters BEFORE the
             first path is planned. Without it, craters near the start were
             seen only if the marker search happened to spin past them (seed
             10: an immediate marker fix, no spin, a wheel in a crater 1.2 m
             from the start).
  TRAVEL_*   sends the leg's goal (arena frame == map frame) straight to Nav2's
             NavigateToPose; the global planner chooses the route through the
             costmap. A leg succeeds ONLY when Nav2 returns SUCCEEDED *and* the
             localized pose is within mission.arrival_* of the goal *and* the
             leg's zone condition holds (mission_logic.zone_ok). Failures are
             retried up to mission.max_goal_attempts, switching to a validated
             alternate goal after GOAL_OCCUPIED / NO_VALID_PATH; then FAILED.
             After START_OCCUPIED (the padded footprint covers a lethal cell --
             Nav2 has no recovery for it) the rover first makes the smallest
             costmap-checked turn or short drive that clears it
             (mission_logic.escape_motion).
             The arm is stowed (and verified) before every leg.
  EXCAVATION the arm runs its dig sequence; at the cut pose the bucket lip is
             located through TF and, when the curl pose is reached, MaterialModel
             credits one scoop only if the lip was in the excavation zone, at the
             sand surface, the bucket had room and the dig site still had sand.
             A rejected scoop is retried (mission.max_dig_attempts); then the
             cycle is recorded as failed and no further cycle starts.
  DUMP       the arm runs its dump sequence; at the pour pose MaterialModel credits
             the bucket's load only if the lip is over the construction zone
             (in / outside the berm target counted separately).
  Cycles     after each dump, mission_logic.another_cycle() decides from the cycle
             count and the time budget (elapsed + longest cycle + return reserve);
             the next cycle backs straight up to the excavation goal when that is
             costmap-clear (mission_logic.reverse_distance / straight_drive_safe).
  No sand moves in Gazebo: the volumes are SIMULATED ESTIMATES
  (material_model.py). The end-of-run summary (cycles, volumes, BCP-mass
  estimate, failed actions) is in /mission_status and in the log.

Goals come from config/arena_params.yaml `mission:` -- fixed zone geometry known
before the run. This node never sees boulder/crater positions (those reach Nav2
only through sensed data in the costmap) and never receives the spawn pose:
localization comes from the start-zone markers + odometry.
"""
import enum
import json
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener

from cac_sim import mission_logic as ml
from cac_sim import material_model as mm


class State(enum.Enum):
    START = 'START'
    LOCALIZE = 'LOCALIZE'
    SURVEY = 'SURVEY'
    TRAVEL_TO_EXCAVATION = 'TRAVEL_TO_EXCAVATION'
    EXCAVATION = 'EXCAVATION'
    TRAVEL_TO_CONSTRUCTION = 'TRAVEL_TO_CONSTRUCTION'
    DUMP = 'DUMP'
    RETURN = 'RETURN'
    COMPLETE = 'COMPLETE'
    FAILED = 'FAILED'


_LEG_OF_STATE = {State.TRAVEL_TO_EXCAVATION: 'excavation',
                 State.TRAVEL_TO_CONSTRUCTION: 'construction',
                 State.RETURN: 'return'}
_GOAL_PARAM = {'excavation': 'mission.excavation_approach',
               'construction': 'mission.construction_approach',
               'return': 'mission.return_approach'}


class MissionStateMachine(Node):

    def __init__(self):
        super().__init__('mission_state_machine')
        dp = self.declare_parameter
        dp('autonomy', True)
        dp('arena.length', 9.0)
        dp('arena.width', 5.0)
        dp('zones.starting_zone', [-4.5, -2.7])
        dp('zones.excavation_zone', [2.0, 3.5])
        dp('zones.construction_zone', [3.5, 4.5])
        dp('robot.footprint_front', 0.66)
        dp('robot.footprint_rear', 0.47)
        dp('robot.footprint_half_width', 0.37)
        dp('mission.excavation_approach', [2.35, 0.0, 0.0])
        dp('mission.construction_approach', [3.30, 0.0, 0.0])
        dp('mission.zone_cross_margin', 0.10)
        dp('mission.return_approach', [-3.55, 0.0, math.pi])
        dp('mission.arrival_xy_tolerance', 0.30)
        dp('mission.arrival_yaw_tolerance', 0.60)
        dp('mission.max_goal_attempts', 3)
        dp('mission.leg_timeout_sec', 240.0)
        dp('mission.num_cycles', 1)
        dp('mission.time_budget_sec', 600.0)
        dp('mission.return_reserve_sec', 150.0)
        dp('mission.first_cycle_estimate_sec', 90.0)
        dp('mission.reverse_speed', 0.10)
        dp('mission.reverse_timeout_sec', 20.0)
        dp('mission.max_dig_attempts', 2)
        # Dead-reckoning budget for the cycle loop: drift ADDED by the cycles is estimated
        # as drift_per_rad x |rotation| since the first arrival in the excavation zone (or
        # the last marker fix, if later). Rotation is what makes the skid-steer slip (worst
        # growth over a whole E3 run 0.0136 m/rad); the error brought from the crossing
        # (0.03-0.2 m measured) is unknown to the rover and not part of the estimate.
        dp('mission.drift_per_rad', 0.0136)
        dp('mission.drift_max', 0.15)
        dp('robot.mass_kg', 50.55)
        dp('zones.berm_target', [4.0, 0.0])
        dp('material.bucket_capacity_l', 1.183)
        dp('material.fill_factor', 0.8)
        dp('material.bulk_density_kg_m3', 1500.0)
        dp('material.max_cut_height', 0.05)
        dp('material.dig_cell', 0.25)
        dp('material.max_dig_depth', 0.15)
        dp('material.cut_pose_index', 1)
        dp('material.curl_pose_index', 2)
        dp('material.pour_pose_index', 1)
        dp('arm.bucket_link', 'bucket_link')
        dp('arm.bucket_lip', [0.14, 0.0, -0.035])   # lip point in bucket_link (arm_params.yaml)
        dp('arm.dig_lip_forward', 0.94)             # lip ahead of base_footprint at the dig pose (FK)
        dp('localize_spin_rate', 0.35)       # rad/s while searching for markers
        dp('localize_timeout_sec', 90.0)
        # Obstacle-zone survey (deg, map frame; 0 = east, toward the obstacle zone).
        # 30 deg steps with the camera's 66 deg view: about -93..+93 deg covered.
        dp('mission.survey_headings', [-60.0, -30.0, 0.0, 30.0, 60.0])
        dp('mission.survey_hold_sec', 1.2)       # still time per heading (detector: 2 frames at 8 Hz)
        dp('mission.survey_turn_rate', 0.35)     # rad/s, the measured no-slip turn rate
        dp('mission.survey_timeout_sec', 60.0)
        dp('mission.footprint_padding', 0.02)   # = Nav2 footprint_padding (test_consistency)
        dp('escape_timeout_sec', 15.0)
        # Scan stops while crossing the obstacle zone (mission_logic.scan_due): pause the
        # Nav2 goal, face heading + each offset (costmap-checked turns), hold, resume.
        dp('mission.scan_spacing', 1.2)
        dp('mission.scan_offsets_deg', [-50.0, 50.0])
        dp('mission.scan_hold_sec', 1.0)
        dp('mission.scan_timeout_sec', 30.0)
        dp('zones.obstacle_zone', [-2.7, 2.0])
        # When no turn <= 60 deg or drive <= 0.25 m clears the start, try drives up to
        # this far (the rover usually got there driving forward along a corridor that
        # narrowed in the costmap; seed 10, state D: 'no escape found' three times).
        dp('escape_long_drive', 0.8)
        # Final turn: the lattice planner reaches a goal heading only by driving a
        # 2.6-3.3 m loop when the rover is already at the goal position but turned
        # (planner_route_test, seed 14) -- through ground the camera never saw (a
        # wheel in an undetected crater). Within final_turn_radius of the goal and
        # more than final_turn_min_yaw off its heading, the mission cancels Nav2 and
        # turns in place instead (if the sweep is clear in the costmap).
        dp('mission.final_turn_radius', 0.25)
        dp('mission.final_turn_min_yaw', 0.40)    # = Nav2 goal checker yaw tolerance
        dp('mission.final_turn_timeout_sec', 20.0)
        # Arm (optional: used only if these joints appear on /joint_states).
        dp('arm.joints', ['boom_joint', 'stick_joint', 'bucket_joint'])
        dp('arm.stow', [0.0, 0.0, 0.0])
        dp('arm.dig_sequence', [0.0, 0.0, 0.0])    # flattened list of poses
        dp('arm.dump_sequence', [0.0, 0.0, 0.0])
        dp('arm.pose_tolerance', 0.08)
        dp('arm.pose_timeout_sec', 8.0)

        g = self.get_parameter
        self._enabled = bool(g('autonomy').value)
        self._fp = ml.Footprint(g('robot.footprint_front').value, g('robot.footprint_rear').value,
                                g('robot.footprint_half_width').value)
        self._zones = ml.Zones(tuple(g('zones.starting_zone').value), tuple(g('zones.excavation_zone').value),
                               tuple(g('zones.construction_zone').value),
                               g('arena.length').value / 2.0, g('arena.width').value / 2.0)
        self._max_attempts = int(g('mission.max_goal_attempts').value)
        self._max_cycles = int(g('mission.num_cycles').value)
        self._cycles_left = self._max_cycles      # reported in /mission_status (kept for older tools)
        self._material = mm.MaterialModel(mm.MaterialParams.from_params({
            'material': {k: g(f'material.{k}').value for k in (
                'bucket_capacity_l', 'fill_factor', 'bulk_density_kg_m3', 'max_cut_height', 'dig_cell',
                'max_dig_depth')},
            'zones': {'excavation_zone': g('zones.excavation_zone').value,
                      'construction_zone': g('zones.construction_zone').value,
                      'berm_target': g('zones.berm_target').value},
            'arena': {'width': g('arena.width').value}}))
        self._cycles = []           # one dict per started cycle
        self._cycle = None
        self._failed_actions = []   # [t, what, why] -- never counted as work
        self._mission_t0 = None     # sim time when the run started (LOCALIZE)
        self._last_dig_cand = None  # excavation goal candidate index that was last reached
        self._rot_since_fix = 0.0   # |yaw change| (odom) since the last marker fix, rad
        self._odom_yaw = None
        self._fixes_seen = 0
        self._rot_at_cycle_end = []
        self._rot_base = None       # _rot_since_fix at the first excavation arrival
        self._summary_logged = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # bt_navigator's action server exists before the node is ACTIVE, and
        # goals sent then are rejected; legs wait for ACTIVE instead of burning
        # attempts (its global costmap needs the map frame, i.e. the marker fix).
        self._bt_state = self.create_client(GetState, '/bt_navigator/get_state')
        self._nav_active = False
        self._nav_query = None
        dp('nav2_ready_timeout_sec', 180.0)
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._state_pub = self.create_publisher(String, 'mission_state', 10)
        self._status_pub = self.create_publisher(String, 'mission_status', 10)
        self._payload_pub = self.create_publisher(Float32, 'payload_level', 10)   # litres in the bucket
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._costmap = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: setattr(self, '_costmap', m), latched)
        self._loc_status = {}
        self.create_subscription(String, '/localization/status', self._on_loc_status, 10)
        self._joint_pos = {}
        self.create_subscription(JointState, '/joint_states', self._on_joints, 20)
        self._arm_pubs = {j: self.create_publisher(Float64, f'/arm/{j}/cmd', 10)
                          for j in g('arm.joints').value}

        self._state = None
        self._state_t0 = self.get_clock().now()
        self._history = []          # [(leg, outcome, reason)]
        self._fail_reason = ''
        self._leg = None
        self._goal_handle = None
        self._nav_result = None     # (status, error_code) once the action finishes
        self._goal_seq = 0          # results of older (cancelled/replaced) goals are ignored
        self._arm_queue = []
        self._arm_target = None
        self._arm_t0 = None
        self._arm_reached = 0       # poses of the current sequence reached so far
        self._set_state(State.START)
        self.create_timer(0.1, self._tick)
        if not self._enabled:
            self.get_logger().info('autonomy:=false -- mission state machine idling, no goals will be sent.')

    # ----------------------------------------------------------------- helpers

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _set_state(self, state: State, note: str = ''):
        if state != self._state:
            self.get_logger().info(f'Mission state: {self._state.value if self._state else "-"} -> '
                                   f'{state.value}{" (" + note + ")" if note else ""}')
        self._state = state
        self._state_t0 = self.get_clock().now()
        self._state_pub.publish(String(data=state.value))
        self._publish_status(note)

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._state_t0).nanoseconds / 1e9

    def _mission_elapsed(self) -> float:
        return 0.0 if self._mission_t0 is None else self._now_s() - self._mission_t0

    def _run_summary(self) -> dict:
        mat = self._material.summary()
        run_s = self._mission_elapsed()
        mass = self.get_parameter('robot.mass_kg').value
        ok = [c for c in self._cycles if c.get('ok')]
        return {
            'run_time_sec': round(run_s, 1),
            'time_budget_sec': self.get_parameter('mission.time_budget_sec').value,
            'cycles_started': len(self._cycles), 'cycles_ok': len(ok),
            'cycles': self._cycles,
            'material': {k: v for k, v in mat.items() if k != 'events'},
            'material_events': mat['events'],
            'robot_mass_kg': mass,
            # rulebook BP-Mass formula on the SIMULATED in-target volume (estimate, not a score)
            'bcp_mass_estimate': round(mm.bcp_mass(mat['deposited_in_target_cm3'], run_s / 60.0, mass), 3)
            if run_s > 0 else None,
            'failed_actions': self._failed_actions,
        }

    def _publish_status(self, note: str = ''):
        self._status_pub.publish(String(data=json.dumps({
            'state': self._state.value, 'note': note, 'cycles_left': self._cycles_left,
            'history': self._history, 'fail_reason': self._fail_reason, 'summary': self._run_summary()})))

    def _failed_action(self, what: str, why: str):
        self._failed_actions.append([round(self._mission_elapsed(), 1), what, why])

    def _fail(self, reason: str):
        self._cmd_vel_pub.publish(Twist())
        self._fail_reason = reason
        self._failed_action('mission', reason)
        self.get_logger().error(f'MISSION FAILED: {reason}')
        self._set_state(State.FAILED, reason)

    def _pose(self):
        """Localized (x, y, yaw) of base_footprint in map, or None."""
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _on_loc_status(self, msg: String):
        try:
            self._loc_status = json.loads(msg.data)
        except ValueError:
            return
        fixes = int(self._loc_status.get('fixes', 0) or 0)
        if fixes > self._fixes_seen:          # a marker update: dead-reckoning error reset
            self._fixes_seen = fixes
            self._rot_since_fix = 0.0
            if self._rot_base is not None:
                self._rot_base = 0.0

    def _track_rotation(self):
        try:
            q = self._tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time()).transform.rotation
        except TransformException:
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self._odom_yaw is not None:
            self._rot_since_fix += abs(ml.angle_diff(yaw, self._odom_yaw))
        self._odom_yaw = yaw

    def _drift_estimate(self) -> float:
        base = self._rot_base if self._rot_base is not None else self._rot_since_fix
        return self.get_parameter('mission.drift_per_rad').value * max(0.0, self._rot_since_fix - base)

    def _on_joints(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self._joint_pos[n] = p

    # --------------------------------------------------------------------- arm

    def _arm_present(self) -> bool:
        return all(j in self._joint_pos for j in self._arm_pubs)

    def _arm_poses(self, param: str) -> list:
        flat = list(self.get_parameter(param).value)
        n = len(self._arm_pubs)
        return [flat[k:k + n] for k in range(0, len(flat) - n + 1, n)]

    def _arm_start(self, poses: list):
        """Queue joint-space poses; _arm_step() drives through them."""
        self._arm_queue = list(poses) if self._arm_present() else []
        self._arm_target = None
        self._arm_reached = 0

    def _lip(self, frame: str):
        """Bucket-lip point (x, y, z) in `frame` from TF (joint states -> robot_state_publisher), or None."""
        try:
            t = self._tf_buffer.lookup_transform(frame, self.get_parameter('arm.bucket_link').value,
                                                 rclpy.time.Time())
        except TransformException:
            return None
        q, tr = t.transform.rotation, t.transform.translation
        px, py, pz = self.get_parameter('arm.bucket_lip').value
        # rotate the lip point by the quaternion, then translate
        ux, uy, uz = q.x, q.y, q.z
        cx, cy, cz = uy * pz - uz * py, uz * px - ux * pz, ux * py - uy * px
        cx, cy, cz = cx + q.w * px, cy + q.w * py, cz + q.w * pz
        rx = px + 2.0 * (uy * cz - uz * cy)
        ry = py + 2.0 * (uz * cx - ux * cz)
        rz = pz + 2.0 * (ux * cy - uy * cx)
        return tr.x + rx, tr.y + ry, tr.z + rz

    def _arm_step(self) -> str:
        """'running' | 'done' | 'error:<reason>'. Each pose must be reached
        (|error| < arm.pose_tolerance on every joint) within arm.pose_timeout_sec."""
        if self._arm_target is None:
            if not self._arm_queue:
                return 'done'
            self._arm_target = self._arm_queue.pop(0)
            self._arm_t0 = self._now_s()
        for (j, pub), q in zip(self._arm_pubs.items(), self._arm_target):
            pub.publish(Float64(data=float(q)))
        tol = self.get_parameter('arm.pose_tolerance').value
        if all(abs(self._joint_pos.get(j, 1e9) - q) < tol for j, q in zip(self._arm_pubs, self._arm_target)):
            self._arm_target = None
            self._arm_reached += 1
            return 'running' if self._arm_queue else 'done'
        if self._now_s() - self._arm_t0 > self.get_parameter('arm.pose_timeout_sec').value:
            err = {j: round(self._joint_pos.get(j, float('nan')) - q, 3)
                   for j, q in zip(self._arm_pubs, self._arm_target)}
            return f'error:arm did not reach {self._arm_target} (joint errors {err})'
        return 'running'

    # --------------------------------------------------------------------- legs

    def _start_leg(self, state: State, reverse_first: bool = False):
        leg = _LEG_OF_STATE[state]
        goal = tuple(self.get_parameter(_GOAL_PARAM[leg]).value)
        self._leg = {'name': leg, 'candidates': ml.candidate_goals(leg, goal, self._zones, self._fp),
                     'cand': 0, 'attempt': 0, 'phase': 'stow', 'retry_at': None, 'reverse_first': reverse_first}
        # later cycles start from the excavation goal that worked last time (seed 10: the
        # primary goal touches the excavation boulder and failed NO_VALID_PATH every cycle)
        if leg == 'excavation' and self._last_dig_cand is not None:
            self._leg['cand'] = self._last_dig_cand
        self._arm_start(self._arm_poses('arm.stow'))
        self._set_state(state, f'goal {self._leg["candidates"][0]}')

    def _nav2_ready(self) -> bool:
        if self._nav_active:
            return True
        if self._nav_query is None and self._bt_state.service_is_ready():
            self._nav_query = self._bt_state.call_async(GetState.Request())
        elif self._nav_query is not None and self._nav_query.done():
            res = self._nav_query.result()
            self._nav_active = res is not None and res.current_state.id == LifecycleState.PRIMARY_STATE_ACTIVE
            self._nav_query = None
        return self._nav_active

    def _send_leg_goal(self):
        leg = self._leg
        if not self._nav2_ready():
            if self._elapsed() > self.get_parameter('nav2_ready_timeout_sec').value:
                self._fail('Nav2 (bt_navigator) did not become active')
            else:
                self.get_logger().info('Waiting for Nav2 to become active...', throttle_duration_sec=10.0)
            return
        if self._goal_blocked(leg['candidates'][leg['cand']]):
            idx = self._next_free_candidate(leg)
            if idx != leg['cand'] and not self._goal_blocked(leg['candidates'][idx]):
                note = f'{leg["candidates"][leg["cand"]]} blocked in the costmap, using {leg["candidates"][idx]}'
                self.get_logger().warn(f'[{leg["name"]}] goal {note}')
                self._history.append([leg['name'], 'skip_blocked_goal', note])
                leg['cand'] = idx
        gx, gy, gyaw = leg['candidates'][leg['cand']]
        if not self._nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('navigate_to_pose action server not available yet.', throttle_duration_sec=5.0)
            return
        leg['attempt'] += 1
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation.z = math.sin(gyaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(gyaw / 2.0)
        self._nav_result = None
        self._goal_handle = None
        self._goal_seq += 1
        leg['phase'] = 'navigating'
        leg['sent_at'] = self._now_s()
        leg['blocked_checks'] = 0
        leg['next_block_check'] = self._now_s() + 1.0
        leg['final_turn_done'] = False
        self.get_logger().info(f'[{leg["name"]}] attempt {leg["attempt"]}/{self._max_attempts}: '
                               f'goal ({gx:.2f}, {gy:.2f}, yaw {math.degrees(gyaw):.0f} deg)')
        seq = self._goal_seq
        self._nav_client.send_goal_async(goal).add_done_callback(lambda f: self._on_goal_response(f, seq))

    def _on_goal_response(self, future, seq):
        # responses/results of a goal that was cancelled or replaced are ignored
        if seq != self._goal_seq:
            return
        handle = future.result()
        if not handle.accepted:
            self._nav_result = (GoalStatus.STATUS_ABORTED, ml.NONE)
            return
        self._goal_handle = handle

        def on_result(f):
            if seq == self._goal_seq:
                self._nav_result = (f.result().status, f.result().result.error_code)
        handle.get_result_async().add_done_callback(on_result)

    def _goal_blocked(self, goal) -> bool:
        """The planner's own goal test: a lethal cell under the padded footprint at the goal."""
        return self._costmap is not None and bool(ml.lethal_cells(
            self._costmap, goal, self._fp, pad=self.get_parameter('mission.footprint_padding').value))

    def _run_leg(self, on_success):
        leg = self._leg
        if leg['phase'] == 'stow':
            arm = self._arm_step()
            if arm.startswith('error'):
                return self._fail(f'[{leg["name"]}] {arm[6:]}')
            if arm == 'done':
                leg['phase'] = 'send'
                if leg.pop('reverse_first', False):
                    self._start_reverse(leg)
            return
        if leg['phase'] == 'reverse_align':
            return self._reverse_align_step(leg)
        if leg['phase'] == 'reverse':
            return self._reverse_step(leg)
        if leg['phase'] == 'escape':
            return self._escape_step(leg)
        if leg['phase'] == 'final_turn':
            return self._final_turn_step(leg)
        if leg['phase'] == 'wait_retry':
            if self._now_s() >= leg['retry_at']:
                leg['phase'] = 'send'
            return
        if leg['phase'] == 'send':
            return self._send_leg_goal()
        if leg['phase'] == 'scan':
            return self._scan_step(leg)
        # navigating
        if self._nav_result is None and not leg['final_turn_done'] and self._start_final_turn(leg):
            return
        if self._nav_result is None and self._start_scan(leg):
            return
        if self._nav_result is None and self._now_s() >= leg['next_block_check']:
            # A goal that became blocked (an obstacle sensed next to it) makes Nav2
            # replan and run recoveries until it gives up; end the attempt instead,
            # as GOAL_OCCUPIED, so the next try uses a free alternate goal.
            leg['next_block_check'] = self._now_s() + 1.0
            leg['blocked_checks'] = leg['blocked_checks'] + 1 if self._goal_blocked(
                leg['candidates'][leg['cand']]) else 0
            if leg['blocked_checks'] >= 2:
                self.get_logger().warn(f'[{leg["name"]}] goal {leg["candidates"][leg["cand"]]} is now blocked '
                                       'in the costmap; cancelling it.')
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()
                self._goal_seq += 1
                self._nav_result = (GoalStatus.STATUS_ABORTED, ml.GOAL_OCCUPIED)
        timed_out = self._now_s() - leg['sent_at'] > self.get_parameter('mission.leg_timeout_sec').value
        if self._nav_result is None and not timed_out:
            return
        if self._nav_result is None:          # timeout: cancel and count the attempt
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self._goal_seq += 1
            status, code = GoalStatus.STATUS_CANCELED, ml.NONE
        else:
            status, code = self._nav_result
        self._cmd_vel_pub.publish(Twist())
        goal = leg['candidates'][leg['cand']]
        pose = self._pose()
        arrived = pose is not None and ml.arrival_ok(
            goal, pose, self.get_parameter('mission.arrival_xy_tolerance').value,
            self.get_parameter('mission.arrival_yaw_tolerance').value)
        in_zone = pose is not None and ml.zone_ok(leg['name'], pose, self._fp, self._zones,
                                                  cross_margin=self.get_parameter('mission.zone_cross_margin').value)
        d = ml.decide_nav_result(status == GoalStatus.STATUS_SUCCEEDED, code, arrived, in_zone,
                                 leg['attempt'], self._max_attempts, timed_out=self._nav_result is None)
        pose_txt = f'({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.0f} deg)' if pose else 'unknown'
        if d.action == 'success':
            self.get_logger().info(f'[{leg["name"]}] ARRIVED (verified): pose {pose_txt}')
            self._history.append([leg['name'], 'arrived', pose_txt])
            return on_success()
        self.get_logger().warn(f'[{leg["name"]}] {d.reason}; pose {pose_txt}')
        self._history.append([leg['name'], d.action, d.reason])
        if d.action == 'fail':
            if leg['name'] != 'return' and self._cycles:
                return self._abandon_cycles(leg['name'], d.reason)
            return self._fail(f'leg "{leg["name"]}" failed: {d.reason}')
        if d.action == 'retry_alternate':
            leg['cand'] = self._next_free_candidate(leg)
        leg['phase'] = 'wait_retry'
        leg['retry_at'] = self._now_s() + 3.0      # let the costmap settle before retrying
        if self._nav_result is not None and code == ml.START_OCCUPIED and pose is not None:
            self._start_escape(leg, pose)

    def _start_final_turn(self, leg) -> bool:
        goal, pose = leg['candidates'][leg['cand']], self._pose()
        if pose is None or math.hypot(pose[0] - goal[0], pose[1] - goal[1]) > \
                self.get_parameter('mission.final_turn_radius').value:
            return False
        if abs(ml.angle_diff(goal[2], pose[2])) <= self.get_parameter('mission.final_turn_min_yaw').value:
            return False
        if self._costmap is None or not ml.turn_in_place_safe(
                self._costmap, pose, goal[2], self._fp, self.get_parameter('mission.footprint_padding').value):
            return False
        leg['final_turn_done'] = True
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_seq += 1
        txt = f'at ({pose[0]:.2f}, {pose[1]:.2f}), {math.degrees(ml.angle_diff(goal[2], pose[2])):.0f} deg to go'
        self.get_logger().info(f'[{leg["name"]}] near the goal: turning in place to its heading ({txt}).')
        self._history.append([leg['name'], 'final_turn', txt])
        leg['phase'] = 'final_turn'
        leg['turn_t0'] = self._now_s()
        return True

    def _final_turn_step(self, leg):
        """Turn to the goal heading closed-loop, then let the normal arrival check decide."""
        goal, pose, twist = leg['candidates'][leg['cand']], self._pose(), Twist()
        timed_out = self._now_s() - leg['turn_t0'] > self.get_parameter('mission.final_turn_timeout_sec').value
        if pose is not None and not timed_out:
            err = ml.angle_diff(goal[2], pose[2])
            if abs(err) > math.radians(3.0):
                twist.angular.z = math.copysign(min(0.35, max(0.1, abs(err))), err)
        self._cmd_vel_pub.publish(twist)
        if twist.angular.z == 0.0:
            leg['phase'] = 'navigating'
            self._nav_result = (GoalStatus.STATUS_SUCCEEDED, ml.NONE)   # verified by arrival + zone checks

    def _start_escape(self, leg, pose):
        pad = self.get_parameter('mission.footprint_padding').value
        # Lethal cells under the start: clear them with the smallest motion. None
        # (the planner refused a start this costmap copy shows free): move to the
        # clearest reachable pose instead of a token 5 deg turn.
        objective = 'smallest' if self._costmap is not None and ml.lethal_cells(
            self._costmap, pose, self._fp, pad) else 'clearest'
        motion = None if self._costmap is None else ml.escape_motion(
            self._costmap, pose, self._fp, pad=pad, objective=objective)
        if motion is None and self._costmap is not None:
            motion = ml.escape_motion(self._costmap, pose, self._fp, pad=pad, objective=objective,
                                      max_drive=self.get_parameter('escape_long_drive').value)
        if motion is None:
            self.get_logger().warn(f'[{leg["name"]}] START_OCCUPIED: no collision-free escape motion found.')
            self._history.append([leg['name'], 'escape', 'none found'])
            return
        kind, amount = motion
        txt = f'turn {math.degrees(amount):.0f} deg' if kind == 'turn' else f'drive {amount:+.2f} m'
        self.get_logger().warn(f'[{leg["name"]}] START_OCCUPIED: escaping with {txt} before retrying.')
        self._history.append([leg['name'], 'escape', txt])
        leg['phase'] = 'escape'
        leg['escape'] = {'kind': kind, 'amount': amount, 'start': pose, 't0': self._now_s()}

    def _escape_step(self, leg):
        """Drive the escape motion closed-loop on the localized pose; then wait and retry."""
        e, pose, twist = leg['escape'], self._pose(), Twist()
        timed_out = self._now_s() - e['t0'] > self.get_parameter('escape_timeout_sec').value
        if pose is not None and not timed_out:
            if e['kind'] == 'turn':
                err = ml.angle_diff(e['start'][2] + e['amount'], pose[2])
                if abs(err) > math.radians(2.0):
                    twist.angular.z = math.copysign(min(0.3, max(0.1, abs(err))), err)
            else:
                sx, sy, syaw = e['start']
                done = (pose[0] - sx) * math.cos(syaw) + (pose[1] - sy) * math.sin(syaw)
                if abs(e['amount'] - done) > 0.01:
                    twist.linear.x = math.copysign(0.08, e['amount'] - done)
        self._cmd_vel_pub.publish(twist)
        if twist.angular.z == 0.0 and twist.linear.x == 0.0:
            if timed_out:
                self.get_logger().warn(f'[{leg["name"]}] escape motion timed out.')
            leg['phase'] = 'wait_retry'
            leg['retry_at'] = self._now_s() + 3.0

    def _start_scan(self, leg) -> bool:
        g = self.get_parameter
        offsets = list(g('mission.scan_offsets_deg').value)
        pose = self._pose()
        if not offsets or pose is None or not ml.scan_due(pose, leg.get('last_scan'), g('mission.scan_spacing').value,
                                                          tuple(g('zones.obstacle_zone').value)):
            return False
        leg['last_scan'] = (pose[0], pose[1])
        heads = [pose[2] + math.radians(o) for o in offsets] + [pose[2]]
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_seq += 1
        self._cmd_vel_pub.publish(Twist())
        leg['phase'] = 'scan'
        leg['scan'] = {'heads': heads, 'k': 0, 'hold_since': None, 't0': self._now_s(), 'skipped': 0}
        self._history.append([leg['name'], 'scan', f'at ({pose[0]:.2f}, {pose[1]:.2f})'])
        return True

    def _scan_step(self, leg):
        sc, pose, twist = leg['scan'], self._pose(), Twist()
        g = self.get_parameter
        done = sc['k'] >= len(sc['heads']) or self._now_s() - sc['t0'] > g('mission.scan_timeout_sec').value
        if not done and pose is not None:
            target = sc['heads'][sc['k']]
            if sc['hold_since'] is None and abs(ml.angle_diff(target, pose[2])) > math.radians(3.0):
                if sc.get('checked') != sc['k']:
                    sc['checked'] = sc['k']
                    if self._costmap is None or not ml.turn_in_place_safe(
                            self._costmap, pose, target, self._fp, g('mission.footprint_padding').value):
                        sc['k'] += 1                    # this heading would sweep into an obstacle: skip it
                        sc['skipped'] += 1
                        return self._cmd_vel_pub.publish(twist)
                err = ml.angle_diff(target, pose[2])
                twist.angular.z = math.copysign(min(g('mission.survey_turn_rate').value, max(0.1, abs(err))), err)
                return self._cmd_vel_pub.publish(twist)
            self._cmd_vel_pub.publish(twist)
            if sc['hold_since'] is None:
                sc['hold_since'] = self._now_s()
            elif (self._now_s() - sc['hold_since'] >= g('mission.scan_hold_sec').value
                  or sc['k'] == len(sc['heads']) - 1):
                sc['k'] += 1
                sc['hold_since'] = None
            return
        self._cmd_vel_pub.publish(twist)
        if sc['skipped']:
            self._history.append([leg['name'], 'scan_partial', f'{sc["skipped"]} heading(s) not clear'])
        leg['attempt'] -= 1          # resuming the same goal is not a new attempt
        leg['phase'] = 'send'

    def _abandon_cycles(self, leg_name: str, reason: str):
        """A leg of a later cycle failed after work was done: record it and go home
        instead of ending the whole run as FAILED (sand already deposited stays counted)."""
        c = self._cycle
        if leg_name == 'construction' and c is not None and not c.get('t_end'):
            c['result'] = f'failed: construction leg failed, sand left in the bucket ({reason})'
            c['t_end'] = round(self._mission_elapsed(), 1)
            why = c['result']
        else:
            why = f'cycle {len(self._cycles) + 1} not started: {leg_name} leg failed ({reason})'
        self.get_logger().warn(f'{why}; returning to the start zone.')
        self._failed_action(f'{leg_name} leg', reason)
        self._history.append(['cycle', 'failed', why])
        self._cycles_left = 0
        self._start_leg(State.RETURN)

    def _start_reverse(self, leg):
        """Back straight up to the leg's goal if it lies behind and the sweep is clear;
        otherwise leave the leg to Nav2. The rover first turns in place to the goal
        heading (if that sweep is clear): backing up along an arrival heading 17 deg off
        left it 0.22 m beside the goal, and Nav2's re-approach stopped where the bucket
        would dig in the construction zone (seed 10, E1)."""
        if not leg.get('aligned') and self._goal_blocked(leg['candidates'][leg['cand']]):
            leg['cand'] = self._next_free_candidate(leg)
        pose, goal = self._pose(), leg['candidates'][leg['cand']]
        pad = self.get_parameter('mission.footprint_padding').value
        if pose is not None and not leg.get('aligned') and abs(ml.angle_diff(goal[2], pose[2])) > math.radians(3.0) \
                and ml.reverse_distance(pose, goal) is not None and self._costmap is not None \
                and ml.turn_in_place_safe(self._costmap, pose, goal[2], self._fp, pad):
            leg['aligned'] = True
            leg['phase'] = 'reverse_align'
            leg['turn_t0'] = self._now_s()
            turn = math.degrees(ml.angle_diff(goal[2], pose[2]))
            self._history.append([leg['name'], 'align', f'{turn:+.0f} deg'])
            return
        dist = None if pose is None else ml.reverse_distance(pose, goal)
        if dist is None or self._costmap is None or not ml.straight_drive_safe(
                self._costmap, pose, dist, self._fp, pad):
            why = ('no localized pose' if pose is None else 'goal not straight behind' if dist is None
                   else 'no costmap' if self._costmap is None else 'reverse sweep not clear in the costmap')
            self._history.append([leg['name'], 'reverse_skipped', why])
            leg['phase'] = 'send'
            return
        self.get_logger().info(f'[{leg["name"]}] backing up {-dist:.2f} m to the goal.')
        self._history.append([leg['name'], 'reverse', f'{dist:+.2f} m'])
        leg['phase'] = 'reverse'
        leg['reverse'] = {'start': pose, 'dist': dist, 't0': self._now_s()}

    def _reverse_align_step(self, leg):
        goal, pose, twist = leg['candidates'][leg['cand']], self._pose(), Twist()
        timed_out = self._now_s() - leg['turn_t0'] > self.get_parameter('mission.final_turn_timeout_sec').value
        if pose is not None and not timed_out:
            err = ml.angle_diff(goal[2], pose[2])
            if abs(err) > math.radians(2.0):
                twist.angular.z = math.copysign(min(0.35, max(0.1, abs(err))), err)
        self._cmd_vel_pub.publish(twist)
        if twist.angular.z == 0.0:
            self._start_reverse(leg)

    def _drive_straight_step(self, r: dict) -> bool:
        """Drive r['dist'] m along r['start'] heading, closed-loop on the localized pose; True when done."""
        pose, twist = self._pose(), Twist()
        timed_out = self._now_s() - r['t0'] > self.get_parameter('mission.reverse_timeout_sec').value
        if pose is not None and not timed_out:
            sx, sy, syaw = r['start']
            done = (pose[0] - sx) * math.cos(syaw) + (pose[1] - sy) * math.sin(syaw)
            if abs(r['dist'] - done) > 0.02:
                twist.linear.x = math.copysign(self.get_parameter('mission.reverse_speed').value, r['dist'] - done)
        self._cmd_vel_pub.publish(twist)
        if twist.linear.x == 0.0 and timed_out:
            self._failed_action('straight drive', f'timed out ({r["dist"]:+.2f} m)')
        return twist.linear.x == 0.0

    def _reverse_step(self, leg):
        if self._drive_straight_step(leg['reverse']):
            leg['phase'] = 'send'

    def _next_free_candidate(self, leg) -> int:
        """Next candidate with no inscribed cost under the footprint (clear margin);
        failing that, the next one the planner would accept; failing that, the next."""
        n = len(leg['candidates'])
        order = [(leg['cand'] + k) % n for k in range(1, n + 1)]
        for idx in order:
            if self._costmap is None or ml.footprint_cost_ok(self._costmap, leg['candidates'][idx], self._fp):
                return idx
        for idx in order:
            if not self._goal_blocked(leg['candidates'][idx]):
                return idx
        return order[0]

    # --------------------------------------------------------------------- fsm

    def _tick(self):
        self._track_rotation()
        if self._enabled:
            getattr(self, f'_handle_{self._state.name.lower()}')()

    def _handle_start(self):
        # Wait for the simulation + EKF (odom->base_footprint) before turning to
        # look for markers. Nav2 is NOT awaited here: its global costmap needs
        # the map frame, which only exists after the marker fix.
        try:
            self._tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return
        self.get_logger().info('Mission started. Localizing from the start-zone markers...')
        self._mission_t0 = self._now_s()
        self._set_state(State.LOCALIZE)

    def _handle_localize(self):
        if self._loc_status.get('initialized') and self._pose() is not None:
            self._cmd_vel_pub.publish(Twist())
            pose = self._pose()
            self.get_logger().info(f'Localized from markers: ({pose[0]:.2f}, {pose[1]:.2f}, '
                                   f'{math.degrees(pose[2]):.0f} deg), {self._loc_status.get("fixes", 0)} fixes.')
            return self._start_survey(pose)
        if self._elapsed() > self.get_parameter('localize_timeout_sec').value:
            return self._fail('no marker fix within localize_timeout_sec')
        # Turn in steps: rotate 1.2 s, then hold still 1.0 s. marker_localizer
        # ignores frames taken while turning (image/odometry timing errors), so
        # the pauses are when the markers are actually measured.
        twist = Twist()
        if self._elapsed() % 2.2 < 1.2:
            twist.angular.z = float(self.get_parameter('localize_spin_rate').value)
        self._cmd_vel_pub.publish(twist)

    def _start_survey(self, pose):
        heads = [math.radians(h) for h in self.get_parameter('mission.survey_headings').value]
        if not heads:
            return self._start_leg(State.TRAVEL_TO_EXCAVATION)
        # sweep from the end nearer the current heading
        if abs(ml.angle_diff(heads[-1], pose[2])) < abs(ml.angle_diff(heads[0], pose[2])):
            heads = heads[::-1]
        self._survey = {'heads': heads, 'k': 0, 'hold_since': None}
        self._set_state(State.SURVEY, f'{len(heads)} headings')

    def _handle_survey(self):
        sv, pose, twist = self._survey, self._pose(), Twist()
        if self._elapsed() > self.get_parameter('mission.survey_timeout_sec').value:
            self.get_logger().warn('Survey timed out; continuing with what was seen.')
            self._cmd_vel_pub.publish(twist)
            return self._start_leg(State.TRAVEL_TO_EXCAVATION)
        if pose is None:
            return self._cmd_vel_pub.publish(twist)
        err = ml.angle_diff(sv['heads'][sv['k']], pose[2])
        if abs(err) > math.radians(3.0) and sv['hold_since'] is None:
            rate = self.get_parameter('mission.survey_turn_rate').value
            twist.angular.z = math.copysign(min(rate, max(0.1, abs(err))), err)
            return self._cmd_vel_pub.publish(twist)
        self._cmd_vel_pub.publish(twist)
        if sv['hold_since'] is None:
            sv['hold_since'] = self._now_s()
        elif self._now_s() - sv['hold_since'] >= self.get_parameter('mission.survey_hold_sec').value:
            sv['k'] += 1
            sv['hold_since'] = None
            if sv['k'] >= len(sv['heads']):
                self._history.append(['survey', 'done', f'{len(sv["heads"])} headings'])
                return self._start_leg(State.TRAVEL_TO_EXCAVATION)

    def _handle_travel_to_excavation(self):
        def arrived():
            self._last_dig_cand = self._leg['cand']
            if self._rot_base is None:
                self._rot_base = self._rot_since_fix
            n = len(self._cycles) + 1
            self._cycle = {'cycle': n, 't_start': round(self._mission_elapsed(), 1), 'dig_attempts': 0,
                           'scoop_l': 0.0, 'dump_l': 0.0, 'in_target': None, 'ok': False, 'result': 'digging',
                           'rot_start': self._rot_since_fix}
            self._cycles.append(self._cycle)
            self._start_dig()
        self._run_leg(arrived)

    def _start_dig(self):
        c = self._cycle
        c['dig_attempts'] += 1
        self._cut_lip = None
        self._scoop_ev = None
        self._dig_move = None
        # Arrival tolerance can leave the dig-pose lip at/over the excavation zone's
        # far edge: nudge straight along the heading first (costmap-checked).
        pose = self._pose()
        if pose is not None:
            g = self.get_parameter
            fwd = g('arm.dig_lip_forward').value
            d = ml.dig_correction(pose, fwd, tuple(g('zones.excavation_zone').value),
                                  g('mission.excavation_approach').value[0] + fwd)
            if d is None:
                self._history.append(['excavation', 'reposition_skipped', 'no short drive puts the lip in the zone'])
            elif abs(d) > 0.02:
                if self._costmap is not None and ml.straight_drive_safe(
                        self._costmap, pose, d, self._fp, g('mission.footprint_padding').value):
                    self._dig_move = {'start': pose, 'dist': d, 't0': self._now_s()}
                    self._history.append(['excavation', 'reposition', f'{d:+.2f} m'])
                    self.get_logger().info(f'Cycle {c["cycle"]}: repositioning {d:+.2f} m so the bucket digs '
                                           'inside the excavation zone.')
                else:
                    self._history.append(['excavation', 'reposition_skipped', 'sweep not clear in the costmap'])
        if self._dig_move is None:
            self._arm_start(self._arm_poses('arm.dig_sequence'))
        self._set_state(State.EXCAVATION, f'cycle {c["cycle"]}: dig pass {c["dig_attempts"]}')

    def _handle_excavation(self):
        c = self._cycle
        if self._dig_move is not None:
            if self._drive_straight_step(self._dig_move):
                self._dig_move = None
                self._arm_start(self._arm_poses('arm.dig_sequence'))
            return
        arm = self._arm_step()
        if arm.startswith('error'):
            c['result'] = 'arm error while digging'
            return self._fail(f'excavation: {arm[6:]}')
        g = self.get_parameter
        if self._cut_lip is None and self._arm_reached > g('material.cut_pose_index').value:
            self._cut_lip = (self._lip('map'), self._lip('base_footprint'))
        if self._scoop_ev is None and self._arm_reached > g('material.curl_pose_index').value:
            lip_map, lip_base = self._cut_lip if self._cut_lip else (None, None)
            self._scoop_ev = self._material.scoop(
                f'cycle{c["cycle"]}-dig{c["dig_attempts"]}', lip_map or (float('nan'),) * 3,
                pose_reached=lip_map is not None, t=self._mission_elapsed())
            if not self._scoop_ev.accepted and lip_map is None:
                self._scoop_ev.reason = 'bucket lip not available from TF at the cut pose'
            self._scoop_ev.detail['lip_base'] = [round(v, 3) for v in lip_base] if lip_base else None
            self._payload_pub.publish(Float32(data=self._material.in_bucket_m3 * 1000.0))
            self._publish_status()
        if arm != 'done':
            return
        ev = self._scoop_ev
        if ev is None:      # no arm (or a sequence shorter than the curl index): nothing was dug
            ev = self._material.scoop(f'cycle{c["cycle"]}-dig{c["dig_attempts"]}', (float('nan'),) * 3,
                                      pose_reached=False, t=self._mission_elapsed())
            ev.reason = 'arm not present or dig sequence incomplete'
        if ev.accepted:
            c['scoop_l'] = round(ev.volume_m3 * 1000.0, 3)
            self.get_logger().info(f'Cycle {c["cycle"]}: scooped {c["scoop_l"]:.3f} L (SIMULATED estimate) at lip '
                                   f'({ev.lip[0]:.2f}, {ev.lip[1]:.2f}, {ev.lip[2]:.3f}).')
            self._history.append(['excavation', 'scooped', f'{c["scoop_l"]:.3f} L (simulated)'])
            return self._start_leg(State.TRAVEL_TO_CONSTRUCTION)
        self.get_logger().warn(f'Cycle {c["cycle"]}: scoop NOT credited: {ev.reason}')
        self._failed_action(f'cycle {c["cycle"]} scoop {c["dig_attempts"]}', ev.reason)
        self._history.append(['excavation', 'scoop_rejected', ev.reason])
        if c['dig_attempts'] < self.get_parameter('mission.max_dig_attempts').value:
            return self._start_dig()
        c['result'] = f'failed: no sand collected ({ev.reason})'
        c['t_end'] = round(self._mission_elapsed(), 1)
        self._cycles_left = 0
        self._history.append(['cycle', 'failed', c['result']])
        self._start_leg(State.RETURN)

    def _handle_travel_to_construction(self):
        def arrived():
            self._pour_ev = None
            self._arm_start(self._arm_poses('arm.dump_sequence'))
            self._set_state(State.DUMP, f'cycle {self._cycle["cycle"]}: pouring')
        self._run_leg(arrived)

    def _handle_dump(self):
        c = self._cycle
        arm = self._arm_step()
        if arm.startswith('error'):
            c['result'] = 'arm error while dumping'
            return self._fail(f'dump: {arm[6:]}')
        if self._pour_ev is None and self._arm_reached > self.get_parameter('material.pour_pose_index').value:
            lip_map, lip_base = self._lip('map'), self._lip('base_footprint')
            self._pour_ev = self._material.dump(f'cycle{c["cycle"]}', lip_map or (float('nan'),) * 3,
                                                pose_reached=lip_map is not None, t=self._mission_elapsed())
            self._pour_ev.detail['lip_base'] = [round(v, 3) for v in lip_base] if lip_base else None
            self._payload_pub.publish(Float32(data=self._material.in_bucket_m3 * 1000.0))
            self._publish_status()
        if arm != 'done':
            return
        ev = self._pour_ev
        c['t_end'] = round(self._mission_elapsed(), 1)
        if ev is not None and ev.accepted:
            c['dump_l'] = round(ev.volume_m3 * 1000.0, 3)
            c['in_target'] = bool(ev.detail.get('in_target'))
            c['ok'] = c['in_target']
            c['result'] = 'ok: deposited in the berm target' if c['ok'] else 'deposited OUTSIDE the berm target'
            self.get_logger().info(f'Cycle {c["cycle"]}: poured {c["dump_l"]:.3f} L (SIMULATED estimate) at lip '
                                   f'({ev.lip[0]:.2f}, {ev.lip[1]:.2f}); {c["result"]}.')
            self._history.append(['dump', 'poured', f'{c["dump_l"]:.3f} L, {c["result"]}'])
            if not c['ok']:
                self._failed_action(f'cycle {c["cycle"]} dump', c['result'])
        else:
            why = ev.reason if ev is not None else 'arm not present or dump sequence incomplete'
            c['result'] = f'failed: dump not credited ({why})'
            self.get_logger().warn(f'Cycle {c["cycle"]}: {c["result"]}')
            self._failed_action(f'cycle {c["cycle"]} dump', why)
            self._history.append(['dump', 'dump_rejected', why])
        # a later cycle also pays for backing up and re-approaching: time it from the previous dump
        prev = [x['t_end'] for x in self._cycles[:-1] if 't_end' in x]
        c['duration_sec'] = round(c['t_end'] - (prev[-1] if prev else c['t_start']), 1)
        if not c['ok']:
            self._cycles_left = 0
            self._history.append(['cycle', 'failed', c['result']])
            return self._start_leg(State.RETURN)
        durations = [x['duration_sec'] for x in self._cycles if x.get('ok')]
        estimate = max(durations + [0.0])
        if len(self._cycles) == 1:     # the first cycle had no back-up/realign; allow for it
            estimate = max(estimate, self.get_parameter('mission.first_cycle_estimate_sec').value)
        self._rot_at_cycle_end.append(self._rot_since_fix)
        k = self.get_parameter('mission.drift_per_rad').value
        # rotation per cycle, dump to dump; the first cycle (which also crossed the obstacle
        # zone) only from its arrival in the excavation zone, + 1 rad for aligning/backing up
        rot_cycles = [b - a for a, b in zip(self._rot_at_cycle_end, self._rot_at_cycle_end[1:]) if b >= a]
        if not rot_cycles and self._rot_since_fix >= c.get('rot_start', 0.0):
            rot_cycles = [self._rot_since_fix - c.get('rot_start', 0.0) + 1.0]
        per_cycle = k * max(rot_cycles) if rot_cycles else 0.0
        c['drift_estimate_m'] = round(self._drift_estimate(), 3)
        more, why = ml.another_cycle(len(self._cycles), self._max_cycles, self._mission_elapsed(),
                                     self.get_parameter('mission.time_budget_sec').value, estimate,
                                     self.get_parameter('mission.return_reserve_sec').value,
                                     drift_now=self._drift_estimate(), drift_per_cycle=per_cycle,
                                     drift_max=self.get_parameter('mission.drift_max').value)
        self._cycles_left = self._max_cycles - len(self._cycles) if more else 0
        self._history.append(['cycle', 'next' if more else 'stop', why])
        self.get_logger().info(f'Cycle {c["cycle"]} done in {c["duration_sec"]:.0f} s; '
                               f'{"another cycle" if more else "returning"}: {why}')
        if more:
            return self._start_leg(State.TRAVEL_TO_EXCAVATION, reverse_first=True)
        self._start_leg(State.RETURN)

    def _handle_return(self):
        def arrived():
            self._set_state(State.COMPLETE, 'all legs verified')
        self._run_leg(arrived)

    def _log_summary(self):
        if self._summary_logged:
            return
        self._summary_logged = True
        s = self._run_summary()
        m = s['material']
        self.get_logger().info(
            f'RUN SUMMARY ({self._state.value}): {s["run_time_sec"]:.0f} s of {s["time_budget_sec"]:.0f} s; '
            f'cycles ok {s["cycles_ok"]}/{s["cycles_started"]}; SIMULATED sand: collected {m["collected_l"]:.2f} L, '
            f'in berm target {m["deposited_in_target_l"]:.2f} L, outside {m["deposited_outside_target_l"]:.2f} L, '
            f'left in bucket {m["in_bucket_l"]:.2f} L; BCP-mass estimate {s["bcp_mass_estimate"]}; '
            f'failed actions {len(s["failed_actions"])}')
        self._publish_status('final summary')

    def _handle_complete(self):
        if self._elapsed() < 0.15:
            self.get_logger().info(f'Mission complete. Legs: {self._history}')
        self._log_summary()

    def _handle_failed(self):
        self._cmd_vel_pub.publish(Twist())
        self._log_summary()


def main():
    rclpy.init()
    node = MissionStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
