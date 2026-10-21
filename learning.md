# Learning & Change Log — CAC 2027 Autonomous Rover Navigation (`cac_sim` & `rover_navigation`)

> **Purpose of this document**: This file explains how the codebase works, why specific behaviors happen in simulation, and records **every single code or configuration change** (even the smallest parameter tweak) along with the exact reasoning behind it.

---

## 1. Codebase Architecture & How the Autonomy Stack Works

The simulation (`cac_sim`) runs a complete autonomous rover pipeline in **ROS 2 Jazzy + Gazebo Harmonic** inside a $9\text{ m} \times 5\text{ m}$ arena divided into four zones along the X-axis:
- **Starting Zone** ($x \in [-4.5, -2.7]\text{ m}$): Contains 5 ArUco markers on the perimeter boards.
- **Obstacle Zone** ($x \in [-2.7, +2.0]\text{ m}$): Contains 3 boulders ($\varnothing\,0.30\text{–}0.40\text{ m}$) and 3 craters ($\varnothing\,0.40\text{–}0.50\text{ m}$ negative depressions).
- **Excavation Zone ("Collecting Part")** ($x \in [+2.0, +3.5]\text{ m}$): Contains diggable sand and 1 smaller excavation boulder ($\varnothing\,0.20\text{–}0.30\text{ m}$).
- **Construction Zone ("Dumping Part")** ($x \in [+3.5, +4.5]\text{ m}$): Contains the circular berm target at $(4.0, 0.0)\text{ m}$ with radius $r = 0.40\text{ m}$.

### Core Nodes (`cac_sim/cac_sim/`)
1. **`marker_localizer.py`**:
   - Detects the start-zone ArUco markers in `/camera/image` and solves joint multi-marker PnP (`SOLVEPNP_SQPNP`) to publish the `map -> odom` transform.
   - Ignores frames taken while turning faster than `0.25 rad/s` to avoid camera-odometry timestamp skew.
   - Between marker sightings (while in the obstacle/excavation/construction zones), the rover dead-reckons using `robot_localization` (`ekf_node`), which fuses wheel odometry (`/odom_raw`) and IMU yaw rate (`/imu`).
2. **`wall_filter.py`**:
   - Competition rules forbid using arena walls for navigation or collision avoidance. This node transforms `/scan` into the `map` frame and strips any LiDAR hit within `0.30 m` of the walls, as well as ground-plane LiDAR hits caused by chassis pitch, publishing `/scan_filtered`.
3. **`arena_boundary_publisher.py`**:
   - Publishes an a-priori static `OccupancyGrid` (`/arena_boundary`) with a `0.11 m` lethal keep-out band along the known $9 \times 5\text{ m}$ arena walls so the planner stays off the walls without sensing them.
4. **`crater_detector.py`**:
   - Subsamples the RGB-D depth cloud (`/camera/points`), fits a local least-squares ground plane in `base_footprint`, and marks 5 cm cells with points $> 1.5\text{ cm}$ below the plane plus their **occlusion shadow** (the hidden near slope of the crater bowl). Confirmed cells (seen in $\ge 2$ frames) are published on `/crater_cells`.
5. **`mission_state_machine.py` & `mission_logic.py`**:
   - Top-level finite state machine (`START -> LOCALIZE -> SURVEY -> TRAVEL_TO_EXCAVATION -> EXCAVATION -> TRAVEL_TO_CONSTRUCTION -> DUMP -> RETURN -> COMPLETE`).
   - Sends `NavigateToPose` goals to Nav2, monitors `/global_costmap/costmap` for blocked goals, verifies arrival against both position/yaw tolerances and competition zone rules (`zone_ok`), sequences the 3-DOF excavator arm (`boom_joint`, `stick_joint`, `bucket_joint`), and credits sand volumes through `material_model.py`.

---

## 2. Root-Cause Diagnosis of the Two Issues You Observed

### Issue A: Why the rover was "taking the long path instead of the actual shortest path"
In `cac_sim/config/nav2_params.yaml`:
1. **Slow inflation decay (`cost_scaling_factor: 1.5`) + Large inflation radius (`0.85 m`)**:
   - In Nav2's `InflationLayer`, the cost of a cell at distance $d$ from a lethal obstacle cell (for $d > r_{\text{inscribed}} = 0.43\text{ m}$) is:
     $$\text{cost}(d) = 252 \cdot \exp\big(-\text{cost\_scaling\_factor} \cdot (d - 0.43)\big)$$
   - With `cost_scaling_factor: 1.5`, cost stays high ($134\text{–}225$) all the way out to $0.85\text{ m}$ from every boulder and crater.
2. **Excessive planner cost penalty (`cost_penalty: 4.0` in `SmacPlannerLattice`)**:
   - `SmacPlannerLattice` already checks the **full rectangular footprint** (`1.13 m x 0.74 m` + `0.06 m` hard `footprint_padding`) at every pose, so the rover is already guaranteed not to collide with any obstacle.
   - However, `SmacPlannerLattice` multiplies the distance of every motion primitive step by:
     $$1 + \text{cost\_penalty} \cdot \frac{\text{cost}}{252}$$
   - With `cost_penalty: 4.0` and `cost_scaling_factor: 1.5`, driving straight through a clear $1.2\text{–}1.5\text{ m}$ gap between two obstacles in the center of the arena was penalized by **$3.5\times\text{ to }4.5\times$ its real distance** (making a direct $6\text{ m}$ path look like a $22\text{ m}$ path to A*!).
   - As a result, the planner rejected the direct shortest route through the middle and routed the rover on a long wide detour around the perimeter of the arena.

### Issue B: Why at the collecting part (excavation zone) it "just sweeps, or does random shit but doesn't move or make its way back"
Three interacting bugs caused this exact failure at the excavation zone:

1. **Fatal `NoneType` Crash in `_start_scan()` caused by `scan_offsets_deg: []`**:
   - When `scan_offsets_deg: []` (an empty YAML array) was set in `config/arena_params.yaml`, ROS 2's YAML parser (`rcl_yaml_param_parser` / `rclpy`) loaded the parameter value as **`None`** instead of `[]` when `declare_parameter('mission.scan_offsets_deg', [-50.0, 50.0])` was called.
   - In `mission_state_machine.py` line 659 (`_start_scan`):
     ```python
     offsets = list(g('mission.scan_offsets_deg').value)
     ```
   - ` _start_scan()` is called on the very first 0.1 s timer tick **after** `_send_leg_goal()` sends the `TRAVEL_TO_EXCAVATION` goal `(2.35, 0.0, 0.0)` to Nav2 (`leg['phase'] == 'navigating'`).
   - Calling `list(None)` threw an unhandled `TypeError: 'NoneType' object is not iterable`, **crashing `mission_state_machine` 0.1 seconds after sending the first excavation goal**!
   - Because Nav2 (`bt_navigator`) is a separate process, Nav2 kept driving the rover to the excavation zone `(2.35, 0.0, 0.0)` on its own. Once the rover reached the excavation zone ("the collecting part"), `mission_state_machine` was already dead—so nothing ever transitioned to `TRAVEL_TO_CONSTRUCTION` or `RETURN`!
2. **Excavation Boulder on `seed:=42` Overlapping the Primary Goal's Padding**:
   - On the default launch seed (`seed:=42`), `excavation_boulder_0` sits at `(2.64, -0.56)` with radius `0.132 m`.
   - At the primary excavation goal `(2.35, 0.0, 0.0)`, the rover's unpadded footprint clearance to `excavation_boulder_0` is `0.0564 m`—which is slightly less than `footprint_padding: 0.06 m`.
   - Because `excavation_boulder_0` is $> 4.0\text{ m}$ from the start zone, the LiDAR only detects it when the rover arrives at the excavation zone. At that moment:
     - If `mission_state_machine` died (from #1 above), Nav2's behavior tree endlessly replanned, failed with `GOAL_OCCUPIED`, and ran recovery backups/twitches in place at the excavation zone forever.
     - Even with `mission_state_machine` alive, `blocked_checks >= 2` cancelled the goal right as the rover arrived in the excavation zone and switched to an alternate candidate `0.5 m` sideways `(2.35, 0.5, 0.0)`. Because `SmacPlannerLattice` has `allow_reverse_expansion: false`, shifting `0.5 m` sideways from `(2.20, 0.0, 0.0)` forced the planner to attempt a 3-meter loop or spin into `excavation_boulder_0`!
3. **Unnecessary Multi-Cycle / Post-Reverse Re-Planning**:
   - `_reverse_step()` in `mission_state_machine.py` always set `leg['phase'] = 'send'` after finishing a straight backup, forcing Nav2 (`SmacPlannerLattice`) to plan a new path from $2\text{ cm}$ away (which causes lattice planners without reverse primitives to plan a $3\text{ m}$ loop if the goal is $2\text{ cm}$ behind the robot).
   - High in-place rotation speeds (`survey_turn_rate: 0.50` and `rotate_to_heading_angular_vel: 0.70`) exceeded the skid-steer rover's measured `0.35 rad/s` no-slip limit on sand, introducing lateral odometry drift before reaching the excavation zone.

---

## 3. Detailed Change Log

### Update 1 — 2026-10-21: Fix Mid-Path Scan Crash, Shortest-Path Planning, and Excavation/Return Flow

#### A. `cac_sim/cac_sim/mission_state_machine.py` (and `rover_navigation/cac_sim/cac_sim/mission_state_machine.py`)
1. **Lines 671–674 (`_start_scan`) — Fix `TypeError: 'NoneType' object is not iterable`**:
   - **Before**:
     ```python
     offsets = list(g('mission.scan_offsets_deg').value)
     ```
   - **After**:
     ```python
     raw_offsets = g('mission.scan_offsets_deg').value
     offsets = [o for o in (raw_offsets or []) if o != 0.0]
     ```
   - **Why**: Prevents `mission_state_machine` from crashing when `scan_offsets_deg` is `[]` (loaded as `None` by `rclpy`) or `[0.0]` (used in YAML to represent disabled scan offsets while preserving `DOUBLE_ARRAY` typing).
2. **Lines 847–849 (`_start_survey`) — Guard `survey_headings` against `None`/empty**:
   - **Before**:
     ```python
     heads = [math.radians(h) for h in self.get_parameter('mission.survey_headings').value]
     ```
   - **After**:
     ```python
     raw_heads = self.get_parameter('mission.survey_headings').value
     heads = [math.radians(h) for h in (raw_heads or [])]
     ```
   - **Why**: Ensures `survey_headings` also safely handles empty/null parameter values without raising a `TypeError`.
3. **Lines 534–554 (`_run_leg` blocked-goal check) — Accept arrival if already in zone and safe**:
   - **Before**: When `blocked_checks >= 2` detected a newly mapped lethal cell under the goal's padding, it unconditionally aborted with `GOAL_OCCUPIED` and forced a retry to a laterally shifted candidate—even if the rover had already arrived inside `arrival_xy_tolerance`, `arrival_yaw_tolerance`, and `zone_ok` on a collision-free pose.
   - **After**: Checks if the current localized `pose` already satisfies `ml.arrival_ok(...)`, `ml.zone_ok(...)`, and has no lethal cells under its padded footprint (`not ml.lethal_cells(...)`). If so, it cancels the Nav2 goal and immediately records `GoalStatus.STATUS_SUCCEEDED` so the rover proceeds directly to digging (`State.EXCAVATION`) instead of driving a 3-meter loop in the excavation zone.
   - **Why**: On `seed=42`, `excavation_boulder_0` touches the 6 cm padding of `(2.35, 0.0, 0.0)` (`0.0564 m` clearance), while the rover arriving at `(2.18–2.30, 0.0, 0.0)` is already safely inside the excavation zone and ready to dig.
4. **Lines 794–808 (`_reverse_step`) — Verify arrival immediately after straight reverse**:
   - **Before**: After backing straight up to the excavation goal, `_reverse_step` unconditionally set `leg['phase'] = 'send'`, invoking Nav2 (`SmacPlannerLattice`) when the rover was already at the goal.
   - **After**: Checks if `ml.arrival_ok(...)` and `ml.zone_ok(...)` are already satisfied at the end of the reverse drive; if so, sets `self._nav_result = (GoalStatus.STATUS_SUCCEEDED, ml.NONE)` directly without re-invoking Nav2.
   - **Why**: Prevents `SmacPlannerLattice` (which has no reverse primitives) from planning a 2.6–3.3 m loop when the rover stops a few centimeters past the goal after backing up.

#### B. `cac_sim/cac_sim/mission_logic.py` (and `rover_navigation/cac_sim/cac_sim/mission_logic.py`)
1. **Line 132 (`candidate_goals`) — Add `±0.25 m` lateral candidate goals before `±0.50 m`**:
   - **Before**:
     ```python
     for dy in (0.5, -0.5, 1.0, -1.0, 1.5, -1.5):
     ```
   - **After**:
     ```python
     for dy in (0.25, -0.25, 0.5, -0.5, 1.0, -1.0, 1.5, -1.5):
     ```
   - **Why**: When the primary goal `(2.35, 0.0, 0.0)` is blocked by an excavation boulder (as on `seed=42`), a `0.25 m` lateral shift `(2.35, 0.25, 0.0)` is completely clear (`0.31 m` boulder clearance), requires half the lateral deviation of `0.50 m`, allows a smooth forward transition to construction `(3.30, 0.0, 0.0)`, and stays within `reverse_distance`'s `max_lateral = 0.25 m` window.

#### C. `cac_sim/config/nav2_params.yaml` (and `rover_navigation/cac_sim/config/nav2_params.yaml`)
1. **`local_costmap` & `global_costmap` -> `inflation_layer.cost_scaling_factor`: `1.5` $\rightarrow$ `3.5`**:
   - **Why**: Causes inflation cost outside the padded inscribed radius (`0.43 m`) to decay more steeply so that $1.1\text{–}1.5\text{ m}$ corridors between obstacles have low traversal cost, allowing the global planner to take the direct shortest path through the arena instead of detouring around the perimeter.
2. **`planner_server` (`SmacPlannerLattice`) -> `cost_penalty`: `4.0` $\rightarrow$ `1.5`, `change_penalty`: `0.05` $\rightarrow$ `0.10`, `non_straight_penalty`: `1.05` $\rightarrow$ `1.15`**:
   - **Why**: Reduces the soft cost multiplier so `SmacPlannerLattice` chooses the actual shortest collision-free route while keeping paths straight and smooth. Hard collision safety (`footprint_padding: 0.06` on the full rectangular footprint) is completely unchanged.
3. **`controller_server` (`FollowPath` / `RegulatedPurePursuitController`) -> `desired_linear_vel`: `0.35` $\rightarrow$ `0.30`, `regulated_linear_scaling_min_speed`: `0.20` $\rightarrow$ `0.15`, `rotate_to_heading_angular_vel`: `0.7` $\rightarrow$ `0.35`**:
   - **Why**: Caps in-place heading rotations to the measured `0.35 rad/s` no-slip rate on sand (preventing sideways wheel slip and odometry drift during turns) and keeps forward tracking smooth.

#### D. `cac_sim/config/arena_params.yaml` (and `rover_navigation/cac_sim/config/arena_params.yaml`)
1. **`mission.scan_offsets_deg`: `[]` $\rightarrow$ `[0.0]`**:
   - **Why**: Gives `rcl_yaml_param_parser` a non-empty `DOUBLE_ARRAY` (`[0.0]`) so `rclpy` never parses the parameter as `None`, while `0.0` is treated as disabled by `_start_scan()` (`[o for o in (raw_offsets or []) if o != 0.0]`).
2. **`mission.survey_turn_rate`: `0.50` $\rightarrow$ `0.35` and `mission.survey_hold_sec`: `0.6` $\rightarrow$ `0.8`**:
   - **Why**: Restores the `0.35 rad/s` no-slip turn rate during the initial obstacle survey so the rover does not slide sideways and corrupt its marker localization before starting the first leg, and holds `0.8 s` so the 8 Hz crater detector gets at least 6 frames per heading.
3. **`mission.num_cycles`: `2` $\rightarrow$ `1` and `mission.reverse_speed`: `0.20` $\rightarrow$ `0.15`**:
   - **Why**: Ensures that as soon as the rover reaches the excavation zone, collects sand (`EXCAVATION`), drives to the construction zone (`TRAVEL_TO_CONSTRUCTION`), and dumps the load onto the berm (`DUMP`), it **immediately transitions to `RETURN`** and drives back to the starting zone to complete the mission cleanly.

#### E. `cac_sim/test/test_mission_cycles.py` (and `rover_navigation/cac_sim/test/test_mission_cycles.py`)
1. **Added `test_scan_stop_disabled_by_zero_offset`**:
   - **Why**: Unit-tests that setting `mission.scan_offsets_deg` to `[0.0]` disables mid-path scan stops without error and leaves the leg in `navigating` phase.

#### F. `rover_navigation/README.md`
1. **Updated "Performance & Fast-Mission Tuning" section (lines 36–49)**:
   - **Why**: Kept the repository README synchronized with the updated parameters (`scan_offsets_deg: [0.0]`, `cost_scaling_factor: 3.5`, `cost_penalty: 1.5`, `num_cycles: 1`, `desired_linear_vel: 0.30`, `rotate_to_heading_angular_vel: 0.35`, `survey_turn_rate: 0.35`, and `survey_hold_sec: 0.8`).
