# cac_sim — Caterpillar Autonomy Challenge 2027 simulation

ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8) simulation of the CAC 2027 arena and an
autonomous excavation rover: randomized boulders and real crater depressions,
marker-based localization that does not use the arena walls, Nav2 navigation
with the rover's real footprint, camera crater detection, and a mission state
machine that only advances on verified arrival and runs repeated
excavate → carry → dump cycles within the 10-minute run, with sand volumes
tracked by an accounting model (simulated estimates — Gazebo's sand is rigid).

```
ros2 launch cac_sim cac_sim.launch.py seed:=42            # GUI + RViz, full autonomy
```

Contents: [Build and run](#build-and-run) · [Arena](#arena) · [Rover](#rover) ·
[Sensors, topics, TF](#sensors-topics-and-tf) · [Autonomy stack](#autonomy-stack) ·
[Mission flow](#mission-state-machine) · [Excavation and dumping](#excavation-and-dumping-material-accounting) ·
[Testing](#testing) ·
[Rule interpretations](#competition-rules-and-interpretations) ·
[Known limitations](#known-limitations) · [Validation status](#validation-status)

---

## Build and run

`$WS` below is the colcon workspace root that contains `src/cac_sim` — e.g. the
root of the `Caterpillar_2027` clone, or `~/MOVIS` on the development machine.
Validation output goes to `$WS/suite_results/` (git-ignored).

```bash
export WS=~/Caterpillar_2027        # your workspace root
cd $WS
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cac_sim
source install/setup.bash

ros2 launch cac_sim cac_sim.launch.py seed:=6                      # GUI + RViz
ros2 launch cac_sim cac_sim.launch.py seed:=6 headless:=true use_rviz:=false   # server only
ros2 launch cac_sim cac_sim.launch.py autonomy:=false              # no mission (teleop/tests)
ros2 launch cac_sim cac_sim.launch.py randomize:=false             # empty arena, fixed start
```

| Argument | Default | Meaning |
|---|---|---|
| `seed` | 42 | Seeds the start pose and the obstacle layout (fully reproducible). |
| `randomize` | true | `false`: no obstacles, fixed start pose. |
| `autonomy` | true | `false`: the mission state machine idles. |
| `use_rviz` | true | Start RViz. |
| `headless` | false | Gazebo server only. Rendering sensors still need a display (`DISPLAY`). |
| `egl` | false | Adds `--headless-rendering`; on the development machine this segfaults the RGB-D camera (Ogre2). |
| `localhost_only` | true | Sets `GZ_IP=127.0.0.1` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (see below). |
| `run_dir` | `<tmp>/cac_sim_runs/seed_N` | Where the generated world, meshes and resolved Nav2 params go. |
| `world` | `cac_arena.sdf` | Base world (its sand bed is regenerated per run). |
| `material_visual` | true | Draw the simulated sand accounting in Gazebo (visual-only berm pile and dig marks, see below). |

**Loopback transport.** On a flaky Wi-Fi interface gz-transport logged thousands of
"multicast: Network is unreachable" errors, the ROS bridge stopped receiving
`/clock`, sim time froze and Nav2's bond watchdog shut navigation down. The launch
therefore keeps Gazebo transport and ROS discovery on 127.0.0.1. Any tool you run
from another shell (`ros2 topic`, `gz topic`, the test tools) needs the same:

```bash
export GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

Each run prints the generated layout, e.g.

```
[cac_sim] seed=6 start=(-3.38, -0.35, yaw=180 deg) attempts=1
  boulder_0  (+0.94, -0.06) d=0.379 centre_z=0.150
  crater_2   (-1.56, +0.58) d=0.480 depth=0.079 ...
```

`ros2 run cac_sim obstacle_spawner --print 1 2 3` prints layouts without starting anything.

---

## Arena

World frame (= ROS `map`): X along the 9 m length, start zone at −X; Y across the
5 m width; Z = 0 is the sand surface.

| Element | Geometry |
|---|---|
| Sand bed | 9 × 5 m, 0.30 m deep, rigid (no granular physics), μ = 0.9 |
| Walls | inner faces at x = ±4.50, y = ±2.50; 0.04 m thick, **0.80 m tall**; corners overlap; visual = collision |
| Start zone | x ∈ [−4.5, −2.7] |
| Obstacle zone | x ∈ [−2.7, 2.0] |
| Excavation zone | x ∈ [2.0, 3.5] |
| Construction zone | x ∈ [3.5, 4.5]; berm target (4.0, 0), r = 0.4 m (visual marker) |
| Zone lines | 3 cm painted strips at x = −2.7, 2.0, 3.5 (visual only) |
| Markers | 5 ArUco (DICT_4X4_50, 20 cm, on 25 cm boards) on the start-zone perimeter |

The rulebook text gives no zone sizes (only a reference diagram); the layout is in
`config/arena_params.yaml` and is the single source of truth for every node.

### Per-run generation (`cac_sim/arena_layout.py`)

The launch file generates the world **before** Gazebo starts:

1. **Start pose** — N/S/E/W heading (rulebook), footprint inside the start zone
   with 0.20 m margin, and room to turn in place (farthest corner + 0.10 m from
   walls and obstacles) for the marker-search spin.
2. **Obstacles** — 3 boulders (Ø 0.30–0.40 m, sunk 0–40 % of their radius:
   "varying heights") and 3 craters in the obstacle zone, plus 1 smaller boulder
   (Ø 0.20–0.30 m) in the excavation zone ("similar or smaller boulders may also be
   present"). Centres ≥ 0.9 m apart and ≥ 0.35 m inside their zone.
3. **Reachability check** — the real footprint, grown by 0.10 m (0.20 m from
   walls), must be able to drive from the start to the excavation, construction and
   return goals (search over x, y and 16 headings, with turning in place).
   Spacing alone does not guarantee this: with the original rules 7–10 % of
   layouts were unsolvable. Rejected layouts are redrawn deterministically
   (attempt k uses `Random(seed·1000003 + k)`; attempt 0 is the original stream,
   so seeds that already passed keep their layout).
4. **Craters as real depressions** — each crater is a closed STL mesh: a square
   sand patch with a cosine bowl (total width 0.40–0.50 m, depth 0.05–0.15 m), flush
   with the surface. The solid sand box is replaced by axis-aligned floor tiles
   around the patches (a box cannot have holes). Verified live: a 6 cm probe
   sphere settles at the bowl bottom (−0.115 m in a 0.117 m crater) and rests at
   +0.06 m on tiles and across tile/patch seams.

The old craters (a ring of 8 cm curbs plus a non-colliding disc buried inside the
solid sand) were physically drivable bumps and invisible to the depth camera; the
old full-zone coloured floor panels were removed because the depth camera rendered
them as an opaque floor over every crater.

---

## Rover

Stowed envelope 1.13 × 0.74 × 0.72 m, 50.5 kg (limits: 1.5 × 0.75 × 0.75 m, 80 kg;
`test_consistency.py`). Centre of mass 2.5 cm ahead of the wheel centre, 0.32 m high;
at rest the body pitches 1.3° nose-down on the rocker springs (measured,
`rover_physics_check.py`).

```
base_footprint (ground, between the wheels)
└─ base_link              chassis 0.90 × 0.50 × 0.20 m, underside 0.20 m above ground
   ├─ bin_link            (fixed) open-top sand bin, middle of the deck
   ├─ electronics_link    (fixed) enclosed electronics box, rear of the deck
   ├─ arm_mount_link      (fixed) front of the deck
   │  └─ boom_joint ─ boom_link ─ stick_joint ─ stick_link ─ bucket_joint ─ bucket_link
   ├─ left_rocker_joint ─ left_rocker_link
   │  ├─ front_left_wheel_joint ─ front_left_wheel_link
   │  └─ rear_left_wheel_joint  ─ rear_left_wheel_link
   ├─ right_rocker_joint ─ right_rocker_link
   │  ├─ front_right_wheel_joint ─ front_right_wheel_link
   │  └─ rear_right_wheel_joint  ─ rear_right_wheel_link
   ├─ lidar_link          (fixed) scan plane 0.10 m above ground
   ├─ imu_link            (fixed)
   └─ camera_link         (fixed) ─ camera_optical_link
```

Files: `urdf/rover_properties.xacro` (all numbers), `chassis.xacro`,
`suspension.xacro`, `wheels.xacro`, `arm.xacro`, `sensors.xacro`,
`gazebo_plugins.xacro`, assembled by `rover.urdf.xacro`.

### Suspension and drive

- **Rocker suspension:** one rocker per side pivots about the lateral axis on the
  chassis side (0.25 m above ground) and carries that side's front and rear wheel
  (wheelbase 0.70 m, track 0.66 m, wheel r = 0.12 m). Each rocker is an independent
  revolute joint (±0.30 rad) with a centring spring (300 N·m/rad) and damper; there
  is no differential bar, so the model stays a tree (no closed loop).
- **Skid-steer drive:** gz-sim's `DiffDrive` system velocity-commands all four
  wheel joints (two per side); `/cmd_vel` in, `/odom_raw` out. Verified live
  (`tools/system_check.py`): 0.2 m/s forward drives all four wheels at +1.67 rad/s;
  a 0.4 rad/s left turn drives the left pair at −1.1 and the right pair at +1.1 rad/s.
- **Wheel friction:** μ = 0.3 sideways, 1.0 rolling (equal friction cannot yaw in
  DART), plus **lateral slip compliance 0.003 m/s/N**. Without compliance the turn
  centre is indeterminate (equal wheel loads + Coulomb friction) and DART pivoted
  the rover about its front axle: base_footprint slid sideways at 0.18 m/s while
  spinning, which odometry cannot see. Measured after the fix: ≤ 0.007 m/s while
  spinning and 0.003 m/s in arcs. Faster in-place turns still slide: ~0.1 m/s at
  45–75 °/s (measured over 14 missions), so every in-place turn the stack commands
  is limited to 0.35 rad/s (mission search/survey/escape, Nav2 behaviors) and Nav2's
  Spin recovery is not used at all (see Autonomy stack).

### Excavator arm

Boom (0.36 m), stick (0.32 m) and bucket on three pitch joints, each held by a gz
`JointPositionController` (command `/arm/<joint>/cmd`, std_msgs/Float64, rad). The
boom pivots on a 5 cm pedestal above the deck (0.45 m above ground). Joint zero is
the **stowed** pose: boom 45° up, stick folded back and down in front of the boom,
bucket tucked against the stick, all above the camera's view and inside the
footprint's 0.66 m front. Poses (`config/arm_params.yaml`; bucket lip = point
(0.14, 0, −0.035) of `bucket_link`):

| Pose | boom, stick, bucket (rad) | Bucket lip |
|---|---|---|
| stow | 0, 0, 0 | over the deck front |
| carry | 0.30, −1.00, 2.10 | 1.02 m ahead, 0.43 m up, open side up |
| dig | 0.85, −1.25, 3.10 | 0.94 m ahead, 1.7 cm above the sand |
| scoop | 0.85, −1.25, 2.80 | 0.97 m ahead, 5 cm up (curl in) |
| pour | 0.30, −1.35, 0.90 | inverted, 0.91 m ahead, 0.58 m up |

`test/test_arm_sweep.py` runs forward kinematics of the generated URDF along both
mission sequences (stow → carry → dig → scoop → carry → stow and stow → carry →
pour → carry → stow, interpolated in joint space) and checks every step against the
chassis, bin, electronics box, wheels, rockers, sensors and the ground (1 cm
clearance; the bucket may reach 3 cm into the sand while digging), and that the
stowed arm is outside the camera's view. Gazebo does not self-collide links of one
model, so this test is what catches an arm passing through the body: the previous
arm geometry failed it (the stowed bucket sat ~8 cm inside the boom, and the boom
root passed ~3 cm through the chassis front when lowered, with its pivot only 2 cm
above the deck). Measured live with the current poses: joints settle within
0.08 rad, body tilt ≤ 3.2° during the sequences.
**No sand moves in Gazebo**: digging and dumping are kinematic arm motions; the
amount of sand is tracked by an accounting model whose volumes are simulated
estimates (see [Excavation and dumping](#excavation-and-dumping-material-accounting)).

---

## Sensors, topics and TF

| Sensor | Topic(s) | Frame | Notes |
|---|---|---|---|
| 2D LiDAR (360°, 10 Hz, 0.12–8 m) | `/scan` | `lidar_link` | scan plane 0.10 m; rover hidden from it via visibility flags; wall and ground returns removed in `/scan_filtered` |
| IMU (100 Hz) | `/imu` | `imu_link` | gyro used; no absolute yaw (no compass) |
| RGB-D camera (640×480, 15 Hz, 66° HFOV, 15° down) | `/camera/image`, `/camera/depth_image`, `/camera/points`, `/camera/camera_info` | `camera_link` (cloud: x forward) | marker detection + craters |
| Wheel odometry | `/odom_raw` | `odom` → `base_footprint` | from DiffDrive |
| Joints | `/joint_states` | — | wheels, rockers, arm |

Derived topics: `/scan_filtered` (wall returns removed), `/crater_cells`
(confirmed crater cells, map frame), `/arena_boundary` (a-priori boundary grid),
`/localization/status` (incl. frame/TF-failure counters), `/localization/candidates`
(one JSON line per marker detection: ids, implied pose, range, outcome — for
validation tools), `/mission_state`, `/mission_status` (JSON), `/payload_level` (simulated litres in the bucket),
`/arena/ground_truth_obstacles` (RViz only).

TF: `map → odom` (marker_localizer) → `base_footprint` (robot_localization EKF) →
`base_link` → sensor links (robot_state_publisher). Gazebo's own TF is not bridged.

---

## Autonomy stack

```
/camera/image ─► marker_localizer ─► map→odom (TF)        /scan + /imu ─► wall_filter
/odom_raw + /imu ─► EKF (robot_localization) ─► odom→base     (walls + ground returns
/camera/points ─► crater_detector ─► /crater_cells             removed) ─► /scan_filtered
arena_params ─► arena_boundary_publisher ─► /arena_boundary             │
                     │                          │                       │
                     ▼                          ▼                       ▼
            global + local costmaps: boundary (static) · obstacle (/scan_filtered)
                                     · crater (/crater_cells, never cleared) · inflation
                                                │
mission_state_machine ─ NavigateToPose ─► bt_navigator (no-spin BT, 2 Hz replan)
  LOCALIZE → SURVEY → legs                   ├─ planner_server: Smac Lattice (footprint-checked)
  goal checks vs /global_costmap/costmap     ├─ controller_server: Regulated Pure Pursuit
  escape / final turn ─► /cmd_vel            └─ collision_monitor ─► /cmd_vel ─► DiffDrive
```
TF tree: `map → odom → base_footprint → base_link → sensor links` (marker_localizer,
EKF, robot_state_publisher); Gazebo's TF is not bridged.

- **Localization (`marker_localizer.py`).** Replaces slam_toolbox. With wall returns
  removed a 2D LiDAR sees only three ~0.3 m boulders — far too little for scan
  matching — and the rulebook's answer is markers in the start area. Each ArUco
  frame with two or more markers is solved as ONE joint PnP over all their corners
  (map-frame object points; markers on different walls are not coplanar, which
  pins down yaw); a lone marker is solved with IPPE_SQUARE, keeping the solution
  consistent with the known camera height/tilt. Frames taken while turning faster
  than 0.25 rad/s are ignored. Initial fix = median of ≥ 3 multi-marker candidates
  (single-marker candidates only as a last resort, after 40 of them). After the
  initial fix, a detection is blended in (gain 0.3, with outlier gating; consistent
  outliers trigger re-initialisation, which also clears the costmaps and crater
  memory) only if it is accurate where it was taken — measured against Gazebo
  ground truth (`tools/marker_range_test.py`, the candidate log of `gt_monitor.py`):

  | Markers in view | Measured error | Used after the initial fix |
  |---|---|---|
  | 1 | 0.085 m / 5.3° at 0.5–1 m, 0.16 m at 2.5–3 m, 0.2–0.3 m beyond | never (`single_max_range` 0) |
  | 2 | 2–8 cm at 3.5 m, 0.15–0.35 m at 4–4.5 m | within 3.5 m (`pair_max_range`) |
  | ≥ 3 | 0.8 cm at 4–4.5 m | within 4.5 m (`multi_max_range`) |

  Single-marker updates (the old gain-0.08 blend) pulled the estimate 4° / 0.3–0.45 m
  off; without them the belief stayed within 2.7–4.2 cm / 0.4° over 5.7 m of driving. Between sightings the rover
  dead-reckons on the EKF (wheel velocities + gyro yaw). `map→odom` is stamped
  0.2 s ahead (as AMCL does) to avoid TF extrapolation errors.
- **Wall filter (`wall_filter.py`).** Every LiDAR hit within 0.30 m of the known
  arena boundary (in the localized map frame) becomes "no return" before the
  costmaps, the collision monitor and the perception node see it. Nothing is
  published until localization exists. It also removes **ground returns**: the scan
  plane is only 0.10 m up, so the 1.3° rest pitch (2–3° in the arm poses) puts the
  ground in the scan at 3–4.3 m; those hits formed arcs of phantom obstacles that
  closed every route (NO_VALID_PATH from everywhere). From the IMU's gravity-referenced
  roll/pitch the node computes each beam's ground-intersection range and drops
  returns beyond it minus 0.30 m. The gpu_lidar returns ground ~0.45° earlier than
  its beam axis (measured at rest: 3.21–3.29 m ahead where the axis predicts 4.35 m),
  which the model includes (`ground_beam_spread_deg`).
- **TF listeners.** `marker_localizer`, `wall_filter` and `crater_detector` look up
  transforms at each message's stamp, waiting up to 50 ms. With the listener on the
  node's own single-threaded executor the TF callbacks queued behind the sensor
  callback and the lookups failed (measured: 62 % of camera images, 55 % of scans,
  75–87 % of point clouds lost). Each now uses `TransformListener(buffer, None,
  spin_thread=True)`, i.e. its own node and thread (after: 1.7 %, 0 %, 0 %).
  Passing the node itself with `spin_thread=True` does not work: `rclpy.spin(node)`
  takes the node back into the main executor.
- **Boundary layer (`arena_boundary_publisher.py`).** A static grid, lethal within
  0.11 m of the walls (+ 0.06 m footprint padding = 0.17 m to a wall), loaded by both
  costmaps. It keeps the planner off the walls
  using the arena's known size instead of sensing the walls.
- **Crater detector (`crater_detector.py`).** Subsampled `/camera/points` →
  base frame → least-squares ground plane (absorbs chassis pitch) → points more
  than 1.5 cm below it within 0.3–2.2 m (2.5 cm missed most of 5–6 cm deep craters) → 5 cm map cells
  **+ their occlusion shadow** → confirmed after hits in 2 frames → `/crater_cells`.
  From the approach side the camera sees only a bowl's far wall; the near slope drops
  away behind the rim, so a 40 cm crater used to be mapped as 13–17 of its ~50 cells
  and a wheel crossed the unmapped slope (seed 12). Each frame therefore also marks the
  cells between a below-plane cell and the camera that it did not see as level ground,
  up to 0.45 m (the rulebook crater width), stopping at the first cell seen as ground
  (the near rim) or where the detector does not look (inside 0.3 m, in the rover's
  own box). In a ray-cast model of the URDF camera over the seed-12 bowl
  (`test_crater_detector.py`) this raised the mapped share of the part deeper than
  the threshold from 0.73 to 1.00 (head-on approach) and 0.55 to 0.82 (25° off), with
  no cell outside the crater. The costmaps' `crater_layer` is marking-only, so
  LiDAR rays passing over a crater can never clear it. A planar LiDAR cannot see a
  depression at all. Before the first marker fix there is no map frame; hits are
  then counted in odom cells and moved into the map with the first map→odom (seed 10:
  a crater was in view only during the marker search and used to be discarded).
- **Nav2 (`config/nav2_params.yaml`).** Both costmaps use the real footprint
  `[[0.66, ±0.37], [−0.47, ±0.37]]` with **0.06 m padding**, inflation 0.85 m
  (beyond the 0.76 m turn-in-place radius; cost_scaling_factor 1.5). Smac rejects
  only poses whose footprint touches a lethal cell, so the padding is the hard
  clearance: with 0.02 m, planned paths passed 1.7–4 cm from boulders
  (`tools/planner_route_test.py`, seed 10). The planner weighs cost more
  (cost_penalty 4) so a wide route beats a narrow gap when both exist. Behavior tree:
  Nav2's default replanning/recovery tree **without the Spin recovery**
  (`config/behavior_trees/navigate_to_pose_no_spin.xml`) — recovery spins turned the
  rover, parked between a boulder and the north wall, to face the wall, after which
  every start was refused; the LiDAR is 360° anyway. Behavior-server rotations are
  limited to 0.35 rad/s. Global planner: **Smac Lattice** with the shipped
  differential-drive primitives (full-footprint collision checks, turning in
  place); NavFn plans for a point and would route through gaps the body cannot turn
  in. Controller: Regulated Pure Pursuit (0.20 m/s, rotate-to-heading); the behavior tree
  replans at 2 Hz and the global costmap updates at 4 Hz — a shallow crater is
  confirmed only ~1.5 m ahead, so reaction distance matters. Collision
  monitor on `/scan_filtered`. BT action-ack timeout 200 ms (Nav2's 20 ms default
  produced error-code-0 failures under load).

---

## Mission state machine

```
START → LOCALIZE → SURVEY → TRAVEL_TO_EXCAVATION → EXCAVATION → TRAVEL_TO_CONSTRUCTION → DUMP
      → (back up → TRAVEL_TO_EXCAVATION → … while another cycle fits) → RETURN → COMPLETE
any state → FAILED (terminal, reason in /mission_status)
```

| Transition | Earned by |
|---|---|
| START → LOCALIZE | odom→base TF exists (simulation + EKF running) |
| LOCALIZE → SURVEY | marker fix (`/localization/status` initialized); turns in place in steps (0.35 rad/s for 1.2 s, pause 1.0 s; frames taken while turning are ignored) while searching; 90 s timeout → FAILED |
| SURVEY → first leg | faced each of −60, −30, 0, 30, 60° (map frame, toward the obstacle zone) and held still 1.2 s, so the depth camera has swept the ground next to the start zone (≈ ±93° at 1.0–2.2 m) and craters there are in the costmap before the first plan; 60 s timeout → continue. Without it, a crater 1.2 m from the start was seen only if the marker search happened to spin past it (an immediate marker fix → a wheel in the crater) |
| TRAVEL_* → task | Nav2 **SUCCEEDED** *and* localized pose within 0.30 m / 0.60 rad of the goal *and* the leg's zone condition holds (the localized footprint ≥ 0.10 m across the excavation/construction line — `zone_cross_margin`, against localization error — every corner ≥ 0.10 m from the east wall; return: centre in the start zone) |
| EXCAVATION → TRAVEL_TO_CONSTRUCTION | arm sequence reached each pose (within 0.10 rad, 10 s per pose) **and** the scoop was credited by the material model (lip in the excavation zone and at the sand at the cut pose); a rejected scoop is dug again once (`max_dig_attempts` 2), then the cycle fails and the rover returns |
| DUMP → next cycle / RETURN | dump sequence finished and the pour credited over the construction zone; another cycle starts only if the cycle succeeded (sand in the berm target) and `another_cycle()` allows it: ≤ `num_cycles` (10); elapsed + longest cycle + 150 s return reserve ≤ 600 s; and the estimated dead-reckoning drift added by the cycles stays ≤ 0.15 m (below) |

Goals (arena frame, `config/arena_params.yaml` → `mission:`): excavation (2.35, 0, 0°)
so the bucket digs at x ≈ 3.29 inside the excavation zone; construction
(3.30, 0, 0°) so the stowed front is 0.46 m into the construction zone (0.26 m when
Nav2 stops 0.20 m short, as it typically does) and 0.54 m from the wall while the pour
lands on the berm target at x ≈ 4.02–4.21; return (−3.55, 0, 180°),
facing the start-zone markers. The old construction goal (the berm centre, x = 4.0)
put the footprint through the east wall at every heading but ±90°. The goal was 3.15
until E5: the believed crossing was then only 0.11–0.14 m in all 303 E3/E4 arrivals,
localization x error reached 0.14 m, and one arrival (E4 seed 9) was 0.3 cm short of
the zone line by ground truth.

Failure handling: no hops — each leg's goal goes straight to Nav2 and the global
planner chooses the route. Each leg gets 3 attempts (240 s each). Error code 0
(rejected/cancelled/BT timeout) → retry the same goal; GOAL_OCCUPIED / NO_VALID_PATH
or "arrived but zone condition failed" → retry with an alternate goal that still
satisfies the zone condition and is free in the current costmap; then FAILED. Legs
wait for Nav2 to be ACTIVE instead of spending attempts. The arm is stowed and
verified before every leg. COMPLETE is published only after every leg was verified.

- **Blocked goals.** Before a goal is sent, and every second while it is active, it
  is checked with the planner's own test (a lethal cell under the padded footprint).
  A blocked candidate is skipped before sending; an active goal blocked twice in a
  row is cancelled and counts as a GOAL_OCCUPIED attempt, so the next try uses a
  free alternate. (Seed 10's primary excavation goal is 3 mm from the excavation
  boulder: blocked as soon as the boulder is mapped. Nav2 used to keep replanning
  and recovering instead of failing.) Results of cancelled or replaced goals are
  ignored (goal sequence numbers).
- **START_OCCUPIED.** Nav2's default tree runs no recovery for it. Before the retry
  the mission makes an escape motion (`mission_logic.escape_motion`): a turn in
  place of 5–60° or a straight drive of 0.05–0.25 m, at ≤ 0.35 rad/s / 0.08 m/s,
  closed-loop on the localized pose with a 15 s timeout. A candidate is accepted only
  if no intermediate pose covers a lethal cell the start does not already cover, the
  end pose is clear with padding + 3 cm, and its centre cell is below inscribed
  cost. If lethal cells are visible under the start, the smallest such motion is
  used; if none are (the planner refused a start this costmap copy shows as free),
  the motion ending at the lowest footprint cost. If no such motion exists, drives up
  to 0.8 m are tried (state-D seed 10: the rover had driven into a corridor that
  narrowed in the costmap as localization drifted 0.12 m, and no ≤ 0.25 m motion was
  clear three times; used once since, E3 seed 1: +0.70 m). Escapes are bounded by the leg's 3 attempts.
- **Final turn.** The lattice planner reaches a goal heading only by driving a
  2.6–3.3 m loop when the rover is already at the goal position but turned
  (`planner_route_test`, seed 14: all tested rotation penalties and expansion
  lengths) — through ground the camera never saw. Within 0.25 m of the goal and more
  than 0.40 rad off its heading the mission cancels the Nav2 goal and turns in place
  (≤ 0.35 rad/s, sweep checked against the costmap, 20 s timeout); arrival and the
  zone condition are then verified as for any Nav2 success.
- **Between cycles.** The excavation goal lies 0.8 m straight behind the
  construction goal with the same heading. The lattice planner has no reverse
  primitives and would turn the 1.13 m rover around twice; instead the mission backs
  straight up (0.10 m/s, closed-loop on the localized pose, 20 s timeout) when
  `mission_logic.reverse_distance` finds the goal behind (≤ 0.25 m lateral, ≤ 0.35 rad
  heading difference) and `straight_drive_safe` finds the swept footprint clear in the
  costmap; otherwise Nav2 drives the leg. Before backing up it turns in place to the
  goal heading (sweep checked): backing along an arrival heading 17° off left the rover
  0.22 m beside the goal and Nav2's re-approach stopped where the bucket would have dug
  in the construction zone (seed 10). Later cycles aim at the excavation goal candidate
  that was reached last time, skipping candidates blocked in the costmap (seed 10's
  primary goal touches the excavation-zone boulder). Arrival is verified the usual way.
- **Scan stops.** While crossing the obstacle zone (≥ 0.3 m inside its x-limits),
  every 1.2 m the Nav2 goal is paused, the rover turns to heading −50°, +50° and back
  (each turn costmap-checked, skipped if not clear; 1 s hold), and the same goal is
  resumed without spending an attempt. The camera sees ±33°; routes that ran past a
  crater and then turned toward it crossed ground seen only at the image edge (seed
  12, crater_2). Costs ~10–15 s per stop, ~4 stops per crossing.
- **Drift budget.** Markers are seen only from the start zone, so every cycle adds
  unseen dead-reckoning error. The mission integrates |yaw change| (odom) since the
  first excavation arrival (reset at a marker fix) and estimates the added drift as
  0.0136 m/rad — the worst whole-run growth measured — and starts another cycle only if
  the estimate after it (current + largest per-cycle rotation so far) stays ≤ 0.15 m.
  In E3 seed 10 the cycles needed Nav2 loops (~7.5 rad each), the error grew from
  0.14 to 0.34 m and the 4th pour landed 0.15 m outside the berm target by ground truth;
  with the budget the same seed stops after 2 cycles. Typical cycles (straight back-up,
  0–1.4 rad) are not limited. The error carried in from the crossing (0.03–0.2 m
  measured) is not part of the estimate.
- **Dig position.** Arrival tolerance (0.30 m) can leave the dig-pose bucket lip
  (0.94 m ahead) at the excavation zone's far edge. Before each dig pass
  `mission_logic.dig_correction` predicts the lip; if it is not ≥ 5 cm inside the near
  and ≥ 10 cm inside the far zone boundary, the rover drives straight (≤ 0.5 m,
  costmap-checked) so the lip lands at the nominal x = 3.29.
- **Failures after work was done.** Once a cycle has started, a failed excavation or
  construction leg (all attempts used) ends the cycling and the rover returns; the
  failure is recorded in the cycle list and `failed_actions`, deposited sand stays
  counted, and the mission can still reach COMPLETE. A failed first excavation leg or
  return leg still ends the mission as FAILED.
- **Ground-truth check (validation only).** `tools/gt_monitor.py` records the
  Gazebo pose at every leg the mission reports as arrived and re-checks the zone rule
  on it; `suite_report.py` fails a seed unless every leg was really in its zone,
  whatever the state machine reported. It also places every credited scoop/dump's
  bucket lip on the ground-truth pose and requires it in the excavation zone / berm
  target.

---

## Excavation and dumping (material accounting)

**What is simulated, and what is not.** Gazebo's sand bed is rigid: the bucket
cannot penetrate it and nothing is carried or deposited physically. The arm's
dig/scoop/carry/pour motions are real joint motions in Gazebo. How much sand those
motions would move is computed by `cac_sim/material_model.py` — every volume below
is a **simulated estimate**, never a measurement, and is labelled so in logs and
summaries.

| Quantity | Value | Source |
|---|---|---|
| Bucket struck volume | 1.183 L | `arm.xacro` bucket interior 0.13 × 0.14 × 0.065 m (test checks it) |
| Fill per scoop | 0.8 → **0.946 L** | ASSUMPTION (`material.fill_factor`) |
| Bulk density | 1500 kg/m³ | ASSUMPTION, loose dry sand; mass reporting only |
| Dig site | 0.25 m cells, 0.15 m usable depth → 9.4 L each | ASSUMPTION (half the 30 cm bed) |
| Berm target | circle r = 0.4 m at (4.0, 0) | `zones.berm_target` |
| Robot mass | 50.55 kg | sum of URDF inertials (test checks it; rule ≤ 80 kg) |

Rules (each rejection is an event with a reason; `test_material_model.py`):

- **Scoop** — credited when the curl pose is reached, using the bucket-lip position
  (TF, `bucket_link` point (0.14, 0, −0.035)) recorded at the cut pose: it must be in
  the excavation zone (rulebook: all berm sand comes from there) and within 5 cm of the
  sand surface; the bucket must have room; the dig site under the lip must still hold
  sand. Volume = min(0.946 L, room, site remaining).
- **Dump** — credited when the pour pose is reached with sand in the bucket and the lip
  over the construction zone. Inside the berm target it counts as
  `deposited_in_target`; elsewhere in the construction zone as
  `deposited_outside_target` (the rulebook scores only berm volume within the target
  location). Partial pours are supported (`fraction`).
- Each action has an id (`cycle3-dig1`, `cycle3`); an id is credited at most once.
  `collected = in_target + outside_target + in_bucket`, all ≥ 0 (tested).

**Score-relevant output.** The rulebook scores *Construction Berm Productivity*:
berm volume within the target (cm³) / (run minutes × robot kg) (BP-Mass), or per
W·h (BP-Energy). The run summary reports the simulated in-target volume (L and cm³),
run time (sim s, from the start of localization), robot mass and the BP-Mass formula
applied to them (`bcp_mass_estimate` — an estimate, not a score; the real score uses
a volumetric scan). Energy is not modelled, so BP-Energy is not estimated. Full
Autonomy needs ≥ 2 full cycles for 600 points; the summary reports cycles started /
successful.

**End-of-run summary** (`/mission_status` → `summary`, and one `RUN SUMMARY` log
line at COMPLETE/FAILED): run time and budget, cycles (per cycle: scoop/dump litres,
in target, duration, result), material totals, every material event, BCP-mass
estimate, failed actions. `gt_monitor.py` copies it into `seed_N_summary.json`
(`run_summary`, `material_gt`).

**In Gazebo** (`material_visualizer.py`, `material_visual:=true`): after each
credited dump a sand-coloured cone (34° angle of repose) with
the total deposited volume is spawned at the mean pour point, and after each credited
scoop a dark disc marks the cut. They are static, have no collision and carry
visibility flags without bit 0, so the LiDAR (mask = bit 0) never sees them; the
camera does, but the crater detector only marks points below the ground plane. They
are pictures of the accounting — the robot cannot feel them and the sand bed is
unchanged. A physical sand model was not attempted: the installed gz-sim 8.11 ships no
terrain, granular or deformable-body system (its `particle-emitter` is visual-only;
`detachable-joint` could carry a rigid lump in the bucket, which would not behave
like sand and would put rigid debris on the berm), and a rigid-particle pile would
not be stable or fast enough to validate in the regression suite.

The bucket is small (~0.95 L per cycle), so even ten cycles deposit < 10 L: the
estimate is dominated by the bucket size, which is a design parameter of the real
rover, not of this simulation.

---

## Testing

Unit tests (no Gazebo):

```bash
cd $WS/src/cac_sim && python3 -m pytest test/
```

`test_arena_layout.py` (determinism, obstacle counts/sizes/zones/spacing, start
rules, every emitted layout reachable, blocked/narrow layouts rejected, watertight
crater mesh, floor tiling, generated world, marker rules), `test_mission_logic.py`
(arrival/zone conditions incl. the crossing margin, bounded retries, alternate
goals, footprint cost check, START_OCCUPIED escape on the seed-10 geometry),
`test_consistency.py` (Nav2 footprint = arena params = URDF collision envelope,
competition envelope, every low rover visual hidden from the LiDAR after
URDF→SDF conversion, mission vs Nav2 tolerances and padding, wall/start margins,
spin-rate limit, LiDAR height), `test_arm_sweep.py` (arm sequences vs rover, ground
and camera view), `test_crater_detector.py` (pre-fix odom cells merged into the map),
`test_wall_filter.py` (ground-return geometry, calibrated against the measured
returns), `test_material_model.py` (scoop/dump rules, capacity, partial and
outside-target dumps, invalid locations/poses, exhausted sites, duplicate ids, reset,
multi-cycle conservation, BP-Mass formula, bucket volume = URDF),
`test_mission_cycles.py` (the mission node's excavation/dump/cycle handlers with a
stubbed arm, TF and clock: credited cycle, time-budget stop, rejected scoop retried
then failed, dump outside the construction zone, missing TF), and in
`test_crater_detector.py` a ray-cast camera over the seed-12 bowl for the occlusion
shadow (coverage up, no false cells, flat ground marks nothing).

Gazebo tools (`tools/`, installed to `share/cac_sim/tools`; export the loopback
variables first):

| Tool | What it checks |
|---|---|
| `run_seed_suite.sh <out> <seeds…>` | fresh sim per seed, full mission, ground-truth monitor, summary table |
| `gt_monitor.py --seed N --out P` | localization error, footprint–wall/boulder distance, wheels in craters, crater detection, final state |
| `rover_physics_check.py --out P` | rest pose, forward/spin/arc/reverse tracking, spin drift, arm pose tracking and body tilt (`autonomy:=false`) |
| `crater_view_test.py --seed N --crater crater_k` | localize, park 2.0/1.5/1.0 m from a crater, report detected cells and false cells |
| `suite_report.py <out>` | PASS = COMPLETE, every leg confirmed in its zone by ground truth, no wall contact, boulder contact or wheel in a crater, ≥ 1 cycle with sand in the berm target and every credited scoop/dump confirmed on the ground-truth pose; also prints cycles, simulated berm litres, sim run time vs budget, BCP-mass estimate |
| `with_sim.sh <out> <timeout> "<launch args>" <cmd…>` | run one tool against a fresh headless sim and stop it afterwards |
| `system_check.py --out P` | topic rates, every link in TF, one parent per frame, `/tf` publishers, wheel directions (forward / left turn), rocker limits (`autonomy:=false`) |
| `marker_range_test.py --out P` | marker-fix error vs range and marker count against ground truth, believed-pose error while backing away (`randomize:=false autonomy:=false`) |
| `planner_route_test.py --seed N --out P` | localize + survey, then ComputePathToPose from given starts under given parameter sets; ground-truth clearance of each path (`autonomy:=false`) |
| `seed_diagnostic.py --seed N --out P` | per camera frame whether each crater was seen and usable; costmap/plan/footprint snapshots at every planner failure |

`run_seed_suite.sh` writes `seed_N_summary.json` (and `_candidates.csv`, the marker
fixes vs ground truth) as each seed finishes; `SUITE_RESUME=1` skips seeds that
already have a summary, so an interrupted suite reruns only the missing ones. It
force-stops Gazebo/Nav2/cac_sim processes before every seed: never run two suites
(or a suite and another sim) at once.

```bash
export GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# multi-cycle missions run up to ~600 s sim (RTF ~0.97): raise the per-seed timeout (default 900 s)
SUITE_TIMEOUT=1500 bash install/cac_sim/share/cac_sim/tools/run_seed_suite.sh $WS/suite_results/my_suite 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
bash install/cac_sim/share/cac_sim/tools/run_seed_suite.sh $WS/suite_results/seed10 10      # one seed
T=install/cac_sim/share/cac_sim/tools
bash $T/with_sim.sh $WS/suite_results/syscheck 300 "randomize:=false autonomy:=false" python3 $T/system_check.py --out $WS/suite_results/syscheck/check
bash $T/with_sim.sh $WS/suite_results/physics 500 "randomize:=false autonomy:=false" python3 $T/rover_physics_check.py --out $WS/suite_results/physics/physics
```

---

## Competition rules and interpretations

From `CAC_2027_Rulebook.pdf`:

- **Walls (Autonomy Rules 3–4, rule 10c):** walls must not be used for mapping,
  navigation or collision avoidance. Implemented: no wall returns reach mapping,
  costmaps, the collision monitor or perception; localization uses start-zone
  markers. *Open question:* the boundary keepout uses the arena's published size
  (not sensing) — whether judges accept such a virtual boundary is not settled by the
  rulebook. Without it, wall safety would rest on localization alone.
- **Markers:** ≤ 5, ≤ 25 × 25 cm, ≤ 60 cm high, one on the wall attached to the start
  zone, the others on "the bin frame structure along the perimeter of the starting
  zone (2 sides)" — interpreted as the north and south boundaries of the start zone.
- **Start:** random position and orientation, aligned N/S/E/W. Remote-control
  repositioning inside the start zone to "lock in" localization is allowed; the
  generator's turn-in-place clearance stands in for it.
- **Obstacles:** ≥ 3 boulders 30–40 cm with varying heights, ≥ 3 craters up to
  40–50 cm wide with varying depth, smaller boulders possible in the excavation zone.
  Crater depth is not specified (5–15 cm assumed).
- **Not traversable / contact penalties:** craters and boulders must be avoided
  (−30 per occurrence); the suite counts both from ground truth.
- **Sand:** must be acquired in the excavation zone and carried (not bulldozed) to
  the construction zone; the material model credits a scoop only with the bucket lip
  in the excavation zone and a pour only over the construction zone, and the bucket is
  lifted (carry pose) while driving. Sand itself is not simulated physically.
- **Scoring:** BP-Mass = berm volume in the target zone (cm³) / (run minutes × robot
  kg); BP-Energy uses W·h (not modelled). The run summary applies the BP-Mass formula
  to the simulated in-target volume (`bcp_mass_estimate`), an estimate only.
- **Cycles (interpretation):** Full Autonomy awards 450 points for one full cycle and
  600 for ≥ 2 cycles with the whole run hands-free. The rulebook lists traversal,
  excavation, return, construction-zone entry and dumping without defining whether a
  cycle must re-cross the obstacle zone. Here a cycle = scoop in the excavation zone →
  carry → pour on the berm target; the obstacle zone is crossed on the first leg and
  on the final return. If judges require each cycle to start from the start zone, the
  cycle loop must be changed (each cycle would add two crossings, ~2 × 60 s).
- **Run time:** prelims 10 min, finale 15 min. `time_budget_sec` (600) bounds the
  cycle loop; the mission clock starts at localization (after any permitted
  start-zone repositioning).
- **Zone order and sizes** come from the reference diagram, which is not in the
  PDF's text.

---

## Known limitations

- **Dead reckoning between marker fixes.** Markers are only in the start zone, so in
  the obstacle and excavation zones the pose comes from wheel velocities + gyro. Any
  wheel slip (e.g. a wheel dropping into an undetected crater) becomes a persistent
  offset until the rover sees the markers again on the return leg. The wall filter
  and the boundary layer both rely on this pose.
- **Crater detection is forward-only and partial.** The camera sees the ground from
  ~0.35 m ahead of the bumper to ~2.2 m, within ±33°. The survey covers the ground
  next to the start zone; elsewhere a crater is seen only while driving toward it.
  From 1–1.5 m only the far wall of a bowl is visible, so a crater is mapped as
  13–17 cells of its ~50; the occlusion shadow (crater detector) now fills the hidden
  near slope, and scan stops turn the camera ±50° every 1.2 m in the obstacle zone so
  ground beside the route is seen before the route turns into it. Crater incursions
  occurred on seed 12 in states B, C and E2 (crater_2, 5.9 cm deep, seen only at the
  image edge) and on seed 14 in state B; with scan stops + shadow (E3) crater_2 was
  62 % mapped instead of 8–12 %. Detection still depends on the camera having looked
  at the ground: a crater that is never inside the ±33° view at 1.0–2.2 m is not
  mapped (seed 10: crater_2 0 % in E3, off every route) — see Validation status.
- **Localization between the zones.** After the initial fix only multi-marker
  detections are used (two markers within 3.5 m, three within 4.5 m), so the rover
  dead-reckons through the obstacle and excavation zones and is corrected only when
  it faces the start zone again.
- **Nav2 start refusals.** In one seed-10 run (2026-10-12) the planner refused a
  start although no lethal cell lay under the padded footprint in any recorded
  costmap; that mechanism inside Smac is still unverified. Later START_OCCUPIED events
  had a visible cause: state-D seed 4 (a newly confirmed crater cell inside the
  padding, cleared by a 5° escape turn) and state-D seed 10 (a 1.06 m boulder gap that
  closed in the costmap as localization drifted 0.12 m; no ≤ 0.25 m escape existed and
  the mission failed — hence the 0.8 m escape fallback, used once in E3 seed 1).
- **Localization drift.** Between marker fixes the pose drifts up to ~0.2 m
  (worst sample 0.237 m, E3 seed 14). Obstacles are mapped relative to the drifting
  pose, so narrow gaps can close in the costmap.
- **Dig-site model.** Each 0.25 m cell holds 9.4 L (assumed 15 cm usable depth); ten
  cycles at one spot use ~9.5 L, so an eleventh scoop there would be refused as
  exhausted. `num_cycles` is 10.
- **Ground-return model.** The LiDAR ground filter assumes flat ground and a
  calibrated 0.45° beam spread; on real terrain a slope ahead would be removed as
  ground (and an obstacle beyond the predicted ground range is not seen until
  closer).
- **Arm self-collision is checked geometrically, not simulated** (gz does not collide
  links of one model); the joint-space interpolation in `test_arm_sweep.py`
  approximates the controllers' real paths.
- **Rocker articulation is not exercised** by the flat sand bed: on flat ground the
  rockers sit at −0.023 rad; behaviour on rough terrain is untested.
- **No granular physics.** Sand is rigid; craters cannot be filled; the arm does not
  move material. Collected/deposited/berm volumes are accounting estimates from the
  bucket size and assumed fill factor (`material_model.py`), confirmed only in where
  they happen (ground-truth lip position), not in how much sand moves. The Gazebo berm
  pile is visual-only. Energy is not modelled (no BP-Energy estimate).
- **Boulders are static** (pushing them is impossible, as is detecting a push).
- **Rocker suspension** is spring-centred per side instead of a differential bar
  (a bar would close a kinematic loop, which URDF cannot express); it is a four-wheel
  rocker, not a rocker-bogie.
- **Gazebo GUI:** in one launch with the GUI the server never advertised its
  services; server-only (`headless:=true`) runs were reliable. See Validation status.
- `slam_toolbox_params.yaml` is kept for reference but SLAM is no longer launched.

---

## Validation status

Full report: [`docs/validation_2026-10-13.md`](docs/validation_2026-10-13.md)
(previous: [`docs/validation_2026-10-12.md`](docs/validation_2026-10-12.md)).

Simulated sand volumes are estimates of the material model; no sand moves in Gazebo.

| Run (2026-10-13) | Code | Result |
|---|---|---|
| Unit tests (`pytest test/`) | E5 (current) | **93 passed** |
| `colcon build` | E5 | builds (dev workspace, frozen workspace, git clone) |
| `system_check.py`, `rover_physics_check.py` | E3 (model files unchanged since) | pass |
| Full 15 seeds | D (pushed `603e238`) | 14/15; seed 10 START_OCCUPIED in a boulder gap |
| Full 15 seeds | E3 | 12/15; seed 8 Nav2 bringup hang, seed 10 pour outside target (drift), seed 12 crater_2 |
| Full 15 seeds | E4 | **14/15**; 0 crater/wall/boulder contacts; 141 cycles, 133.4 L sim in target, all material events GT-confirmed; seed 9 construction arrival 0.3 cm short of the zone line by ground truth |
| Targeted 9, 10, 12, 14 (+ seed 12 ×2) | E5 (current) | **stopped after 6 min, no seed completed — E5 is not validated in simulation** |

**Not run on E5:** any completed simulation (targeted or full). E5 differs from E4 only in the construction
goal (3.15 → 3.30 m); the seeded layouts are unchanged.

Open issues: seed 12's shallow crater_2 is still crossed in some runs (2/3 pass on
E3/E4); localization drift between marker fixes (up to ~0.25 m); the 2026-10-12
START_OCCUPIED without a lethal cell is unexplained; rough terrain untested; volumes are
estimates.
