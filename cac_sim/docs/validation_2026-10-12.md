# Validation report — 2026-10-12

> Superseded by [`validation_2026-10-13.md`](validation_2026-10-13.md). One claim below (seed 14,
> state C) was corrected on 2026-10-13; the correction is marked in place.

Package: `cac_sim` (ROS 2 Jazzy + Gazebo Harmonic). All results below come from the
Gazebo ground-truth monitor (`tools/gt_monitor.py`) and `tools/suite_report.py`; the
raw logs, CSVs and bags are not in the repository (`suite_results/`, ~5 GB, git-ignored).

A seed **passes** only if the mission reached COMPLETE, every leg it reported as
arrived was really in its zone by ground truth (`legs_gt`), and there was no
footprint–wall contact, no footprint–boulder contact and no wheel inside a crater bowl
at any 0.5 s sample.

> **The final full 15-seed regression has NOT been run on the latest code.**
> The last complete 15-seed run (state **B** below) predates four changes (state
> **D**). A run of the latest code was started and stopped on request after 2 minutes
> of seed 1; it is not a result.

## Code states

| State | What it contains | Sim validation |
|---|---|---|
| **A** (baseline) | before the seed-10 work | 15 seeds: 14/15 (suite of 2026-10-11) |
| **B** | A + every change below up to and including the construction-goal/zone-margin fix | full 15-seed suite: **13/15** |
| **C** | B + crater depth threshold 1.5 cm + mission final turn | seeds 12, 14, 10 once (+ seed 12 once with the diagnostic tool) |
| **D** (current) | C + BT replanning 2 Hz, global costmap 4 Hz, cruise 0.20 m/s | **unit tests only** — not run in simulation |

## Results

### Baseline (state A), 2026-10-11

Seeds 2–5, 8–15 (suite3) + seeds 1, 6, 7 (suite2): **14/15 pass**; seed 10 FAILED
(START_OCCUPIED, a wheel in an undetected crater). Per-seed mean localization error
0.026–0.119 m, maximum 0.319 m; closest approach to a boulder 0.075 m, to a wall
0.089 m. (Leg ground-truth verification did not exist yet.) The suite2 files were in
`/tmp` and were lost in a reboot; their numbers come from the session transcript.

### Full 15-seed suite, state B (2026-10-12 21:02)

| seed | result | time s | loc mean / max m | wall min m | boulder min m | wheel-in-crater samples | legs by GT |
|---|---|---|---|---|---|---|---|
| 1 | PASS | 187.5 | 0.059 / 0.153 | 0.246 | 0.140 | 0 | yes |
| 2 | PASS | 138.0 | 0.074 / 0.130 | 0.409 | 0.249 | 0 | yes |
| 3 | PASS | 139.0 | 0.030 / 0.064 | 0.357 | 0.416 | 0 | yes |
| 4 | PASS | 190.0 | 0.049 / 0.109 | 0.294 | 0.168 | 0 | yes |
| 5 | PASS | 183.0 | 0.063 / 0.151 | 0.270 | 0.227 | 0 | yes |
| 6 | PASS | 212.0 | 0.082 / 0.156 | 0.160 | 0.214 | 0 | yes |
| 7 | PASS | 124.0 | 0.066 / 0.145 | 0.385 | 0.180 | 0 | yes |
| 8 | PASS | 124.0 | 0.027 / 0.056 | 0.433 | 0.299 | 0 | yes |
| 9 | PASS | 126.5 | 0.040 / 0.098 | 0.257 | 0.278 | 0 | yes |
| 10 | PASS | 185.5 | 0.062 / 0.152 | 0.336 | 0.133 | 0 | yes |
| 11 | PASS | 149.5 | 0.065 / 0.128 | 0.347 | 0.609 | 0 | yes |
| 12 | **FAIL** | 121.0 | 0.039 / 0.061 | 0.202 | 0.221 | **6** | yes |
| 13 | PASS | 127.0 | 0.031 / 0.059 | 0.210 | 0.277 | 0 | yes |
| 14 | **FAIL** | 159.0 | 0.104 / 0.179 | 0.257 | 0.284 | **5** | yes |
| 15 | PASS | 236.5 | 0.057 / 0.133 | 0.251 | 0.149 | 0 | yes |

**13/15 pass; 15/15 COMPLETE with every leg confirmed by ground truth; 0 wall or
boulder contacts; 2 crater incursions.** Localization: per-seed mean 0.027–0.104 m
(average 0.057), worst sample 0.179 m (baseline: 0.319 m). Closest boulder 0.133 m,
closest wall 0.160 m.

Compared with the baseline: seed 10 fixed; seeds 12 and 14 newly failed (crater
incursions, both passed in the baseline). Diagnosis:
- seed 12: `crater_2` is only 5.9 cm deep; at the 2.5 cm depth threshold only its core
  was mapped and a wheel crossed the unmapped slope;
- seed 14: near the excavation goal (0.11 m away, 43° turned) the lattice planner
  produced a ~3 m loop through ground the camera never saw, into undetected `crater_0`.

### Seed 10 repeated

| Code | Runs | Result |
|---|---|---|
| A | baseline suite | 0/1 |
| intermediate (in order) | `seed10_repeat_1..3`, `seed10_tfthread_1..3`, `seed10_redesign_1..3`, `seed10_r2_1..3`, `seed10_r3_1..3`, `seed10_r4_1..3` | 2/3, 0/3, 0/3, 1/3, 2/3, 2/3 — each failure was diagnosed and led to one of the fixes below |
| B | `seed10_r5_1..3` + full suite | **4/4 PASS**, legs GT-verified, loc max 0.065–0.152 m |
| C | `targeted_12_14_10` | 1/1 PASS |

### State C targeted runs

| seed | result | notes |
|---|---|---|
| 10 | PASS | GOAL_OCCUPIED → alternate goal; final turn used once |
| 12 | **FAIL** | wheel in `crater_2` (6 samples) again; crater now 46 % mapped |
| 14 | PASS | ~~final turn avoided the loop~~ — **corrected 2026-10-13:** no `final_turn` entry was logged in this run; the rover took a different, direct route (6.3 m vs 9.8 m in state B) and arrived 21° off the goal heading, inside Nav2's 23° yaw tolerance, so no loop was needed. The pass is not evidence for the final turn. |
| 12 (diagnostic tool attached) | PASS | `crater_2` visible ~2 s, confirmed ~1.5 m ahead, route diverted |

State D (faster replanning, slower cruise) targets exactly this seed-12 reaction-time
problem and is **unverified**.

### Robot model and integration checks (state B code; unchanged since for the model)

- `tools/system_check.py` (empty arena): sensor rates /scan 9.9 Hz, /imu 91 Hz,
  /camera/points 14.5 Hz, /odom_raw 46 Hz, /odometry/filtered 25 Hz; every URDF link in
  TF; every frame one parent; /tf publishers robot_state_publisher, ekf_filter_node,
  marker_localizer; forward drive all wheels +1.67 rad/s; left turn left −1.1 /
  right +1.1 rad/s, 61° yaw; rockers −0.023 rad (within ±0.30). **Pass.**
- `tools/rover_physics_check.py`: rest pitch 1.32°, spin drift 0.007 m/s, full-turn
  drift 0.032 m; arm joints settle within 0.08 rad; body tilt ≤ 3.2° in the arm
  sequences. **Pass.**
- Localization: `tools/marker_range_test.py` (empty arena, backing 5.7 m away from the
  markers): believed-pose error 2.7–4.2 cm / ≤ 0.4° with the final gating (was
  0.18–0.45 m / 3–5° with single-marker updates).
- TF listener fix (measured, 40 s, seed 10): localizer TF failures 482/783 → 13/787
  images; /scan_filtered 178 → 336 of 397 scans; crater detector 75–87 % → 0 % clouds lost.
- LiDAR ground-return filter: phantom hits in the first 60 s of seed 10 22 025 → 0.

### Unit tests (state D)

`python3 -m pytest test/` — **49 passed** (also from the team-repository copy).
`colcon build --packages-select cac_sim` succeeds in a fresh workspace.

## Implemented / tested / unverified

| Change | Status |
|---|---|
| TF listeners on their own thread (localizer, wall filter, crater detector) | implemented, measured |
| Marker-fix gating by marker count and range | implemented, measured (range test, 15-seed suite B) |
| LiDAR ground-return rejection (IMU roll/pitch + measured beam spread) | implemented, measured, suite B |
| Crater cells kept before the first marker fix (odom → map) | implemented, unit-tested, suite B |
| Obstacle-zone survey after localization | implemented, suite B |
| Footprint padding 0.06 m, keep-out 0.11 m, inflation/cost tuning | implemented, planner tests, suite B |
| Behavior tree without Spin recovery; behavior rotations ≤ 0.35 rad/s | implemented, suite B |
| Blocked-goal detection, START_OCCUPIED escape, goal-result sequencing | implemented, unit-tested, suite B |
| Construction goal 3.15 m, zone crossing margin 0.10 m | implemented, unit-tested, suite B |
| Ground-truth leg verification in the monitor/report | implemented, suite B |
| Arm geometry/poses (pedestal, stow, carry/dig/scoop/pour) | implemented, sweep-tested, physics check, suite B |
| Crater depth threshold 1.5 cm | implemented, crater view test, targeted runs (state C) |
| Final in-place turn near the goal | implemented, unit-tested, targeted runs (state C) |
| BT replanning 2 Hz, global costmap 4 Hz, cruise 0.20 m/s | implemented, **unverified in simulation** |

## Open issues

- Crater incursions remain possible (seed 12 in states B and C): shallow craters are
  confirmed late and only partially mapped.
- One START_OCCUPIED in a seed-10 run had no lethal cell under the footprint in any
  recorded costmap; the cause inside the planner was not identified.
- Physical excavation is not simulated (payload counter only).
- Rocker articulation on rough terrain is not exercised by the flat arena.
