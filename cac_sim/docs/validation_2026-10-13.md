# Validation report — 2026-10-13

Package `cac_sim` (ROS 2 Jazzy, Gazebo Harmonic / gz-sim 8.11). Results come from the
Gazebo ground-truth monitor (`tools/gt_monitor.py`) and `tools/suite_report.py`. Raw
logs, CSVs and summaries are in `suite_results/` of the development workspace
(git-ignored, not in the repository). Previous report:
[`validation_2026-10-12.md`](validation_2026-10-12.md) (one claim corrected there).

**Every simulated sand volume in this report is an estimate from the material
accounting model** (`cac_sim/material_model.py`): Gazebo's sand bed is rigid and no
sand moves. "Confirmed by ground truth" for material means the bucket lip was where
the rule requires (excavation zone / berm target) at the Gazebo pose; it does not
mean any sand was measured.

## Pass criteria

A seed **passes** only if: the mission reached COMPLETE; every leg it reported as
arrived was in its zone at the ground-truth pose; no footprint–wall contact, no
footprint–boulder contact and no wheel inside a crater bowl at any 0.5 s sample; and
(runs with material accounting) at least one cycle deposited sand in the berm target
and every credited scoop/dump was confirmed on the ground-truth pose. Run time within
the 600 s budget is reported separately.

## Code states

Each simulation result below belongs to exactly one code state; a frozen copy of
each state's source was built in its own workspace (`suite_results/ws_*`, with
`src_snapshot.tgz`) so later edits could not change the code under test.

| State | Content | Workspace |
|---|---|---|
| D | pushed branch `603e238` (2 Hz replanning, 4 Hz global costmap, 0.20 m/s) | `ws_stateD` |
| E1 | D + material model, multi-cycle mission with time budget, back-up between cycles, visualizer, GT material check, crater occlusion shadow | `ws_E1` |
| E2 | E1 + heading alignment before backing up, dig repositioning, 0.8 m escape fallback, cone pile visual | `ws_E2` |
| E3 | E2 + scan stops in the obstacle zone, later-cycle leg failure → return, reuse of the last reached excavation goal | `ws_E3` |
| E4 | E3 + dead-reckoning drift budget in the cycle decision | `ws_E4` |
| **E5 (current, sim-unverified)** | E4 + construction goal x 3.15 → 3.30 m | `ws_E5` |

Code fingerprints (sha256 of the sorted `sha256sum` list of the non-documentation
files of `src/cac_sim`, command under *Reproduce*): E3 `995814b20848089c`,
E4 `1e2b2230ec89fc59`, **E5 `e9df52e3fd91c184`**. Documentation (README, this file) was
edited after E5 was frozen; no code was.

## Build and unit tests (E5)

- `colcon build --packages-select cac_sim`: succeeds (frozen `ws_E5`, `~/MOVIS`, and the git clone).
- `python3 -m pytest test/`: **93 passed** (49 before this session). New: `test_material_model.py`
  (20), `test_mission_cycles.py` (12), mission-logic tests for cycles/back-up/dig position/scan
  stops/long escape/drift budget/construction-goal margin (+8), crater-detector ray-cast tests (+3), robot mass = URDF (+1). A tf2
  `InvalidHandle` traceback printed at interpreter exit is pre-existing (also on state D) and
  does not affect results.
- `check_urdf`: parses; visual and collision geometry agree link by link (bucket: 4 plate
  visuals inside one collision box); walls 0.80 m high, static, visual = collision, inner
  faces at ±4.50 / ±2.50 m. Total mass 50.55 kg (rule ≤ 80 kg). All 16 inertia tensors are
  positive definite and satisfy the triangle inequality. Bin (steel grey, open top, 4 walls)
  and electronics box (blue, closed) are separate links with distinct materials.

## Gazebo smoke tests (E3; URDF, world and sensor files unchanged in E4/E5)

- `system_check.py` (empty arena): no failures. Rates /scan 8.1 Hz, /imu 80.7, /camera/points
  12.3, /odom_raw 40.4, /odometry/filtered 29.9; every URDF link in TF, one parent per frame;
  forward drive all wheels +1.667 rad/s (0.596 m), left turn −1.1/+1.1 rad/s (61.6°), rockers
  −0.024…−0.023 rad. (Sensor rates are ~15 % below the 2026-10-12 check, likely host load;
  not investigated.)
- `rover_physics_check.py`: rest pitch 1.32°; forward 0.30 and reverse −0.20 m/s tracked
  exactly by ground truth; spin drift 0.036 m per phase, full-turn drift 0.025 m; arm joints
  within 0.079 rad of every pose; body tilt ≤ 3.2° in the arm sequences. The tool's odometry
  translation read 16–20 % below ground truth on straight drives — cause not investigated.

Rough terrain is not tested: the arena is flat (rockers sit at −0.023 rad).

## Navigation regression, state D (pushed code), 15 seeds

`suite_results/suite_stateD_2026-10-13`: **14/15 pass**; 0 wheel-in-crater samples
(seeds 12 and 14, which failed in state B, pass), 0 wall or boulder contacts.
Seed 10 FAILED: three START_OCCUPIED in the 1.06 m gap between boulder_0 and
boulder_2 (ground truth 0.12 m from boulder_0, localization error 0.12 m), no escape
motion found. Seed 4 had one START_OCCUPIED next to crater_2, cleared by a 5° escape.

## Investigations

- **Seed 14 (state C) attribution — corrected.** The 2026-10-12 report credited the final
  in-place turn. The run's history has no `final_turn` entry; the rover took a direct route
  (6.3 m vs 9.8 m in state B) and arrived 21° off heading, inside Nav2's 23° tolerance.
- **Seed 12 crater_2 (5.9 cm deep).** (1) From the approach side the camera sees only the
  far wall of a bowl; the near slope is occluded → crater detector **occlusion shadow**
  (ray-cast test: mapped share of the part deeper than the threshold 0.73 → 1.00 head-on,
  0.55 → 0.82 at 25°, no false cells). (2) In E2 the route ran past crater_2 with it at
  29–33° off the heading (image edge) for 9.5 s, no frame produced below-plane points, and
  the route then turned into it (9 samples). States D and E1 passed only because their
  routes swung south. → **scan stops**: every 1.2 m in the obstacle zone the goal is
  paused and the camera turned ±50°. E3 seed 12: crater_0/1/2 mapped 0.99/0.92/0.62
  (E2: 0.72/0.56/0.12), no incursion.
- **Seed 10 (multi-cycle).** E1: cycle 2's arrival left the dig-pose lip at x = 3.50 (zone
  edge); both scoops were correctly refused. → heading alignment before backing up, dig
  repositioning. E2: the primary excavation goal touches the excavation boulder; every
  cycle needed NO_VALID_PATH → alternate goal, and a 6th-cycle PATIENCE_EXCEEDED ended the
  whole run FAILED after 5 deposits → later-cycle leg failures return home; later cycles
  reuse the last reached goal.
- **Drift over cycles (E3 full, seed 10).** Fitting loc-error growth against travel and
  rotation over all E3 runs: rotation dominates (distance coefficient ≈ 0), worst whole-run
  growth 0.0136 m/rad. Most rotation happens on the crossing (scan stops, 11–31 rad) at
  controlled rates and produced ≤ 0.22 m; normal cycles turn 0–1.4 rad. → E4 budgets only
  the rotation of the cycle phase (≤ 0.15 m estimated); replayed on the E3 data it keeps 10
  cycles on typical seeds and stops seed 10 after 2.
- **START_OCCUPIED.** See README *Known limitations*: visible causes in D seeds 4/10; the
  2026-10-12 seed-10 case without a lethal cell remains unexplained.

## Targeted runs

| Run | Seed | Result | Cycles ok | Berm L (sim) | Sim run s | Notes |
|---|---|---|---|---|---|---|
| E1 | 12 | PASS | 10/10 | 9.46 | 380 | |
| E1 | 14 | PASS | 10/10 | 9.46 | 369 | |
| E1 | 10 | PASS | 1/2 | 0.95 | 263 | cycle 2: lip at zone edge, scoops refused |
| E2 | 10 | FAIL | 5/5 | 4.73 | 438 | 6th excavation leg failed → mission FAILED |
| E2 | 12 | FAIL | 10/10 | 9.46 | 433 | 9 wheel-in-crater samples (crater_2) |
| E2 | 14 | PASS | 10/10 | 9.46 | 381 | |
| E3 | 12 | PASS | 10/10 | 9.46 | 481 | crater mapping 0.99/0.92/0.62 |
| E3 | 10 | PASS | 5/5 | 4.73 | 557 | stopped by the time budget |
| E3 | 14 | PASS | 10/10 | 9.46 | 503 | |
| E4 | 10 | PASS | 2/2 | 1.89 | 477 | drift budget stopped cycling (0.21 + 0.19 > 0.15 m) |

## Full regression, E3, 15 seeds

`suite_results/suite_E3_2026-10-13`: **12/15 pass**, 14/15 COMPLETE, 0 wall or boulder
contacts, 1 seed with a wheel in a crater, 127.8 L simulated in the berm target.

| seed | result | cycles ok | berm L (sim) | sim run s | loc max m | wall min m | boulder min m | crater samples | note |
|---|---|---|---|---|---|---|---|---|---|
| 1 | PASS | 10/10 | 9.46 | 499 | 0.182 | 0.241 | 0.092 | 0 | |
| 2 | PASS | 10/10 | 9.46 | 559 | 0.220 | 0.411 | 0.136 | 0 | |
| 3 | PASS | 10/10 | 9.46 | 509 | 0.098 | 0.200 | 0.157 | 0 | |
| 4 | PASS | 10/10 | 9.46 | 475 | 0.169 | 0.087 | 0.182 | 0 | |
| 5 | PASS | 10/10 | 9.46 | 514 | 0.096 | 0.285 | 0.193 | 0 | |
| 6 | PASS | 9/9 | 8.52 | 536 | 0.109 | 0.330 | 0.122 | 0 | stopped by the time budget |
| 7 | PASS | 10/10 | 9.46 | 449 | 0.124 | 0.225 | 0.152 | 0 | |
| 8 | **FAIL** | 0/0 | 0 | 198 | — | 0.481 | 2.01 | 0 | Nav2 bringup hang (middleware), see below |
| 9 | PASS | 10/10 | 9.46 | 498 | 0.103 | 0.258 | 0.210 | 0 | |
| 10 | **FAIL** | 6/6 | 5.68 | 591 | 0.360 | 0.159 | 0.110 | 0 | cycle-4 pour outside the target by ground truth |
| 11 | PASS | 10/10 | 9.46 | 496 | 0.099 | 0.347 | 0.271 | 0 | |
| 12 | **FAIL** | 10/10 | 9.46 | 510 | 0.103 | 0.175 | 0.155 | **13** | crater_2 again |
| 13 | PASS | 10/10 | 9.46 | 486 | 0.107 | 0.171 | 0.355 | 0 | |
| 14 | PASS | 10/10 | 9.46 | 496 | 0.098 | 0.235 | 0.340 | 0 | |
| 15 | PASS | 10/10 | 9.46 | 512 | 0.154 | 0.250 | 0.192 | 0 | |

- **Seed 8:** Nav2 never became active: the lifecycle manager hung at "Configuring
  controller_server" after `failed to send response to /controller_server/change_state
  (timeout)` (DDS). The same signature appears in 1 of the 121 earlier launch logs
  (2026-10-12). The mission timed out after 180 s and ended FAILED, as designed.
- **Seed 10:** markers are only seen from the start zone; with Nav2 loops of ~7.5 rad per
  cycle (primary goal blocked by the excavation boulder) the dead-reckoning error grew
  0.14 → 0.34 m. The believed cycle-4 pour point (3.91, 0.25) was really at (3.97, 0.55),
  outside the 0.4 m berm target: the ground-truth material check failed the seed. → E4.
- **Seed 12:** a wheel crossed crater_2 (5.9 cm deep) on the first excavation leg; it was 6 %
  mapped. One scan stop happened 0.66 m from the crater — inside the camera's near blind
  zone (ground visible from ~0.85 m). Seed 12 / crater_2 is intermittent across code states:
  failed in B, C, E2, E3-full; passed in D, E1, E3-targeted. Two distinct fixes (occlusion
  shadow, scan stops) raised its mapping in some runs (62 % in E3-targeted) but not
  reliably. **Unresolved.**

## Full regression, E4, 15 seeds

`suite_results/suite_E4_2026-10-13` (code E4, fingerprint `1e2b2230ec89fc59`; archived
with checksum in `suite_results/backups/suite_E4_2026-10-13_complete.tar.gz`). Runner
20:50:53–23:02 (2 h 11 min wall), 7656 s monitored, 7360 s simulated mission time.

**14/15 pass; 15/15 COMPLETE; 0 seeds with a wheel in a crater; 0 wall contacts; 0
boulder contacts; 1 ground-truth mission-leg failure (seed 9); 141 cycles, all 141
scoops and 141 pours confirmed on the ground-truth pose; 133.4 L (≈ 200 kg at the
assumed 1500 kg/m³) simulated in the berm target** — estimates, not measurements.

| seed | result | cycles ok | berm L (sim) | collected kg (sim) | sim run s | wall s | loc max m | wall min m | boulder min m | crater samples | legs GT | material GT | BCP-mass est. | cycle loop ended by |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | PASS | 10/10 | 9.46 | 14.2 | 542 | 570 | 0.227 | 0.243 | 0.092 | 0 | yes | 20/20 | 20.7 | 10/10 cycles |
| 2 | PASS | 10/10 | 9.46 | 14.2 | 472 | 498 | 0.142 | 0.358 | 0.155 | 0 | yes | 20/20 | 23.8 | 10/10 cycles |
| 3 | PASS | 10/10 | 9.46 | 14.2 | 508 | 536 | 0.097 | 0.214 | 0.119 | 0 | yes | 20/20 | 22.1 | 10/10 cycles |
| 4 | PASS | 10/10 | 9.46 | 14.2 | 472 | 493 | 0.087 | 0.325 | 0.215 | 0 | yes | 20/20 | 23.8 | 10/10 cycles |
| 5 | PASS | 10/10 | 9.46 | 14.2 | 531 | 547 | 0.195 | 0.285 | 0.184 | 0 | yes | 20/20 | 21.2 | 10/10 cycles |
| 6 | PASS | 9/9 | 8.52 | 12.8 | 512 | 528 | 0.135 | 0.347 | 0.153 | 0 | yes | 18/18 | 19.7 | time budget |
| 7 | PASS | 10/10 | 9.46 | 14.2 | 455 | 470 | 0.144 | 0.376 | 0.161 | 0 | yes | 20/20 | 24.7 | 10/10 cycles |
| 8 | PASS | 10/10 | 9.46 | 14.2 | 491 | 506 | 0.190 | 0.386 | 0.212 | 0 | yes | 20/20 | 22.9 | 10/10 cycles |
| 9 | **FAIL** | 10/10 | 9.46 | 14.2 | 509 | 529 | 0.139 | 0.134 | 0.224 | 0 | **NO** | 20/20 | 22.1 | 10/10 cycles |
| 10 | PASS | 2/2 | 1.89 | 2.8 | 438 | 450 | 0.244 | 0.307 | 0.152 | 0 | yes | 4/4 | 5.1 | drift budget |
| 11 | PASS | 10/10 | 9.46 | 14.2 | 503 | 520 | 0.093 | 0.334 | 0.291 | 0 | yes | 20/20 | 22.4 | 10/10 cycles |
| 12 | PASS | 10/10 | 9.46 | 14.2 | 466 | 482 | 0.074 | 0.203 | 0.164 | 0 | yes | 20/20 | 24.1 | 10/10 cycles |
| 13 | PASS | 10/10 | 9.46 | 14.2 | 466 | 482 | 0.118 | 0.204 | 0.337 | 0 | yes | 20/20 | 24.1 | 10/10 cycles |
| 14 | PASS | 10/10 | 9.46 | 14.2 | 491 | 512 | 0.070 | 0.246 | 0.228 | 0 | yes | 20/20 | 22.9 | 10/10 cycles |
| 15 | PASS | 10/10 | 9.46 | 14.2 | 505 | 532 | 0.159 | 0.251 | 0.220 | 0 | yes | 20/20 | 22.2 | 10/10 cycles |

Every mission completed within the 600 s budget (max 542 s simulated).

- **Seed 9 (construction-zone boundary).** Cycle 1's construction arrival was believed at
  (2.96, 0.01) — front 0.14 m across x = 3.5 — but ground truth had x = 2.821, front 0.3 cm
  *short* of the line (localization x error 0.139 m). The mission itself was otherwise
  clean (10/10 cycles, every pour GT-confirmed in the target). Across all 303 construction
  arrivals of E3/E4 the believed crossing was only 0.11–0.14 m (Nav2 stops ~0.19 m short of
  the 3.15 goal, just past the mission's 0.10 m `zone_cross_margin`), the ground-truth x
  error reached ±0.14 m, and 24 arrivals were < 5 cm across by ground truth. The ground-truth
  check was left unchanged; the goal was moved instead (E5).
- **Seeds 12 and 14 — consistency from actual results.** Seed 14: passed in 7 consecutive
  runs over 6 code states (C, D, E1, E2, E3-targeted, E3-full, E4); its only failure was
  state B. Seed 12: crater_2 incursions in B, C, E2 and E3-full; passes in D, E1,
  E3-targeted and E4. With the current crater/scan-stop stack (E3/E4) it passed 2 of 3, and
  crater_2's mapped share varied 6–66 % between runs. **Seed 12 is not consistent.**
- Gazebo visual, checked live in the E5 seed-9 run after 2 cycles with `gz model --list`:
  `dig_mark_1`, `dig_mark_2`, `berm_pile_2` present (`berm_pile_1` removed), pile at
  (4.034, 0.018), centre height 0.047 m = half of the 0.094 m cone for 1.89 L.
- Gazebo visual: all 141 pile spawns confirmed by the world's create service; 2 of the
  removals of the previous (smaller) pile timed out (seeds 2, 9), leaving it inside the new
  one — cosmetic only.

## E5 (construction goal 3.30 m)

E5 = E4 + `mission.construction_approach` 3.15 → 3.30 m, fingerprint `e9df52e3fd91c184`.
Typical stop ~3.11: front ~0.27 m across x = 3.5, pour at x ≈ 4.02 (berm centre 4.0); at
the goal itself front 0.46 m across, 0.37 m (padded) from the wall, pour at 4.21 (inside
r = 0.4). The seeded layouts for seeds 1–15 are unchanged (layout reachability uses the
goals; re-generated and compared). New test
`test_construction_goal_margin_survives_nav2_tolerance_and_localization_error` fails on
3.15 and passes on 3.30. Unit tests 93/93.

**E5 is unverified in simulation.** A targeted run (seeds 9, 10, 12, 14, then 12 twice) was
started at 23:04 and stopped on request at 23:11, ~6 min into seed 9; no seed completed
(`suite_results/targeted_E5_2026-10-13/INTERRUPTED.txt`; the partial `seed_9_summary.json`
is not a result). Whether the 3.30 m goal removes seed 9's boundary failure, and whether it
changes anything else (longer back-up 0.76 m, pours nearer the wall), has **not** been
tested in Gazebo. The last fully validated code is **E4 (14/15)**.

## Material accounting (E3)

Per cycle: one scoop of 0.946 L (1.183 L struck bucket × assumed fill 0.8) credited at
the curl pose with the lip in the excavation zone at the sand surface, carried, poured
on the berm target (r = 0.4 m). The scoop/dump sequence plus back-up/re-approach takes
~26 s per cycle in the excavation/construction zones; the first cycle includes the
obstacle-zone crossing with scan stops. The cycle loop stops at `num_cycles` (10) or
when elapsed + longest cycle + 150 s return reserve would exceed 600 s.
BCP-mass estimate = simulated in-target cm³ / (sim run minutes × 50.55 kg).

## Known issues and unverified behaviour

- **E5 (current code) has no completed simulation run.** Last fully validated: E4, 14/15.
- **Seed 12 / crater_2 (5.9 cm deep) is not consistently avoided**: 2 of 3 passes on E3/E4;
  crater_2 mapped 6–66 % between runs.
- **Construction-zone crossing margin** was 0.11–0.14 m believed vs up to 0.14 m localization
  error on E4 (seed 9 failed by 0.3 cm); the E5 fix is untested in Gazebo.
- **Nav2 bringup hang** (DDS lifecycle response timeout) in 2 of ~150 launches (E3 seed 8);
  the mission fails safely after 180 s; no fix in this package.
- The later-cycle "return instead of FAILED" path is unit-tested but was not triggered in any
  E3/E4 run. The long-drive escape fallback triggered once (E3 seed 1, return leg, +0.70 m
  drive after START_OCCUPIED; the seed passed).
- Gazebo visual: 2 of 141 old-pile removals timed out (cosmetic).
- Volumes are estimates (assumed fill factor, density, dig-site depth); no sand moves; the
  Gazebo pile is visual-only.
- Crater detection needs the camera to have looked at the ground; craters off every route
  stay unmapped (fine), but a route can still turn into ground seen only obliquely between
  scan stops.
- Localization drifts up to ~0.24 m between marker fixes; narrow gaps can close in the
  costmap (state-D seed 10). The 0.8 m escape fallback was exercised once (E3 seed 1).
- The 2026-10-12 START_OCCUPIED with no lethal cell under the footprint is unexplained.
- Rough terrain, rocker articulation and wheel slip on slopes are untested (flat arena).
- Energy (BP-Energy) is not modelled. The cycle definition is an interpretation (README).
- Not competition-ready evidence: one simulator, one flat arena model, 15 seeds.

## Reproduce

```bash
cd $WS && source /opt/ros/jazzy/setup.bash && colcon build --packages-select cac_sim && source install/setup.bash
cd src/cac_sim && python3 -m pytest test/ && cd $WS
export GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
T=install/cac_sim/share/cac_sim/tools
SUITE_TIMEOUT=1500 bash $T/run_seed_suite.sh $WS/suite_results/suite_E3 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
SUITE_TIMEOUT=1500 bash $T/run_seed_suite.sh $WS/suite_results/targeted 12 10 14
bash $T/with_sim.sh $WS/suite_results/syscheck 300 "randomize:=false autonomy:=false" python3 $T/system_check.py --out $WS/suite_results/syscheck/check
bash $T/with_sim.sh $WS/suite_results/physics 500 "randomize:=false autonomy:=false" python3 $T/rover_physics_check.py --out $WS/suite_results/physics/physics
# code fingerprint (run in src/cac_sim)
find . -type f -not -name '*.md' -not -path './docs/*' -not -path '*/__pycache__/*' -not -path '*/.pytest_cache/*' \
  | sort | xargs sha256sum | sha256sum | cut -c1-16
```

Never run two suites or sims at once (the runner force-stops Gazebo/Nav2 between seeds).
