# Autonomous Rover Navigation & CAC 2027 Simulation Workspace

This repository contains the complete autonomous navigation stack and simulation environment for the Caterpillar Autonomy Challenge (CAC) 2027 rover, targeted for **ROS 2 Jazzy**.

---

## Repository Structure

```
.
├── cac_sim/             # Full simulation, arena layout, and autonomous mission state machine
└── rover_navigation/    # Nav2 navigation stack configuration and bringup for the real rover
```

### 1. `cac_sim`
ROS 2 Jazzy + Gazebo Harmonic simulation of the 9 × 5 m competition arena and the 4-wheel skid-steer rover.
- **ArUco Marker Localization** (`marker_localizer.py`): Replaces SLAM with start-zone marker PnP solving to comply with competition rules forbidding wall-based localization.
- **Perception & Filtering**:
  - `wall_filter.py`: Strips arena wall and ground-plane LiDAR hits.
  - `crater_detector.py`: Detects negative obstacles (craters) from RGB-D point clouds and projects their occlusion shadows into the costmap.
- **Mission Autonomy** (`mission_state_machine.py` + `mission_logic.py`): Fully autonomous mission sequencing (`LOCALIZE` → `SURVEY` → `TRAVEL_TO_EXCAVATION` → `EXCAVATION` → `TRAVEL_TO_CONSTRUCTION` → `DUMP` → `RETURN`).
- **Material Accounting** (`material_model.py`): Tracks sand extraction and berm deposition volume estimates.

### 2. `rover_navigation`
The deployable navigation package for the real rover running on an onboard computer (e.g. NVIDIA Jetson Orin Nano).
- Tuned `nav2_params.yaml` featuring **SmacPlanner2D** and **RegulatedPurePursuitController** optimized for sand locomotion.
- Dedicated launch files for hardware navigation (`navigation.launch.py`) and simulation desktop testing (`sim_test.launch.py`).
- Pre-configured behavior trees and costmap configurations.

---

## Performance & Fast-Mission Tuning (5–6 Min Runtime)

The mission parameters in `cac_sim/config/arena_params.yaml` and `cac_sim/config/nav2_params.yaml` have been optimized to reduce overall mission runtime from ~10 minutes down to **5–6 minutes**:

1. **Mid-Path Sweeps Disabled**:
   - `mission.scan_offsets_deg: [0.0]` (disables the periodic $\pm 50^\circ$ scan stops every 1.2 m while keeping a valid `DOUBLE_ARRAY` type in ROS 2 YAML parsing).
2. **Shortest-Path Global Planning & Costmap Tuning**:
   - `inflation_layer.cost_scaling_factor: 3.5` and `SmacPlannerLattice.cost_penalty: 1.5` so the planner routes through direct collision-free central corridors instead of taking long perimeter detours.
3. **Single-Cycle Collect $\rightarrow$ Dump $\rightarrow$ Return Flow**:
   - `mission.num_cycles: 1`, `mission.time_budget_sec: 360.0`, `mission.return_reserve_sec: 90.0`.
   - Early arrival acceptance in `_run_leg()` when the rover is already inside the target zone and a nearby obstacle enters the goal's padding, plus `±0.25 m` lateral candidate goals.
4. **Controller & No-Slip Rotation Speeds**:
   - `desired_linear_vel: 0.30` m/s and `regulated_linear_scaling_min_speed: 0.15` m/s.
   - `rotate_to_heading_angular_vel: 0.35` rad/s and `survey_turn_rate: 0.35` rad/s (capped at the measured no-slip limit on sand to prevent lateral odometry drift).
   - Initial obstacle survey set to 3 headings (`[-45.0, 0.0, 45.0]`) with `0.8 s` hold time.

---

## Build Instructions

```bash
# Clone and build within your workspace root (e.g., ~/rover_ws)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Running

### Simulation
```bash
ros2 launch cac_sim cac_sim.launch.py seed:=42
```
Optional arguments:
- `headless:=true use_rviz:=false` (run without GUI)
- `autonomy:=false` (teleoperation / manual mode)
- `randomize:=false` (fixed start pose, no obstacles)

### Real Rover Navigation
```bash
ros2 launch rover_navigation navigation.launch.py
```
