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
   - `mission.scan_offsets_deg: []` (turned off the periodic $\pm 50^\circ$ scan stops every 1.2 m that added ~15 seconds per stop).
2. **Optimized Time Budget & Capped Cycles**:
   - `mission.num_cycles: 2` (awards full autonomy points according to competition criteria while eliminating runaway multi-cycle loops).
   - `mission.time_budget_sec: 360.0` (6 minutes maximum mission time).
   - `mission.return_reserve_sec: 90.0` (time reserved for returning to the start zone).
3. **Increased Controller Driving Speeds**:
   - `desired_linear_vel: 0.35` m/s (increased from 0.20 m/s for faster transit).
   - `regulated_linear_scaling_min_speed: 0.20` m/s.
4. **Faster Initial Survey & Recovery**:
   - Initial obstacle survey reduced to 3 headings (`[-45.0, 0.0, 45.0]`) with 0.6 s hold time.
   - Survey turn rate increased to `0.50` rad/s.
   - Reverse speed between cycles increased to `0.20` m/s.

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
