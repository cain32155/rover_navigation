# rover_navigation

ROS 2 Jazzy Nav2 configuration and launch files for a differential-drive autonomous sand rover running on a Jetson Orin Nano with an Intel RealSense depth camera (`PointCloud2`) and external localization (`map -> odom -> base_link` + `/odom`).

---

## Package Structure

- `config/nav2_params.yaml`: Nav2 parameters tuned for low CPU usage (`SmacPlanner2D`, `RegulatedPurePursuitController`), gentle sand velocities/accelerations (`velocity_smoother`, `behavior_server`), and depth-camera `PointCloud2` obstacle detection.
- `launch/navigation.launch.py`: Real-rover navigation launch file. Starts **only** the Nav2 navigation servers (`navigation_launch.py` from `nav2_bringup`) without `amcl` or `map_server`.
- `launch/sim_test.launch.py`: Desktop simulation test launch file. Wraps `nav2_bringup`'s `tb3_loopback_simulation.launch.py` using this package's `config/nav2_params.yaml`.

---

## Build Instructions

```bash
cd ~/rover_ws
colcon build --symlink-install --packages-select rover_navigation
source install/setup.bash
```

Verify that the YAML configuration parses cleanly:

```bash
python3 -c "import yaml,sys; yaml.safe_load(open('src/rover_navigation/config/nav2_params.yaml'))"
```

---

## Running on the Real Rover

When the external localization package (UKF + SLAM / AprilTag fallback) and RealSense camera are running and publishing `map -> odom -> base_link`, `/odom`, and `/camera/depth/color/points`:

```bash
source ~/rover_ws/install/setup.bash
ros2 launch rover_navigation navigation.launch.py
```

Optional launch arguments:
- `use_sim_time:=false` (default: `false`)
- `autostart:=true` (default: `true`)
- `params_file:=/path/to/custom_params.yaml` (default: `rover_navigation/config/nav2_params.yaml`)

---

## Desktop Testing with Loopback Simulation (`sim_test.launch.py`)

`tb3_loopback_simulation.launch.py` natively accepts a `params_file` argument (`ros2 launch nav2_bringup tb3_loopback_simulation.launch.py --show-args`) and passes it to `bringup_launch.py`, `map_server`, and `loopback_simulation.launch.py`.

Launch the loopback simulation with our rover parameters:

```bash
source ~/rover_ws/install/setup.bash
ros2 launch rover_navigation sim_test.launch.py
```

### Handling the Initial `map -> odom` Transform Timeout in Loopback Sim

In `tb3_loopback_simulation.launch.py`, `nav2_loopback_sim` provides an `initialpose` subscriber and immediately broadcasts `map -> odom -> base_footprint` at `(0, 0, 0)` on startup, or updates `map -> odom` whenever **2D Pose Estimate** (`/initialpose`) is set in RViz.

If `global_costmap` / `planner_server` waits on `map -> base_link` or times out during bringup before an initial pose is published:

1. **In RViz**: Click the **2D Pose Estimate** button in the top toolbar and click/drag on the map to publish `/initialpose`, then if lifecycle bringup timed out, click **Startup** (or **Reset** -> **Startup**) in the Nav2 RViz panel.
2. **Or via CLI** (useful for headless testing or before RViz interaction):
   ```bash
   ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
   ```
   If `lifecycle_manager_navigation` timed out waiting for transforms, resume it via:
   ```bash
   ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
   ```

---

## Verification Steps

With `sim_test.launch.py` (or `navigation.launch.py`) running, check the lifecycle states of the core servers:

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

Expected output for all three:
```text
active [3]
```

Verify active plugins and key parameters:

```bash
ros2 param get /planner_server GridBased.plugin
# Expected: String value is: nav2_smac_planner::SmacPlanner2D

ros2 param get /controller_server FollowPath.plugin
# Expected: String value is: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
```

---

## Unverified Parameters

All active parameters in `config/nav2_params.yaml` were verified directly against the installed ROS 2 Jazzy binaries and headers under `/opt/ros/jazzy`.

- **Commented-out `spatio_temporal_voxel_layer` (`stvl_layer`) block**: `ros-jazzy-spatio-temporal-voxel-layer` is not currently installed in `/opt/ros/jazzy` on this machine, so its parameter names come from the Nav2 Jazzy STVL documentation and remain commented out in `config/nav2_params.yaml`.
- **Per-behavior linear backup speed parameter in `behavior_server`**: In Nav2 Jazzy (`nav2_behaviors::BackUp` / `DriveOnHeading`), linear backup speed is passed in the `BackUp` Behavior Tree action goal (`backup_speed="0.15"` in the default `navigate_to_pose_w_replanning_and_recovery.xml`) rather than as a ROS parameter on `behavior_server`, and is further clamped by `velocity_smoother` (`min_velocity: [-0.2, 0.0, -0.8]`, `max_accel`, `max_decel`). Therefore no unverified ROS parameter was invented under `behavior_server.backup`.
