#!/usr/bin/env python3
"""Main launch file for the CAC simulation.

    ros2 launch cac_sim cac_sim.launch.py seed:=42 randomize:=true autonomy:=true use_rviz:=true

Per run, BEFORE Gazebo starts, cac_sim/arena_layout.py generates (from `seed`):
the rover start pose (N/S/E/W heading, room to turn in place), the boulders and
crater depressions (resampled until the real footprint can reach every mission
goal), and a world file with the sand bed tiled around the crater meshes plus
the start-zone ArUco markers. Everything is written to the run directory
(<tmp>/cac_sim_runs/seed_<seed>/) and the layout is printed.

The spawn pose goes ONLY to Gazebo (`create`) and the layout generator -- never
to the autonomy stack, which localizes itself from the markers.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from cac_sim import arena_layout


def _bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('cac_sim')
    arena_yaml = f'{pkg_share}/config/arena_params.yaml'
    params = arena_layout.load_params(arena_yaml)

    seed = int(LaunchConfiguration('seed').perform(context))
    randomize = _bool(context, 'randomize')
    autonomy = _bool(context, 'autonomy')
    use_rviz = _bool(context, 'use_rviz')
    headless = _bool(context, 'headless')
    egl = _bool(context, 'egl')
    material_visual = _bool(context, 'material_visual')
    world_file = LaunchConfiguration('world').perform(context)
    run_dir = LaunchConfiguration('run_dir').perform(context) or os.path.join(
        tempfile.gettempdir(), 'cac_sim_runs', f'seed_{seed}')

    layout = arena_layout.generate_layout(params, seed, randomize)
    world_path = arena_layout.generate_world(params, f'{pkg_share}/worlds/{world_file}', run_dir, layout)
    robot_x, robot_y, robot_yaw = layout.start
    fp = arena_layout.Footprint.from_params(params)
    xs = [c[0] for c in fp.corners(*layout.start)]
    ys = [c[1] for c in fp.corners(*layout.start)]
    print(f'[cac_sim] seed={seed} randomize={randomize} run_dir={run_dir}\n'
          f'[cac_sim] {arena_layout.describe(layout)}\n'
          f'[cac_sim] start footprint x=[{min(xs):.2f}, {max(xs):.2f}] y=[{min(ys):.2f}, {max(ys):.2f}]')

    # Resolve the planner's lattice-primitive path for this install.
    with open(f'{pkg_share}/config/nav2_params.yaml') as f:
        nav2_text = f.read().replace('<NAV2_SMAC_SHARE>', get_package_share_directory('nav2_smac_planner')) \
            .replace('<CAC_SIM_SHARE>', pkg_share)
    nav2_params = os.path.join(run_dir, 'nav2_params.yaml')
    with open(nav2_params, 'w') as f:
        f.write(nav2_text)

    robot_description = ParameterValue(Command(['xacro ', f'{pkg_share}/urdf/rover.urdf.xacro']), value_type=str)

    # headless:=true -> server only (no GUI client). Rendering sensors (camera,
    # gpu_lidar) still need a display; egl:=true adds --headless-rendering, which
    # segfaults the RGB-D camera on some Ogre2/driver combinations (README).
    gz_flags = '-r' + (' -s' if headless else '') + (' --headless-rendering' if egl else '')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': f'{world_path} {gz_flags}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )
    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'cac_arena', '-name', 'cac_rover', '-topic', 'robot_description',
                   '-x', str(robot_x), '-y', str(robot_y), '-z', '0.03', '-Y', str(robot_yaw)],
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': f'{pkg_share}/config/ros_gz_bridge.yaml', 'use_sim_time': True}],
    )
    ground_truth = Node(
        package='cac_sim', executable='obstacle_spawner', output='screen',
        parameters=[{'random_seed': seed, 'randomize': randomize, 'arena_params_file': arena_yaml,
                     'use_sim_time': True}],
    )
    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        parameters=[f'{pkg_share}/config/ekf.yaml'],
    )
    marker_localizer = Node(
        package='cac_sim', executable='marker_localizer', output='screen',
        parameters=[arena_yaml, {'use_sim_time': True}],
    )
    wall_filter = Node(
        package='cac_sim', executable='wall_filter', output='screen',
        parameters=[arena_yaml, {'use_sim_time': True}],
    )
    boundary = Node(
        package='cac_sim', executable='arena_boundary_publisher', output='screen',
        parameters=[arena_yaml, {'use_sim_time': True}],
    )
    crater_detector = Node(
        package='cac_sim', executable='crater_detector', output='screen',
        parameters=[{'use_sim_time': True}],
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch/navigation_launch.py']),
        launch_arguments={'use_sim_time': 'true', 'params_file': nav2_params, 'autostart': 'true'}.items(),
    )
    perception_node = Node(package='cac_sim', executable='perception_node', output='screen',
                           parameters=[{'use_sim_time': True}])
    mission_state_machine = Node(
        package='cac_sim', executable='mission_state_machine', output='screen',
        parameters=[arena_yaml, f'{pkg_share}/config/arm_params.yaml',
                    {'autonomy': autonomy, 'use_sim_time': True}],
    )
    material_visualizer = Node(
        package='cac_sim', executable='material_visualizer', output='screen',
        condition=IfCondition(str(material_visual)),
        parameters=[{'world': 'cac_arena', 'use_sim_time': True}],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', condition=IfCondition(str(use_rviz)),
        arguments=['-d', f'{pkg_share}/rviz/cac_sim.rviz'], parameters=[{'use_sim_time': True}],
    )
    return [gz_sim, robot_state_publisher, spawn_robot, bridge, ground_truth, ekf_node,
            marker_localizer, wall_filter, boundary, crater_detector, nav2, perception_node,
            mission_state_machine, material_visualizer, rviz]


def _localhost_env(context, *args, **kwargs):
    """Keep Gazebo transport and ROS discovery on loopback (localhost_only:=true).

    Found by testing: on a flaky Wi-Fi interface gz-transport logged thousands of
    "multicast: Network is unreachable" errors, the ROS bridge stopped receiving
    /clock, sim time froze, TF lookups blocked and Nav2's bond watchdog shut the
    whole navigation stack down mid-mission. Tools run from another shell need
    the same two variables (README).
    """
    if not _bool(context, 'localhost_only'):
        return []
    return [SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
            SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='42',
                              description='RNG seed for the obstacle layout and the rover start pose'),
        DeclareLaunchArgument('randomize', default_value='true',
                              description='false: no obstacles and a fixed start pose'),
        DeclareLaunchArgument('autonomy', default_value='true',
                              description='false: mission state machine idles (manual/teleop testing)'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value='cac_arena.sdf',
                              description='Base world in share/cac_sim/worlds (the sand bed is regenerated)'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Gazebo server only, no GUI (needs a display for sensors)'),
        DeclareLaunchArgument('egl', default_value='false',
                              description='Add --headless-rendering (known RGB-D camera segfault on some setups)'),
        DeclareLaunchArgument('material_visual', default_value='true',
                              description='Show the SIMULATED sand accounting in Gazebo (visual-only pile/dig marks)'),
        DeclareLaunchArgument('run_dir', default_value='',
                              description='Where generated world/meshes/params go (default <tmp>/cac_sim_runs/seed_N)'),
        DeclareLaunchArgument('localhost_only', default_value='true',
                              description='Keep gz-transport and ROS 2 discovery on 127.0.0.1'),
        OpaqueFunction(function=_localhost_env),
        OpaqueFunction(function=launch_setup),
    ])
