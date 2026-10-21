import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_rover_nav = get_package_share_directory('rover_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    use_composition = LaunchConfiguration('use_composition')
    map_yaml_file = LaunchConfiguration('map')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_rover_nav, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS 2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the Nav2 stack',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Whether to use composed bringup (False exposes standalone lifecycle nodes)',
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_nav2_bringup, 'maps', 'tb3_sandbox.yaml'),
        description='Full path to map yaml file to load for loopback simulation',
    )

    tb3_loopback_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'tb3_loopback_simulation.launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'autostart': autostart,
            'use_rviz': use_rviz,
            'use_composition': use_composition,
            'map': map_yaml_file,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(tb3_loopback_sim_cmd)

    return ld
