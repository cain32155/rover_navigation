#!/usr/bin/env python3
"""Ground-truth obstacle markers for RViz, and a layout CLI.

Obstacles are no longer spawned into a running Gazebo: since craters became
real depressions, the sand bed has to be generated with holes for them before
Gazebo starts, so the launch file bakes the whole layout into a per-run world
(cac_sim/arena_layout.py). This node regenerates the same seeded layout and
publishes it on /arena/ground_truth_obstacles (MarkerArray, latched) purely for
visualization and debugging. Nothing in the autonomy stack subscribes to it.

CLI (no ROS needed):  ros2 run cac_sim obstacle_spawner --print <seed> [<seed> ...]
"""
import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from cac_sim import arena_layout


class GroundTruthObstacles(Node):

    def __init__(self):
        super().__init__('obstacle_spawner')
        self.declare_parameter('random_seed', 42)
        self.declare_parameter('randomize', True)
        self.declare_parameter('arena_params_file', '')
        path = self.get_parameter('arena_params_file').value or os.path.join(
            get_package_share_directory('cac_sim'), 'config', 'arena_params.yaml')
        p = arena_layout.load_params(path)
        layout = arena_layout.generate_layout(p, int(self.get_parameter('random_seed').value),
                                              bool(self.get_parameter('randomize').value))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        pub = self.create_publisher(MarkerArray, '/arena/ground_truth_obstacles', qos)
        arr = MarkerArray()
        for k, ob in enumerate(layout.obstacles):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id = 'ground_truth', k
            m.type = Marker.SPHERE if ob.kind == 'boulder' else Marker.CYLINDER
            m.pose.position.x, m.pose.position.y = ob.x, ob.y
            m.pose.position.z = ob.z if ob.kind == 'boulder' else -ob.depth / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = ob.diameter
            m.scale.z = ob.diameter if ob.kind == 'boulder' else ob.depth
            m.color.a = 0.35
            m.color.r, m.color.g, m.color.b = (0.6, 0.6, 0.6) if ob.kind == 'boulder' else (0.9, 0.3, 0.1)
            arr.markers.append(m)
        pub.publish(arr)
        self.get_logger().info('Ground-truth layout (visualization only):\n' + arena_layout.describe(layout))


def main():
    if '--print' in sys.argv:
        path = os.path.join(get_package_share_directory('cac_sim'), 'config', 'arena_params.yaml')
        p = arena_layout.load_params(path)
        for s in sys.argv[sys.argv.index('--print') + 1:]:
            print(arena_layout.describe(arena_layout.generate_layout(p, int(s))))
        return
    rclpy.init()
    node = GroundTruthObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
