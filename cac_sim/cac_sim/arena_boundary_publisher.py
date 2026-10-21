#!/usr/bin/env python3
"""Publish the a-priori arena boundary as an OccupancyGrid on /arena_boundary.

With wall returns filtered out (wall_filter.py), nothing SENSED keeps the
planner off the walls. This grid supplies that from the arena's known
dimensions instead: cells within `keepout_band` of the 9 x 5 m boundary (and
everything outside it) are lethal, the interior is free. Both costmaps load it
through a StaticLayer, which also fixes the global costmap to the whole arena
(no SLAM map growing as the rover explores).

This uses the arena's published size, not any sensing of the walls -- whether
judges accept such a virtual boundary is listed as an open rule question in
the README.
"""
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def boundary_grid(length: float, width: float, band: float, res: float = 0.05, pad: float = 0.30):
    """(data[int8 rows], origin_x, origin_y, nx, ny) for the boundary map."""
    nx = int(round((length + 2 * pad) / res))
    ny = int(round((width + 2 * pad) / res))
    ox, oy = -length / 2.0 - pad, -width / 2.0 - pad
    xs = ox + res * (np.arange(nx) + 0.5)
    ys = oy + res * (np.arange(ny) + 0.5)
    X, Y = np.meshgrid(xs, ys)
    lethal = (np.abs(X) > length / 2.0 - band) | (np.abs(Y) > width / 2.0 - band)
    return np.where(lethal, 100, 0).astype(np.int8), ox, oy, nx, ny


class ArenaBoundaryPublisher(Node):

    def __init__(self):
        super().__init__('arena_boundary_publisher')
        self.declare_parameter('arena.length', 9.0)
        self.declare_parameter('arena.width', 5.0)
        self.declare_parameter('localization.keepout_band', 0.11)
        self.declare_parameter('resolution', 0.05)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(OccupancyGrid, '/arena_boundary', qos)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        data, ox, oy, nx, ny = boundary_grid(g('arena.length'), g('arena.width'),
                                             g('localization.keepout_band'), g('resolution'))
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = float(g('resolution'))
        msg.info.width, msg.info.height = nx, ny
        msg.info.origin.position.x, msg.info.origin.position.y = ox, oy
        msg.info.origin.orientation.w = 1.0
        msg.data = data.flatten().tolist()
        self._msg = msg
        # Published once on a transient-local (latched) topic. Not republished
        # periodically: every new map message makes a StaticLayer re-process it.
        self._publish()

    def _publish(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._msg.info.map_load_time = self._msg.header.stamp
        self._pub.publish(self._msg)


def main():
    rclpy.init()
    node = ArenaBoundaryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
