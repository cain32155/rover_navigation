#!/usr/bin/env python3
"""Drop LiDAR returns that come from the arena walls: /scan -> /scan_filtered.

Rulebook (Navigation Protocol / Autonomy Rules 3-4): the walls "shall not be
used for the purposes of mapping, autonomous navigation and collision
avoidance", and teams must show their autonomy sensors don't use them. The
walls stay fully physical in Gazebo; this node makes sure no wall return ever
reaches the costmaps, the collision monitor or the obstacle clusterer.

Each beam's hit point is transformed into the map (== arena) frame with the
localized pose at the scan time; any hit within `wall_filter_band` of the known
arena boundary (|x| > L/2 - band or |y| > W/2 - band) is replaced by +inf
("no return"), so it neither marks nor clears. Until localization is available
nothing is published, so nothing downstream can see an unfiltered wall.

Ground returns: the scan plane is only 0.10 m above the sand, so a body pitch
of 1.3 deg (the stowed rover on its rocker springs) puts ground hits at 4.3 m and
the arm's carry/dig poses (2-3 deg) at 2-3 m. Marked as obstacles, those arcs
closed the planner's routes (seed 10: NO_VALID_PATH from everywhere). From the
IMU's gravity-referenced roll/pitch, each beam's ground-intersection range is
computed (ground_range below); returns at or beyond it minus `ground_margin`
become +inf as well -- which also lets those beams clear stale phantom cells.

Limitation (documented in the README): the filter relies on the localized pose.
If localization error exceeds the band, wall returns can leak in as phantom
obstacles near the boundary, and real obstacles closer than the band to a wall
are not seen (they are inside the planner's keepout band anyway).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def ground_range(angles: np.ndarray, roll: float, pitch: float, height: float,
                 spread: float = 0.0) -> np.ndarray:
    """Range at which each beam (angle in the level-mounted scanner's plane) meets
    flat ground `height` below the scanner, for body roll/pitch (rad, ROS
    convention: +pitch = nose down, +roll = left side up), with the beam's lower
    edge `spread` rad below its axis; +inf if it never does."""
    dz = (-math.sin(pitch) * np.cos(angles) + math.cos(pitch) * math.sin(roll) * np.sin(angles)
          - math.sin(spread))
    with np.errstate(divide='ignore'):
        return np.where(dz < -1e-6, height / np.maximum(-dz, 1e-9), np.inf)


class WallFilter(Node):

    def __init__(self):
        super().__init__('wall_filter')
        self.declare_parameter('arena.length', 9.0)
        self.declare_parameter('arena.width', 5.0)
        self.declare_parameter('localization.wall_filter_band', 0.30)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('lidar_height', 0.10)     # scan plane above the sand (urdf lidar mount)
        self.declare_parameter('ground_margin', 0.30)    # m: returns this close to the ground range count as ground
        # The gpu_lidar returns ground ~0.45 deg earlier than its axis would: at rest
        # (IMU pitch 1.32 deg, empty arena) ground came back at 3.21-3.29 m ahead,
        # 3.34-3.46 m at +-20 deg, 3.56-3.75 m at +-30 deg; pitch alone predicts 4.35,
        # 4.50, 4.81 m, pitch + 0.45 deg predicts 3.25, 3.41, 3.62 m.
        self.declare_parameter('ground_beam_spread_deg', 0.45)
        self._rp = None                                  # (roll, pitch) from /imu
        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self._stats = {'scans': 0, 'wall_beams': 0, 'ground_beams': 0}
        self.create_timer(10.0, lambda: self.get_logger().info(f'wall_filter: {self._stats}'))
        self._tf_buffer = Buffer()
        # TF on its own node/thread (node=None): the per-message lookup below waits
        # for the transform at the message stamp, which TF callbacks queued on this
        # node's own executor could not deliver -- measured before this change:
        # marker_localizer lost 62 % of images, wall_filter 55 % of scans.
        self._tf_listener = TransformListener(self._tf_buffer, None, spin_thread=True)
        self._pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self._warned = False

    def _on_imu(self, m: Imu):
        q = m.orientation
        if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
            return
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self._rp = (roll, pitch)

    def _on_scan(self, scan: LaserScan):
        band = self.get_parameter('localization.wall_filter_band').value
        lim_x = self.get_parameter('arena.length').value / 2.0 - band
        lim_y = self.get_parameter('arena.width').value / 2.0 - band
        try:
            tf = self._tf_buffer.lookup_transform(self.get_parameter('map_frame').value, scan.header.frame_id,
                                                  Time.from_msg(scan.header.stamp),
                                                  timeout=rclpy.duration.Duration(seconds=0.05))
        except TransformException:
            if not self._warned:
                self.get_logger().info('No localized pose yet -- /scan_filtered withheld until localization.')
                self._warned = True
            return
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        r = np.asarray(scan.ranges, dtype=np.float32)
        ang = scan.angle_min + scan.angle_increment * np.arange(len(r)) + yaw
        with np.errstate(invalid='ignore'):
            px = tx + r * np.cos(ang)
            py = ty + r * np.sin(ang)
            wall = np.isfinite(r) & ((np.abs(px) > lim_x) | (np.abs(py) > lim_y))
        ground = np.zeros_like(wall)
        if self._rp is not None:
            beam = scan.angle_min + scan.angle_increment * np.arange(len(r))
            rg = ground_range(beam, self._rp[0], self._rp[1], self.get_parameter('lidar_height').value,
                              math.radians(self.get_parameter('ground_beam_spread_deg').value))
            with np.errstate(invalid='ignore'):
                ground = np.isfinite(r) & ~wall & (r >= rg - self.get_parameter('ground_margin').value)
        self._stats['scans'] += 1
        self._stats['wall_beams'] += int(wall.sum())
        self._stats['ground_beams'] += int(ground.sum())
        out = LaserScan()
        out.header = scan.header
        out.angle_min, out.angle_max, out.angle_increment = scan.angle_min, scan.angle_max, scan.angle_increment
        out.time_increment, out.scan_time = scan.time_increment, scan.scan_time
        out.range_min, out.range_max = scan.range_min, scan.range_max
        r = r.copy()
        r[wall | ground] = np.inf
        out.ranges = r.tolist()
        out.intensities = list(scan.intensities)
        self._pub.publish(out)


def main():
    rclpy.init()
    node = WallFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
