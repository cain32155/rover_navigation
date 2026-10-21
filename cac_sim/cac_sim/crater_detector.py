#!/usr/bin/env python3
"""Detect craters (negative obstacles) from the RGB-D point cloud.

A planar LiDAR at 0.10 m can never see a depression -- everything it scans is
above the ground -- so craters are found with the depth camera:

  /camera/points -> subsample -> transform to base_footprint ->
  fit the local ground plane (least squares on near-ground points, refit on
  inliers; this absorbs the chassis pitch that the 2D EKF does not model) ->
  points more than `depth_threshold` BELOW the plane, inside `max_range`,
  outside the rover's own body box -> map-frame 5 cm cells ->
  + the occlusion shadow of those cells (see frame_cells) ->
  a cell is confirmed after hits in `min_hits` different frames ->
  /crater_cells (PointCloud2 in map, z = 0.05) for the costmaps' crater_layer.

Occlusion shadow: from the approach side the camera sees only the far wall of a
bowl -- the near slope drops away behind the rim -- so a 40 cm crater was mapped as
13-17 of its ~50-78 cells and a wheel crossed the unmapped slope (seed 12: crater_2,
5.9 cm deep, 46 % mapped, a left wheel through its middle). Every cell between a
below-plane cell and the camera that this frame did NOT see as level ground is
therefore counted too, up to `shadow_max_len` (the rulebook's 40-50 cm crater width),
stopping at the first cell seen as ground (the near rim). Flat ground in view is
always seen, so the shadow fills only what a depression hides.

Confirmed cells are kept (the crater_layer is marking-only, so LiDAR rays that
pass over a crater can never clear it) until marker_localizer announces a
re-initialisation, after which they are re-detected from fresh views. A single noisy frame cannot create a
cell. Points above the plane are ignored here (boulders come from the LiDAR).

Before the first marker fix there is no map frame, yet the localization spin is
when the camera sweeps the ground around the start zone (seed 10: a crater seen
in ~20 frames during the spin and never again before a wheel dropped into it).
Hits from that period are counted in odom cells and moved into the map, with
the first map->odom, when it appears; the rover only turns in place meanwhile,
so odom has barely drifted.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from cac_sim.marker_localizer import _tf_to_T


def _cloud_xyz(msg: PointCloud2, stride: int) -> np.ndarray:
    """Nx3 float array from an (organized or not) PointCloud2 with float32 x/y/z."""
    off = {f.name: f.offset for f in msg.fields}
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.height > 1:
        buf = buf.reshape(msg.height, msg.row_step)[::stride, :msg.width * msg.point_step]
        buf = buf.reshape(-1, msg.width, msg.point_step)[:, ::stride, :].reshape(-1, msg.point_step)
    else:
        buf = buf[:msg.width * msg.point_step].reshape(-1, msg.point_step)[::stride * stride]
    pts = np.stack([buf[:, off[a]:off[a] + 4].copy().view(np.float32).ravel() for a in 'xyz'], axis=1)
    return pts[np.all(np.isfinite(pts), axis=1)]


def _cells(pts_xy: np.ndarray, cell: float, min_pts: int) -> set:
    if len(pts_xy) == 0:
        return set()
    c, n = np.unique(np.floor(pts_xy / cell).astype(np.int64), axis=0, return_counts=True)
    return {(int(i), int(j)) for (i, j), k in zip(c, n) if k >= min_pts}


def frame_cells(pb: np.ndarray, coef, T: np.ndarray, depth_threshold: float, cell: float,
                min_pts: int, shadow_max_len: float, observable=None):
    """Cells (in T's frame) one camera frame marks as crater: (below-plane cells, shadow cells).

    pb: Nx3 points in base_footprint (already range/self-box filtered); coef: ground
    plane (a, b, c) in base_footprint; T: 4x4 base_footprint -> map (or odom).
    Shadows walk from each below-plane cell toward the camera (CAMERA_XY through T),
    i.e. back along the rays that could not see the ground there. The walk also
    stops where the detector does not look at all (observable(x, y) in
    base_footprint is False: inside min_range, beyond max_range, or in the rover's
    self box) -- ground there is unseen because it is ignored, not because it is hidden.
    """
    resid = pb[:, 2] - (coef[0] * pb[:, 0] + coef[1] * pb[:, 1] + coef[2])
    low = pb[(resid < -depth_threshold) & (resid > -0.5)]
    if len(low) == 0:
        return set(), set()
    ground = pb[np.abs(resid) < depth_threshold]
    R, t = T[:2, :2], T[:2, 3]
    low_c = _cells(low[:, :2] @ R.T + t, cell, min_pts)
    if shadow_max_len <= 0 or not low_c:
        return low_c, set()
    ground_c = _cells(ground[:, :2] @ R.T + t, cell, 1) - low_c
    cam = T @ np.array([CAMERA_XY[0], CAMERA_XY[1], 0.0, 1.0])
    Rinv = R.T
    shadow = set()
    step = cell / 2.0
    for i, j in low_c:
        p = np.array([(i + 0.5) * cell, (j + 0.5) * cell])
        d = cam[:2] - p
        n = np.hypot(*d)
        if n < 1e-6:
            continue
        d /= n
        k = 1
        while k * step <= min(shadow_max_len, n):
            q = p + d * k * step
            key = (int(np.floor(q[0] / cell)), int(np.floor(q[1] / cell)))
            if key in ground_c:
                break
            if observable is not None:
                qb = Rinv @ (q - t)
                if not observable(qb[0], qb[1]):
                    break
            if key not in low_c:
                shadow.add(key)
            k += 1
    return low_c, shadow


# camera position in base_footprint (urdf: camera_link at x 0.47, z 0.33); only x/y are used
CAMERA_XY = (0.47, 0.0)


def fit_ground(pts: np.ndarray, band: float = 0.06, inlier: float = 0.02):
    """Least-squares plane z = a*x + b*y + c through near-ground points; returns (a, b, c) or None."""
    sel = pts[np.abs(pts[:, 2]) < band]
    if len(sel) < 50:
        return None
    for _ in range(2):
        A = np.column_stack([sel[:, 0], sel[:, 1], np.ones(len(sel))])
        coef, *_ = np.linalg.lstsq(A, sel[:, 2], rcond=None)
        res = sel[:, 2] - A @ coef
        sel = sel[np.abs(res) < inlier]
        if len(sel) < 50:
            return None
    return coef


class CraterDetector(Node):

    def __init__(self):
        super().__init__('crater_detector')
        dp = self.declare_parameter
        dp('base_frame', 'base_footprint')
        dp('map_frame', 'map')
        dp('odom_frame', 'odom')
        dp('stride', 4)                  # use every n-th row and column of the cloud
        dp('process_rate', 8.0)          # Hz (craters are often seen only briefly, obliquely)
        dp('min_range', 0.3)             # m, horizontal distance from base_footprint
        dp('max_range', 2.2)
        # m below the fitted ground plane (sim depth is noise-free: plane residual < 1 mm).
        # 1.5 cm instead of 2.5: a 5-6 cm deep crater went from 0 / 27-35 % mapped to
        # 46 / 35-44 % with no false cells (crater_view_test, seed 12, 2.0/1.5/1.0 m);
        # a wheel had crossed the unmarked slope of such a crater.
        dp('depth_threshold', 0.015)
        dp('cell_size', 0.05)
        dp('min_hits', 2)                # distinct frames before a cell is confirmed
        dp('min_points_per_cell', 2)     # points in one frame to count a hit
        dp('shadow_max_len', 0.45)       # m; 0 disables the occlusion shadow (see module doc)
        # Rover's own body (base_footprint frame) incl. a deployed arm/bucket: ignore.
        dp('self_box', [-0.75, 1.30, -0.50, 0.50])   # x_min, x_max, y_min, y_max
        self._tf_buffer = Buffer()
        # TF on its own node and thread (node=None): the lookups below wait for
        # transforms, which TF callbacks queued behind this node's point-cloud
        # callback could never deliver in time -- 75-87 % of clouds failed with
        # "extrapolation into the future" although /tf itself lagged sim time by
        # only 6 ms (median). Passing `self` with spin_thread=True does NOT help:
        # rclpy.spin(self) takes the node back into the main executor.
        self._tf_listener = TransformListener(self._tf_buffer, None, spin_thread=True)
        self._pub = self.create_publisher(PointCloud2, '/crater_cells', 2)
        self.create_subscription(PointCloud2, '/camera/points', self._on_cloud, 2)
        self.create_subscription(String, '/localization/reinitialized', self._on_reinit, 2)
        self._last_t = 0.0
        self._hits = {}        # (i, j) -> number of frames with a hit
        self._confirmed = set()
        self._odom_hits = {}   # same, in odom cells, while there is no map frame yet
        self._pre = {'frames': 0, 'tf_fail': 0, 'with_hits': 0}   # pre-fix frame counters (logged at merge)
        self._stats = {'clouds': 0, 'tf_fail': 0, 'no_plane': 0, 'frames': 0, 'with_hits': 0}
        self.create_timer(5.0, lambda: self.get_logger().info(
            f'crater_detector: {self._stats}, {len(self._confirmed)} confirmed cells'))
        self._frames = 0

    def _observable_fn(self):
        """f(x, y): a base_footprint point the detector looks at (range band, outside the self box)."""
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        r0, r1 = g('min_range'), g('max_range')
        x0, x1, y0, y1 = g('self_box')
        return lambda x, y: r0 < np.hypot(x, y) < r1 and not (x0 < x < x1 and y0 < y < y1)

    def _on_reinit(self, _msg):
        """Localization jumped: cells recorded since the last good fix may be displaced."""
        self.get_logger().warn(f'Localization re-initialised: dropping {len(self._confirmed)} crater cells.')
        self._hits.clear()
        self._confirmed.clear()

    def _count_hits(self, hits: dict, pm: np.ndarray):
        """Add one frame's below-plane points (frame coordinates) to per-cell frame counts."""
        self._add_frame(hits, _cells(pm[:, :2], self.get_parameter('cell_size').value,
                                     self.get_parameter('min_points_per_cell').value))

    @staticmethod
    def _add_frame(hits: dict, cells: set):
        """One frame counts at most once per cell."""
        for key in cells:
            hits[key] = hits.get(key, 0) + 1

    def _merge_odom_hits(self, T_map_odom: np.ndarray):
        """Move the pre-fix odom-cell counts into map cells (cell centres transformed)."""
        cs = self.get_parameter('cell_size').value
        for (i, j), n in self._odom_hits.items():
            p = T_map_odom @ np.array([(i + 0.5) * cs, (j + 0.5) * cs, 0.0, 1.0])
            key = (int(np.floor(p[0] / cs)), int(np.floor(p[1] / cs)))
            self._hits[key] = self._hits.get(key, 0) + n
            if self._hits[key] >= self.get_parameter('min_hits').value:
                self._confirmed.add(key)
        self.get_logger().info(f'Map frame available: merged {len(self._odom_hits)} pre-fix odom cells, '
                               f'{len(self._confirmed)} confirmed crater cells (pre-fix frames {self._pre}).')
        self._odom_hits = {}

    def _on_cloud(self, msg: PointCloud2):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_t < 1.0 / self.get_parameter('process_rate').value:
            return
        self._last_t = now
        self._stats['clouds'] += 1
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        stamp = Time.from_msg(msg.header.stamp)
        wait = rclpy.duration.Duration(seconds=0.05)
        try:
            T_base_cloud = _tf_to_T(self._tf_buffer.lookup_transform(g('base_frame'), msg.header.frame_id, Time()))
        except TransformException:
            return
        T_map_base = T_odom_base = None
        try:
            # no map frame before the first marker fix: do not wait for it
            if not self._tf_buffer.can_transform(g('map_frame'), g('base_frame'), Time()):
                raise TransformException('no map frame yet')
            T_map_base = _tf_to_T(self._tf_buffer.lookup_transform(g('map_frame'), g('base_frame'), stamp,
                                                                   timeout=wait))
        except TransformException:
            try:
                T_odom_base = _tf_to_T(self._tf_buffer.lookup_transform(g('odom_frame'), g('base_frame'), stamp,
                                                                        timeout=wait))
            except TransformException as e:
                self._pre['tf_fail'] += 1
                self._stats['tf_fail'] += 1
                self._stats['last_tf_error'] = str(e)[:160]
                return
        if T_map_base is not None and self._odom_hits:
            try:
                self._merge_odom_hits(_tf_to_T(self._tf_buffer.lookup_transform(
                    g('map_frame'), g('odom_frame'), Time())))
            except TransformException:
                pass
        pts = _cloud_xyz(msg, int(g('stride')))
        if len(pts) == 0:
            return
        pb = pts @ T_base_cloud[:3, :3].T + T_base_cloud[:3, 3]
        rng = np.hypot(pb[:, 0], pb[:, 1])
        x0, x1, y0, y1 = g('self_box')
        keep = (rng > g('min_range')) & (rng < g('max_range')) & \
            ~((pb[:, 0] > x0) & (pb[:, 0] < x1) & (pb[:, 1] > y0) & (pb[:, 1] < y1))
        pb = pb[keep]
        coef = fit_ground(pb)
        if coef is None:
            self._stats['no_plane'] += 1
            return
        low_c, shadow_c = frame_cells(pb, coef, T_map_base if T_map_base is not None else T_odom_base,
                                      g('depth_threshold'), g('cell_size'), g('min_points_per_cell'),
                                      g('shadow_max_len'), self._observable_fn())
        self._frames += 1
        self._stats['frames'] += 1
        self._stats['with_hits'] += int(bool(low_c))
        self._stats['shadow_cells'] = self._stats.get('shadow_cells', 0) + len(shadow_c)
        if T_map_base is None:
            self._pre['frames'] += 1
            self._pre['with_hits'] += int(bool(low_c))
            self._add_frame(self._odom_hits, low_c | shadow_c)
            return
        if low_c:
            self._add_frame(self._hits, low_c | shadow_c)
            self._confirmed |= {k for k, n in self._hits.items() if n >= g('min_hits')}
        self._publish(msg.header.stamp)

    def _publish(self, stamp):
        cs = self.get_parameter('cell_size').value
        pts = np.array([[(i + 0.5) * cs, (j + 0.5) * cs, 0.05] for i, j in self._confirmed],
                       dtype=np.float32).reshape(-1, 3)
        msg = PointCloud2()
        msg.header = Header(frame_id=self.get_parameter('map_frame').value, stamp=stamp)
        msg.height, msg.width = 1, len(pts)
        msg.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                      for k, n in enumerate('xyz')]
        msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 12, 12 * len(pts), True
        msg.data = pts.tobytes()
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = CraterDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
