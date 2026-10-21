#!/usr/bin/env python3
"""Start-zone ArUco marker localization: publishes map -> odom.

Replaces slam_toolbox. The rulebook forbids using the arena walls for mapping,
navigation or collision avoidance, and with wall returns removed a 2D LiDAR
sees only three ~0.3 m boulders -- far too little for scan matching. The
rulebook's own answer is navigation aids: ArUco/AprilTag markers placed in the
starting area (see config/arena_params.yaml `markers:`).

Pipeline per camera frame:
  detect markers -> solvePnP (IPPE_SQUARE, both solutions) for each marker ->
  pick the solution consistent with the camera's known mounting height/tilt ->
  T_map_base = T_map_marker * T_marker_cam * T_cam_base -> 2D (x, y, yaw) ->
  candidate T_map_odom = T_map_base * inv(T_odom_base at the image stamp).
The first `min_initial_fixes` candidates are combined by median into the
initial estimate; later candidates are blended in with gain `update_gain`
(outliers beyond `gate_xy`/`gate_yaw` are counted, and `reinit_after`
consecutive consistent outliers re-initialise). Between sightings map->odom is
held constant, so the rover dead-reckons on the EKF (wheel odometry + gyro).

The map frame IS the arena frame, so mission goals need no conversion and the
node never needs the spawn pose.
"""
import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def _rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _tf_to_T(tf):
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    t = tf.transform.translation
    return _T(R, [t.x, t.y, t.z])


def _yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])


def marker_frame_in_map(x, y, z, yaw):
    """OpenCV marker frame (x right, y up, z out of the face) expressed in map,
    for a marker whose face points along `yaw`."""
    n = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(-n, up)          # viewer looking at the face (along -n): right-hand side
    return _T(np.column_stack([right, up, n]), [x, y, z])


class MarkerLocalizer(Node):

    def __init__(self):
        super().__init__('marker_localizer')
        dp = self.declare_parameter
        dp('markers.dictionary', 'DICT_4X4_50')
        dp('markers.size', 0.20)
        dp('markers.ids', [0, 1, 2])
        dp('markers.poses', [0.0] * 12)
        dp('camera_frame', 'camera_optical_link')
        dp('base_frame', 'base_footprint')
        dp('odom_frame', 'odom')
        dp('map_frame', 'map')
        dp('max_range', 4.5)               # ignore markers farther than this (m)
        dp('min_marker_px', 18.0)          # ignore markers smaller than this edge length (px)
        dp('min_initial_fixes', 6)
        dp('single_fallback_fixes', 40)
        dp('update_gain', 0.3)            # multi-marker fixes
        dp('single_marker_gain', 0.08)    # single-marker fixes (yaw error of a few deg)
        # After the initial fix, single-marker fixes are used only within this range
        # (0 = never). Measured with tools/marker_range_test.py (one marker in view,
        # ground truth): 0.5-1.0 m -> 0.085 m / 5.3 deg, 2.5-3.0 m -> 0.09-0.16 m /
        # 2-4 deg, 3.0-3.5 m -> 0.20-0.28 m / 4-5 deg. They pass the outlier gate and
        # pulled the estimate 4 deg / 0.3-0.45 m off, while multi-marker fixes are
        # good to ~1 cm / 0.1 deg and odometry drifts a few cm per leg.
        dp('single_max_range', 0.0)
        # Same for multi-marker fixes, by marker count. Measured against ground truth
        # (gt_monitor candidates, seed 10 runs r3_1..3): 3 markers at 4.0-4.5 m ->
        # 0.8 cm median; 2 markers at 3.5 m -> 2-8 cm, at 4.0-4.5 m -> 0.15-0.35 m
        # (two markers on one wall are nearly collinear seen from afar); those fixes
        # dragged the estimate 0.35 m off on a return leg.
        dp('pair_max_range', 3.5)         # exactly two markers
        dp('multi_max_range', 4.5)        # three or more
        dp('max_turn_rate', 0.25)         # rad/s: no fixes while turning faster
        dp('gate_xy', 0.5)
        dp('gate_yaw', 0.25)
        dp('reinit_after', 6)
        dp('publish_rate', 30.0)
        # map->odom is stamped this far in the future (as AMCL does): the transform
        # only changes on marker fixes, and without the offset consumers looking up
        # map->base at the latest odom time hit "extrapolation into the future".
        dp('transform_tolerance', 0.2)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, g('markers.dictionary')))
        self._det_params = (cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create')
                            else cv2.aruco.DetectorParameters())
        self._det_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._size = float(g('markers.size'))
        flat = [float(v) for v in g('markers.poses')]
        self._markers = {int(i): marker_frame_in_map(*flat[4 * k:4 * k + 4])
                         for k, i in enumerate(g('markers.ids'))}
        s = self._size / 2.0
        self._obj_pts = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)
        # the same corners in the map frame, for joint multi-marker PnP
        self._corners_map = {mid: (np.c_[self._obj_pts, np.ones(4)] @ T.T)[:, :3]
                             for mid, T in self._markers.items()}

        self._bridge = CvBridge()
        self._K = None
        self._D = None
        self._tf_buffer = Buffer()
        # TF on its own node/thread (node=None): the per-message lookup below waits
        # for the transform at the message stamp, which TF callbacks queued on this
        # node's own executor could not deliver -- measured before this change:
        # marker_localizer lost 62 % of images, wall_filter 55 % of scans.
        self._tf_listener = TransformListener(self._tf_buffer, None, spin_thread=True)
        self._br = TransformBroadcaster(self)
        self._status_pub = self.create_publisher(String, '/localization/status', 10)
        # Re-initialisation means everything recorded in the map frame since the
        # last good fix may be displaced (phantom obstacles from leaked wall
        # returns, shifted crater cells): announce it and clear the costmaps.
        self._reinit_pub = self.create_publisher(String, '/localization/reinitialized', 10)
        # One JSON line per marker detection that produced a pose candidate (ids,
        # multi/single, implied base pose, marker range, outcome) -- for validation
        # tools; the estimator does not read it.
        self._cand_pub = self.create_publisher(String, '/localization/candidates', 50)
        self._counts = {'images': 0, 'tf_fail': 0, 'turning': 0, 'candidates': 0}
        self._clear_clients = [self.create_client(ClearEntireCostmap, n) for n in
                               ('/global_costmap/clear_entirely_global_costmap',
                                '/local_costmap/clear_entirely_local_costmap')]

        self._estimate = None        # (x, y, yaw) of map->odom
        self._initial = []
        self._outliers = []
        self._fixes = 0
        self._last_fix_time = None
        self._last_detection = {}

        self._yaw_rate = 0.0
        self.create_subscription(Odometry, '/odometry/filtered',
                                 lambda m: setattr(self, '_yaw_rate', m.twist.twist.angular.z), 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._on_info, 5)
        self.create_subscription(Image, '/camera/image', self._on_image, 2)
        self.create_timer(1.0 / float(g('publish_rate')), self._publish)
        self.create_timer(1.0, self._publish_status)

    def _on_info(self, msg: CameraInfo):
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._D = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)

    # ----------------------------------------------------------------- vision

    def _on_image(self, msg: Image):
        if self._K is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        self._counts['images'] += 1
        try:
            T_base_cam = _tf_to_T(self._tf_buffer.lookup_transform(
                self.get_parameter('base_frame').value, self.get_parameter('camera_frame').value, Time()))
            T_odom_base = _tf_to_T(self._tf_buffer.lookup_transform(
                self.get_parameter('odom_frame').value, self.get_parameter('base_frame').value, stamp,
                timeout=rclpy.duration.Duration(seconds=0.05)))
        except TransformException:
            self._counts['tf_fail'] += 1
            return
        gray = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._det_params)
        if ids is None:
            return
        if abs(self._yaw_rate) > self.get_parameter('max_turn_rate').value:
            self._counts['turning'] += 1
            return          # image/odometry timing errors are amplified while turning
        good = []
        for c, mid in zip(corners, ids.flatten()):
            mid = int(mid)
            if mid not in self._markers:
                continue
            img_pts = c.reshape(4, 2).astype(np.float64)
            edge = np.mean(np.linalg.norm(img_pts - np.roll(img_pts, 1, axis=0), axis=1))
            if edge >= self.get_parameter('min_marker_px').value:
                good.append((mid, img_pts, edge))
        if not good:
            return
        if len(good) >= 2:
            T_base = self._solve_multi(good, T_base_cam)
            gain = self.get_parameter('update_gain').value
        else:
            T_base = self._solve(good[0][1], good[0][0], T_base_cam)
            gain = self.get_parameter('single_marker_gain').value
        if T_base is None:
            return
        bx, by, byaw = T_base[0, 3], T_base[1, 3], _yaw_of(T_base)
        # candidate map->odom = T_map_base * inv(T_odom_base), all planar
        oyaw = _yaw_of(T_odom_base)
        myaw = byaw - oyaw
        ox, oy = T_odom_base[0, 3], T_odom_base[1, 3]
        c2, s2 = math.cos(myaw), math.sin(myaw)
        cand = (bx - (c2 * ox - s2 * oy), by - (s2 * ox + c2 * oy), math.atan2(math.sin(myaw), math.cos(myaw)))
        self._last_detection = {'ids': [g[0] for g in good],
                                'base': [round(bx, 3), round(by, 3), round(math.degrees(byaw), 1)],
                                'edge_px': [round(float(g[2]), 1) for g in good]}
        rng = float(np.mean([math.hypot(self._markers[m][0, 3] - bx, self._markers[m][1, 3] - by)
                             for m, _, _ in good]))
        self._counts['candidates'] += 1
        limit = self.get_parameter('single_max_range' if len(good) == 1 else
                                   'pair_max_range' if len(good) == 2 else 'multi_max_range').value
        if self._estimate is not None and rng > limit:
            outcome = 'rejected_range'
        else:
            outcome = self._integrate(cand, gain, multi=len(good) >= 2)
        self._cand_pub.publish(String(data=json.dumps({
            't': stamp.nanoseconds / 1e9, 'ids': [g[0] for g in good], 'multi': len(good) >= 2,
            'base': [round(bx, 4), round(by, 4), round(byaw, 5)], 'range': round(rng, 3),
            'edge_px': [round(float(g[2]), 1) for g in good], 'outcome': outcome})))

    def _plausible(self, T_map_cam, T_base_cam):
        T_map_base = T_map_cam @ np.linalg.inv(T_base_cam)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, T_map_base[2, 2]))))
        ok = abs(T_map_cam[2, 3] - T_base_cam[2, 3]) < 0.25 and tilt < 12.0
        return ok, T_map_base, tilt

    def _solve_multi(self, good, T_base_cam):
        """Joint PnP over every corner of every visible marker (map-frame object
        points): no planar flip ambiguity, and markers on different walls are not
        coplanar, which constrains yaw far better than one 20 cm square."""
        obj = np.vstack([self._corners_map[mid] for mid, _, _ in good])
        img = np.vstack([pts for _, pts, _ in good])
        ok, rvec, tvec = cv2.solvePnP(obj, img, self._K, self._D, flags=cv2.SOLVEPNP_SQPNP)
        if not ok:
            return None
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, self._K, self._D, rvec, tvec)
        R, _ = cv2.Rodrigues(rvec)
        T_map_cam = np.linalg.inv(_T(R, tvec.flatten()))
        if np.linalg.norm(T_map_cam[:3, 3] - obj.mean(axis=0)) > self.get_parameter('max_range').value + 1.0:
            return None
        ok, T_map_base, _ = self._plausible(T_map_cam, T_base_cam)
        return T_map_base if ok else None

    def _solve(self, img_pts, mid, T_base_cam):
        """Robot base pose in map from one marker, or None. IPPE_SQUARE returns the
        two planar-pose solutions; keep the one whose implied camera pose matches
        the known mount (height above ground and roll), which removes the classic
        single-marker flip ambiguity."""
        ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(self._obj_pts, img_pts, self._K, self._D,
                                                     flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        expected_cam_z = T_base_cam[2, 3]
        best, best_score = None, 1e9
        for rvec, tvec, err in zip(rvecs, tvecs, np.array(errs).flatten()):
            dist = float(np.linalg.norm(tvec))
            if dist > self.get_parameter('max_range').value:
                continue
            R, _ = cv2.Rodrigues(rvec)
            T_cam_marker = _T(R, tvec.flatten())
            T_map_cam = self._markers[mid] @ np.linalg.inv(T_cam_marker)
            T_map_base = T_map_cam @ np.linalg.inv(T_base_cam)
            # base_footprint should sit on the ground, upright
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, T_map_base[2, 2]))))
            # Reprojection error (px) dominates: the wrong flip solution typically
            # reprojects 5-10x worse (0.3-0.45 px vs 0.02-0.06 px measured).
            score = float(err) * 2.0 + abs(T_map_base[2, 3]) * 5.0 + tilt * 0.1
            if score < best_score and abs(T_map_cam[2, 3] - expected_cam_z) < 0.25 and tilt < 12.0:
                best, best_score = T_map_base, score
        return best

    # ------------------------------------------------------------- estimation

    def _integrate(self, cand, gain, multi):
        self._fixes += 1
        self._last_fix_time = self.get_clock().now()
        if self._estimate is None:
            self._initial.append((cand, multi))
            n_multi = sum(1 for _, m in self._initial if m)
            # Initial fix from >= min_initial_fixes/2 MULTI-marker candidates
            # (measured: 0.5-1.5 cm, 0.14 deg). Single-marker candidates are used
            # only as a last resort after `single_fallback_fixes` of them (measured:
            # one single-marker initial fix was 0.85 m / 13 deg off - planar PnP
            # flip ambiguity).
            need = self.get_parameter('min_initial_fixes').value
            if n_multi >= need // 2 or len(self._initial) >= self.get_parameter('single_fallback_fixes').value:
                use = [c for c, m in self._initial if m] if n_multi >= need // 2 else [c for c, _ in self._initial]
                a = np.array(use)
                yaw = math.atan2(np.median(np.sin(a[:, 2])), np.median(np.cos(a[:, 2])))
                self._estimate = (float(np.median(a[:, 0])), float(np.median(a[:, 1])), yaw)
                self.get_logger().info(f'Initial marker fix: map->odom = ({self._estimate[0]:.3f}, '
                                       f'{self._estimate[1]:.3f}, {math.degrees(yaw):.1f} deg) '
                                       f'from {len(use)} detections ({n_multi} multi-marker)')
                return 'initial_fix'
            return 'initial'
        ex, ey, eyaw = self._estimate
        dxy = math.hypot(cand[0] - ex, cand[1] - ey)
        dyaw = abs(math.atan2(math.sin(cand[2] - eyaw), math.cos(cand[2] - eyaw)))
        if dxy > self.get_parameter('gate_xy').value or dyaw > self.get_parameter('gate_yaw').value:
            self._outliers.append(cand)
            if len(self._outliers) >= self.get_parameter('reinit_after').value:
                a = np.array(self._outliers)
                if np.ptp(a[:, 0]) < 0.3 and np.ptp(a[:, 1]) < 0.3:
                    self.get_logger().warn('Consistent marker fixes disagree with the estimate -- re-initialising '
                                           '(clearing costmaps and crater memory).')
                    self._estimate = None
                    self._initial = [(c, True) for c in self._outliers]
                    self._reinit_pub.publish(String(data='reinitialized'))
                    for cl in self._clear_clients:
                        if cl.service_is_ready():
                            cl.call_async(ClearEntireCostmap.Request())
                    self._outliers = []
                    return 'reinit'
                self._outliers = []
            return 'outlier'
        self._outliers = []
        k = gain
        yaw = eyaw + k * math.atan2(math.sin(cand[2] - eyaw), math.cos(cand[2] - eyaw))
        self._estimate = (ex + k * (cand[0] - ex), ey + k * (cand[1] - ey), math.atan2(math.sin(yaw), math.cos(yaw)))
        return 'update'

    def _publish(self):
        if self._estimate is None:
            return
        x, y, yaw = self._estimate
        t = TransformStamped()
        t.header.stamp = (self.get_clock().now() + rclpy.duration.Duration(
            seconds=self.get_parameter('transform_tolerance').value)).to_msg()
        t.header.frame_id = self.get_parameter('map_frame').value
        t.child_frame_id = self.get_parameter('odom_frame').value
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self._br.sendTransform(t)

    def _publish_status(self):
        age = None if self._last_fix_time is None else \
            (self.get_clock().now() - self._last_fix_time).nanoseconds / 1e9
        self._status_pub.publish(String(data=json.dumps({
            'initialized': self._estimate is not None, 'fixes': self._fixes,
            'last_fix_age_sec': None if age is None else round(age, 1),
            'map_to_odom': None if self._estimate is None else [round(v, 4) for v in self._estimate],
            'last_detection': self._last_detection, 'counts': self._counts})))


def main():
    rclpy.init()
    node = MarkerLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
