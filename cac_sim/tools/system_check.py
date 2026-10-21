#!/usr/bin/env python3
"""Integration check of a running sim (TEST TOOL), against `autonomy:=false`.

  python3 tools/system_check.py --out $WS/suite_results/system_check/check

1. Topic rates over --window s: /scan, /imu, /camera/points, /camera/image,
   /odom_raw, /odometry/filtered, /joint_states.
2. TF: every URDF link resolves from base_footprint; every child frame on /tf
   and /tf_static has exactly one parent; the nodes publishing /tf (expected:
   robot_state_publisher, the EKF and marker_localizer -- no Gazebo bridge).
3. Drive: 0.2 m/s forward, then 0.4 rad/s left turn; every wheel joint's
   velocity sign (forward: all four positive about their +y axes; left turn:
   left side slower than right side), and ground-truth motion direction.
4. Rockers: joint positions within their limits, and their range during the drive.
Prints and writes <out>.json; exit status 1 if a check failed.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image, Imu, JointState, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener

TOPICS = {'/scan': (LaserScan, 10.0), '/imu': (Imu, 100.0), '/camera/points': (PointCloud2, 15.0),
          '/camera/image': (Image, 15.0), '/odom_raw': (Odometry, 50.0), '/odometry/filtered': (Odometry, 30.0),
          '/joint_states': (JointState, None)}
WHEELS = ('front_left_wheel_joint', 'rear_left_wheel_joint', 'front_right_wheel_joint', 'rear_right_wheel_joint')


class Check(Node):
    def __init__(self):
        super().__init__('system_check')
        self.counts = {t: 0 for t in TOPICS}
        for t, (typ, _) in TOPICS.items():
            self.create_subscription(typ, t, lambda m, t=t: self.counts.__setitem__(t, self.counts[t] + 1), 10)
        self.parents = {}
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(TFMessage, '/tf_static', self.on_tf,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.js = {}
        self.create_subscription(JointState, '/joint_states', self.on_js, 20)
        self.tf = Buffer()
        self.tl = TransformListener(self.tf, None, spin_thread=True)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def on_tf(self, m):
        for t in m.transforms:
            self.parents.setdefault(t.child_frame_id, set()).add(t.header.frame_id)

    def on_js(self, m):
        for i, n in enumerate(m.name):
            self.js[n] = (m.position[i] if i < len(m.position) else None,
                          m.velocity[i] if i < len(m.velocity) else None)

    def spin_for(self, secs, twist=None, sample=None):
        t0 = time.time()
        while time.time() - t0 < secs:
            if twist is not None:
                self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            if sample is not None:
                sample()
        if twist is not None:
            self.pub.publish(Twist())


def gz_pose():
    out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/cac_arena/pose/info', '-n', '1', '--json-output'],
                         capture_output=True, text=True, timeout=15).stdout
    for p in json.loads(out.splitlines()[0])['pose']:
        if p['name'] == 'cac_rover':
            q = p['orientation']
            w, x, y, z = q.get('w', 1), q.get('x', 0), q.get('y', 0), q.get('z', 0)
            return (p['position'].get('x', 0.0), p['position'].get('y', 0.0),
                    math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--window', type=float, default=10.0)
    a = ap.parse_args()
    from ament_index_python.packages import get_package_share_directory
    urdf = ET.fromstring(subprocess.run(['xacro', os.path.join(get_package_share_directory('cac_sim'), 'urdf',
                                                               'rover.urdf.xacro')],
                                        capture_output=True, text=True, check=True).stdout)
    links = [l.get('name') for l in urdf.findall('link')]
    limits = {j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))
              for j in urdf.findall('joint') if j.get('type') == 'revolute'}
    rclpy.init()
    c = Check()
    res, fails = {}, []
    c.spin_for(a.window)
    res['rates_hz'] = {t: round(n / a.window, 1) for t, n in c.counts.items()}
    for t, (_, nominal) in TOPICS.items():
        if res['rates_hz'][t] == 0 or (nominal and res['rates_hz'][t] < 0.6 * nominal):
            fails.append(f'rate {t} {res["rates_hz"][t]} Hz')
    missing = []
    for l in links:
        try:
            c.tf.lookup_transform('base_footprint', l, rclpy.time.Time())
        except TransformException:
            missing.append(l)
    res['tf_missing_links'] = missing
    if missing:
        fails.append(f'TF missing {missing}')
    multi = {k: sorted(v) for k, v in c.parents.items() if len(v) > 1}
    res['tf_children_with_several_parents'] = multi
    if multi:
        fails.append(f'TF several parents {multi}')
    res['tf_publishers'] = sorted({i.node_name for i in c.get_publishers_info_by_topic('/tf')})
    if any('bridge' in n for n in res['tf_publishers']):
        fails.append('gz bridge publishes /tf')
    rock = {j: [] for j in ('left_rocker_joint', 'right_rocker_joint')}

    def sample_rock():
        for j in rock:
            if j in c.js and c.js[j][0] is not None:
                rock[j].append(c.js[j][0])
    for phase, vx, wz in (('forward', 0.2, 0.0), ('left_turn', 0.0, 0.4)):
        g0 = gz_pose()
        tw = Twist()
        tw.linear.x, tw.angular.z = vx, wz
        vel = {w: [] for w in WHEELS}

        def sample():
            sample_rock()
            for w in WHEELS:
                if w in c.js and c.js[w][1] is not None:
                    vel[w].append(c.js[w][1])
        c.spin_for(1.0, tw)                     # accelerate
        c.spin_for(2.0, tw, sample)
        c.spin_for(1.0)
        g1 = gz_pose()
        mean = {w: round(sum(v) / len(v), 3) if v else None for w, v in vel.items()}
        res[phase] = {'wheel_velocity_mean': mean, 'gt_dx_body': round(
            math.cos(g0[2]) * (g1[0] - g0[0]) + math.sin(g0[2]) * (g1[1] - g0[1]), 3),
            'gt_dyaw_deg': round(math.degrees(math.atan2(math.sin(g1[2] - g0[2]), math.cos(g1[2] - g0[2]))), 1)}
        if any(v is None for v in mean.values()):
            fails.append(f'{phase}: wheel velocities missing')
        elif phase == 'forward':
            if not all(v > 0.5 for v in mean.values()) or res[phase]['gt_dx_body'] < 0.2:
                fails.append(f'forward drive wrong: {res[phase]}')
        else:
            left = (mean[WHEELS[0]] + mean[WHEELS[1]]) / 2
            right = (mean[WHEELS[2]] + mean[WHEELS[3]]) / 2
            if not (right > left) or res[phase]['gt_dyaw_deg'] < 20:
                fails.append(f'left turn wrong: {res[phase]}')
    res['rocker_range_rad'] = {j: [round(min(v), 3), round(max(v), 3)] if v else None for j, v in rock.items()}
    for j, v in rock.items():
        if not v or min(v) < limits[j][0] - 0.01 or max(v) > limits[j][1] + 0.01:
            fails.append(f'rocker {j} outside limits or no state')
    res['failures'] = fails
    print(json.dumps(res, indent=1))
    with open(a.out + '.json', 'w') as f:
        json.dump(res, f, indent=1)
    c.destroy_node()
    rclpy.try_shutdown()
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
