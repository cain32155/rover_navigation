#!/usr/bin/env python3
"""Rover physics check (TEST TOOL): run against a launched sim with autonomy off.

  ros2 launch cac_sim cac_sim.launch.py randomize:=false autonomy:=false use_rviz:=false
  python3 tools/rover_physics_check.py --out /tmp/physics

Phases (each from rest; cmd_vel is published continuously):
  rest      settle; ground-truth base height/roll/pitch, rocker and arm angles
  forward   v=0.3 m/s for 5 s       spin+ / spin-  w=+-0.5 rad/s for 8 s
  arc       v=0.3, w=0.4 for 6 s    reverse  v=-0.2 for 4 s
  arm       stow -> dig -> carry -> pour -> stow, joint tracking error and body tilt
Reports, per drive phase: ground-truth speed / yaw rate vs command, odometry vs
ground truth, and translation of base_footprint while spinning (spin drift).
Writes <out>.json.
"""
import argparse
import json
import math
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def quat_rpy(q):
    w, x, y, z = q.get('w', 1.0), q.get('x', 0.0), q.get('y', 0.0), q.get('z', 0.0)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Check(Node):
    def __init__(self):
        super().__init__('rover_physics_check')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm = {j: self.create_publisher(Float64, f'/arm/{j}/cmd', 10)
                    for j in ('boom_joint', 'stick_joint', 'bucket_joint')}
        self.js, self.odom, self.gt = {}, None, None
        self.create_subscription(JointState, '/joint_states', self.on_js, 20)
        self.create_subscription(Odometry, '/odom_raw', lambda m: setattr(self, 'odom', m), 20)
        self.gz = subprocess.Popen(['gz', 'topic', '-e', '-t', '/world/cac_arena/pose/info', '--json-output'],
                                   stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        threading.Thread(target=self.read_gt, daemon=True).start()

    def on_js(self, m):
        for n, p, v in zip(m.name, m.position, m.velocity or [0.0] * len(m.name)):
            self.js[n] = (p, v)

    def read_gt(self):
        for line in self.gz.stdout:
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            st = msg.get('header', {}).get('stamp', {})
            t = float(st.get('sec', 0)) + float(st.get('nsec', 0)) * 1e-9
            for p in msg.get('pose', []):
                if p.get('name') == 'cac_rover':
                    pos = p.get('position', {})
                    self.gt = (t, pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0),
                               *quat_rpy(p.get('orientation', {})))

    def run(self, sec, vx=0.0, wz=0.0, arm=None):
        end = time.time() + sec
        tw = Twist()
        tw.linear.x, tw.angular.z = vx, wz
        while time.time() < end:
            self.cmd.publish(tw)
            if arm is not None:
                for (j, pub), q in zip(self.arm.items(), arm):
                    pub.publish(Float64(data=float(q)))
            rclpy.spin_once(self, timeout_sec=0.02)

    def snap(self):
        o = self.odom
        oy = None
        if o is not None:
            q = o.pose.pose.orientation
            oy = (o.pose.pose.position.x, o.pose.pose.position.y,
                  math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        return self.gt, oy


def unwrap(a, b):
    return math.atan2(math.sin(b - a), math.cos(b - a))


def drive_phase(c, name, sec, vx, wz, settle=1.5):
    c.run(settle)                       # stop and settle
    g0, o0 = c.snap()
    c.run(1.0, vx, wz)                  # accelerate
    g1, o1 = c.snap()
    c.run(sec, vx, wz)
    g2, o2 = c.snap()
    dt = g2[0] - g1[0]
    dx, dy = g2[1] - g1[1], g2[2] - g1[2]
    yaw_mid = g1[6] + unwrap(g1[6], g2[6]) / 2
    v_fwd = (dx * math.cos(yaw_mid) + dy * math.sin(yaw_mid)) / dt
    v_lat = (-dx * math.sin(yaw_mid) + dy * math.cos(yaw_mid)) / dt
    w_gt = unwrap(g1[6], g2[6]) / dt if abs(wz) < 0.7 * math.pi / sec else None
    # total yaw for spins (may exceed pi): integrate from joint-free estimate
    od = math.hypot(o2[0] - o1[0], o2[1] - o1[1]) if o1 and o2 else None
    r = {'phase': name, 'cmd_v': vx, 'cmd_w': wz, 'dt': round(dt, 2),
         'gt_v_forward': round(v_fwd, 3), 'gt_v_lateral': round(v_lat, 3),
         'gt_w': None if w_gt is None else round(w_gt, 3),
         'gt_translation_m': round(math.hypot(g2[1] - g0[1], g2[2] - g0[2]), 3),
         'odom_translation_m': None if od is None else round(od, 3),
         'max_abs_roll_pitch_deg': round(math.degrees(max(abs(g2[4]), abs(g2[5]))), 2)}
    print(json.dumps(r))
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--arm-only', action='store_true')
    a = ap.parse_args()
    rclpy.init()
    c = Check()
    t0 = time.time()
    while (c.gt is None or not c.js) and time.time() - t0 < 30:
        rclpy.spin_once(c, timeout_sec=0.1)
    res = {}
    c.run(3.0, arm=[0, 0, 0])
    g, _ = c.snap()
    res['rest'] = {'base_z': round(g[3], 4), 'roll_deg': round(math.degrees(g[4]), 2),
                   'pitch_deg': round(math.degrees(g[5]), 2),
                   'joints': {k: round(v[0], 3) for k, v in c.js.items() if 'wheel' not in k}}
    print('rest', json.dumps(res['rest']))
    if a.arm_only:
        res['drive'] = []
    else:
        res['drive'] = [drive_phase(c, 'forward', 5, 0.3, 0.0),
                        drive_phase(c, 'spin+', 8, 0.0, 0.5),
                        drive_phase(c, 'spin-', 8, 0.0, -0.5),
                        drive_phase(c, 'arc', 6, 0.3, 0.4),
                        drive_phase(c, 'reverse', 4, -0.2, 0.0)]
        # spin drift over one full turn: translation of base_footprint
        c.run(2.0)
        g0, _ = c.snap()
        c.run(2 * math.pi / 0.5 + 1.0, 0.0, 0.5)
        c.run(2.0)
        g1, _ = c.snap()
        res['full_turn_drift_m'] = round(math.hypot(g1[1] - g0[1], g1[2] - g0[2]), 3)
        print('full turn drift', res['full_turn_drift_m'])
    # the mission's own sequences (config/arm_params.yaml): dig via the raised carry pose
    import yaml
    from ament_index_python.packages import get_package_share_directory
    with open(get_package_share_directory('cac_sim') + '/config/arm_params.yaml') as f:
        ap_ = yaml.safe_load(f)['/**']['ros__parameters']['arm']
    dig_seq = [ap_['dig_sequence'][k:k + 3] for k in range(0, len(ap_['dig_sequence']), 3)]
    dump_seq = [ap_['dump_sequence'][k:k + 3] for k in range(0, len(ap_['dump_sequence']), 3)]
    poses = {'stow': list(ap_['stow']), 'carry': dig_seq[0], 'dig': dig_seq[1], 'scoop': dig_seq[2],
             'pour': dump_seq[1]}
    arm = []
    for name in ('carry', 'dig', 'scoop', 'carry', 'stow', 'carry', 'pour', 'carry', 'stow'):
        q = poses[name]
        tilt = 0.0
        end = time.time() + 6.0
        while time.time() < end:
            c.run(0.1, arm=q)
            if c.gt:
                tilt = max(tilt, abs(math.degrees(c.gt[4])), abs(math.degrees(c.gt[5])))
        err = {j: round(c.js[j][0] - qq, 3) for j, qq in zip(('boom_joint', 'stick_joint', 'bucket_joint'), q)}
        arm.append({'pose': name, 'target': q, 'error': err, 'max_tilt_deg': round(tilt, 2),
                    'final_tilt_deg': round(max(abs(math.degrees(c.gt[4])), abs(math.degrees(c.gt[5]))), 2),
                    'base_z': round(c.gt[3], 4)})
        print(json.dumps(arm[-1]))
    res['arm'] = arm
    with open(a.out + '.json', 'w') as f:
        json.dump(res, f, indent=1)
    c.gz.kill()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
