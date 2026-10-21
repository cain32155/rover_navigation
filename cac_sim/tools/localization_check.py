#!/usr/bin/env python3
"""Marker-localization accuracy check (TEST TOOL), against `autonomy:=false`.

  python3 tools/localization_check.py --out /tmp/loc_seed6

Performs the mission's stepped marker search (turn 1.2 s, hold 1.0 s) until
marker_localizer reports a fix, holds still 3 s, then compares the localized
pose (TF map->base_footprint) with Gazebo ground truth. Then drives a 2 m
straight line + 90 deg turn + 2 m and compares again (pure dead reckoning,
the markers are mostly out of view). Prints and writes <out>.json.
"""
import argparse
import json
import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


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


class Check(Node):
    def __init__(self):
        super().__init__('localization_check')
        self.loc = {}
        self.create_subscription(String, '/localization/status', lambda m: setattr(self, 'loc', json.loads(m.data)), 5)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf = Buffer()
        self.tl = TransformListener(self.tf, self)

    def run(self, sec, v=0.0, w=0.0):
        tw = Twist()
        tw.linear.x, tw.angular.z = v, w
        end = time.time() + sec
        while time.time() < end:
            self.cmd.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)

    def belief(self):
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.02)
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def compare(label, bel, gt):
    r = {'label': label, 'belief': [round(v, 3) for v in bel], 'ground_truth': [round(v, 3) for v in gt],
         'pos_err_m': round(math.hypot(bel[0] - gt[0], bel[1] - gt[1]), 3),
         'yaw_err_deg': round(math.degrees(math.atan2(math.sin(bel[2] - gt[2]), math.cos(bel[2] - gt[2]))), 2)}
    print(json.dumps(r))
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    rclpy.init()
    c = Check()
    t0 = time.time()
    while not c.loc.get('initialized') and time.time() - t0 < 120:
        c.run(0.1, 0.0, 0.35 if (time.time() - t0) % 2.2 < 1.2 else 0.0)
    c.run(3.0)
    res = {'time_to_fix_s': round(time.time() - t0, 1), 'status': c.loc}
    bel = c.belief()
    if bel is None:
        res['error'] = 'no fix'
    else:
        res['after_fix'] = compare('after_fix', bel, gz_pose())
        # dead-reckoning leg: face +x (the arena interior) first, then drive
        yaw = bel[2]
        turn = math.atan2(math.sin(-yaw), math.cos(-yaw))
        c.run(abs(turn) / 0.4, 0.0, 0.4 * (1 if turn > 0 else -1))
        c.run(1.0)
        c.run(2.0 / 0.25, 0.25, 0.0)
        c.run(math.pi / 2 / 0.4, 0.0, 0.4)
        c.run(1.5 / 0.25, 0.25, 0.0)
        c.run(2.0)
        res['after_drive'] = compare('after_drive', c.belief(), gz_pose())
    with open(a.out + '.json', 'w') as f:
        json.dump(res, f, indent=1)
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
