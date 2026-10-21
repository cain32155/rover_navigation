#!/usr/bin/env python3
"""Marker-fix accuracy versus range (TEST TOOL), against `randomize:=false autonomy:=false`.

  python3 tools/marker_range_test.py --out $WS/suite_results/marker_range/run1

Localizes with the mission's stepped marker search, turns to face the start-zone
markers (west), then backs away east in --step metre stops up to --x-max. At every
stop (rover still for --hold s) each /localization/candidates message is compared
with Gazebo ground truth: error of the base pose that single detection implies,
by marker range and single/multi-marker. The believed pose (TF map->base_footprint)
error is recorded at each stop too. Writes <out>_candidates.csv, <out>_stops.csv
and prints a table binned by range.
"""
import argparse
import csv
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


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class RangeTest(Node):
    def __init__(self):
        super().__init__('marker_range_test')
        self.loc = {}
        self.cands = []
        self.collect = None
        self.tf = Buffer()
        self.tl = TransformListener(self.tf, self, spin_thread=True)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/localization/status', lambda m: setattr(self, 'loc', json.loads(m.data)), 5)
        self.create_subscription(String, '/localization/candidates', self.on_cand, 50)

    def on_cand(self, m):
        if self.collect is not None:
            self.cands.append(dict(json.loads(m.data), stop=self.collect))

    def belief(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def run(self, vx, wz, secs):
        tw = Twist()
        tw.linear.x, tw.angular.z = vx, wz
        t0 = time.time()
        while time.time() - t0 < secs:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.pub.publish(Twist())

    def spin_for(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--step', type=float, default=0.5)
    ap.add_argument('--x-max', type=float, default=1.0)
    ap.add_argument('--hold', type=float, default=4.0)
    args = ap.parse_args()
    rclpy.init()
    n = RangeTest()
    t0 = time.time()
    while not n.loc.get('initialized'):                 # the mission's stepped search
        if time.time() - t0 > 120:
            raise SystemExit('no marker fix')
        n.run(0.0, 0.35, 1.2)
        n.spin_for(1.0)
    n.spin_for(2.0)
    for _ in range(40):                                 # face west
        b = n.belief()
        err = wrap(math.pi - b[2])
        if abs(err) < 0.03:
            break
        n.run(0.0, max(-0.3, min(0.3, 1.2 * err)), 0.3)
    stops = []
    k = 0
    while True:
        n.spin_for(1.0)
        n.collect = k
        n.spin_for(args.hold)
        n.collect = None
        gt, bel = gz_pose(), n.belief()
        stops.append({'stop': k, 'gt_x': round(gt[0], 3), 'gt_y': round(gt[1], 3),
                      'gt_yaw': round(math.degrees(gt[2]), 2),
                      'bel_err': round(math.hypot(bel[0] - gt[0], bel[1] - gt[1]), 3),
                      'bel_yaw_err': round(math.degrees(wrap(bel[2] - gt[2])), 2)})
        for c in n.cands:
            if c['stop'] == k:
                c['gt'] = gt
        print(stops[-1], flush=True)
        if gt[0] >= args.x_max:
            break
        # back away east (rover faces west), heading held on the belief
        t1 = time.time()
        start = gz_pose()
        while time.time() - t1 < 15:
            b = n.belief()
            n.run(-0.15, max(-0.3, min(0.3, 1.5 * wrap(math.pi - b[2]))), 0.2)
            g = gz_pose()
            if math.hypot(g[0] - start[0], g[1] - start[1]) >= args.step:
                break
        n.pub.publish(Twist())
        k += 1
    rows = []
    for c in n.cands:
        gx, gy, gyaw = c['gt']
        bx, by, byaw = c['base']
        rows.append({'stop': c['stop'], 'multi': int(c['multi']), 'ids': ' '.join(map(str, c['ids'])),
                     'range': c['range'], 'min_edge_px': min(c['edge_px']), 'outcome': c['outcome'],
                     'err_xy': round(math.hypot(bx - gx, by - gy), 4),
                     'err_yaw_deg': round(math.degrees(wrap(byaw - gyaw)), 3)})
    for suffix, data in (('candidates', rows), ('stops', stops)):
        if data:
            with open(f'{args.out}_{suffix}.csv', 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(data[0].keys()))
                w.writeheader()
                w.writerows(data)
    print('range_bin multi  n  err_xy_median  err_xy_p90  err_xy_max  yaw_med_deg')
    for lo in [x * 0.5 for x in range(0, 12)]:
        for multi in (1, 0):
            sel = sorted(r['err_xy'] for r in rows if lo <= r['range'] < lo + 0.5 and r['multi'] == multi)
            yaw = sorted(abs(r['err_yaw_deg']) for r in rows if lo <= r['range'] < lo + 0.5 and r['multi'] == multi)
            if sel:
                print(f'{lo:.1f}-{lo + 0.5:.1f}  {multi}  {len(sel):3d}  {sel[len(sel) // 2]:.3f}  '
                      f'{sel[int(len(sel) * 0.9)]:.3f}  {sel[-1]:.3f}  {yaw[len(yaw) // 2]:.2f}')
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
