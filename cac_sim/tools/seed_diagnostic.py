#!/usr/bin/env python3
"""Per-seed failure diagnostic, run next to a cac_sim launch (TEST TOOL ONLY).

  python3 tools/seed_diagnostic.py --seed 10 --out $WS/suite_results/seed10_diagnostic/diag

Answers two questions from ground truth, independently of the autonomy stack:

1. Did the camera see each crater, and could crater_detector have used it?
   For every camera frame (throttled to --cam-rate) the cloud is put through the
   detector's own filters (range, self box, fitted ground plane) and projected
   into the world with the GROUND-TRUTH pose. Per crater: points that fell in
   its bowl ('seen') and how many of them were below the plane by more than the
   detector's threshold ('low'). Also logged: whether map->base_footprint
   existed (the detector discards every frame without it).
2. Why did navigation fail? On every new mission history entry and at the end,
   the global and local costmaps, the latest plan, both published footprints,
   the believed and ground-truth poses are saved, plus the costmap values under
   the believed footprint outline (what the Smac planner checks).

Writes <out>_frames.csv, <out>_cells.csv, <out>_plans.json, <out>_snap_<k>.npz and
<out>_report.json.
"""
import argparse
import csv
import json
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PolygonStamped
from rcl_interfaces.msg import Log
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from cac_sim import arena_layout
    from cac_sim.crater_detector import _cloud_xyz, fit_ground
    from cac_sim.marker_localizer import _tf_to_T
except ImportError:  # running from the source tree
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from cac_sim import arena_layout
    from cac_sim.crater_detector import _cloud_xyz, fit_ground
    from cac_sim.marker_localizer import _tf_to_T

# crater_detector defaults (cac_sim/crater_detector.py)
STRIDE, MIN_R, MAX_R, DEPTH = 4, 0.3, 2.2, 0.025
SELF_BOX = (-0.75, 1.30, -0.50, 0.50)


def grid_array(msg):
    a = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    o = msg.info.origin.position
    return a, o.x, o.y, msg.info.resolution


def outline_cells(poly, ox, oy, res, step=0.01):
    """Grid cells (row, col) under a closed polygon outline, sampled every `step` m."""
    cells = set()
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        for k in range(n + 1):
            x, y = x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n
            cells.add((int(math.floor((y - oy) / res)), int(math.floor((x - ox) / res))))
    return sorted(cells)


class Diag(Node):
    def __init__(self, args, layout):
        super().__init__('seed_diagnostic')
        self.args, self.layout = args, layout
        self.craters = [o for o in layout.obstacles if o.kind == 'crater']
        self.gt = None
        self.tf = Buffer()
        self.tl = TransformListener(self.tf, self)
        self.t0 = time.time()
        self.state, self.status, self.hist_len = 'unknown', {}, 0
        self.gcm = self.lcm = None
        self.fp = {}
        self.plans = []
        self.cells = []
        self.frames = []
        self.cell_rows = []
        self.snaps = 0
        self.last_cam = 0.0
        self.end_at = None
        self.create_subscription(String, '/mission_state', self.on_state, 10)
        self.create_subscription(String, '/mission_status', self.on_status, 10)
        self.create_subscription(PointCloud2, '/camera/points', self.on_cloud, 2)
        self.create_subscription(PointCloud2, '/crater_cells', self.on_cells, 2)
        self.create_subscription(Path, '/plan', self.on_plan, 5)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: setattr(self, 'gcm', m), 2)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: setattr(self, 'lcm', m), 2)
        for k in ('global', 'local'):
            self.create_subscription(PolygonStamped, f'/{k}_costmap/published_footprint',
                                     lambda m, k=k: self.fp.__setitem__(k, m), 2)
        self.create_timer(1.0, self.sample_cells)
        # the planner's own failure messages (mission_status only updates on a state change)
        self.last_log_snap = 0.0
        self.create_subscription(Log, '/rosout', self.on_log, 50)

    def on_log(self, m):
        if m.name == 'planner_server' and ('Start occupied' in m.msg or 'no valid path' in m.msg) \
                and time.time() - self.last_log_snap > 5.0:
            self.last_log_snap = time.time()
            self.snapshot(f'planner: {m.msg}')

    def now(self):
        return round(time.time() - self.t0, 2)

    def gt_reader(self, proc):
        for line in proc.stdout:
            try:
                poses = json.loads(line)['pose']
            except (ValueError, KeyError):
                continue
            for p in poses:
                if p.get('name') == 'cac_rover':
                    pos, q = p.get('position', {}), p.get('orientation', {})
                    w, x, y, z = q.get('w', 1.0), q.get('x', 0.0), q.get('y', 0.0), q.get('z', 0.0)
                    self.gt = (pos.get('x', 0.0), pos.get('y', 0.0),
                               math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
                    break

    def belief(self, frame='map'):
        try:
            t = self.tf.lookup_transform(frame, 'base_footprint', Time())
        except TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_state(self, m):
        if m.data != self.state:
            self.state = m.data
            if m.data in ('COMPLETE', 'FAILED') and self.end_at is None:
                self.end_at = time.time() + self.args.linger

    def on_status(self, m):
        try:
            self.status = json.loads(m.data)
        except ValueError:
            return
        hist = self.status.get('history') or []
        if len(hist) > self.hist_len:
            self.hist_len = len(hist)
            self.snapshot(f'history_{len(hist)}: {hist[-1]}')

    def on_plan(self, m):
        pts = [(round(p.pose.position.x, 3), round(p.pose.position.y, 3)) for p in m.poses[::5]]
        if m.poses:
            pts.append((round(m.poses[-1].pose.position.x, 3), round(m.poses[-1].pose.position.y, 3)))
        if not self.plans or self.plans[-1]['pts'] != pts:
            self.plans.append({'t': self.now(), 'gt': self.gt, 'pts': pts})

    def on_cells(self, m):
        if m.width == 0:
            self.cells = []
            return
        self.cells = np.frombuffer(bytes(m.data), dtype=np.float32).reshape(-1, m.point_step // 4)[:, :2].tolist()

    def sample_cells(self):
        row = {'t': self.now(), 'state': self.state, 'n_cells': len(self.cells)}
        for c in self.craters:
            row[c.name] = sum(1 for x, y in self.cells if math.hypot(x - c.x, y - c.y) <= c.radius + 0.05)
        self.cell_rows.append(row)

    def on_cloud(self, msg):
        if self.gt is None or time.time() - self.last_cam < 1.0 / self.args.cam_rate:
            return
        self.last_cam = time.time()
        gt = self.gt
        map_ok = self.belief() is not None
        try:
            T = _tf_to_T(self.tf.lookup_transform('base_footprint', msg.header.frame_id, Time()))
        except TransformException:
            return
        pts = _cloud_xyz(msg, STRIDE)
        if len(pts) == 0:
            return
        pb = pts @ T[:3, :3].T + T[:3, 3]
        rng = np.hypot(pb[:, 0], pb[:, 1])
        x0, x1, y0, y1 = SELF_BOX
        pb = pb[(rng > MIN_R) & (rng < MAX_R) &
                ~((pb[:, 0] > x0) & (pb[:, 0] < x1) & (pb[:, 1] > y0) & (pb[:, 1] < y1))]
        coef = fit_ground(pb)
        resid = None if coef is None else pb[:, 2] - (coef[0] * pb[:, 0] + coef[1] * pb[:, 1] + coef[2])
        c, s = math.cos(gt[2]), math.sin(gt[2])
        wx = gt[0] + c * pb[:, 0] - s * pb[:, 1]
        wy = gt[1] + s * pb[:, 0] + c * pb[:, 1]
        row = {'t': self.now(), 'state': self.state, 'gt_x': round(gt[0], 3), 'gt_y': round(gt[1], 3),
               'gt_yaw': round(math.degrees(gt[2]), 1), 'map_tf': int(map_ok), 'plane': int(coef is not None)}
        for cr in self.craters:
            inside = np.hypot(wx - cr.x, wy - cr.y) < cr.radius
            row[f'{cr.name}_seen'] = int(inside.sum())
            row[f'{cr.name}_low'] = 0 if resid is None else int((inside & (resid < -DEPTH)).sum())
        self.frames.append(row)

    def snapshot(self, why):
        k = self.snaps
        self.snaps += 1
        bel, odom_bel, gt = self.belief(), self.belief('odom'), self.gt
        out = {'why': why, 't': self.now(), 'gt': gt, 'belief_map': bel, 'belief_odom': odom_bel,
               'state': self.state, 'plan': self.plans[-1] if self.plans else None}
        arrays = {}
        for name, grid, fpkey in (('global', self.gcm, 'global'), ('local', self.lcm, 'local')):
            if grid is None:
                continue
            a, ox, oy, res = grid_array(grid)
            arrays[f'{name}_costmap'] = a
            arrays[f'{name}_origin'] = np.array([ox, oy, res])
            fpm = self.fp.get(fpkey)
            if fpm is None:
                continue
            poly = [(p.x, p.y) for p in fpm.polygon.points]
            arrays[f'{name}_footprint'] = np.array(poly)
            vals = []
            for r, cc in outline_cells(poly, ox, oy, res):
                v = int(a[r, cc]) if 0 <= r < a.shape[0] and 0 <= cc < a.shape[1] else -2
                vals.append((r, cc, ox + (cc + 0.5) * res, oy + (r + 0.5) * res, v))
            lethal = [v for v in vals if v[4] >= 99 or v[4] == -2]
            out[f'{name}_outline_max'] = max(v[4] for v in vals) if vals else None
            cx, cy = (bel or (0.0, 0.0))[:2] if name == 'global' else (odom_bel or (0.0, 0.0))[:2]
            ci, cj = int(math.floor((cx - ox) / res)), int(math.floor((cy - oy) / res))
            out[f'{name}_centre_cost'] = int(a[cj, ci]) if 0 <= cj < a.shape[0] and 0 <= ci < a.shape[1] else None
            out[f'{name}_outline_lethal_cells'] = [(round(x, 3), round(y, 3), v) for _, _, x, y, v in lethal]
            out[f'{name}_footprint'] = [(round(x, 3), round(y, 3)) for x, y in poly]
        np.savez_compressed(f'{self.args.out}_snap_{k}.npz', **arrays)
        with open(f'{self.args.out}_snap_{k}.json', 'w') as f:
            json.dump(out, f, indent=1)
        self.get_logger().info(f'snapshot {k}: {why}; global outline max {out.get("global_outline_max")}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--duration', type=float, default=900.0)
    ap.add_argument('--linger', type=float, default=5.0)
    ap.add_argument('--cam-rate', type=float, default=4.0)
    args = ap.parse_args()
    from ament_index_python.packages import get_package_share_directory
    params = arena_layout.load_params(os.path.join(get_package_share_directory('cac_sim'), 'config',
                                                   'arena_params.yaml'))
    layout = arena_layout.generate_layout(params, args.seed, True)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    gz = subprocess.Popen(['gz', 'topic', '-e', '-t', '/world/cac_arena/dynamic_pose/info', '--json-output'],
                          stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, start_new_session=True)
    rclpy.init()
    d = Diag(args, layout)
    threading.Thread(target=d.gt_reader, args=(gz,), daemon=True).start()
    try:
        while time.time() - d.t0 < args.duration and (d.end_at is None or time.time() < d.end_at):
            rclpy.spin_once(d, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        os.killpg(gz.pid, 9)
    d.snapshot('end')
    for suffix, rows in (('frames', d.frames), ('cells', d.cell_rows)):
        if rows:
            with open(f'{args.out}_{suffix}.csv', 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
    with open(f'{args.out}_plans.json', 'w') as f:
        json.dump(d.plans, f)
    rep = {'seed': args.seed, 'final_state': d.state, 'history': d.status.get('history'),
           'frames': len(d.frames), 'craters': {}}
    for cr in d.craters:
        seen = [r for r in d.frames if r[f'{cr.name}_seen'] > 0]
        rep['craters'][cr.name] = {
            'pos': [cr.x, cr.y], 'radius': cr.radius,
            'frames_seen': len(seen),
            'frames_seen_with_map_tf': sum(r['map_tf'] for r in seen),
            'frames_low': sum(1 for r in seen if r[f'{cr.name}_low'] > 0),
            'frames_low_with_map_tf': sum(1 for r in seen if r[f'{cr.name}_low'] > 0 and r['map_tf']),
            'first_seen_t': seen[0]['t'] if seen else None,
            'last_seen_t': seen[-1]['t'] if seen else None,
            'max_cells_marked': max((r[cr.name] for r in d.cell_rows), default=0),
        }
    with open(f'{args.out}_report.json', 'w') as f:
        json.dump(rep, f, indent=1)
    print(json.dumps(rep, indent=1))
    d.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
