#!/usr/bin/env python3
"""Ground-truth validation monitor for a running cac_sim launch (TEST TOOL ONLY).

Compares what the autonomy stack believes with what Gazebo says, and checks the
rover against the arena it was given -- the stack itself never sees any of this.

  python3 tools/gt_monitor.py --seed 6 --duration 600 --out /tmp/run6

Streams Gazebo ground truth (`gz topic -e --json-output` on
/world/cac_arena/dynamic_pose/info; the ROS bridge drops entity names), then
every 0.5 s records: ground-truth pose, localized pose (TF
map->base_footprint), localization error, minimum footprint distance to each
wall and every boulder, overlap of the footprint/wheels with crater bowls, and
the mission state. Writes <out>.csv and <out>_summary.json, and stops early
when the mission reaches COMPLETE or FAILED (after --linger seconds).

Leg verification: whenever the mission reports a leg 'arrived' (from its own,
localized pose), the GROUND-TRUTH pose at that moment is recorded and the leg's
zone condition (mission_logic.zone_ok, same rules as the mission) is checked on
it -- a COMPLETE mission whose legs did not really end where they claim fails.

Material verification: every scoop/dump the mission CREDITS (material_model) is
re-checked with the ground-truth rover pose: the bucket lip (its base_footprint
offset, from the mission's TF at that moment) is placed on the Gazebo pose and must
lie in the excavation zone (scoop) / in the berm target (dump credited in target).
This checks localization, not sand: no sand moves in the simulation.

Contact criteria (from the regenerated, identical seeded layout):
  wall contact     footprint corner within --contact-tol of an inner wall face
  boulder contact  footprint rectangle within --contact-tol of the sphere's
                   ground-plane radius
  crater crossing  any wheel contact point inside a crater's rim radius
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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from cac_sim import arena_layout
    from cac_sim import mission_logic as ml
except ImportError:  # running from the source tree
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from cac_sim import arena_layout
    from cac_sim import mission_logic as ml


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rect_circle_distance(fp, pose, cx, cy, r):
    """Distance from the footprint rectangle at pose to a circle (negative = overlap)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = cx - x, cy - y
    bx, by = c * dx + s * dy, -s * dx + c * dy
    mid = (fp.front - fp.rear) / 2.0
    hx, hy = (fp.front + fp.rear) / 2.0, fp.half_width
    qx, qy = max(abs(bx - mid) - hx, 0.0), max(abs(by) - hy, 0.0)
    return math.hypot(qx, qy) - r


class Monitor(Node):
    def __init__(self, args, params, layout):
        super().__init__('gt_monitor')
        self.args, self.params, self.layout = args, params, layout
        self.fp = arena_layout.Footprint.from_params(params)
        self.gt = None
        self.state, self.status = 'unknown', {}
        self.tf = Buffer()
        self.tl = TransformListener(self.tf, self)
        self.create_subscription(String, '/mission_state', lambda m: setattr(self, 'state', m.data), 10)
        self.create_subscription(String, '/mission_status', self.on_status, 10)
        self.cands = []         # every marker candidate vs ground truth at reception
        self.create_subscription(String, '/localization/candidates', self.on_cand, 50)
        self.loc_status = {}
        self.create_subscription(String, '/localization/status',
                                 lambda m: setattr(self, 'loc_status', json.loads(m.data)), 5)
        self.crater_cells = []
        self.create_subscription(PointCloud2, '/crater_cells', self.on_craters, 2)
        self.rows = []
        self.legs = []          # [{leg, claimed, gt, gt_zone_ok}] per 'arrived' history entry
        self.seen_history = 0
        self.material_gt = []   # per credited material event: lip placed on the ground-truth pose
        self.seen_events = set()
        self.mat = None
        try:
            from cac_sim import material_model as mm
            self.mat = mm.MaterialModel(mm.MaterialParams.from_params(params))
        except (ImportError, KeyError):
            pass                 # older package without material accounting
        z = params['zones']
        self.zones = ml.Zones(tuple(z['starting_zone']), tuple(z['excavation_zone']), tuple(z['construction_zone']),
                              params['arena']['length'] / 2.0, params['arena']['width'] / 2.0)
        self.mfp = ml.Footprint(self.fp.front, self.fp.rear, self.fp.half_width)
        self.t0 = time.time()
        self.end_at = None

    def on_cand(self, m):
        if self.gt is None:
            return
        c = json.loads(m.data)
        gx, gy, gyaw = self.gt
        bx, by, byaw = c['base']
        self.cands.append({'t': round(time.time() - self.t0, 2), 'multi': int(c['multi']), 'ids': c['ids'],
                           'range': c['range'], 'outcome': c['outcome'],
                           'err_xy': round(math.hypot(bx - gx, by - gy), 3),
                           'err_yaw_deg': round(math.degrees(math.atan2(math.sin(byaw - gyaw),
                                                                         math.cos(byaw - gyaw))), 2)})

    def on_status(self, m):
        try:
            self.status = json.loads(m.data)
        except ValueError:
            return
        hist = self.status.get('history') or []
        for leg, outcome, note in hist[self.seen_history:]:
            if outcome == 'arrived' and leg in ('excavation', 'construction', 'return'):
                gt = self.gt
                self.legs.append({'leg': leg, 'claimed': note, 't': round(time.time() - self.t0, 1),
                                  'gt': None if gt is None else [round(gt[0], 3), round(gt[1], 3),
                                                                 round(math.degrees(gt[2]), 1)],
                                  'gt_zone_ok': gt is not None and ml.zone_ok(leg, gt, self.mfp, self.zones)})
        self.seen_history = len(hist)
        for ev in (self.status.get('summary') or {}).get('material_events', []):
            key = (ev.get('kind'), ev.get('id'))
            if not ev.get('accepted') or key in self.seen_events or self.mat is None:
                continue
            self.seen_events.add(key)
            rec = {'kind': ev['kind'], 'id': ev['id'], 'volume_l': ev.get('volume_l'),
                   't': round(time.time() - self.t0, 1), 'lip_claimed': ev.get('lip'), 'lip_gt': None, 'gt_ok': False}
            lb, gt = ev.get('lip_base'), self.gt
            if lb and gt is not None:
                c, s_ = math.cos(gt[2]), math.sin(gt[2])
                lx, ly = gt[0] + c * lb[0] - s_ * lb[1], gt[1] + s_ * lb[0] + c * lb[1]
                rec['lip_gt'] = [round(lx, 3), round(ly, 3)]
                if ev['kind'] == 'scoop':
                    rec['gt_ok'] = self.mat.in_excavation_zone(lx, ly)
                elif ev.get('in_target'):
                    rec['gt_ok'] = self.mat.in_berm_target(lx, ly)
                else:
                    rec['gt_ok'] = self.mat.in_construction_zone(lx, ly)
            self.material_gt.append(rec)

    def on_craters(self, msg):
        import numpy as np
        if msg.width == 0:
            self.crater_cells = []
            return
        pts = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(-1, msg.point_step // 4)[:, :2]
        self.crater_cells = pts.tolist()

    def crater_report(self):
        """Per crater: fraction of its 5 cm cells marked; plus marked cells far from any crater."""
        rep, used = {}, set()
        for ob in self.layout.obstacles:
            if ob.kind != 'crater':
                continue
            n_in = sum(1 for x, y in self.crater_cells if math.hypot(x - ob.x, y - ob.y) <= ob.radius + 0.05)
            area_cells = math.pi * ob.radius ** 2 / 0.05 ** 2
            rep[ob.name] = round(min(1.0, n_in / area_cells), 2)
            used |= {k for k, (x, y) in enumerate(self.crater_cells) if math.hypot(x - ob.x, y - ob.y) <= ob.radius + 0.10}
        return rep, len(self.crater_cells) - len(used)

    def gt_reader(self, proc):
        """Thread: parse the streamed JSON poses, keep the rover model's pose."""
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

    def belief(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def sample(self):
        if self.gt is None:
            return
        gt, bel = self.gt, self.belief()
        half_l, half_w = self.params['arena']['length'] / 2, self.params['arena']['width'] / 2
        corners = self.fp.corners(*gt)
        wall = min(min(half_l - abs(cx), half_w - abs(cy)) for cx, cy in corners)
        boulder, crater_wheels = 99.0, 0
        wheels = self.args.wheel_points
        c, s = math.cos(gt[2]), math.sin(gt[2])
        wpts = [(gt[0] + c * wx - s * wy, gt[1] + s * wx + c * wy) for wx, wy in wheels]
        for ob in self.layout.obstacles:
            if ob.kind == 'boulder':
                boulder = min(boulder, rect_circle_distance(self.fp, gt, ob.x, ob.y, ob.radius))
            else:
                crater_wheels += sum(math.hypot(px - ob.x, py - ob.y) < ob.radius for px, py in wpts)
        err = None if bel is None else math.hypot(bel[0] - gt[0], bel[1] - gt[1])
        yerr = None if bel is None else math.degrees(
            math.atan2(math.sin(bel[2] - gt[2]), math.cos(bel[2] - gt[2])))
        self.rows.append({'t': round(time.time() - self.t0, 1), 'state': self.state,
                          'gt_x': round(gt[0], 3), 'gt_y': round(gt[1], 3), 'gt_yaw': round(math.degrees(gt[2]), 1),
                          'bel_x': None if bel is None else round(bel[0], 3),
                          'bel_y': None if bel is None else round(bel[1], 3),
                          'loc_err': None if err is None else round(err, 3),
                          'yaw_err': None if yerr is None else round(yerr, 1),
                          'wall_dist': round(wall, 3), 'boulder_dist': round(boulder, 3),
                          'crater_wheels': crater_wheels})
        if self.state in ('COMPLETE', 'FAILED') and self.end_at is None:
            self.end_at = time.time() + self.args.linger


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--randomize', default='true')
    ap.add_argument('--duration', type=float, default=600.0)
    ap.add_argument('--linger', type=float, default=5.0)
    ap.add_argument('--out', required=True)
    ap.add_argument('--arena-params', default=None)
    ap.add_argument('--contact-tol', type=float, default=0.02)
    args = ap.parse_args()
    path = args.arena_params
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('cac_sim'), 'config', 'arena_params.yaml')
    params = arena_layout.load_params(path)
    layout = arena_layout.generate_layout(params, args.seed, args.randomize.lower() == 'true')
    r = params['robot']
    # wheel ground-contact points (x, y) in base_footprint, from the footprint params
    args.wheel_points = [(sx * r.get('wheel_x_offset', 0.45), sy * (r['footprint_half_width'] - 0.04))
                         for sx in (1, -1) for sy in (1, -1)]
    gz = subprocess.Popen(['gz', 'topic', '-e', '-t', '/world/cac_arena/dynamic_pose/info', '--json-output'],
                          stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, start_new_session=True)
    rclpy.init()
    mon = Monitor(args, params, layout)
    threading.Thread(target=mon.gt_reader, args=(gz,), daemon=True).start()
    mon.create_timer(0.5, mon.sample)
    try:
        while time.time() - mon.t0 < args.duration and (mon.end_at is None or time.time() < mon.end_at):
            rclpy.spin_once(mon, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        os.killpg(gz.pid, 9)
    rows = mon.rows
    if rows:
        with open(args.out + '.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
    errs = [x['loc_err'] for x in rows if x['loc_err'] is not None]
    if mon.cands:
        with open(args.out + '_candidates.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(mon.cands[0].keys()))
            w.writeheader()
            w.writerows(mon.cands)
    used = [c for c in mon.cands if c['outcome'] in ('update', 'initial', 'initial_fix')]
    crater_cov, false_cells = mon.crater_report()
    final = rows[-1] if rows else {}
    summary = {
        'seed': args.seed, 'samples': len(rows), 'final_state': mon.state,
        'mission_status': mon.status,
        'final_gt': [final.get('gt_x'), final.get('gt_y'), final.get('gt_yaw')],
        'loc_err_mean': round(sum(errs) / len(errs), 3) if errs else None,
        'loc_err_max': max(errs) if errs else None,
        'min_wall_dist': min((x['wall_dist'] for x in rows), default=None),
        'min_boulder_dist': min((x['boulder_dist'] for x in rows), default=None),
        'wall_contact': any(x['wall_dist'] <= args.contact_tol for x in rows),
        'boulder_contact': any(x['boulder_dist'] <= args.contact_tol for x in rows),
        'crater_wheel_samples': sum(1 for x in rows if x['crater_wheels'] > 0),
        'crater_detected_fraction': crater_cov,
        'legs_gt': mon.legs,
        'localizer_counts': mon.loc_status.get('counts'),
        'marker_candidates': {'n': len(mon.cands), 'used': len(used),
                              'used_err_xy_max': max((c['err_xy'] for c in used), default=None),
                              'used_err_gt_0.15': sum(1 for c in used if c['err_xy'] > 0.15)},
        # every leg the mission needs, each confirmed by ground truth when it claimed arrival
        'legs_gt_verified': all(any(x['leg'] == leg and x['gt_zone_ok'] for x in mon.legs)
                                for leg in ('excavation', 'construction', 'return'))
                            and all(x['gt_zone_ok'] for x in mon.legs),
        'crater_false_cells': false_cells,
        'material_gt': mon.material_gt,
        'material_gt_verified': all(x['gt_ok'] for x in mon.material_gt) if mon.material_gt else None,
        'run_summary': {k: v for k, v in (mon.status.get('summary') or {}).items() if k != 'material_events'},
        'duration_sec': rows[-1]['t'] if rows else 0,
        'layout': arena_layout.describe(layout),
    }
    with open(args.out + '_summary.json', 'w') as f:
        json.dump(summary, f, indent=1)
    print(json.dumps({k: v for k, v in summary.items() if k not in ('layout', 'mission_status', 'run_summary')},
                     indent=1))
    print('run summary:', json.dumps(summary['run_summary']))
    print('mission history:', json.dumps(mon.status.get('history'), indent=0))
    mon.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
