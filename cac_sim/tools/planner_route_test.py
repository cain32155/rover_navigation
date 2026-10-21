#!/usr/bin/env python3
"""Global-planner route choice on a seeded layout (TEST TOOL), against `autonomy:=false`.

  python3 tools/planner_route_test.py --seed 10 --out $WS/suite_results/planner/s10

Localizes with the mission's stepped marker search, performs the mission's
obstacle-zone survey (so the costmap holds what the rover would really know at
the start of the first leg), then asks Nav2's planner (ComputePathToPose, from
the current pose) for each goal under each parameter set in --sets (JSON list of
{"name": ..., "params": {"/node": {"param": value}}}). For every path: length,
and the minimum ground-truth clearance of the footprint along it to each boulder
and crater (same geometry as gt_monitor). Writes <out>.json.
"""
import argparse
import itertools
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from cac_sim import arena_layout
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from cac_sim import arena_layout
sys.path.insert(0, os.path.dirname(__file__))
from gt_monitor import rect_circle_distance  # noqa: E402


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class RouteTest(Node):
    def __init__(self):
        super().__init__('planner_route_test')
        self.loc = {}
        self.tf = Buffer()
        self.tl = TransformListener(self.tf, self, spin_thread=True)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/localization/status', lambda m: setattr(self, 'loc', json.loads(m.data)), 5)
        self.plan = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.cells = []
        self.create_subscription(PointCloud2, '/crater_cells', self.on_cells, 2)

    def on_cells(self, m):
        import numpy as np
        self.cells = [] if m.width == 0 else \
            np.frombuffer(bytes(m.data), dtype=np.float32).reshape(-1, m.point_step // 4)[:, :2].tolist()

    def belief(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def run(self, wz, secs):
        tw = Twist()
        tw.angular.z = wz
        t0 = time.time()
        while time.time() - t0 < secs:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.pub.publish(Twist())

    def spin_for(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)

    def face(self, yaw):
        for _ in range(80):
            b = self.belief()
            if b is None:
                self.spin_for(0.2)
                continue
            err = wrap(yaw - b[2])
            if abs(err) < math.radians(3):
                return
            self.run(math.copysign(min(0.35, max(0.1, abs(err))), err), 0.2)

    def set_params(self, node, params):
        cli = self.create_client(SetParameters, f'{node}/set_parameters')
        if not cli.wait_for_service(timeout_sec=5.0):
            return f'no service {node}'
        req = SetParameters.Request()
        for k, v in params.items():
            pv = ParameterValue()
            if isinstance(v, bool):
                pv.type, pv.bool_value = ParameterType.PARAMETER_BOOL, v
            elif isinstance(v, int):
                pv.type, pv.integer_value = ParameterType.PARAMETER_INTEGER, v
            else:
                pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(v)
            req.parameters.append(Parameter(name=k, value=pv))
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        return None if res and all(r.successful for r in res.results) else \
            f'{node}: ' + '; '.join(r.reason for r in (res.results if res else []))

    def compute(self, goal, start=None):
        self.plan.wait_for_server(timeout_sec=10.0)
        g = ComputePathToPose.Goal()
        g.goal = PoseStamped()
        g.goal.header.frame_id = 'map'
        g.goal.pose.position.x, g.goal.pose.position.y = goal[0], goal[1]
        g.goal.pose.orientation.z, g.goal.pose.orientation.w = math.sin(goal[2] / 2), math.cos(goal[2] / 2)
        g.planner_id = 'GridBased'
        g.use_start = start is not None
        if start is not None:
            g.start = PoseStamped()
            g.start.header.frame_id = 'map'
            g.start.pose.position.x, g.start.pose.position.y = start[0], start[1]
            g.start.pose.orientation.z, g.start.pose.orientation.w = math.sin(start[2] / 2), math.cos(start[2] / 2)
        fut = self.plan.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        h = fut.result()
        if h is None or not h.accepted:
            return None, 'rejected'
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=15.0)
        r = rf.result()
        if r is None:
            return None, 'timeout'
        poses = [(p.pose.position.x, p.pose.position.y,
                  2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w)) for p in r.result.path.poses]
        return poses, r.result.error_code


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--sets', default='[{"name": "baseline", "params": {}}]')
    ap.add_argument('--starts', default='[null]', help='JSON list of [x, y, yaw_deg] or null (current pose)')
    args = ap.parse_args()
    from ament_index_python.packages import get_package_share_directory
    params = arena_layout.load_params(os.path.join(get_package_share_directory('cac_sim'), 'config',
                                                   'arena_params.yaml'))
    layout = arena_layout.generate_layout(params, args.seed, True)
    fp = arena_layout.Footprint.from_params(params)
    m = params['mission']
    goals = {'excavation': tuple(m['excavation_approach'])}
    rclpy.init()
    n = RouteTest()
    t0 = time.time()
    while not n.loc.get('initialized'):
        if time.time() - t0 > 120:
            raise SystemExit('no marker fix')
        n.run(0.35, 1.2)
        n.spin_for(1.0)
    for h in m['survey_headings']:
        n.face(math.radians(h))
        n.spin_for(m['survey_hold_sec'])
    n.spin_for(4.0)                      # costmap catches up
    start = n.belief()
    out = {'seed': args.seed, 'start_belief': start, 'layout': arena_layout.describe(layout), 'results': [],
           'crater_cells': {ob.name: sum(1 for x, y in n.cells if math.hypot(x - ob.x, y - ob.y) <= ob.radius + 0.05)
                            for ob in layout.obstacles if ob.kind == 'crater'},
           'crater_cells_total': len(n.cells)}
    print('crater cells', out['crater_cells'], out['crater_cells_total'], flush=True)
    for st in json.loads(args.sets):
        errs = [e for node, p in st['params'].items() if (e := n.set_params(node, p))]
        n.spin_for(3.0)
        for (gname, goal), s0 in itertools.product(goals.items(), json.loads(args.starts)):
            start = None if s0 is None else (s0[0], s0[1], math.radians(s0[2]))
            poses, code = n.compute(goal, start)
            row = {'set': st['name'], 'goal': gname, 'start': s0, 'error_code': code, 'param_errors': errs}
            if poses:
                row['length'] = round(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses, poses[1:])), 2)
                for ob in layout.obstacles:
                    row[f'clear_{ob.name}'] = round(min(rect_circle_distance(fp, p, ob.x, ob.y, ob.radius)
                                                        for p in poses), 3)
                row['min_clear'] = min(v for k, v in row.items() if k.startswith('clear_'))
                row['path'] = [(round(x, 2), round(y, 2)) for x, y, _ in poses[::10]]
            out['results'].append(row)
            print(json.dumps({k: v for k, v in row.items() if k != 'path'}), flush=True)
    with open(args.out + '.json', 'w') as f:
        json.dump(out, f, indent=1)
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
