#!/usr/bin/env python3
"""Crater visibility test (TEST TOOL): does the camera pipeline mark a crater?

Run against `ros2 launch cac_sim cac_sim.launch.py seed:=N autonomy:=false`:

  python3 tools/crater_view_test.py --seed 6 --crater crater_2 --standoff 2.0 1.5 1.0

Turns in place until marker_localizer has a fix, then for each stand-off
distance drives (Nav2 NavigateToPose) to a pose that distance from the crater
centre, facing it, waits --dwell seconds and reports the fraction of the
crater's 5 cm cells present on /crater_cells and the number of marked cells
farther than 0.10 m from every crater (false positives). Uses the seeded
layout only to choose poses and to score -- the detector never sees it.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

try:
    from cac_sim import arena_layout
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from cac_sim import arena_layout


class Tester(Node):
    def __init__(self):
        super().__init__('crater_view_test')
        self.cells = np.zeros((0, 2))
        self.loc = {}
        self.create_subscription(PointCloud2, '/crater_cells', self.on_cells, 2)
        self.create_subscription(String, '/localization/status', self.on_loc, 2)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def on_cells(self, m):
        self.cells = np.frombuffer(bytes(m.data), np.float32).reshape(-1, m.point_step // 4)[:, :2] \
            if m.width else np.zeros((0, 2))

    def on_loc(self, m):
        self.loc = json.loads(m.data)

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def localize(self, timeout=60.0):
        end = time.time() + timeout
        t0 = time.time()
        while time.time() < end and not self.loc.get('initialized'):
            tw = Twist()
            tw.angular.z = 0.35 if (time.time() - t0) % 2.2 < 1.2 else 0.0   # same stepped turn as the mission
            self.cmd.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd.publish(Twist())
        return bool(self.loc.get('initialized'))

    def wait_nav2_active(self, timeout=180.0):
        from lifecycle_msgs.srv import GetState
        cl = self.create_client(GetState, '/bt_navigator/get_state')
        end = time.time() + timeout
        while time.time() < end:
            if cl.wait_for_service(timeout_sec=1.0):
                f = cl.call_async(GetState.Request())
                while not f.done():
                    rclpy.spin_once(self, timeout_sec=0.05)
                if f.result().current_state.id == 3:   # ACTIVE
                    return True
            self.spin_for(1.0)
        return False

    def goto(self, x, y, yaw, timeout=180.0):
        self.nav.wait_for_server(timeout_sec=30.0)
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        fut = self.nav.send_goal_async(g)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        h = fut.result()
        if not h.accepted:
            return 'rejected'
        res = h.get_result_async()
        end = time.time() + timeout
        while not res.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not res.done():
            h.cancel_goal_async()
            return 'timeout'
        return 'succeeded' if res.result().status == GoalStatus.STATUS_SUCCEEDED else \
            f'failed({res.result().result.error_code})'

    def score(self, layout):
        out = {}
        far = np.ones(len(self.cells), dtype=bool)
        for ob in layout.obstacles:
            if ob.kind != 'crater':
                continue
            d = np.hypot(self.cells[:, 0] - ob.x, self.cells[:, 1] - ob.y) if len(self.cells) else np.zeros(0)
            out[ob.name] = round(min(1.0, float((d <= ob.radius + 0.05).sum()) / (math.pi * ob.radius ** 2 / 0.0025)), 2)
            far &= d > ob.radius + 0.10 if len(d) else far
        return out, int(far.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--crater', default='crater_0')
    ap.add_argument('--standoff', type=float, nargs='+', default=[2.0, 1.5, 1.0])
    ap.add_argument('--dwell', type=float, default=4.0)
    ap.add_argument('--arena-params', default=None)
    a = ap.parse_args()
    path = a.arena_params
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('cac_sim'), 'config', 'arena_params.yaml')
    params = arena_layout.load_params(path)
    layout = arena_layout.generate_layout(params, a.seed)
    target = next(o for o in layout.obstacles if o.name == a.crater)
    rclpy.init()
    t = Tester()
    if not t.localize():
        print(json.dumps({'error': 'no localization fix'}))
        return
    if not t.wait_nav2_active():
        print(json.dumps({'error': 'Nav2 not active'}))
        return
    t.spin_for(2.0)
    results = []
    for d in a.standoff:
        # approach from the side facing the start zone (west), clamped inside the arena
        x, y = target.x - d, target.y
        y = max(-1.8, min(1.8, y))
        yaw = math.atan2(target.y - y, target.x - x)
        x = target.x - d * math.cos(yaw)
        nav = t.goto(x, y, yaw)
        t.spin_for(a.dwell)
        cov, false_cells = t.score(layout)
        results.append({'standoff_m': d, 'nav': nav, 'coverage': cov, 'false_cells': false_cells,
                        'localization': t.loc.get('map_to_odom')})
        print(json.dumps(results[-1]))
    print('SUMMARY', json.dumps({'seed': a.seed, 'crater': a.crater, 'results': results}))
    t.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
