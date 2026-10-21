#!/usr/bin/env python3
"""Gazebo picture of the SIMULATED sand accounting (visual only).

Listens to /mission_status (mission_state_machine's JSON, which carries the
material_model events) and, through the Gazebo world's create/remove services:
  - after every credited dump: a sand-coloured pile whose volume equals the total
    deposited volume, at the volume-weighted mean pour point (material_model.pile_shape:
    cone at the angle of repose);
  - after every credited scoop: a dark disc where the bucket lip cut the sand.

These models are STATIC, have NO collision geometry and carry visibility flags
without bit 0, so the LiDAR (visibility mask = bit 0, urdf/rover_properties.xacro)
never sees them; the depth camera does, but the crater detector only marks points
BELOW the fitted ground plane. They change nothing the robot can feel: the sand
bed itself stays rigid and no material moves. They exist so a person watching
Gazebo sees where the accounting says sand was taken and put.
"""
import json
import queue
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from cac_sim import material_model as mm

_HIDDEN_FROM_LIDAR = 0xFFFFFFFE


def pile_sdf(name: str, x: float, y: float, volume_m3: float) -> str:
    r, h = mm.pile_shape(volume_m3)
    shape = f"<cone><radius>{r:.4f}</radius><length>{h:.4f}</length></cone>"
    return (f"<sdf version='1.11'><model name='{name}'><static>true</static>"
            f"<pose>{x:.3f} {y:.3f} {h / 2:.4f} 0 0 0</pose><link name='pile'><visual name='sand'>"
            f"<geometry>{shape}</geometry><material><ambient>0.85 0.72 0.45 1</ambient>"
            f"<diffuse>0.85 0.72 0.45 1</diffuse></material>"
            f"<visibility_flags>{_HIDDEN_FROM_LIDAR}</visibility_flags></visual></link></model></sdf>")


def dig_mark_sdf(name: str, x: float, y: float) -> str:
    return (f"<sdf version='1.11'><model name='{name}'><static>true</static>"
            f"<pose>{x:.3f} {y:.3f} 0.002 0 0 0</pose><link name='mark'><visual name='cut'>"
            f"<geometry><cylinder><radius>0.08</radius><length>0.002</length></cylinder></geometry>"
            f"<material><ambient>0.35 0.27 0.15 1</ambient><diffuse>0.35 0.27 0.15 1</diffuse></material>"
            f"<visibility_flags>{_HIDDEN_FROM_LIDAR}</visibility_flags></visual></link></model></sdf>")


class MaterialVisualizer(Node):

    def __init__(self):
        super().__init__('material_visualizer')
        self.declare_parameter('world', 'cac_arena')
        self._world = self.get_parameter('world').value
        self._seen = set()           # event ids already drawn
        self._dumps = []             # (x, y, volume_m3)
        self._pile = None            # current pile model name
        self._jobs = queue.Queue()
        threading.Thread(target=self._worker, daemon=True).start()
        self.create_subscription(String, '/mission_status', self._on_status, 10)

    def _gz(self, service, reqtype, req) -> bool:
        try:
            out = subprocess.run(['gz', 'service', '-s', f'/world/{self._world}/{service}', '--reqtype', reqtype,
                                  '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                                 capture_output=True, text=True, timeout=10)
        except (OSError, subprocess.TimeoutExpired) as e:
            self.get_logger().warn(f'gz service {service} failed: {e}')
            return False
        ok = 'data: true' in out.stdout
        if not ok:
            self.get_logger().warn(f'gz service {service} did not confirm: {out.stdout.strip()} {out.stderr.strip()}')
        return ok

    def _create(self, sdf: str) -> bool:
        return self._gz('create', 'gz.msgs.EntityFactory', f'sdf: "{sdf}" allow_renaming: false')

    def _remove(self, name: str):
        self._gz('remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')

    def _worker(self):
        while True:
            self._jobs.get()()

    def _on_status(self, msg: String):
        try:
            events = json.loads(msg.data).get('summary', {}).get('material_events', [])
        except ValueError:
            return
        for ev in events:
            key = (ev.get('kind'), ev.get('id'))
            if not ev.get('accepted') or key in self._seen:
                continue
            self._seen.add(key)
            x, y = ev['lip'][0], ev['lip'][1]
            if ev['kind'] == 'scoop':
                name = f'dig_mark_{len([k for k in self._seen if k[0] == "scoop"])}'
                self._jobs.put(lambda n=name, x=x, y=y: self._create(dig_mark_sdf(n, x, y)))
            else:
                self._dumps.append((x, y, ev['volume_l'] / 1000.0))
                vol = sum(v for _, _, v in self._dumps)
                px = sum(x * v for x, _, v in self._dumps) / vol
                py = sum(y * v for _, y, v in self._dumps) / vol
                name, old = f'berm_pile_{len(self._dumps)}', self._pile
                self._pile = name

                def job(n=name, old=old, px=px, py=py, vol=vol):
                    if self._create(pile_sdf(n, px, py, vol)) and old:
                        self._remove(old)
                    r, h = mm.pile_shape(vol)
                    self.get_logger().info(f'berm pile visual: {vol * 1000:.2f} L (SIMULATED) at ({px:.2f}, {py:.2f}), '
                                           f'r={r:.3f} m h={h:.3f} m')
                self._jobs.put(job)


def main():
    rclpy.init()
    node = MaterialVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
