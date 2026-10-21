"""Material (sand) accounting for excavation and dumping, kept free of ROS so it can be unit tested.

WHAT THIS IS: a bookkeeping model of how much sand the rover has taken from the
excavation zone, carries in its bucket and has poured, with every quantity in
explicit units (m^3, kg). It replaces the old time-based payload counter.

WHAT THIS IS NOT: granular physics. Gazebo's sand is rigid; no sand moves in the
simulation. Every volume here is a SIMULATED ESTIMATE derived from the bucket
geometry and the parameters below, not a measurement. The rulebook scores the
berm by a volumetric scan before and after the run; nothing in this model is
such a scan.

Rules enforced (each rejection is recorded as an event with a reason):
  scoop  credited only when the dig pose was reached, the bucket lip is inside
         the excavation zone (rulebook Construction Protocol 5: all berm sand
         comes from the excavation zone) and low enough to cut the sand, the
         bucket has room and the dig site under the lip still holds sand.
  dump   credited only when the pour pose was reached, the bucket holds sand
         and the lip is over the construction zone. Sand poured within the berm
         target circle counts as `deposited_in_target`; elsewhere in the
         construction zone as `deposited_outside_target` (still out of the bucket,
         but the rulebook counts only berm volume within the target location).
  every action carries an id; an id is credited at most once, so a repeated
  callback or a re-sent arm sequence cannot collect or dump the same sand twice.

Conservation (checked by tests): collected == in_target + outside_target + in_bucket,
and every quantity is >= 0.
"""
import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class MaterialParams:
    # Bucket interior (urdf/arm.xacro bucket_link: 0.14 x 0.16 x 0.07 m outside,
    # 1 cm plates) = 0.13 x 0.14 x 0.065 m struck volume.
    bucket_capacity_m3: float = 0.13 * 0.14 * 0.065
    # Fraction of the struck volume one scoop fills (ASSUMPTION: loose dry sand,
    # a single shallow pass of a small bucket; not measured).
    fill_factor: float = 0.8
    # Loose dry sand bulk density (ASSUMPTION, typical 1400-1600 kg/m^3); only
    # used to report mass and the carried load.
    bulk_density_kg_m3: float = 1500.0
    # Lip must be within this height of the sand surface (z = 0) to cut sand.
    # The dig pose puts it 1.7 cm above the surface (arm_params.yaml); the arm
    # cannot penetrate the rigid Gazebo sand.
    max_cut_height_m: float = 0.05
    # Dig sites: the excavation zone is divided into square cells; each holds
    # cell^2 * max_dig_depth of diggable sand (rulebook: ~30 cm sand bed; the
    # usable depth per site is an ASSUMPTION).
    dig_cell_m: float = 0.25
    max_dig_depth_m: float = 0.15
    excavation_x: tuple = (2.0, 3.5)
    construction_x: tuple = (3.5, 4.5)
    arena_half_width: float = 2.5
    berm_target_xy: tuple = (4.0, 0.0)
    berm_target_radius_m: float = 0.4

    @classmethod
    def from_params(cls, p: dict) -> 'MaterialParams':
        """From the arena_params.yaml dict (arena_layout.load_params) or the same keys flattened."""
        m, z = p['material'], p['zones']
        return cls(bucket_capacity_m3=m['bucket_capacity_l'] / 1000.0, fill_factor=m['fill_factor'],
                   bulk_density_kg_m3=m['bulk_density_kg_m3'], max_cut_height_m=m['max_cut_height'],
                   dig_cell_m=m['dig_cell'], max_dig_depth_m=m['max_dig_depth'],
                   excavation_x=tuple(z['excavation_zone']), construction_x=tuple(z['construction_zone']),
                   arena_half_width=p['arena']['width'] / 2.0, berm_target_xy=tuple(z['berm_target']),
                   berm_target_radius_m=z.get('berm_target_radius', 0.4))


@dataclass
class MaterialEvent:
    kind: str          # 'scoop' | 'dump'
    action_id: str
    accepted: bool
    volume_m3: float
    reason: str
    t: float = 0.0
    lip: tuple = ()
    detail: dict = field(default_factory=dict)

    def as_dict(self):
        return {'kind': self.kind, 'id': self.action_id, 'accepted': self.accepted,
                'volume_l': round(self.volume_m3 * 1000.0, 3), 'reason': self.reason, 't': round(self.t, 1),
                'lip': [round(v, 3) for v in self.lip], **self.detail}


class MaterialModel:

    def __init__(self, params: MaterialParams = MaterialParams()):
        self.p = params
        self.reset()

    def reset(self):
        self.in_bucket_m3 = 0.0
        self.collected_m3 = 0.0
        self.deposited_in_target_m3 = 0.0
        self.deposited_outside_target_m3 = 0.0
        self.removed_by_site = {}       # (i, j) cell -> m^3 already dug
        self.events = []
        self._credited_ids = set()

    # ----------------------------------------------------------------- geometry

    def site_of(self, x: float, y: float) -> tuple:
        c = self.p.dig_cell_m
        return (math.floor(x / c), math.floor(y / c))

    def site_capacity_m3(self) -> float:
        return self.p.dig_cell_m ** 2 * self.p.max_dig_depth_m

    def site_remaining_m3(self, x: float, y: float) -> float:
        return max(0.0, self.site_capacity_m3() - self.removed_by_site.get(self.site_of(x, y), 0.0))

    def in_excavation_zone(self, x: float, y: float) -> bool:
        return self.p.excavation_x[0] <= x <= self.p.excavation_x[1] and abs(y) <= self.p.arena_half_width

    def in_construction_zone(self, x: float, y: float) -> bool:
        return self.p.construction_x[0] <= x <= self.p.construction_x[1] and abs(y) <= self.p.arena_half_width

    def in_berm_target(self, x: float, y: float) -> bool:
        tx, ty = self.p.berm_target_xy
        return math.hypot(x - tx, y - ty) <= self.p.berm_target_radius_m

    # ------------------------------------------------------------------ actions

    def _record(self, ev: MaterialEvent) -> MaterialEvent:
        self.events.append(ev)
        if ev.accepted:
            self._credited_ids.add((ev.kind, ev.action_id))
        return ev

    def scoop(self, action_id: str, lip: tuple, pose_reached: bool, t: float = 0.0) -> MaterialEvent:
        """One dig pass. lip = bucket-lip (x, y, z) in the arena frame at the dig pose."""
        x, y, z = lip

        def reject(reason):
            return self._record(MaterialEvent('scoop', action_id, False, 0.0, reason, t, tuple(lip)))
        if ('scoop', action_id) in self._credited_ids:
            return reject('duplicate action id (already credited)')
        if not pose_reached:
            return reject('dig pose not reached')
        if not self.in_excavation_zone(x, y):
            return reject(f'bucket lip at ({x:.2f}, {y:.2f}) is outside the excavation zone')
        if z > self.p.max_cut_height_m:
            return reject(f'bucket lip {z:.3f} m above the sand (max {self.p.max_cut_height_m} m)')
        room = self.p.bucket_capacity_m3 - self.in_bucket_m3
        if room <= 1e-12:
            return reject('bucket already full')
        available = self.site_remaining_m3(x, y)
        if available <= 1e-12:
            return reject(f'dig site {self.site_of(x, y)} is exhausted')
        vol = min(self.p.bucket_capacity_m3 * self.p.fill_factor, room, available)
        site = self.site_of(x, y)
        self.removed_by_site[site] = self.removed_by_site.get(site, 0.0) + vol
        self.in_bucket_m3 += vol
        self.collected_m3 += vol
        left_l = round(self.site_remaining_m3(x, y) * 1000, 3)
        return self._record(MaterialEvent('scoop', action_id, True, vol, 'collected', t, tuple(lip),
                                          {'site': list(site), 'site_left_l': left_l}))

    def dump(self, action_id: str, lip: tuple, pose_reached: bool, fraction: float = 1.0,
             t: float = 0.0) -> MaterialEvent:
        """Pour `fraction` of the bucket. lip = bucket-lip (x, y, z) in the arena frame at the pour pose."""
        x, y = lip[0], lip[1]

        def reject(reason):
            return self._record(MaterialEvent('dump', action_id, False, 0.0, reason, t, tuple(lip)))
        if ('dump', action_id) in self._credited_ids:
            return reject('duplicate action id (already credited)')
        if not pose_reached:
            return reject('pour pose not reached')
        if not 0.0 < fraction <= 1.0:
            return reject(f'invalid pour fraction {fraction}')
        if self.in_bucket_m3 <= 1e-12:
            return reject('bucket is empty')
        if not self.in_construction_zone(x, y):
            return reject(f'bucket lip at ({x:.2f}, {y:.2f}) is outside the construction zone')
        vol = self.in_bucket_m3 * fraction
        self.in_bucket_m3 = max(0.0, self.in_bucket_m3 - vol)
        in_target = self.in_berm_target(x, y)
        if in_target:
            self.deposited_in_target_m3 += vol
        else:
            self.deposited_outside_target_m3 += vol
        return self._record(MaterialEvent('dump', action_id, True, vol,
                                          'deposited in berm target' if in_target else
                                          'deposited in construction zone, OUTSIDE the berm target',
                                          t, tuple(lip), {'in_target': in_target}))

    # ------------------------------------------------------------------ summary

    def summary(self) -> dict:
        rho = self.p.bulk_density_kg_m3
        acc = [e for e in self.events if e.accepted]
        return {
            'note': 'SIMULATED ESTIMATE from bucket geometry and assumed parameters; no sand moves in Gazebo',
            'collected_l': round(self.collected_m3 * 1000, 3),
            'in_bucket_l': round(self.in_bucket_m3 * 1000, 3),
            'deposited_in_target_l': round(self.deposited_in_target_m3 * 1000, 3),
            'deposited_outside_target_l': round(self.deposited_outside_target_m3 * 1000, 3),
            'deposited_in_target_cm3': round(self.deposited_in_target_m3 * 1e6, 1),
            'collected_kg': round(self.collected_m3 * rho, 3),
            'scoops_ok': sum(e.kind == 'scoop' for e in acc),
            'dumps_ok': sum(e.kind == 'dump' for e in acc),
            'rejected': sum(not e.accepted for e in self.events),
            'events': [e.as_dict() for e in self.events],
        }


def bcp_mass(berm_volume_cm3: float, run_time_min: float, robot_mass_kg: float) -> float:
    """Rulebook "Construction Berm Productivity - Normalized for Robot Mass":
    berm volume within the target zone (cm^3) / (run time in minutes * robot mass in kg).
    Fed with simulated volumes this is an ESTIMATE, not a score."""
    if run_time_min <= 0 or robot_mass_kg <= 0:
        raise ValueError('run time and robot mass must be positive')
    return berm_volume_cm3 / (run_time_min * robot_mass_kg)


def pile_shape(volume_m3: float, repose_deg: float = 34.0) -> tuple:
    """(radius, height) of a conical sand pile of the given volume for the Gazebo visual:
    a cone at the angle of repose (ASSUMED 34 deg, loose dry sand). The visual has no
    collision and is hidden from the LiDAR (material_visualizer.py); it is a picture
    of the accounted volume only."""
    if volume_m3 <= 0:
        return 0.0, 0.0
    t = math.tan(math.radians(repose_deg))
    r = (3.0 * volume_m3 / (math.pi * t)) ** (1.0 / 3.0)
    return r, r * t
