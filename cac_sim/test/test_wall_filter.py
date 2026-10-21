"""wall_filter.ground_range: where a tilted 0.10 m scan plane meets the ground."""
import math

import numpy as np

from cac_sim.wall_filter import ground_range


def test_level_scanner_never_hits_ground():
    a = np.linspace(-math.pi, math.pi, 361)
    assert np.all(np.isinf(ground_range(a, 0.0, 0.0, 0.10)))


def test_nose_down_pitch_hits_ground_ahead_only():
    # the measured stowed pitch: 1.32 deg nose down -> 4.3 m straight ahead
    a = np.array([0.0, math.pi / 2, math.pi])
    r = ground_range(a, 0.0, math.radians(1.32), 0.10)
    assert abs(r[0] - 0.10 / math.sin(math.radians(1.32))) < 1e-6 and 4.2 < r[0] < 4.4
    assert np.isinf(r[1]) and np.isinf(r[2])         # sideways level, behind points up


def test_roll_left_side_up_hits_ground_on_the_right():
    a = np.array([math.pi / 2, -math.pi / 2])
    r = ground_range(a, math.radians(2.0), 0.0, 0.10)
    assert np.isinf(r[0]) and abs(r[1] - 0.10 / math.sin(math.radians(2.0))) < 1e-6


def test_obstacle_closer_than_ground_range_is_kept():
    # what _on_scan does: beams at/after (ground range - margin) are dropped
    a = np.array([0.0, 0.0])
    rg = ground_range(a, 0.0, math.radians(2.5), 0.10)      # ~2.29 m
    ranges = np.array([1.5, 2.25])                          # boulder at 1.5 m, ground return
    ground = ranges >= rg - 0.30
    assert list(ground) == [False, True]


def test_beam_spread_reproduces_measured_ground_returns():
    # measured at rest (pitch 1.32 deg): 3.21-3.29 m ahead, 3.34-3.46 m at 20 deg, 3.56-3.75 m at 30 deg
    a = np.radians([0.0, 20.0, 30.0])
    r = ground_range(a, 0.0, math.radians(1.32), 0.10, math.radians(0.45))
    assert 3.15 < r[0] < 3.35 and 3.30 < r[1] < 3.50 and 3.50 < r[2] < 3.80
