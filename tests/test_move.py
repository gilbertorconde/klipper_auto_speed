# Find your printers max speed before losing steps
#
# Copyright (C) 2026 gilbertorconde (https://github.com/gilbertorconde) - Better Auto Speed fork
#
# This file may be distributed under the terms of the MIT license.
#
# Geometry tests for better_auto_speed.move (no Klipper required).

import math

import pytest

from better_auto_speed.funcs import calculate_distance
from better_auto_speed.move import MoveX, MoveY, MoveZ, MoveDiagX, MoveDiagY


def make_limits(dist=300.0):
    axis = {"min": 0.0, "max": dist, "center": dist / 2, "dist": dist, "home": 0.0}
    return {"x": dict(axis), "y": dict(axis), "z": dict(axis)}


def test_movex_distance_math():
    limits = make_limits()
    mv = MoveX()
    mv.Init(limits, margin=20.0, isolate_xy=True)
    assert mv.max_dist == 300.0 - 40.0
    mv.Calc(limits, veloc=100.0, accel=1000.0, margin=20.0)
    expected = calculate_distance(100.0, 1000.0) / 2 + 20.0  # 5 + margin
    assert math.isclose(mv.dist, expected)
    assert mv.home == [True, False, False]
    assert math.isclose(mv.pos["x"][0], limits["x"]["max"] - mv.dist)
    assert math.isclose(mv.pos["x"][1], limits["x"]["max"] - 20.0)


def test_movey_distance_math():
    limits = make_limits()
    mv = MoveY()
    mv.Init(limits, margin=20.0, isolate_xy=True)
    mv.Calc(limits, veloc=100.0, accel=1000.0, margin=20.0)
    expected = calculate_distance(100.0, 1000.0) / 2 + 20.0
    assert math.isclose(mv.dist, expected)
    assert mv.home == [False, True, False]


def test_movez_distance_math():
    limits = make_limits()
    mv = MoveZ()
    mv.Init(limits, 20.0, False)
    mv.Calc(limits, veloc=50.0, accel=500.0, margin=20.0)
    # Z uses full calculate_distance (no /2 halving).
    expected = calculate_distance(50.0, 500.0) + 20.0
    assert math.isclose(mv.dist, expected)
    assert mv.home == [False, False, True]
    # home (0) <= min (0) -> move up from min.
    assert math.isclose(mv.pos["z"][0], limits["z"]["min"] + mv.dist)


def test_movediagx_distance_math():
    limits = make_limits()
    mv = MoveDiagX()
    mv.Init(limits, 20.0, None)
    mv.Calc(limits, veloc=200.0, accel=1000.0, margin=20.0)
    expected = calculate_distance(200.0, 1000.0) / 2 * math.sin(45) + 20.0
    assert math.isclose(mv.dist, expected)
    assert mv.home == [True, True, False]


def test_movediagy_distance_math():
    limits = make_limits()
    mv = MoveDiagY()
    mv.Init(limits, 20.0, None)
    mv.Calc(limits, veloc=200.0, accel=1000.0, margin=20.0)
    expected = calculate_distance(200.0, 1000.0) / 2 * math.sin(45) + 20.0
    assert math.isclose(mv.dist, expected)
    # DiagY sweeps x from min upward.
    assert math.isclose(mv.pos["x"][0], limits["x"]["min"] + mv.dist)


def test_full_dist_override():
    limits = make_limits()
    mv = MoveX()
    mv.Init(limits, margin=20.0, isolate_xy=True)
    # An override larger than max_dist gets clamped to max_dist after +margin.
    mv.Calc(limits, veloc=100.0, accel=1000.0, margin=20.0, dist=500.0)
    assert mv.dist == mv.max_dist


def test_validate_min_clamp():
    limits = make_limits()
    mv = MoveX()
    mv.Init(limits, margin=0.0, isolate_xy=True)
    # Tiny computed distance is clamped up to the 5mm floor (margin 0).
    mv.Calc(limits, veloc=10.0, accel=1000.0, margin=0.0)
    assert mv.dist == 5.0
