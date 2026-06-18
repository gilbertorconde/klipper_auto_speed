# Find your printers max speed before losing steps
#
# Copyright (C) 2026 gilbertorconde (https://github.com/gilbertorconde) - Better Auto Speed fork
#
# This file may be distributed under the terms of the MIT license.
#
# Pure-math tests for better_auto_speed.funcs (no Klipper required).

import math

from better_auto_speed.funcs import (
    calculate_velocity,
    calculate_accel,
    calculate_distance,
    calculate_diagonal,
    calculate_graph,
    calculate_move_time,
)


def test_calculate_velocity():
    assert calculate_velocity(1000.0, 250.0) == math.sqrt(250.0 / 1000.0) * 1000.0


def test_calculate_accel():
    assert calculate_accel(100.0, 250.0) == 100.0 ** 2 / 250.0


def test_calculate_distance():
    assert calculate_distance(100.0, 1000.0) == 100.0 ** 2 / 1000.0


def test_velocity_accel_distance_roundtrip():
    accel = 2000.0
    travel = 180.0
    veloc = calculate_velocity(accel, travel)
    # The velocity reachable over `travel` ramps back to that same travel.
    assert math.isclose(calculate_distance(veloc, accel), travel)


def test_calculate_diagonal():
    assert calculate_diagonal(3.0, 4.0) == 5.0


def test_calculate_graph():
    assert calculate_graph(200.0, 100) == 10000.0 / (200.0 / 100)


def test_move_time_trapezoid():
    # dist >= veloc^2/accel -> reaches cruise velocity (trapezoid).
    accel, veloc, dist = 1000.0, 100.0, 100.0
    assert math.isclose(calculate_move_time(accel, veloc, dist), veloc / accel + dist / veloc)


def test_move_time_triangle():
    # dist < veloc^2/accel -> never reaches veloc (triangle profile).
    accel, veloc, dist = 1000.0, 100.0, 5.0
    assert math.isclose(calculate_move_time(accel, veloc, dist), 2.0 * math.sqrt(dist / accel))


def test_move_time_guards():
    assert calculate_move_time(1000.0, 100.0, 0.0) == float('inf')
    assert calculate_move_time(0.0, 100.0, 100.0) == float('inf')
    assert calculate_move_time(1000.0, 0.0, 100.0) == float('inf')
