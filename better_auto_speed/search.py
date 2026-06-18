# Find your printers max speed before losing steps
#
# Copyright (C) 2024 Anonoei <dev@anonoei.com>
# Copyright (C) 2026 gilbertorconde (https://github.com/gilbertorconde) - Better Auto Speed fork
#
# This file may be distributed under the terms of the MIT license.
#
# Per-axis speed search: bounds resolution, the binary search loop, and the
# single measured attempt that drives it.

from time import perf_counter

from .funcs import calculate_accel, calculate_velocity
from .move import MoveX, MoveY, MoveZ, MoveDiagX, MoveDiagY
from .wrappers import AttemptWrapper


class SpeedSearch:
    def __init__(self, machine, motion, settings):
        self.machine = machine
        self.motion = motion
        self.settings = settings

    def resolve_accel_bounds(self, gcmd, max_dist, veloc_stat):
        accel_max = (gcmd.get_float('ACCEL_MAX', None, above=1.0)
                     or self.settings.cfg_accel_max or 100000.0)
        # The size floor is only meaningful with a fixed companion velocity:
        # the lowest accel must still let the move fit the available travel.
        size_min = calculate_accel(veloc_stat, max_dist) if veloc_stat else None
        accel_min = (gcmd.get_float('ACCEL_MIN', None, above=1.0)
                     or self.settings.cfg_accel_min or size_min or 1000.0)
        accel_min = max(1.0, min(accel_min, accel_max - 1.0))
        return accel_min, accel_max

    def resolve_veloc_bounds(self, gcmd, max_dist, accel_max):
        size_max = calculate_velocity(accel_max, max_dist)  # sqrt(accel_max * D)
        veloc_max = (gcmd.get_float('VELOCITY_MAX', None, above=1.0)
                     or self.settings.cfg_veloc_max or size_max)
        veloc_min = (gcmd.get_float('VELOCITY_MIN', None, above=1.0)
                     or self.settings.cfg_veloc_min or 50.0)
        veloc_min = max(1.0, min(veloc_min, veloc_max - 1.0))
        return veloc_min, veloc_max

    def init_axis(self, aw: AttemptWrapper, axis):
        aw.axis = axis
        if axis == "diag_x":
            aw.move = MoveDiagX()
        elif axis == "diag_y":
            aw.move = MoveDiagY()
        elif axis == "x":
            aw.move = MoveX()
        elif axis == "y":
            aw.move = MoveY()
        elif axis == "z":
            aw.move = MoveZ()
        aw.move.Init(self.machine.axis_limits, aw.margin, self.settings.isolate_xy)

    def binary_search(self, aw: AttemptWrapper):
        aw.time_start = perf_counter()
        m_min = aw.min
        m_max = aw.max
        m_var = m_min + (m_max-m_min) // 3

        if aw.veloc == 0.0:
            aw.veloc = 1.0
        if aw.accel == 0.0:
            aw.accel = 1.0

        # Force full-travel test moves (coupled accel search) so the move stays
        # long as accel rises and the toolhead actually reaches the cruise velocity.
        fd = aw.move.max_dist if aw.full_dist else None

        if aw.type in ("accel", "graph"): # stat is velocity, var is accel
            m_stat = aw.veloc
            o_veloc = aw.veloc
            if o_veloc == 1.0:
                aw.accel = calculate_accel(aw.veloc, aw.move.max_dist)
            aw.move.Calc(self.machine.axis_limits, m_stat, m_var, aw.margin, fd)

        elif aw.type in ("velocity"): # stat is accel, var is velocity
            m_stat = aw.accel
            o_accel = aw.accel
            if o_accel == 1.0:
                aw.veloc = calculate_velocity(aw.accel, aw.move.max_dist)
            aw.move.Calc(self.machine.axis_limits, m_var, m_stat, aw.margin, fd)

        measuring = True
        measured_val = None
        aw.valid_max = None
        aw.tries = 0
        aw.home_steps, aw.move_time_prehome = self.motion.prehome(aw.move.home)
        while measuring:
            aw.tries += 1
            if aw.type in ("accel", "graph"):
                if o_veloc == 1.0:
                    m_stat = aw.veloc = calculate_velocity(m_var, aw.move.dist)/2.5
                aw.accel = m_var
                aw.move.Calc(self.machine.axis_limits, m_stat, m_var, aw.margin, fd)
            elif aw.type == "velocity":
                if o_accel == 1.0:
                    m_stat = aw.accel = calculate_accel(m_var, aw.move.dist)*2.5
                aw.veloc = m_var
                aw.move.Calc(self.machine.axis_limits, m_var, m_stat, aw.margin, fd)
            #self.gcode.respond_info(str(aw))

            valid = self.attempt(aw)

            if aw.type in ("accel", "graph"):
                veloc = m_stat
                accel = m_var
            elif aw.type in ("velocity"):
                veloc = m_var
                accel = m_stat
            respond = f"BETTER AUTO SPEED {aw.type} on {aw.axis} try {aw.tries} ({aw.time_last:.2f}s)\n"
            respond += f"Moved {aw.move_dist - aw.margin:.2f}mm at a{accel:.0f}/v{veloc:.0f} after {aw.move_time_prehome:.2f}/{aw.move_time:.2f}/{aw.move_time_posthome:.2f}s\n"
            respond += f"Missed"
            if aw.move.home[0]:
                respond += f" X {aw.missed['x']:.2f},"
            if aw.move.home[1]:
                respond += f" Y {aw.missed['y']:.2f},"
            if aw.move.home[2]:
                respond += f" Z {aw.missed['z']:.2f},"
            self.machine.gcode.respond_info(respond[:-1])
            if measured_val is not None:
                if m_var * (1 + aw.accuracy) > m_max or m_var * (1 - aw.accuracy) < m_min:
                    measuring = False
            measured_val = m_var
            if valid:
                m_min = m_var
                if aw.valid_max is None or m_var > aw.valid_max:
                    aw.valid_max = m_var
            else:
                m_max = m_var
            m_var = (m_min + m_max)//2

        aw.time_total = perf_counter() - aw.time_start
        return m_var

    def attempt(self, aw: AttemptWrapper):
        timeAttempt = perf_counter()

        self.motion.set_velocity(self.machine.th_veloc, self.machine.th_accel, self.machine.th_scv)
        start_coord = [aw.move.pos["x"][0], aw.move.pos["y"][0], aw.move.pos["z"][0]]
        self.motion.move(start_coord, self.motion.position_speed(start_coord, self.machine.th_veloc))
        self.machine.toolhead.wait_moves()
        self.motion.set_velocity(aw.veloc, aw.accel, aw.scv)
        timeMove = perf_counter()

        self.motion.move([aw.move.pos["x"][1], aw.move.pos["y"][1], aw.move.pos["z"][1]], aw.veloc)
        self.machine.toolhead.wait_moves()
        aw.move_time = perf_counter() - timeMove
        aw.move_dist = aw.move.dist

        valid, aw.home_steps, aw.missed, aw.move_time_posthome = self.motion.posttest(aw.home_steps, aw.max_missed, aw.move.home)
        aw.time_last = perf_counter() - timeAttempt
        return valid
