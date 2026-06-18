# Find your printers max speed before losing steps
#
# Copyright (C) 2024 Anonoei <dev@anonoei.com>
# Copyright (C) 2026 gilbertorconde (https://github.com/gilbertorconde) - Better Auto Speed fork
#
# This file may be distributed under the terms of the MIT license.
#
# All BETTER_AUTO_SPEED* gcode handlers and the orchestration around them
# (coupled sweep, pair finalization, prep/variance, config save).

from time import perf_counter

from .funcs import calculate_graph, calculate_move_time
from .wrappers import ResultsWrapper, AttemptWrapper
from .axes import parse_axis, axis_to_str
from .plotting import render_graph


class AutoSpeedCommands:
    def __init__(self, machine, motion, search, overrides, settings):
        self.machine = machine
        self.motion = motion
        self.search = search
        self.overrides = overrides
        self.settings = settings
        self.gcode = machine.gcode

    cmd_BETTER_AUTO_SPEED_help = ("Automatically find your printer's maximum acceleration/velocity")
    def cmd_BETTER_AUTO_SPEED(self, gcmd):
        self.machine.check_homed(gcmd)

        validate = gcmd.get_int('VALIDATE', 0, minval=0, maxval=1)
        save = gcmd.get_int('SAVE', 1, minval=0, maxval=1)
        couple = gcmd.get_int('COUPLE', 1, minval=0, maxval=1)

        # Coupling inputs: hold one quantity fixed and find the other.
        # VELOC is an alias for VELOCITY at this level.
        accel_in = gcmd.get_float('ACCEL', None, above=1.0)
        veloc_in = gcmd.get_float('VELOC', None, above=1.0)
        if veloc_in is None:
            veloc_in = gcmd.get_float('VELOCITY', None, above=1.0)

        # Apply current/homing-speed overrides once here, and neutralize the
        # related params so nested sub-calls don't re-apply or re-save.
        override_state = self.overrides.apply(gcmd)
        for k in ("X_CURRENT", "Y_CURRENT", "Z_CURRENT",
                  "X_HOMING_SPEED", "Y_HOMING_SPEED", "Z_HOMING_SPEED"):
            gcmd._params.pop(k, None)
        gcmd._params["SAVE"] = 0

        try:
            self._prepare(gcmd) # Make sure the printer is level, [check endstop variance]

            move_z = gcmd.get_int('Z', None)
            if move_z is not None:
                coord = [None, None, move_z]
                self.motion.move(coord, self.motion.position_speed(coord, self.machine.th_veloc))

            # Explicit fixed values take precedence over the (default) sweep.
            if accel_in is not None and veloc_in is not None:
                # Both supplied: nothing to search, report/save/validate the pair.
                self._finalize_pair(gcmd, accel_in, veloc_in, save, validate)
            elif accel_in is not None:
                # Fixed accel -> find the velocity that works with it.
                veloc_results = self.cmd_BETTER_AUTO_SPEED_VELOCITY(gcmd)
                self._finalize_pair(gcmd, accel_in, veloc_results.vals['rec'], save, validate)
            elif veloc_in is not None:
                # Fixed velocity -> find the accel that works with it. The accel
                # search holds the value passed as VELOCITY.
                gcmd._params["VELOCITY"] = veloc_in
                accel_results = self.cmd_BETTER_AUTO_SPEED_ACCEL(gcmd)
                self._finalize_pair(gcmd, accel_results.vals['rec'], veloc_in, save, validate)
            elif couple:
                # Default: sweep velocities, measure max accel at each, and
                # recommend the best combined pair (highest throughput).
                self._couple_sweep(gcmd, save, validate)
            else:
                start = perf_counter()
                accel_results = self.cmd_BETTER_AUTO_SPEED_ACCEL(gcmd)
                veloc_results = self.cmd_BETTER_AUTO_SPEED_VELOCITY(gcmd)

                respond = f"BETTER AUTO SPEED found recommended acceleration and velocity after {perf_counter() - start:.2f}s\n"
                for axis in self.settings.valid_axes:
                    aR = accel_results.vals.get(axis, None)
                    vR = veloc_results.vals.get(axis, None)
                    if aR is not None or vR is not None:
                        respond += f"| {axis.replace('_', ' ').upper()} max:"
                        if aR is not None:
                            respond += f" a{aR:.0f}"
                        if vR is not None:
                            respond += f" v{vR:.0f}"
                        respond += "\n"

                respond += f"Recommended accel: {accel_results.vals['rec']:.0f}\n"
                respond += f"Recommended velocity: {veloc_results.vals['rec']:.0f}\n"
                self.gcode.respond_info(respond)

                if save:
                    self._save_to_config(gcmd, accel=accel_results.vals['rec'], velocity=veloc_results.vals['rec'])

                if validate:
                    gcmd._params["ACCEL"] = accel_results.vals['rec']
                    gcmd._params["VELOCITY"] = veloc_results.vals['rec']
                    self.cmd_BETTER_AUTO_SPEED_VALIDATE(gcmd)
        finally:
            self.overrides.restore(override_state)
            self.motion.restore_velocity_limits()
            if gcmd.get_int('MOTOR_OFF', 1 if self.settings.motor_off else 0, minval=0, maxval=1):
                self.gcode.run_script_from_command("M84")

    def _finalize_pair(self, gcmd, accel, velocity, save, validate):
        # Report a coupled accel/velocity pair, then optionally save/validate it.
        respond = "BETTER AUTO SPEED coupled accel/velocity\n"
        respond += f"Recommended accel: {accel:.0f}\n"
        respond += f"Recommended velocity: {velocity:.0f}\n"
        self.gcode.respond_info(respond)
        if save:
            self._save_to_config(gcmd, accel=accel, velocity=velocity)
        if validate:
            gcmd._params["ACCEL"] = accel
            gcmd._params["VELOCITY"] = velocity
            self.cmd_BETTER_AUTO_SPEED_VALIDATE(gcmd)

    def _couple_sweep(self, gcmd, save, validate):
        # Sweep a set of cruise velocities, measure the max accel that passes at
        # each (per axis), combine conservatively across axes, then pick the pair
        # that minimizes the time of a representative move sized to the printer.
        axes = parse_axis(gcmd.get("AXIS", axis_to_str(self.settings.axes)), self.settings.valid_axes)
        axes = self._prepare_axes(axes)

        margin     = gcmd.get_float("MARGIN", self.settings.margin, above=0.0)
        derate     = gcmd.get_float('DERATE', self.settings.derate, above=0.0, below=1.0)
        max_missed = gcmd.get_float('MAX_MISSED', self.settings.max_missed, above=0.0)
        accel_accu = gcmd.get_float('ACCEL_ACCU', self.settings.accel_accu, above=0.0, below=1.0)
        scv        = gcmd.get_float('SCV', self.settings.scv, above=1.0)
        veloc_div  = gcmd.get_int('VELOCITY_DIV', 5, minval=2)

        # Representative move distance and velocity bounds come from the most
        # constraining (shortest-travel) axis, so the pair is safe everywhere.
        dists = {}
        for axis in axes:
            probe = AttemptWrapper()
            probe.margin = margin
            self.search.init_axis(probe, axis)
            dists[axis] = probe.move.max_dist
        min_dist = min(dists.values())

        accel_ceiling = (gcmd.get_float('ACCEL_MAX', None, above=1.0)
                         or self.settings.cfg_accel_max or 100000.0)
        veloc_min, veloc_max = self.search.resolve_veloc_bounds(gcmd, min_dist, accel_ceiling)
        veloc_step = (veloc_max - veloc_min) / (veloc_div - 1)
        velocs = [round((v * veloc_step) + veloc_min) for v in range(veloc_div)]

        respond = "BETTER AUTO SPEED coupling accel/velocity on"
        for axis in axes:
            respond += f" {axis.upper().replace('_', ' ')},"
        respond = respond[:-1] + f"\nSweeping velocities {velocs}"
        self.gcode.respond_info(respond)

        start = perf_counter()
        # measured[axis][i] holds the max passing accel at velocs[i], or None if
        # no accel in range passed (that velocity is infeasible on that axis).
        measured = {axis: [] for axis in axes}
        for axis in axes:
            for veloc in velocs:
                aw = AttemptWrapper()
                aw.type = "accel"
                aw.accuracy = accel_accu
                aw.max_missed = max_missed
                aw.margin = margin
                aw.veloc = veloc
                aw.scv = scv
                # Use a full-travel move so the toolhead truly reaches `veloc` and
                # high-accel attempts stay stressful (otherwise accel pegs at max).
                aw.full_dist = True
                self.search.init_axis(aw, axis)
                aw.min, aw.max = self.search.resolve_accel_bounds(gcmd, aw.move.max_dist, veloc)
                self.gcode.respond_info(
                    f"BETTER AUTO SPEED coupling {axis.upper().replace('_', ' ')} - "
                    f"v{veloc:.0f} (accel {aw.min:.0f} - {aw.max:.0f})")
                self.search.binary_search(aw)
                measured[axis].append(aw.valid_max)

        # Conservative combine: the accel safe on every tested axis at a given
        # velocity is the minimum measured across axes. A velocity is feasible
        # only if every axis found a passing accel for it.
        combined = []
        for i in range(len(velocs)):
            vals = [measured[axis][i] for axis in axes]
            combined.append(None if any(v is None for v in vals) else min(vals))

        best_i = None
        best_t = None
        respond = f"BETTER AUTO SPEED coupling results after {perf_counter() - start:.2f}s\n"
        for i, veloc in enumerate(velocs):
            if combined[i] is None:
                respond += f"| v{veloc:.0f} -> infeasible (no passing accel)\n"
                continue
            rec_a = combined[i] * derate
            rec_v = veloc * derate
            t = calculate_move_time(rec_a, rec_v, min_dist)
            if best_t is None or t < best_t:
                best_t = t
                best_i = i
            respond += (f"| v{veloc:.0f} -> max accel {combined[i]:.0f}"
                        f" | move time {t * 1000:.1f}ms\n")

        if best_i is None:
            respond += ("No feasible accel/velocity pair found. Try lowering "
                        "VELOCITY_MAX/ACCEL_MAX or increasing MAX_MISSED.")
            self.gcode.respond_info(respond)
            return

        respond += (f"Best pair: accel {combined[best_i] * derate:.0f},"
                    f" velocity {velocs[best_i] * derate:.0f}"
                    f" (move time {best_t * 1000:.1f}ms over {min_dist:.0f}mm)")
        self.gcode.respond_info(respond)

        self._finalize_pair(gcmd, combined[best_i] * derate,
                            velocs[best_i] * derate, save, validate)

    cmd_BETTER_AUTO_SPEED_ACCEL_help = ("Automatically find your printer's maximum acceleration")
    def cmd_BETTER_AUTO_SPEED_ACCEL(self, gcmd):
        self.machine.check_homed(gcmd)
        override_state = self.overrides.apply(gcmd)
        try:
            axes = parse_axis(gcmd.get("AXIS", axis_to_str(self.settings.axes)), self.settings.valid_axes)
            axes = self._prepare_axes(axes)

            margin         = gcmd.get_float("MARGIN", self.settings.margin, above=0.0)
            derate         = gcmd.get_float('DERATE', self.settings.derate, above=0.0, below=1.0)
            max_missed      = gcmd.get_float('MAX_MISSED', self.settings.max_missed, above=0.0)

            accel_accu = gcmd.get_float('ACCEL_ACCU', self.settings.accel_accu, above=0.0, below=1.0)

            veloc = gcmd.get_float('VELOCITY', 1.0, above=1.0)
            veloc_stat = veloc if veloc != 1.0 else None
            scv =   gcmd.get_float('SCV', self.settings.scv, above=1.0)

            respond = "BETTER AUTO SPEED finding maximum acceleration on"
            for axis in axes:
                respond += f" {axis.upper().replace('_', ' ')},"
            self.gcode.respond_info(respond[:-1])

            rw = ResultsWrapper()
            start = perf_counter()
            for axis in axes:
                aw = AttemptWrapper()
                aw.type = "accel"
                aw.accuracy = accel_accu
                aw.max_missed = max_missed
                aw.margin = margin

                aw.veloc = veloc
                aw.scv = scv
                self.search.init_axis(aw, axis)
                aw.min, aw.max = self.search.resolve_accel_bounds(gcmd, aw.move.max_dist, veloc_stat)
                self.gcode.respond_info(
                    f"BETTER AUTO SPEED accel range on {axis.upper().replace('_', ' ')}: "
                    f"{aw.min:.0f} - {aw.max:.0f}")
                rw.vals[aw.axis] = self.search.binary_search(aw)
            rw.duration = perf_counter() - start

            rw.name = "acceleration"
            respond = f"BETTER AUTO SPEED found maximum acceleration after {rw.duration:.2f}s\n"
            for axis in self.settings.valid_axes:
                if rw.vals.get(axis, None) is not None:
                    respond += f"| {axis.replace('_', ' ').upper()} max: {rw.vals[axis]:.0f}\n"
            respond += f"\n"

            rw.derate(derate)
            respond += f"Recommended values:\n"
            for axis in self.settings.valid_axes:
                if rw.vals.get(axis, None) is not None:
                    respond += f"| {axis.replace('_', ' ').upper()} max: {rw.vals[axis]:.0f}\n"
            respond += f"Recommended acceleration: {rw.vals['rec']:.0f}\n"

            self.gcode.respond_info(respond)

            if gcmd.get_int('SAVE', 1, minval=0, maxval=1):
                self._save_to_config(gcmd, accel=rw.vals['rec'])
            return rw
        finally:
            self.overrides.restore(override_state)
            self.motion.restore_velocity_limits()

    cmd_BETTER_AUTO_SPEED_VELOCITY_help = ("Automatically find your printer's maximum velocity")
    def cmd_BETTER_AUTO_SPEED_VELOCITY(self, gcmd):
        self.machine.check_homed(gcmd)
        override_state = self.overrides.apply(gcmd)
        try:
            axes = parse_axis(gcmd.get("AXIS", axis_to_str(self.settings.axes)), self.settings.valid_axes)
            axes = self._prepare_axes(axes)

            margin         = gcmd.get_float("MARGIN", self.settings.margin, above=0.0)
            derate         = gcmd.get_float('DERATE', self.settings.derate, above=0.0, below=1.0)
            max_missed      = gcmd.get_float('MAX_MISSED', self.settings.max_missed, above=0.0)

            veloc_accu = gcmd.get_float('VELOCITY_ACCU', self.settings.veloc_accu, above=0.0, below=1.0)

            accel = gcmd.get_float('ACCEL', 1.0, above=1.0)
            # Accel ceiling used to size the reachable velocity (sqrt(accel * travel)):
            # the fixed companion accel if one was given, otherwise the accel ceiling.
            accel_ceiling = accel if accel != 1.0 else (
                gcmd.get_float('ACCEL_MAX', None, above=1.0) or self.settings.cfg_accel_max or 100000.0)
            scv =   gcmd.get_float('SCV', self.settings.scv, above=1.0)

            respond = "BETTER AUTO SPEED finding maximum velocity on"
            for axis in axes:
                respond += f" {axis.upper().replace('_', ' ')},"
            self.gcode.respond_info(respond[:-1])

            rw = ResultsWrapper()
            start = perf_counter()
            for axis in axes:
                aw = AttemptWrapper()
                aw.type = "velocity"
                aw.accuracy  = veloc_accu
                aw.max_missed = max_missed
                aw.margin = margin

                aw.accel = accel
                aw.scv = scv
                self.search.init_axis(aw, axis)
                aw.min, aw.max = self.search.resolve_veloc_bounds(gcmd, aw.move.max_dist, accel_ceiling)
                self.gcode.respond_info(
                    f"BETTER AUTO SPEED velocity range on {axis.upper().replace('_', ' ')}: "
                    f"{aw.min:.0f} - {aw.max:.0f}")
                rw.vals[aw.axis] = self.search.binary_search(aw)
            rw.duration = perf_counter() - start

            rw.name = "velocity"
            respond = f"BETTER AUTO SPEED found maximum velocity after {rw.duration:.2f}s\n"
            for axis in self.settings.valid_axes:
                if rw.vals.get(axis, None) is not None:
                    respond += f"| {axis.replace('_', ' ').upper()} max: {rw.vals[axis]:.0f}\n"
            respond += "\n"

            rw.derate(derate)
            respond += f"Recommended values\n"
            for axis in self.settings.valid_axes:
                if rw.vals.get(axis, None) is not None:
                    respond += f"| {axis.replace('_', ' ').upper()} max: {rw.vals[axis]:.0f}\n"
            respond += f"Recommended velocity: {rw.vals['rec']:.0f}\n"

            self.gcode.respond_info(respond)

            if gcmd.get_int('SAVE', 1, minval=0, maxval=1):
                self._save_to_config(gcmd, velocity=rw.vals['rec'])
            return rw
        finally:
            self.overrides.restore(override_state)
            self.motion.restore_velocity_limits()

    cmd_BETTER_AUTO_SPEED_VALIDATE_help = ("Validate your printer's acceleration/velocity don't miss steps")
    def cmd_BETTER_AUTO_SPEED_VALIDATE(self, gcmd):
        self.machine.check_homed(gcmd)
        override_state = self.overrides.apply(gcmd)
        try:
            max_missed   = gcmd.get_float('MAX_MISSED', self.settings.max_missed, above=0.0)
            margin       = gcmd.get_float('VALIDATE_MARGIN', default=self.settings.validate_margin, above=0.0)
            small_margin = gcmd.get_float('VALIDATE_INNER_MARGIN', default=self.settings.validate_inner_margin, above=0.0)
            iterations   = gcmd.get_int('VALIDATE_ITERATIONS', default=self.settings.validate_iterations, minval=1)

            accel = gcmd.get_float('ACCEL', default=self.machine.toolhead.max_accel, above=0.0)
            veloc = gcmd.get_float('VELOCITY', default=self.machine.toolhead.max_velocity, above=0.0)
            scv =   gcmd.get_float('SCV', self.settings.scv, above=1.0)

            respond = f"BETTER AUTO SPEED validating over {iterations} iterations\n"
            respond += f"Acceleration: {accel:.0f}\n"
            respond += f"Velocity: {veloc:.0f}\n"
            respond += f"SCV: {scv:.0f}"
            self.gcode.respond_info(respond)
            self.motion.set_velocity(veloc, accel, scv)
            valid, duration, missed_x, missed_y = self.motion.validate(veloc, iterations, margin, small_margin, max_missed)

            respond = f"BETTER AUTO SPEED validated results after {duration:.2f}s\n"
            respond += f"Valid: {valid}\n"
            respond += f"Missed X {missed_x:.2f}, Y {missed_y:.2f}"
            self.gcode.respond_info(respond)
            return valid
        finally:
            self.overrides.restore(override_state)
            self.motion.restore_velocity_limits()

    cmd_BETTER_AUTO_SPEED_GRAPH_help = ("Graph your printer's maximum acceleration at given velocities")
    def cmd_BETTER_AUTO_SPEED_GRAPH(self, gcmd):
        import matplotlib          # fail early here if matplotlib isn't installed
        self.machine.check_homed(gcmd)
        axes = parse_axis(gcmd.get("AXIS", axis_to_str(self.settings.axes)), self.settings.valid_axes)
        axes = self._prepare_axes(axes)

        margin     = gcmd.get_float("MARGIN", self.settings.margin, above=0.0)
        derate     = gcmd.get_float('DERATE', self.settings.derate, above=0.0, below=1.0)
        max_missed = gcmd.get_float('MAX_MISSED', self.settings.max_missed, above=0.0)

        scv        = gcmd.get_float('SCV', self.settings.scv, above=1.0)

        veloc_min  = gcmd.get_float('VELOCITY_MIN', 200.0, above=0.0)
        veloc_max  = gcmd.get_float('VELOCITY_MAX', 700.0, above=veloc_min)
        veloc_div  = gcmd.get_int(  'VELOCITY_DIV', 5, minval=2)

        accel_accu = gcmd.get_float('ACCEL_ACCU', self.settings.accel_accu, above=0.0, below=1.0)

        accel_min_slope = gcmd.get_int('ACCEL_MIN_SLOPE', 100, minval=0)
        accel_max_slope = gcmd.get_int('ACCEL_MAX_SLOPE', 1800, minval=accel_min_slope)

        veloc_step = (veloc_max - veloc_min)//(veloc_div - 1)
        velocs = [round((v * veloc_step) + veloc_min) for v in range(0, veloc_div)]
        respond = "BETTER AUTO SPEED graphing maximum accel from velocities on"
        for axis in axes:
            respond += f" {axis.upper().replace('_', ' ')},"
        respond = respond[:-1] + "\n"
        respond += f"V_MIN: {veloc_min}, V_MAX: {veloc_max}, V_STEP: {veloc_step}\n"
        self.gcode.respond_info(respond)

        aw = AttemptWrapper()
        aw.type = "graph"
        aw.accuracy = accel_accu
        aw.max_missed = max_missed
        aw.margin = margin
        aw.scv = scv
        try:
            for axis in axes:
                start = perf_counter()
                self.search.init_axis(aw, axis)
                accels = []
                accel_mins = []
                accel_maxs = []
                for veloc in velocs:
                    self.gcode.respond_info(f"BETTER AUTO SPEED graph {aw.axis} - v{veloc}")
                    aw.veloc = veloc
                    aw.min = round(calculate_graph(veloc, accel_min_slope))
                    aw.max = round(calculate_graph(veloc, accel_max_slope))
                    accel_mins.append(aw.min)
                    accel_maxs.append(aw.max)
                    accels.append(self.search.binary_search(aw))
                filepath = render_graph(self.settings.results_dir, aw.axis, accel_accu,
                                        velocs, accels, accel_mins, accel_maxs, derate)
                self.gcode.respond_info(f"Velocs: {velocs}")
                self.gcode.respond_info(f"Accels: {accels}")
                self.gcode.respond_info(f"BETTER AUTO SPEED graph found max accel on {aw.axis} after {perf_counter() - start:.0f}s\nSaving graph to {filepath}")
        finally:
            self.motion.restore_velocity_limits()

    def _save_to_config(self, gcmd, accel=None, velocity=None):
        configfile = self.machine.printer.lookup_object('configfile')
        saved = []
        if accel is not None:
            configfile.set('printer', 'max_accel', round(accel))
            saved.append(f"max_accel={round(accel)}")
        if velocity is not None:
            configfile.set('printer', 'max_velocity', round(velocity))
            saved.append(f"max_velocity={round(velocity)}")
        if saved:
            self.gcode.respond_info(
                "BETTER AUTO SPEED queued to [printer]: " + ", ".join(saved) +
                "\nRun SAVE_CONFIG to apply (this rewrites printer.cfg and restarts Klipper).")

    def _prepare(self, gcmd):
        self.machine.check_homed(gcmd)

        start = perf_counter()
        # Level the printer if it's not leveled
        self.motion.level(gcmd)
        center = [self.machine.axis_limits["x"]["center"], self.machine.axis_limits["y"]["center"], self.machine.axis_limits["z"]["center"]]
        self.motion.move(center, self.motion.position_speed(center, self.machine.th_veloc))

        self._variance(gcmd)

        return perf_counter() - start

    def _variance(self, gcmd):
        variance        = gcmd.get_int('VARIANCE', 1, minval=0, maxval=1)

        max_missed      = gcmd.get_float('MAX_MISSED', self.settings.max_missed, above=0.0)
        endstop_samples = gcmd.get_int('ENDSTOP_SAMPLES', self.settings.endstop_samples, minval=2)

        settling_home   = gcmd.get_int("SETTLING_HOME", default=self.settings.settling_home, minval=0, maxval=1)

        if variance == 0:
            return

        self.gcode.respond_info(f"BETTER AUTO SPEED checking endstop variance over {endstop_samples} samples")

        if settling_home:
            self.machine.toolhead.wait_moves()
            self.motion.home(True, True, False)

        axes = parse_axis(gcmd.get("AXIS", axis_to_str(self.settings.axes)), self.settings.valid_axes)

        check_x = 'x' in axes if self.settings.isolate_xy else True
        check_y = 'y' in axes if self.settings.isolate_xy else True

        # Check endstop variance
        endstops = self.motion.endstop_variance(endstop_samples, x=check_x, y=check_y)

        x_max = max(endstops["x"]) if check_x else 0
        y_max = max(endstops["y"]) if check_y else 0
        self.gcode.respond_info(f"BETTER AUTO SPEED endstop variance:\nMissed X:{x_max:.2f} steps, Y:{y_max:.2f} steps")

        if x_max >= max_missed or y_max >= max_missed:
            raise gcmd.error(f"Please increase MAX_MISSED (currently {max_missed}), or tune your steppers/homing macro.")

    def _prepare_axes(self, axes):
        # Drop Z when there's no stepper_z data (unsupported/cartesian setups where
        # Z wasn't captured) and warn when Z is probe-homed (unreliable detection).
        if "z" in axes and "z" not in self.machine.steppers:
            self.gcode.respond_info(
                "BETTER AUTO SPEED warning: no stepper_z data found; skipping Z axis.")
            axes = [a for a in axes if a != "z"]
        if "z" in axes and self.machine.z_uses_probe():
            self.gcode.respond_info(
                "BETTER AUTO SPEED warning: stepper_z homes via a probe/virtual "
                "endstop, so missed-step detection on Z is unreliable and results "
                "may be over-optimistic. Treat Z recommendations with caution.")
        return axes
