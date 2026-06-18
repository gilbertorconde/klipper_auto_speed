# Find your printers max speed before losing steps
#
# Copyright (C) 2024 Anonoei <dev@anonoei.com>
# Copyright (C) 2026 gilbertorconde (https://github.com/gilbertorconde) - Better Auto Speed fork
#
# This file may be distributed under the terms of the MIT license.
#
# Pure helpers for parsing and formatting axis selections.


def parse_axis(raw_axes, valid_axes):
    raw_axes = raw_axes.lower()
    raw_axes = raw_axes.replace(" ", "")
    raw_axes = raw_axes.split(',')
    return [axis for axis in raw_axes if axis in valid_axes]


def axis_to_str(axes):
    result = ""
    for axis in axes:
        result += f"{axis},"
    return result[:-1]
