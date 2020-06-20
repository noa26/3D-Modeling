from typing import Tuple

import numpy as np


def calculate_pc_adjustments(object_pc: np.ndarray, captured_pc: np.ndarray) -> Tuple[float, float, float]:
    assert len(object_pc.shape) == 2
    assert len(captured_pc.shape) == 2
    assert object_pc.shape[1] == 3
    assert captured_pc.shape[1] == 3

    mins_obj = np.min(object_pc, axis=0)
    mins_cap = np.min(captured_pc, axis=0)
    maxs_obj = np.max(object_pc, axis=0)
    maxs_cap = np.max(captured_pc, axis=0)

    # Maybe use the average over an edge but that's the idea.
    # y values is a bit problematic because we have the plate on the bottom,
    # so we can only calculate the difference over the top.
    # Because all z values should be equal we can use and average all over the z's to get an estimate of the z-shift.
    x_shift = ((mins_obj[0] - mins_cap[0]) + (maxs_obj[0] - maxs_cap[0])) / 2
    y_shift = (maxs_obj[1] - maxs_cap[1])
    z_shift = np.average(object_pc, axis=0)[2] - np.average(captured_pc, axis=0)[2]

    return x_shift, y_shift, z_shift

    # Just an idea but maybe there's a problem in scaling so we can calculate the scale ratio as such:
    # x_scale = (maxs_obj[0] - mins_obj[0])/((maxs_cap[0] - mins_cap[0])
    # y_scale = maxs_obj[1]/maxs_cap[1]
    # Again, maybe we can use an average over an edge,
    # and also y values are again problematic so we can only use the top.
    # I am not sure about z-scale or if it exists.
    # Also I think it'll be easier to average over an edge if we get the frames (or frame matrices)
    # instead of pc's into this function, and find all the left pixels row by row and then convert them to points
    # and average their x value's, the same for y values (z values are not affected).
