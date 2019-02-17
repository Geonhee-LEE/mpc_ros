#!/usr/bin/env python
from .lissajous_trajectory import LissajousTrajectory
from .circular_trajectory import CircularTrajectory
from .epitrochoid_trajectory import EpitrochoidTrajectory
from .lemniscate_trajectory import LemniscateTrajectory
from .squared_trajectory import SquaredTrajectory
from .linear_trajectory import LinearTrajectory

def create_trajectory(name, period):
    if name == 'linear':
        return LinearTrajectory(0.05, 0.01, 0.05, 0.01)
    elif name == 'circular':
        return CircularTrajectory(2.0, period)
    elif name == 'squared':
        return SquaredTrajectory(2.0, period, 0.01, 0.01)
    elif name == 'lemniscate':
        return LemniscateTrajectory(2.0, period)
    elif name == 'epitrochoid':
        return EpitrochoidTrajectory(5, 1, 3, period, 1 / 3.0)
    elif name == 'lissajous':
        return LissajousTrajectory(1, 1, 3, 2, period)