#!/usr/bin/env python
from math import sin, pi, cos

from .trajectory import Trajectory


class CircularTrajectory(object, Trajectory):
    def __init__(self, radius, period, x_0=0, y_0=0):
        Trajectory.__init__(self)
        self.radius = radius
        self. period = period
        self.x_0 = x_0
        self.y_0 = y_0

    def get_position_at(self, t):
        super(CircularTrajectory, self).get_position_at(t)
        self.position.x = self.radius * sin(2 * pi * t / self.period) + self.x_0
        self.position.y = -self.radius * cos(2 * pi * t / self.period) + self.y_0

        return self.position

    def get_name(self):
        return str(CircularTrajectory.__name__).replace('Trajectory', '').lower()