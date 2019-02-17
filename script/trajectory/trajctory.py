#!/usr/bin/env python
import math
from twisted.conch.insults.insults import Vector

from geometry_msgs.msg import Point


class NegativeTimeException(Exception):
    pass

class Trajectory:
    def __init__(self):
        self.position = Point()

    def get_position_at(self, t):
        if t < 0:
            raise NegativeTimeException()

def abs_vector(self):
    return (self.x * self.x + self.y * self.y) ** 0.5


def sub_point(self, other):
    return Vector(self.x - other.x, self.y - other.y)


Vector.__abs__ = abs_vector
Point.__sub__ = sub_point