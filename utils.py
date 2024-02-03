import math


def transform_point(p, t):
    """
    Transforms point p using scaling matrix [[m11, m12], [m21, m22]] and translation vector [t1, t2].

    :param p: [x, y]
    :param t: [m11, m12, m21, m22, t1, t2]
    """
    return [t[0] * p[0] + t[1] * p[1] + t[4], t[2] * p[0] + t[3] * p[1] + t[5]]


def distance(p1, p2):
    """
    Determines the distance between points p1 and p2.
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def angle(p1, p2):
    """
    Determines the angle of rotation of line segment (p1, p2).
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    return math.atan2(dy, dx)
