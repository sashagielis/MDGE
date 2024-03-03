import math


def transform_point(p, t):
    """
    Transforms point p using scaling matrix [[m11, m12], [m21, m22]] and translation vector [t1, t2].
    :param p: Point
    :param t: [m11, m12, m21, m22, t1, t2]
    """
    p.x = t[0] * p.x + t[1] * p.y + t[4]
    p.y = t[2] * p.x + t[3] * p.y + t[5]


def distance(p1, p2):
    """
    Determines the distance between points p1 and p2.
    """
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def angle(p1, p2):
    """
    Determines the angle of rotation of line segment (p1, p2).
    """
    dx = p2.x - p1.x
    dy = p2.y - p1.y

    return math.atan2(dy, dx)
