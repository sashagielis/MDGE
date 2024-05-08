import math

from point import Point


def transform_point(p, t):
    """
    Transforms point p using scaling matrix [[m11, m12], [m21, m22]] and translation vector [t1, t2].

    :param p: Point
    :param t: [m11, m12, m21, m22, t1, t2]
    """
    p.x = t[0] * p.x + t[1] * p.y + t[4]
    p.y = t[2] * p.x + t[3] * p.y + t[5]


def distance(p, q):
    """
    Determines the distance between points p and q.
    """
    return math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2)


def angle(p, q):
    """
    Determines the angle of rotation of line segment pq.
    """
    dx = q.x - p.x
    dy = q.y - p.y

    return math.atan2(dy, dx)


def angle_around_point(p, q, r):
    """
    Determines the counterclockwise angle in degrees by turning from p to r around q.
    """
    ang = math.degrees(angle(q, r) - angle(q, p))

    return ang + 360 if ang < 0 else ang


def vector_length(p):
    """
    Determines the length of vector p.
    """
    return math.sqrt(p.x ** 2 + p.y ** 2)


def vector_bisector(p, q):
    """
    Determines the bisector of vectors p and q, which is orthogonal and to the left of p if they are opposite vectors.
    """
    unit_p = p / vector_length(p)
    unit_q = q / vector_length(q)

    bisector = unit_p + unit_q
    if bisector == Point(0, 0):
        # p and q are opposite vectors
        bisector = Point(-unit_p.y, unit_p.x)

    return bisector


def orientation(p, q, r):
    """
    Determines the orientation of the ordered point triplet (p, q, r).
    Source: https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/

    :returns: 0 : collinear, 1 : clockwise, 2 : counterclockwise
    """
    val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)

    if val > 0:
        # Clockwise orientation
        return 1
    elif val < 0:
        # Counterclockwise orientation
        return 2
    else:
        # Collinear orientation
        return 0


def on_segment(p, q, r):
    """
    Determines whether point r lies on line segment pq.
    """
    if orientation(p, q, r) == 0 and min(p.x, q.x) <= r.x <= max(p.x, q.x) and min(p.y, q.y) <= r.y <= max(p.y, q.y):
        return True

    return False


def on_half_line(p, q, r):
    """
    Determines whether point r lies on the half-line with origin p and another point q.
    """
    return orientation(p, q, r) == 0 and (on_segment(p, q, r) or on_segment(p, r, q))


def check_segment_segment_intersection(p1, q1, p2, q2):
    """
    Determines whether line segments p1q1 and p2q2 intersect.
    Source: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
    """
    # Find the four orientations required for the general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if o1 != o2 and o3 != o4:
        return True

    # Special cases
    # p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if o1 == 0 and on_segment(p1, q1, p2):
        return True

    # p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if o2 == 0 and on_segment(p1, q1, q2):
        return True

    # p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if o3 == 0 and on_segment(p2, q2, p1):
        return True

    # p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if o4 == 0 and on_segment(p2, q2, q1):
        return True

    # If none of the cases apply
    return False


def check_segment_line_intersection(p1, q1, p2, q2):
    """
    Determines whether line segment p1q1 and the line through p2 and q2 intersect.
    """
    o_p1 = orientation(p2, q2, p1)
    o_q1 = orientation(q2, p2, q1)

    if o_p1 == 0 or o_q1 == 0 or o_p1 == o_q1:
        return True
    else:
        return False


def check_segment_half_line_intersection(p1, q1, p2, q2):
    """
    Determines whether line segment p1q1 and the half-line with origin p2 and another point q2 intersect.
    """
    if not check_segment_line_intersection(p1, q1, p2, q2):
        return False

    o_p1 = orientation(p2, q2, p1)
    o_q1 = orientation(p2, q2, q1)

    if o_p1 == 2 or o_q1 == 1:
        return angle_around_point(q1, p2, p1) <= 180
    elif o_p1 == 1 or o_q1 == 2:
        return angle_around_point(p1, p2, q1) <= 180
    else:
        # p1q1 and p2q2 are collinear
        return on_half_line(p2, q2, p1) or on_half_line(p2, q2, q1)
