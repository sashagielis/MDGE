import math
from fractions import Fraction

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
    Determines the angle of rotation of line segment pq in radians.
    """
    dx = q.x - p.x
    dy = q.y - p.y

    return normalize_angle(math.atan2(dy, dx))


def normalize_angle(a):
    """
    Normalizes an angle in radians to the range [0, 2π].
    """
    full_rotation = Fraction(2 * math.pi)

    while a < 0:
        a += full_rotation

    while a > 2 * math.pi:
        a -= full_rotation

    return a


def rotation_angle(a1, a2):
    """
    Determines the counterclockwise angle of rotation from angle a1 to angle a2 in radians.
    """
    a1 = normalize_angle(a1)
    a2 = normalize_angle(a2)

    rotation = a1 - a2

    if rotation > 0:
        rotation = 2 * math.pi - rotation
    else:
        rotation = -rotation

    return rotation


def vector_length(p):
    """
    Determines the length of vector p.
    """
    return math.sqrt(p.x ** 2 + p.y ** 2)


def dot(p, q):
    """
    Determines the dot product between vectors p and q.
    """
    return p.x * q.x + p.y * q.y


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


def check_segment_arc_intersection(p, q, center, radius, left_angle, right_angle, proper_intersect):
    """
    Determines whether line segment pq and the arc with given center, radius and angles intersect.
    The angles of the arc must be given in clockwise order in radians.
    If proper_intersect is True, we only register proper intersections, i.e., not tangent.
    """
    # We define the line through p and q as a function of the time t, where 0 <= t <= 1 for segment pq
    # We then substitute it into the equation of the circle with center c and radius r
    # By solving the resulting equation for t we can determine the times of intersection of the line with the circle
    dx, dy = q.x - p.x, q.y - p.y
    a = dx ** 2 + dy ** 2
    b = 2 * (dx * (p.x - center.x) + dy * (p.y - center.y))
    c = (p.x - center.x) ** 2 + (p.y - center.y) ** 2 - radius ** 2

    discriminant = b ** 2 - 4 * a * c
    if discriminant < 0:
        return False

    t1 = (-b - math.sqrt(discriminant)) / (2 * a)
    t2 = (-b + math.sqrt(discriminant)) / (2 * a)

    # If t1 = t2, the line and circle are tangent and thus the intersection is not proper
    if proper_intersect and t1 == t2:
        return False

    # Normalize the angles
    left_angle = normalize_angle(left_angle)
    right_angle = normalize_angle(right_angle)

    # Segment pq only intersects the circle if the time of intersection is between 0 and 1
    if 0 <= t1 <= 1:
        intersection = p + t1 * (q - p)

        # Check if the intersection lies between the two angles of the arc
        if left_angle >= angle(center, intersection) >= right_angle:
            return True

    if 0 <= t2 <= 1:
        intersection = p + t2 * (q - p)

        # Check if the intersection lies between the two angles of the arc
        if left_angle >= angle(center, intersection) >= right_angle:
            return True

    return False


def check_rectangle_arc_intersection(p1, p2, p3, p4, center, radius, left_angle, right_angle, proper_intersect):
    """
    Determines whether the rectangle p1p2p3p4 and the arc with given center, radius and angles intersect.
    The corner points of the rectangle must be given in counterclockwise order along the boundary.
    The angles of the arc must be given in clockwise order in radians.
    If proper_intersect is True, we only register proper intersections, i.e., not tangent.
    """
    rec_sides = [[p1, p2], [p2, p3], [p3, p4], [p4, p1]]

    # Normalize the angles
    left_angle = normalize_angle(left_angle)
    right_angle = normalize_angle(right_angle)

    # Check if one of the arc's endpoints intersects the rectangle
    arc_p1 = center + radius * Point(math.cos(left_angle), math.sin(left_angle))
    arc_p2 = center + radius * Point(math.cos(right_angle), math.sin(right_angle))
    arc_points = [arc_p1, arc_p2]
    for a in arc_points:
        total_orientation = 0
        for side in rec_sides:
            # If an endpoint of the arc lies on the rectangle's boundary, the intersection is not proper
            if not proper_intersect and on_segment(side[0], side[1], a):
                return True
            else:
                total_orientation += orientation(side[0], side[1], a)

        # If total orientation equals 8, the point is left of each rectangle's side, and thus inside the rectangle
        if total_orientation == 8:
            return True

    # Check if one of the sides of the rectangle intersects the arc
    for side in rec_sides:
        if check_segment_arc_intersection(side[0], side[1], center, radius, left_angle, right_angle, proper_intersect):
            return True

    return False
