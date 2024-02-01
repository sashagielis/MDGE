def transform_point(p, t):
    """
    Transforms point p using scaling matrix and translation vector given by t.

    :param p: [x, y]
    :param t: [m11, m12, m21, m22, t1, t2]
    :return: [[m11, m12], [m21, m22]] * [x, y] + [t1, t2]
    """
    return [t[0] * p[0] + t[1] * p[1] + t[4], t[2] * p[0] + t[3] * p[1] + t[5]]