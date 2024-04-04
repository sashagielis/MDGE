class Constraint:
    def __init__(self, p1, p2, min_sep):
        self.p1 = p1
        self.p2 = p2
        self.min_separation = min_sep


class ObstaclePairConstraint(Constraint):
    def __init__(self, o1, o2, min_sep):
        super().__init__(o1, o2, min_sep)


class ObstacleVertexConstraint(Constraint):
    def __init__(self, o, v, min_sep):
        super().__init__(o, v, min_sep)
