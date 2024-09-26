from utils import distance


class Constraint:
    """
    A minimum-separation constraint on two points.
    """
    def __init__(self, p1, p2, min_sep):
        """
        :param p1: a Point object
        :param p2: a Point object
        :param min_sep: the minimum required separation between p1 and p2
        """
        self.p1 = p1
        self.p2 = p2
        self.min_separation = min_sep

    @property
    def value(self):
        """
        Returns the conflict value of the constraint.
        """
        return max(0, self.min_separation - distance(self.p1, self.p2))

    def __str__(self):
        return f"d({self.p1}, {self.p2}) >= {self.min_separation}"


class ObstaclePairConstraint(Constraint):
    """
    A minimum separation constraint on two point obstacles.
    """
    def __init__(self, o1, o2, min_sep):
        """
        :param o1: a PointObstacle object
        :param o2: a PointObstacle object
        :param min_sep: the minimum required separation between o1 and o2
        """
        super().__init__(o1, o2, min_sep)


class ObstacleVertexConstraint(Constraint):
    """
    A minimum separation constraint on a point obstacle and a point.
    """
    def __init__(self, o, v, min_sep):
        """
        :param o: a PointObstacle object
        :param v: a Vertex object
        :param min_sep: the minimum required separation between o and v
        """
        super().__init__(o, v, min_sep)
