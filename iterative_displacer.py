from constraint import ObstaclePairConstraint
from obstacle_displacer import ObstacleDisplacer
from point import Point
from utils import distance


class IterativeDisplacer(ObstacleDisplacer):
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        super().__init__(instance)

    def displace_obstacles(self):
        """
        Computes new obstacle positions by iteratively displacing obstacles.

        :returns: the maximum obstacle displacement
        """
        while self.conflicts:
            conflict = self.conflicts.pop()
            p1 = conflict.p1
            p2 = conflict.p2

            # Compute unit vector u from p1 to p2
            dist = distance(p1, p2)
            u = Point((p2.x - p1.x) / dist, (p2.y - p1.y) / dist)

            if type(conflict) == ObstaclePairConstraint:
                # Compute required displacement
                displacement = conflict.value / 2

                # Apply displacement in direction of vector u
                p1.x -= displacement * u.x
                p1.y -= displacement * u.y
                p2.x += displacement * u.x
                p2.y += displacement * u.y
            else:
                # Compute required displacement
                displacement = conflict.value

                # Apply displacement in direction of vector u
                p1.x -= displacement * u.x
                p1.y -= displacement * u.y

            # Update conflicting minimum separation constraints
            # TODO: INEFFICIENT!!!
            self.compute_constraints_naive()
