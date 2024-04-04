from constraint import ObstaclePairConstraint
from obstacle_displacer import ObstacleDisplacer
from point import Point
from utils import distance


class HeuristicDisplacer(ObstacleDisplacer):
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        super().__init__(instance)

    def execute(self):
        """
        Computes new obstacle positions by iteratively displacing obstacles.

        :returns: the total obstacle displacement
        """
        # Compute minimum separation constraints
        self.compute_constraints()

        cost = 0
        for constraint in self.constraints:
            p1 = constraint.p1
            p2 = constraint.p2

            # Compute conflict
            dist = distance(p1, p2)
            conflict = max(0, constraint.min_separation - dist)

            # Compute unit vector u from p1 to p2
            u = Point((p2.x - p1.x) / dist, (p2.y - p1.y) / dist)

            if type(constraint) == ObstaclePairConstraint:
                # Compute required displacement
                displacement = conflict / 2

                # Apply displacement in direction of vector u
                p1.x -= displacement * u.x
                p1.y -= displacement * u.y
                p2.x += displacement * u.x
                p2.y += displacement * u.y

                # Update displacement cost
                p1.displacement += displacement
                p2.displacement += displacement
                cost = max(cost, p1.displacement, p2.displacement)
            else:
                # Compute required displacement
                displacement = conflict

                # Apply displacement in direction of vector u
                p1.x -= displacement * u.x
                p1.y -= displacement * u.y

                # Update displacement cost
                p1.displacement += displacement
                cost = max(cost, p1.displacement)

        return cost
