from constraint import ObstaclePairConstraint
from obstacle_displacer import ObstacleDisplacer
from point import Point
from utils import distance, orientation


class SweepDisplacer(ObstacleDisplacer):
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        super().__init__(instance)

    def displace_obstacles(self):
        """
        Computes new obstacle positions by displacing obstacles using a 'sweeping' technique.

        :returns: the maximum obstacle displacement
        """
        while self.conflicts:
            conflict = self.conflicts.pop()
            p1 = conflict.p1
            p2 = conflict.p2

            # Compute unit vector u from p1 to p2 and its normal
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dist = distance(p1, p2)
            u = Point(dx / dist, dy / dist)
            normal_of_u = Point(-dy / dist, dx / dist)

            if type(conflict) == ObstaclePairConstraint:
                # Compute required displacement
                displacement = conflict.value / 2

                q1 = Point(p1.x + normal_of_u.x, p1.y + normal_of_u.y)
                q2 = Point(p2.x + normal_of_u.x, p2.y + normal_of_u.y)
                for obstacle in self.instance.obstacles:
                    # Apply displacement in direction of vector u
                    if orientation(p1, q1, obstacle) == 2 or orientation(p1, q1, obstacle) == 0:
                        obstacle.x -= displacement * u.x
                        obstacle.y -= displacement * u.y
                    elif orientation(p2, q2, obstacle) == 1 or orientation(p2, q2, obstacle) == 0:
                        obstacle.x += displacement * u.x
                        obstacle.y += displacement * u.y
            else:
                # Compute required displacement
                displacement = conflict.value

                q1 = Point(p1.x + normal_of_u.x, p1.y + normal_of_u.y)
                for obstacle in self.instance.obstacles:
                    # Apply displacement in direction of vector u
                    if orientation(p1, q1, obstacle) == 2 or orientation(p1, q1, obstacle) == 0:
                        obstacle.x -= displacement * u.x
                        obstacle.y -= displacement * u.y

            # Update conflicting minimum separation constraints
            # TODO: INEFFICIENT!!!
            self.compute_constraints_naive()
