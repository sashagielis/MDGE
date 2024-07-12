from enum import Enum

from utils import distance


class Objective(Enum):
    """
    The objective function to be minimized.
    """
    MAX = 1  # The maximum displacement
    TOTAL = 2  # The total displacement


class Displacer(Enum):
    """
    The displacer used for displacing the obstacles.
    """
    DELAUNAY = 1  # The DelaunayDisplacer


class ObstacleDisplacer:
    """
    A displacement method to solve a set of minimum-separation constraints on the obstacles.
    """
    def __init__(self, instance, objective):
        """
        :param instance: a SimplifiedInstance object
        :param objective: an Objective object
        """
        self.instance = instance
        self.objective = objective
        self.constraints = []

    def compute_constraints(self):
        """
        Computes the minimum separation constraints that the solution should satisfy.
        Displacers should implement this method.
        """
        raise Exception(f"Method compute_constraints not implemented for {type(self).__name__}")

    def displace_obstacles(self):
        """
        Computes new obstacle positions by minimizing objective subject to minimum separation constraints.
        Displacers should implement this method.
        """
        raise Exception(f"Method displace_obstacles not implemented for {type(self).__name__}")

    def is_valid_solution(self):
        """
        Determines whether the current configuration of obstacles satisfies all constraints.
        """
        for constraint in self.constraints:
            if constraint.value > 0:
                return False

        return True

    def compute_cost(self):
        """
        Computes the cost of the objective to be minimized.
        """
        if self.objective == Objective.MAX:
            return max(distance(obstacle, obstacle.original_position) for obstacle in self.instance.obstacles)
        elif self.objective == Objective.TOTAL:
            return sum(distance(obstacle, obstacle.original_position) for obstacle in self.instance.obstacles)
        else:
            return None

    def execute(self):
        """
        Executes the displacement method.
        """
        # Compute minimum-separation constraints
        self.compute_constraints()

        while not self.is_valid_solution():
            # Displace the obstacles
            self.displace_obstacles()

            # Update the bounding box points of the DT, which may no longer be valid after displacing the obstacles
            self.instance.homotopy.update_bbox_points()

            # While there are non-Delaunay triangles in the DT, flip the problematic half-edges
            while not self.instance.homotopy.dt.is_valid():
                for t in self.instance.homotopy.dt.triangles:
                    delaunay, he = t.is_delaunay()
                    if not delaunay:
                        he.flip()

                        # Update the crossing sequences
                        for edge in self.instance.graph.edges:
                            edge.crossing_sequence.update(he)

            # Recompute constraints
            self.compute_constraints()

        # Return displacement cost
        return self.compute_cost()
