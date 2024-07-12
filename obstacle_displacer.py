from enum import Enum
from itertools import pairwise

from constraint import ObstaclePairConstraint, ObstacleVertexConstraint
from utils import check_segment_segment_intersection, distance, on_segment


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
    SCIPY = 1  # The ScipyDisplacer
    OPTIMAL = 2  # The OptimalDisplacer
    DIAMOND = 3  # The DiamondDisplacer
    DELAUNAY = 4  # The DelaunayDisplacer


class ObstacleDisplacer:
    def __init__(self, instance, objective):
        """
        :param instance: a SimplifiedInstance object
        :param objective: an Objective object
        """
        self.instance = instance
        self.objective = objective
        self.constraints = []

    def compute_constraints_naive(self):
        """
        Computes the minimum separation constraints on all obstacle and obstacle-vertex pairs.
        """
        self.constraints = []
        for i in range(len(self.instance.obstacles)):
            # Create obstacle pair constraints
            for j in range(i + 1, len(self.instance.obstacles)):
                o1 = self.instance.obstacles[i]
                o2 = self.instance.obstacles[j]

                # Compute the total thickness of the edges passing in between o1 and o2
                # To do this, for each edge link pq, check if it intersects line segment o1o2
                # Assumes that o1, o2 and line segment pq are disjoint
                total_thickness = 0
                for edge in self.instance.graph.edges:
                    for (p, q) in pairwise(edge.path):
                        if check_segment_segment_intersection(o1, o2, p, q):
                            # Prevent double counting of intersections in a bend
                            if p == edge.v1 or not on_segment(o1, o2, p):
                                total_thickness += edge.thickness

                # Create constraint
                constraint = ObstaclePairConstraint(o1, o2, total_thickness)
                self.constraints.append(constraint)

            # Create obstacle-vertex constraints
            for v in self.instance.graph.vertices:
                o = self.instance.obstacles[i]

                # Compute the total thickness of the edges passing in between o and v
                # To do this, for each edge link pq, check if it intersects line segment ov and if so, not in a vertex
                # Assumes that o and line segment pq are disjoint
                total_thickness = 0
                for edge in self.instance.graph.edges:
                    for (p, q) in list(pairwise(edge.path)):
                        if check_segment_segment_intersection(o, v, p, q):
                            # Do not consider link incident on v and prevent double counting of intersections in a bend
                            if not (v == p or v == q) and not on_segment(o, v, p):
                                total_thickness += edge.thickness

                # Add extra space to draw vertex
                # min_separation = total_thickness + v.diameter / 2

                # Create constraint
                constraint = ObstacleVertexConstraint(o, v, total_thickness)
                self.constraints.append(constraint)

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
        # Compute minimum separation constraints
        # self.compute_constraints_naive()

        # Displace the obstacles
        self.displace_obstacles()

        # print("Constraints:")
        # for con in self.constraints:
        #     print(con)

        if self.is_valid_solution():
            # Return displacement cost
            return self.compute_cost()
        else:
            raise Exception("There are remaining conflicts!")
