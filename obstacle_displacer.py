from itertools import pairwise

from constraint import ObstaclePairConstraint, ObstacleVertexConstraint
from utils import do_intersect, distance, on_segment


class ObstacleDisplacer:
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        self.instance = instance
        self.constraints = []
        self.conflicts = []

    def compute_constraints_naive(self):
        """
        Computes the minimum separation constraints on all obstacle and obstacle-vertex pairs.
        """
        self.constraints = []
        self.conflicts = []
        for i in range(len(self.instance.obstacles)):
            # Create obstacle pair constraints
            for j in range(i + 1, len(self.instance.obstacles)):
                o1 = self.instance.obstacles[i]
                o2 = self.instance.obstacles[j]

                # Compute the total thickness of the edges passing in between o1 and o2
                # To do this, for each edge link pq, check if it intersects line segment o1o2
                # Assumes that o1, o2 and line segment pq are disjoint
                total_thickness = 0
                for edge in self.instance.graph.half_edges:
                    for (p, q) in pairwise(edge.path):
                        if do_intersect(o1, o2, p, q) and not on_segment(o1, o2, p):
                            total_thickness += edge.thickness

                # Create constraint
                constraint = ObstaclePairConstraint(o1, o2, total_thickness)
                self.constraints.append(constraint)

                # If the constraint does not hold, mark it as conflict
                if constraint.value > 0:
                    self.conflicts.append(constraint)

            # Create obstacle-vertex constraints
            for v in self.instance.graph.vertices:
                o = self.instance.obstacles[i]

                # Compute the total thickness of the edges passing in between o and v
                # To do this, for each edge link pq, check if it intersects line segment ov and if so, not in a vertex
                # Assumes that o and line segment pq are disjoint
                total_thickness = 0
                for edge in self.instance.graph.half_edges:
                    for (p, q) in list(pairwise(edge.path)):
                        if do_intersect(o, v, p, q) and not (v == p or v == q) and not on_segment(o, v, p):
                            total_thickness += edge.thickness

                # Add extra space to draw vertex
                min_separation = total_thickness + v.diameter / 2

                # Create constraint
                constraint = ObstacleVertexConstraint(o, v, min_separation)
                self.constraints.append(constraint)

                # If the constraint does not hold, mark it as conflict
                if constraint.value > 0:
                    self.conflicts.append(constraint)

    def displace_obstacles(self):
        """
        Displacers should implement this method to displace the obstacles.
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
        Computes the maximum displacement over all obstacles.
        """
        return max(distance(obstacle, obstacle.original_position) for obstacle in self.instance.obstacles)

    def execute(self):
        """
        Executes the displacement method.
        """
        # Compute minimum separation constraints
        self.compute_constraints_naive()

        # Displace the obstacles
        self.displace_obstacles()

        if self.is_valid_solution():
            # Return displacement cost
            return self.compute_cost()
        else:
            raise Exception("There are remaining conflicts!")
