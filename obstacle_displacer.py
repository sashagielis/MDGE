from itertools import pairwise

from constraint import ObstaclePairConstraint, ObstacleVertexConstraint
from utils import do_intersect


class ObstacleDisplacer:
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        self.instance = instance
        self.constraints = []

    def compute_constraints(self):
        """
        Computes the minimum separation constraints on obstacle and obstacle-vertex pairs.
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
                        if do_intersect(o1, o2, p, q):
                            total_thickness += edge.thickness

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
                        if do_intersect(o, v, p, q) and not (v == p or v == q):
                            total_thickness += edge.thickness

                # Add extra space to draw vertex
                min_separation = total_thickness + v.diameter / 2

                constraint = ObstacleVertexConstraint(o, v, min_separation)
                self.constraints.append(constraint)
