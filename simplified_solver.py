import math

from functools import partial
from itertools import pairwise
from scipy.optimize import minimize

from solver import Solver
from utils import do_intersect


class SimplifiedSolver(Solver):
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        super().__init__(instance)

    def displace_obstacles(self):
        """
        Computes new obstacle positions by minimizing an objective function subject to constraints.

        :returns: the final value of the objective function
        """
        def objective(new_obstacles, old_obstacles):
            """
            The objective function to be minimized, i.e., the maximum obstacle displacement.

            :param new_obstacles: flat list of new obstacle coordinates [x'1, y'1, x'2, y'2, ..., x'n, y'n]
            :param old_obstacles: flat list of old obstacle coordinates [x1, y1, x2, y2, ..., xn, yn]
            :returns: the value of the objective function
            """
            dxs = [new_obstacles[i] - old_obstacles[i] for i in range(0, len(new_obstacles), 2)]
            dys = [new_obstacles[i] - old_obstacles[i] for i in range(1, len(new_obstacles), 2)]

            return max(math.sqrt(dx ** 2 + dy ** 2) for dx, dy in zip(dxs, dys))

        def obstacle_pair_constraint(new_obstacles, index1, index2, min_sep):
            """
            A minimum separation constraint on a pair of obstacles (o1, o2), i.e., d(o1, o2) >= min_sep.

            :param new_obstacles: flat list of new obstacle coordinates [x'1, y'1, x'2, y'2, ..., x'n, y'n]
            :param index1: the index of obstacle o1 in list self.instance.obstacles
            :param index2: the index of obstacle o2 in list self.instance.obstacles
            :param min_sep: the minimum required separation between o1 and o2
            :returns: d(o1, o2) - min_sep
            """
            # Compute the indices of the obstacles in list new_obstacles
            new_index1 = 2 * index1
            new_index2 = 2 * index2

            dx = new_obstacles[new_index1] - new_obstacles[new_index2]
            dy = new_obstacles[new_index1 + 1] - new_obstacles[new_index2 + 1]

            return math.sqrt(dx ** 2 + dy ** 2) - min_sep

        def obstacle_vertex_constraint(new_obstacles, index, vertex, min_sep):
            """
            A minimum separation constraint on an obstacle-vertex pair (o, v), i.e., d(o, v) >= min_sep.

            :param new_obstacles: flat list of new obstacle coordinates [x'1, y'1, x'2, y'2, ..., x'n, y'n]
            :param index: the index of obstacle o in list self.instance.obstacles
            :param vertex: the vertex v
            :param min_sep: the minimum required separation between o and v
            :returns: d(o, v) - min_sep
            """
            # Compute the index of the obstacle in list new_obstacles
            new_index = 2 * index

            dx = new_obstacles[new_index] - vertex.x
            dy = new_obstacles[new_index + 1] - vertex.y

            return math.sqrt(dx ** 2 + dy ** 2) - min_sep

        constraints = []

        # Create a constraint for each pair of obstacles
        for i in range(len(self.instance.obstacles)):
            # Create obstacle pair constraints
            for j in range(i + 1, len(self.instance.obstacles)):
                o1 = self.instance.obstacles[i].path[0]
                o2 = self.instance.obstacles[j].path[0]

                # Compute the total thickness of the edges passing in between o1 and o2
                # To do this, for each edge link pq, check if it intersects line segment o1o2
                # Assumes that o1, o2 and line segment pq are disjoint
                total_thickness = 0
                for edge in self.instance.graph.edges:
                    for (p, q) in pairwise(edge.path):
                        if do_intersect(o1, o2, p, q):
                            total_thickness += edge.thickness

                # Constraints depend on obstacles and total thickness, so we need to evaluate constraints using partial
                # Inspiration: https://stackoverflow.com/questions/27659235/adding-multiple-constraints-to
                # -scipy-minimize-autogenerate-constraint-dictionary-list
                constraint = partial(obstacle_pair_constraint, index1=i, index2=j, min_sep=total_thickness)
                constraints.append({'type': 'ineq', 'fun': constraint})

            # Create obstacle-vertex constraints
            for v in self.instance.graph.vertices:
                o = self.instance.obstacles[i].path[0]

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

                # Constraints depend on pair and total thickness, so we need to evaluate constraints using partial
                # Inspiration: https://stackoverflow.com/questions/27659235/adding-multiple-constraints-to
                # -scipy-minimize-autogenerate-constraint-dictionary-list
                constraint = partial(obstacle_vertex_constraint, index=i, vertex=v, min_sep=min_separation)
                constraints.append({'type': 'ineq', 'fun': constraint})

        # Flat list of initial obstacle coordinates
        initial_obstacles = [coordinate for o in self.instance.obstacles for coordinate in [o.path[0].x, o.path[0].y]]

        # Apply optimization to compute new obstacle positions
        result = minimize(objective, initial_obstacles, args=initial_obstacles, method='SLSQP', constraints=constraints)

        # Assign the new positions to the obstacles
        flat_new_obstacles = result['x']
        for i in range(0, len(flat_new_obstacles), 2):
            o = self.instance.obstacles[int(i / 2)].path[0]
            o.x = flat_new_obstacles[i]
            o.y = flat_new_obstacles[i + 1]

        return result['fun']

    def solve(self):
        # TODO: compute shortest homotopic edges

        # Displace the obstacles
        displacement_cost = self.displace_obstacles()
        print(f"Displacement cost = {displacement_cost}")

        # TODO: compute thick homotopic edges using growing algorithm
