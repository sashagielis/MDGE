import gurobipy as gp
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

    def displace_obstacles_scipy(self):
        """
        Computes new obstacle positions by minimizing maximum displacement subject to minimum separation constraints.
        Uses SciPy optimization, which is fast but computes a local optimum.

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

        # Assign computed positions to obstacles
        flat_new_obstacles = result['x']
        for i in range(0, len(flat_new_obstacles), 2):
            o = self.instance.obstacles[int(i / 2)].path[0]
            o.x = flat_new_obstacles[i]
            o.y = flat_new_obstacles[i + 1]

        return result['fun']

    def displace_obstacles_gurobi(self):
        """
        Computes new obstacle positions by minimizing maximum displacement subject to minimum separation constraints.
        Uses Gurobi optimization, which computes a global optimum but is slow (since it solves a non-convex MIQCP).

        :returns: the final value of the objective function
        """
        model = gp.Model("Displace obstacles")

        new_xs, new_ys = [], []
        displacements = []
        for i in range(len(self.instance.obstacles)):
            o = self.instance.obstacles[i].path[0]

            # Create variables for new x- and y-coordinates of obstacles
            new_xs.append(model.addVar(name=f"x(o'_{i})"))
            new_ys.append(model.addVar(name=f"y(o'_{i})"))

            # Add variable equal to summed squares of x and y differences
            dx = new_xs[i] - o.x
            dy = new_ys[i] - o.y
            summed_squares = model.addVar(name=f"(x(o'_{i}) - x(o_{i}))^2 + (y(o'_{i}) - y(o_{i}))^2")
            model.addConstr(summed_squares == dx ** 2 + dy ** 2)

            # Add variable equal to displacement, that is, Euclidean distance between current and new obstacle
            displacements.append(model.addVar(name=f"d(o_{i}, o'_{i})"))
            model.addGenConstrPow(summed_squares, displacements[i], 0.5)

        # Add variable equal to maximum over all displacements and use it as objective to be minimized
        objective = model.addVar(name="max displacement")
        model.addGenConstrMax(objective, displacements)
        model.setObjective(objective, gp.GRB.MINIMIZE)

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

                # Add minimum separation constraint on the pair of obstacles, i.e., d(o_1, o_2) >= min_sep
                dx = new_xs[i] - new_xs[j]
                dy = new_ys[i] - new_ys[j]
                model.addConstr(dx ** 2 + dy ** 2 >= total_thickness ** 2)

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

                # Add minimum separation constraint on the obstacle-vertex pair, i.e., d(o, v) >= min_sep
                dx = new_xs[i] - v.x
                dy = new_ys[i] - v.y
                model.addConstr(dx ** 2 + dy ** 2 >= min_separation ** 2)

        # Apply optimization to compute new obstacle positions
        model.optimize()

        # Assign computed positions to obstacles
        for i in range(len(self.instance.obstacles)):
            o = self.instance.obstacles[i].path[0]
            o.x = new_xs[i].X
            o.y = new_ys[i].X

        return model.ObjVal

    def solve(self):
        # TODO: compute shortest homotopic edges

        # Displace the obstacles
        displacement_cost = self.displace_obstacles_gurobi()

        # TODO: compute thick homotopic edges using growing algorithm
