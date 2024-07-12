import copy
from fractions import Fraction
import gurobipy as gp

from constraint import ObstaclePairConstraint, ObstacleVertexConstraint
from graph import Vertex
from obstacle_displacer import Objective, ObstacleDisplacer
from point import Point
from utils import vector_length

approx_factor = 2


class DelaunayDisplacer(ObstacleDisplacer):
    """
    A displacement method that computes a 2√2-approximation of the obstacle positions.
    """
    def __init__(self, instance, objective):
        """
        :param instance: a SimplifiedInstance object
        :param objective: an Objective object
        """
        super().__init__(instance, objective)

    def compute_constraints(self):
        """
        Computes the minimum-separation constraints on the Delaunay edges.
        By increasing the constraints by a factor 2, there is guaranteed to be enough space between all pairs of points.
        """
        # Remove existing constraints on the half-edges
        for he in self.instance.homotopy.dt.half_edges:
            he.constraint = None

        # Initialize the constraints
        half_edges = copy.copy(self.instance.homotopy.dt.half_edges)
        self.constraints = []
        while half_edges:
            he = half_edges.pop()

            # Do not add constraints for bounding box points
            if he.origin in self.instance.homotopy.bbox_points or he.target in self.instance.homotopy.bbox_points:
                continue

            # Do not add constraints on pairs of vertices
            if isinstance(he.origin, Vertex) and isinstance(he.target, Vertex):
                continue

            # Check if already added the constraint
            if he.constraint is not None:
                continue

            # Construct an obstacle-vertex or obstacle-pair constraint
            if isinstance(he.origin, Vertex):
                min_sep = approx_factor * he.origin.radius
                con = ObstacleVertexConstraint(he.target, he.origin, min_sep)
            elif isinstance(he.target, Vertex):
                min_sep = approx_factor * he.target.radius
                con = ObstacleVertexConstraint(he.origin, he.target, min_sep)
            else:
                min_sep = 0
                con = ObstaclePairConstraint(he.origin, he.target, min_sep)

            # Set the constraint on the half-edge and its twin
            he.constraint = con
            if he.twin is not None:
                he.twin.constraint = con

            self.constraints.append(con)

        # Compute the minimum separating thickness of the constraints
        for edge in self.instance.graph.edges:
            # Iterate over all crossings and update their constraints
            for he in edge.crossing_sequence.sequence:
                if he.constraint is not None:
                    he.constraint.min_separation += approx_factor * edge.thickness

    def displace_obstacles(self):
        """
        Displaces the obstacles by optimizing a linear program under DT-approximated minimum-separation constraints.
        Inspiration: https://doi.org/10.1111/cgf.13722.

        :returns: the final value of the approximated objective function
        """
        # Create environment to suppress all Gurobi output
        env = gp.Env(empty=True)
        env.setParam("OutputFlag", 0)
        env.start()

        model = gp.Model("Delaunay displacer", env=env)

        # C_δ contains for each edge of the unit circle of δ, the point closest to the origin
        C_delta = [Point(-1, 0), Point(0, 1), Point(1, 0), Point(0, -1)]

        new_xs, new_ys = [], []
        displacements = []
        for i in range(len(self.instance.obstacles)):
            o = self.instance.obstacles[i]

            # Create variables for new x- and y-coordinates of the obstacle
            new_xs.append(model.addVar(lb=-gp.GRB.INFINITY, name=f"x(o'_{i})"))
            new_ys.append(model.addVar(lb=-gp.GRB.INFINITY, name=f"y(o'_{i})"))

            # Add variable equal to displacement δ_i
            displacements.append(model.addVar(name=f"δ(o_{i}, o'_{i})"))

            dx = new_xs[i] - float(o.x)
            dy = new_ys[i] - float(o.y)

            # Add constraints such that δ_i >= c * (o'_i - o_i) / ||c||^2 for all c in C_δ
            # Since δ_i is equal to the max over these values and we minimize in the objective, δ_i is equal to the max
            for c in C_delta:
                x_normalized = c.x / (vector_length(c) ** 2)
                y_normalized = c.y / (vector_length(c) ** 2)

                model.addConstr(displacements[i] >= x_normalized * dx + y_normalized * dy)

        # Add variable equal to objective to be minimized
        if self.objective == Objective.MAX:
            obj = model.addVar(name="max displacement")
            model.addConstr(obj == gp.max_(displacements))
        elif self.objective == Objective.TOTAL:
            obj = model.addVar(name="total displacement")
            model.addConstr(obj == gp.quicksum(displacements))
        else:
            raise Exception(f"Objective {self.objective.name} not implemented for DelaunayDisplacer")

        # Set objective
        model.setObjective(obj, gp.GRB.MINIMIZE)

        # Add constraints to model
        for constraint in self.constraints:
            if type(constraint) == ObstaclePairConstraint:
                i = constraint.p1.id
                j = constraint.p2.id

                x1 = new_xs[i]
                y1 = new_ys[i]
                x2 = new_xs[j]
                y2 = new_ys[j]

                var_name = f"δ(o'_{i}, o'_{j})"
            else:
                i = constraint.p1.id
                v = constraint.p2

                x1 = new_xs[i]
                y1 = new_ys[i]
                x2 = v.x
                y2 = v.y

                var_name = f"δ(o'_{i}, v_{v.id})"

            # Compute dx and dy
            dx = x1 - x2
            dy = y1 - y2

            # Compute distance δ between the pair of points using C_δ
            c_distances = []
            for c in C_delta:
                x_normalized = c.x / (vector_length(c) ** 2)
                y_normalized = c.y / (vector_length(c) ** 2)

                dist = model.addVar(lb=-gp.GRB.INFINITY)
                c_distances.append(dist)

                model.addConstr(dist == x_normalized * dx + y_normalized * dy)

            separation = model.addVar(name=var_name)
            model.addGenConstrMax(separation, c_distances)

            # Add minimum-separation constraint on the pair of points
            model.addConstr(separation >= constraint.min_separation)

            # Add orthogonality constraint on the x-coordinates
            if constraint.p1.x == constraint.p2.x:
                model.addConstr(x1 == x2)
            elif constraint.p1.x < constraint.p2.x:
                model.addConstr(x1 <= x2 - 1)
            else:
                model.addConstr(x1 >= x2 + 1)

            # Add orthogonality constraint on the y-coordinates
            if constraint.p1.y == constraint.p2.y:
                model.addConstr(y1 == y2)
            elif constraint.p1.y < constraint.p2.y:
                model.addConstr(y1 <= y2 - 1)
            else:
                model.addConstr(y1 >= y2 + 1)

            # Add disjointness constraints on the pair of points
            x_diff = model.addVar(lb=-gp.GRB.INFINITY)
            y_diff = model.addVar(lb=-gp.GRB.INFINITY)
            abs_x_diff = model.addVar()
            abs_y_diff = model.addVar()
            model.addConstr(x_diff == x1 - x2)
            model.addConstr(y_diff == y1 - y2)
            model.addConstr(abs_x_diff == gp.abs_(x_diff))
            model.addConstr(abs_y_diff == gp.abs_(y_diff))
            model.addConstr(abs_x_diff + abs_y_diff >= 1)

        # Apply optimization to compute new obstacle positions
        model.optimize()

        # Assign computed positions to obstacles
        for i in range(len(self.instance.obstacles)):
            o = self.instance.obstacles[i]
            o.x = Fraction(new_xs[i].X)
            o.y = Fraction(new_ys[i].X)

            # Update the dimensions of the instance
            self.instance.update_dimensions(o)

        return model.ObjVal
