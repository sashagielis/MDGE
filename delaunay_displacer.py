import copy
import math
from fractions import Fraction
import gurobipy as gp

from constraint import ObstaclePairConstraint, ObstacleVertexConstraint
from graph import Vertex
from obstacle_displacer import Objective, ObstacleDisplacer
from point import Point
from utils import vector_length

approx_factor = 1.998  # The approximation factor of the Delaunay constraints
min_coordinate_diff = 1  # The minimum required x- and y-difference between pairs of points sharing a Delaunay edge


class DelaunayDisplacer(ObstacleDisplacer):
    """
    A displacement method that computes a 1.998√2-approximation of the new optimal obstacle positions.
    """
    def __init__(self, instance, objective):
        """
        :param instance: a SimplifiedInstance object
        :param objective: an Objective object
        """
        super().__init__(instance, objective)

        self.model = None
        self.new_xs, self.new_ys = [], []
        self.initialize_model()

    def initialize_model(self):
        """
        Initializes the Gurobi model with constraints to compute displacements and objective function.
        """
        # Create environment to suppress all Gurobi output
        env = gp.Env(empty=True)
        env.setParam("OutputFlag", 0)
        env.start()

        self.model = gp.Model("Delaunay displacer", env=env)

        # C_δ contains for each side of the unit circle of δ, the point closest to the origin
        # We use δ = L_∞ (maximum metric)
        C_delta = [Point(-1, 0), Point(0, 1), Point(1, 0), Point(0, -1)]

        # Add constraints to compute values of displacements
        self.new_xs, self.new_ys = [], []
        displacements = []
        for i in range(len(self.instance.obstacles)):
            o = self.instance.obstacles[i]

            # Create variables for new x- and y-coordinates of the obstacle
            self.new_xs.append(self.model.addVar(lb=-gp.GRB.INFINITY))
            self.new_ys.append(self.model.addVar(lb=-gp.GRB.INFINITY))

            # Add variable equal to displacement δ_i
            displacements.append(self.model.addVar())

            dx = self.new_xs[i] - float(o.x)
            dy = self.new_ys[i] - float(o.y)

            # Add constraints such that δ_i >= c * (o'_i - o_i) / ||c||^2 for all c in C_δ
            # Since δ_i is equal to the max over these values and we minimize in the objective, δ_i is equal to the max
            for c in C_delta:
                x_normalized = c.x / (vector_length(c) ** 2)
                y_normalized = c.y / (vector_length(c) ** 2)

                self.model.addConstr(displacements[i] >= x_normalized * dx + y_normalized * dy)

        # Add variable equal to objective to be minimized
        if self.objective == Objective.TOTAL:
            obj = self.model.addVar()
            self.model.addConstr(obj == gp.quicksum(displacements))
        else:
            raise Exception(f"Objective {self.objective.name} not implemented for DelaunayDisplacer")

        # Set objective
        self.model.setObjective(obj, gp.GRB.MINIMIZE)

    def compute_constraints(self, keep_prev_constraints=True):
        """
        Computes the minimum-separation constraints on the Delaunay edges.
        We increase the constraints by a factor 1.998 to ensure there is enough space between all pairs of points.
        """
        half_edges = copy.copy(self.instance.homotopy.dt.half_edges)

        if not keep_prev_constraints:
            # Reset the constraints
            self.constraints = []
            self.initialize_model()

            for he in half_edges:
                he.constraint = None

        # Add a minimum-separation constraint on each Delaunay edge
        while half_edges:
            he = half_edges.pop()

            # Check if we already added the constraint
            if he.constraint is not None:
                continue

            # Do not add constraints for bounding box points
            if he.origin in self.instance.homotopy.bbox_points or he.target in self.instance.homotopy.bbox_points:
                continue

            # Do not add constraints on pairs of vertices
            if isinstance(he.origin, Vertex) and isinstance(he.target, Vertex):
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

            for edge in self.instance.graph.edges:
                # Iterate over all crossings and update the constraint on he
                for crossing in edge.crossing_sequence.sequence:
                    if crossing is he or crossing is he.twin:
                        con.min_separation += approx_factor * edge.thickness

            # Add minimum-separation constraint
            self.constraints.append(con)

            # Add corresponding constraints to model
            self.add_constraints_to_model(con)

    def add_constraints_to_model(self, constraint):
        """
        Adds the given minimum-separation constraint and corresponding orthogonality constraints to the Gurobi model.

        :param constraint: a Constraint object
        """
        # Retrieve variables for new obstacle position(s)
        if type(constraint) == ObstaclePairConstraint:
            i = constraint.p1.id
            j = constraint.p2.id

            x1 = self.new_xs[i]
            y1 = self.new_ys[i]
            x2 = self.new_xs[j]
            y2 = self.new_ys[j]
        else:
            i = constraint.p1.id
            v = constraint.p2

            x1 = self.new_xs[i]
            y1 = self.new_ys[i]
            x2 = v.x
            y2 = v.y

        # Add orthogonality constraint on the x-coordinates
        if constraint.p1.x == constraint.p2.x:
            self.model.addConstr(x1 == x2)
            dx = 0
        elif constraint.p1.x < constraint.p2.x:
            self.model.addConstr(x1 <= x2 - min_coordinate_diff)
            dx = x2 - x1
        else:
            self.model.addConstr(x1 >= x2 + min_coordinate_diff)
            dx = x1 - x2

        # Add orthogonality constraint on the y-coordinates
        if constraint.p1.y == constraint.p2.y:
            self.model.addConstr(y1 == y2)
            dy = 0
        elif constraint.p1.y < constraint.p2.y:
            self.model.addConstr(y1 <= y2 - min_coordinate_diff)
            dy = y2 - y1
        else:
            self.model.addConstr(y1 >= y2 + min_coordinate_diff)
            dy = y1 - y2

        # Add minimum-separation constraint using the Manhattan distance
        # The Manhattan distance overestimates the Euclidean distance by a factor √2
        # Therefore, we need to scale the min separation by an extra factor √2, on top of the 1.998 approx factor
        self.model.addConstr(dx + dy >= math.sqrt(2) * constraint.min_separation)

    def displace_obstacles(self):
        """
        Displaces the obstacles by optimizing a linear program under DT-approximated minimum-separation constraints.
        Follows algorithm by Meulemans: https://doi.org/10.1111/cgf.13722.

        :returns: the final value of the approximated objective function, or None if no solution was found
        """
        # Apply optimization to compute new obstacle positions
        self.model.optimize()

        if self.model.Status == gp.GRB.OPTIMAL:
            # The model was solved optimally
            # Assign computed positions to obstacles
            for i in range(len(self.instance.obstacles)):
                o = self.instance.obstacles[i]
                o.x = Fraction(self.new_xs[i].X)
                o.y = Fraction(self.new_ys[i].X)

            return self.model.ObjVal
        else:
            print(f"Model could not be solved! Gurobi terminated with status code {self.model.Status} "
                  f"(see https://www.gurobi.com/documentation/current/refman/optimization_status_codes.html#sec"
                  f":StatusCodes)")

            return None
