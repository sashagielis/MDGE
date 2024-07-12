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
    def __init__(self, instance, objective, step):
        """
        :param instance: a SimplifiedInstance object
        :param objective: an Objective object
        """
        super().__init__(instance, objective)
        self.step = step

    def compute_constraints(self):
        for he in self.instance.homotopy.dt.half_edges:
            he.constraint = None

        # Initialize the constraints
        half_edges = copy.copy(self.instance.homotopy.dt.half_edges)
        self.constraints = []
        while half_edges:
            he = half_edges.pop()

            if he.origin in self.instance.homotopy.bbox_points or he.target in self.instance.homotopy.bbox_points:
                continue

            if isinstance(he.origin, Vertex) and isinstance(he.target, Vertex):
                continue

            if he.constraint is not None:
                continue

            if isinstance(he.origin, Vertex):
                min_sep = approx_factor * he.origin.radius
                con = ObstacleVertexConstraint(he.target, he.origin, min_sep)
            elif isinstance(he.target, Vertex):
                min_sep = approx_factor * he.target.radius
                con = ObstacleVertexConstraint(he.origin, he.target, min_sep)
            else:
                min_sep = 0
                con = ObstaclePairConstraint(he.origin, he.target, min_sep)

            he.constraint = con
            if he.twin is not None:
                he.twin.constraint = con

            self.constraints.append(con)

        # Compute the constraints
        for edge in self.instance.graph.edges:
            for he in edge.crossing_sequence:
                if he.constraint is not None:
                    he.constraint.min_separation += approx_factor * edge.thickness

    def update_crossing_sequences(self, he):
        """
        Updates the crossing sequences of the edges after flipping half-edge he.

        :param he: a HalfEdge object
        """
        def check_quadrilateral():
            """
            Checks whether the edge enters a quadrilateral for which he is the separating edge via the current crossing.
            Updates the crossing sequence of the edge accordingly.
            """
            # Check if he or its twin is the separating half-edge of the quadrilateral entered via the current crossing
            separating_half_edge = None
            if crossing == he.next.twin or crossing == he.prev.twin:
                separating_half_edge = he
            elif crossing == he.twin.next.twin or crossing == he.twin.prev.twin:
                separating_half_edge = he.twin

            if separating_half_edge is not None:
                # Get the next two crossings of the edge
                crossing2 = edge.crossing_sequence[i + 1]
                crossing3 = edge.crossing_sequence[i + 2] if i < len(edge.crossing_sequence) - 2 else None

                # If the edge crossed the separating half-edge, check if it still does after the flip
                if crossing2 == he or crossing2 == he.twin:
                    # If crossing2 is the last crossing, remove it as it has become connected to v2 after the flip
                    # If the third crossing is in the same triangle as the separating half-edge, also remove it
                    if crossing3 is None or crossing3.triangle == separating_half_edge.triangle:
                        del edge.crossing_sequence[i + 1]

                # Otherwise, the edge crosses the separating half-edge after the flip
                else:
                    edge.crossing_sequence.insert(i + 1, separating_half_edge)

        for edge in self.instance.graph.edges:
            # Iterate over the crossings of the crossing sequence and update them due to flipping he
            i = 0
            while i < len(edge.crossing_sequence):
                crossing = edge.crossing_sequence[i]

                # Handle the case where it is the first crossing after leaving vertex v1
                if i == 0:
                    # If he is the first crossing, remove it as he has become connected to v1 after the flip
                    if crossing == he or crossing == he.twin:
                        del edge.crossing_sequence[i]

                    # If he was one of v1's incident half-edges before the flip, its twin has become the first crossing
                    elif crossing == he.next or crossing == he.prev:
                        edge.crossing_sequence.insert(i, he.twin)
                        i += 1
                    elif crossing == he.twin.next or crossing == he.twin.prev:
                        edge.crossing_sequence.insert(i, he)
                        i += 1

                    # Otherwise, check the quadrilateral that the edge enters via the crossing
                    else:
                        check_quadrilateral()
                        i += 1

                # Handle the case where it is the last crossing before arriving at vertex v2
                elif i == len(edge.crossing_sequence) - 1:
                    # If he is the last crossing, remove it as he has become connected to v2 after the flip
                    if crossing == he or crossing == he.twin:
                        del edge.crossing_sequence[i]

                    # If he was one of v2's incident half-edges before the flip, it has become the last crossing
                    elif crossing == he.next.twin or crossing == he.prev.twin:
                        edge.crossing_sequence.insert(i + 1, he)
                        i += 2
                    elif crossing == he.twin.next.twin or crossing == he.twin.prev.twin:
                        edge.crossing_sequence.insert(i + 1, he.twin)
                        i += 2
                    else:
                        i += 1

                # Handle the general case where it is a crossing via which the edge enters a quadrilateral
                else:
                    check_quadrilateral()
                    i += 1

    def compute_new_positions(self):
        # Create environment to suppress all Gurobi output
        env = gp.Env(empty=True)
        env.setParam("OutputFlag", 0)
        env.start()

        model = gp.Model(f"Delaunay displacer", env=env)

        # C_δ contains for each edge of the unit circle of δ, the point closest to the origin
        C_delta = [Point(-1, 0), Point(0, 1), Point(1, 0), Point(0, -1)]

        new_xs, new_ys = [], []
        displacements = []
        for i in range(len(self.instance.obstacles)):
            o = self.instance.obstacles[i]

            # Create variables for new x- and y-coordinates of obstacles
            new_xs.append(model.addVar(lb=-gp.GRB.INFINITY, name=f"x(o'_{i})"))
            new_ys.append(model.addVar(lb=-gp.GRB.INFINITY, name=f"y(o'_{i})"))

            # Add variable equal to displacement δ_i
            displacements.append(model.addVar(name=f"δ(o_{i}, o'_{i})"))

            dx = new_xs[i] - o.x
            dy = new_ys[i] - o.y

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
            raise Exception(f"Objective {self.objective.name} not implemented for {type(self).__name__}")

        # Set objective
        model.setObjective(obj, gp.GRB.MINIMIZE)

        # Add constraints to model
        for constraint in self.constraints:
            if type(constraint) == ObstaclePairConstraint:
                i = constraint.p1.id
                j = constraint.p2.id

                # Add minimum separation constraint on the pair of obstacles, i.e., δ(o'_i, o'_j) >= min_sep
                dx = new_xs[i] - new_xs[j]
                dy = new_ys[i] - new_ys[j]

                var_name = f"δ(o'_{i}, o'_{j})"

                # Add orthogonality constraint on the x-coordinates, i.e., x(o'_i) <= x(o'_j) if x(o_i) <= x(o_j)
                if constraint.p1.x == constraint.p2.x:
                    model.addConstr(new_xs[i] == new_xs[j])
                elif constraint.p1.x < constraint.p2.x:
                    model.addConstr(new_xs[i] <= new_xs[j] - 1)
                else:
                    model.addConstr(new_xs[i] >= new_xs[j] + 1)

                # Add orthogonality constraint on the y-coordinates, i.e., y(o'_i) <= y(o'_j) if y(o_i) <= y(o_j)
                if constraint.p1.y == constraint.p2.y:
                    model.addConstr(new_ys[i] == new_ys[j])
                elif constraint.p1.y < constraint.p2.y:
                    model.addConstr(new_ys[i] <= new_ys[j] - 1)
                else:
                    model.addConstr(new_ys[i] >= new_ys[j] + 1)

                x_diff = model.addVar(lb=-gp.GRB.INFINITY)
                y_diff = model.addVar(lb=-gp.GRB.INFINITY)
                abs_x_diff = model.addVar()
                abs_y_diff = model.addVar()
                model.addConstr(x_diff == new_xs[i] - new_xs[j])
                model.addConstr(y_diff == new_ys[i] - new_ys[j])
                model.addConstr(abs_x_diff == gp.abs_(x_diff))
                model.addConstr(abs_y_diff == gp.abs_(y_diff))
                model.addConstr(abs_x_diff + abs_y_diff >= 1)
            else:
                i = constraint.p1.id
                v = constraint.p2

                # Add minimum separation constraint on the obstacle-vertex pair, i.e., δ(o'_i, v) >= min_sep
                dx = new_xs[i] - v.x
                dy = new_ys[i] - v.y

                var_name = f"δ(o'_{i}, v_{v.id})"

                # Add orthogonality constraint on the x-coordinates, i.e., x(o'_i) <= x(v) if x(o_i) <= x(v)
                if constraint.p1.x == v.x:
                    model.addConstr(new_xs[i] == v.x)
                elif constraint.p1.x < v.x:
                    model.addConstr(new_xs[i] <= v.x - 1)
                else:
                    model.addConstr(new_xs[i] >= v.x + 1)

                # Add orthogonality constraint on the y-coordinates, i.e., y(o'_i) <= y(v) if y(o_i) <= y(v)
                if constraint.p1.y == v.y:
                    model.addConstr(new_ys[i] == v.y)
                elif constraint.p1.y < v.y:
                    model.addConstr(new_ys[i] <= v.y - 1)
                else:
                    model.addConstr(new_ys[i] >= v.y + 1)

                x_diff = model.addVar(lb=-gp.GRB.INFINITY)
                y_diff = model.addVar(lb=-gp.GRB.INFINITY)
                abs_x_diff = model.addVar()
                abs_y_diff = model.addVar()
                model.addConstr(x_diff == new_xs[i] - v.x)
                model.addConstr(y_diff == new_ys[i] - v.y)
                model.addConstr(abs_x_diff == gp.abs_(x_diff))
                model.addConstr(abs_y_diff == gp.abs_(y_diff))
                model.addConstr(abs_x_diff + abs_y_diff >= 1)

            # Compute distance δ between pair using C_δ
            c_distances = []
            for c in C_delta:
                x_normalized = c.x / (vector_length(c) ** 2)
                y_normalized = c.y / (vector_length(c) ** 2)

                dist = model.addVar(lb=-gp.GRB.INFINITY)
                c_distances.append(dist)

                model.addConstr(dist == x_normalized * dx + y_normalized * dy)

            separation = model.addVar(name=var_name)
            model.addGenConstrMax(separation, c_distances)

            # Add minimum separation constraint to the model
            model.addConstr(separation >= constraint.min_separation)

        # Apply optimization to compute new obstacle positions
        model.optimize()

        # Assign computed positions to obstacles
        for i in range(len(self.instance.obstacles)):
            o = self.instance.obstacles[i]
            o.x = Fraction(new_xs[i].X)
            o.y = Fraction(new_ys[i].X)

            self.instance.update_dimensions(o)

    def displace_obstacles(self):
        # Compute the constraints on the Delaunay edges
        self.compute_constraints()

        while not self.is_valid_solution():
            # Compute the new positions of the obstacles using the linear program with DT-approximated constraints
            self.compute_new_positions()

            # Update the bounding box points of the DT, which may no longer be valid after displacing the obstacles
            self.instance.homotopy.update_bbox_points()

            # While there are non-Delaunay triangles in the DT, flip the problematic half-edges
            while not self.instance.homotopy.dt.is_valid():
                for t in self.instance.homotopy.dt.triangles:
                    delaunay, he = t.is_delaunay()
                    if not delaunay:
                        print(he)
                        he.flip()

                        # Update the crossing sequences
                        self.update_crossing_sequences(he)

            # Recompute the constraints on the new Delaunay edges
            self.compute_constraints()
