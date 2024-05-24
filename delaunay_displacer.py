import gurobipy as gp
import math

from constraint import ObstaclePairConstraint
from delaunay_triangulation import DelaunayTriangulation
from obstacle_displacer import ObstacleDisplacer, Objective


class DelaunayDisplacer(ObstacleDisplacer):
    def __init__(self, instance, objective):
        """
        :param instance: a SimplifiedInstance object
        :param objective: an Objective object
        """
        super().__init__(instance, objective)

    def displace_obstacles(self):
        """
        Computes √2-approximation for obstacle positions using Gurobi optimization.
        Only puts constraints on pairs that have an edge in the Delaunay triangulation.
        By setting the constraints √2 too large, these constraints are sufficient.

        :returns: the final value of the objective function
        """
        # Create environment to suppress all Gurobi output
        env = gp.Env(empty=True)
        env.setParam("OutputFlag", 0)
        env.start()

        model = gp.Model("Optimal displacer", env=env)

        new_xs, new_ys = [], []
        displacements = []
        for i in range(len(self.instance.obstacles)):
            o = self.instance.obstacles[i]

            # Create variables for new x- and y-coordinates of obstacles
            new_xs.append(model.addVar(name=f"x(o'_{i})"))
            new_ys.append(model.addVar(name=f"y(o'_{i})"))

            # Add variable equal to summed squares of x- and y-differences
            dx = new_xs[i] - o.x
            dy = new_ys[i] - o.y
            summed_squares = model.addVar(name=f"(x(o'_{i}) - x(o_{i}))^2 + (y(o'_{i}) - y(o_{i}))^2")
            model.addConstr(summed_squares == dx ** 2 + dy ** 2)

            # Add variable equal to displacement, that is, Euclidean distance between current and new obstacle
            displacements.append(model.addVar(name=f"d(o_{i}, o'_{i})"))
            model.addGenConstrPow(summed_squares, displacements[i], 0.5)

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

        # Compute Delaunay triangulation on the vertices and obstacles
        points = self.instance.graph.vertices + self.instance.obstacles
        dt = DelaunayTriangulation(points)

        # Add constraints to model
        for constraint in self.constraints:
            dt_p1 = dt.get_delaunay_vertex_from_point(constraint.p1)
            dt_p2 = dt.get_delaunay_vertex_from_point(constraint.p2)

            # If the Delaunay triangulation does not contain the edge (p1, p2), do not add constraint
            if not dt_p1.has_edge(dt_p2) and not dt_p2.has_edge(dt_p1):
                continue

            if type(constraint) == ObstaclePairConstraint:
                i = constraint.p1.id
                j = constraint.p2.id

                # Add minimum separation constraint on the pair of obstacles, i.e., d(o_i, o_j) >= min_sep
                dx = new_xs[i] - new_xs[j]
                dy = new_ys[i] - new_ys[j]
            else:
                i = constraint.p1.id
                v = constraint.p2

                # Add minimum separation constraint on the obstacle-vertex pair, i.e., d(o_i, v) >= min_sep
                dx = new_xs[i] - v.x
                dy = new_ys[i] - v.y

            # Add minimum separation constraint to the model that is √2 too large
            model.addConstr(dx ** 2 + dy ** 2 >= math.sqrt(2) * constraint.min_separation ** 2)

        # Apply optimization to compute new obstacle positions
        model.optimize()

        # Assign computed positions to obstacles
        for i in range(len(self.instance.obstacles)):
            o = self.instance.obstacles[i]
            o.x = new_xs[i].X
            o.y = new_ys[i].X

        return model.ObjVal
