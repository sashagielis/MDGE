import gurobipy as gp

from constraint import ObstaclePairConstraint
from obstacle_displacer import ObstacleDisplacer, Objective
from point import Point
from utils import vector_length


class DiamondDisplacer(ObstacleDisplacer):
    def __init__(self, instance, objective):
        """
        :param instance: a SimplifiedInstance object
        :param objective: an Objective object
        """
        super().__init__(instance, objective)

    def displace_obstacles(self):
        """
        Computes √2-approximation for obstacle positions using Gurobi optimization.
        Inspiration: https://doi.org/10.1111/cgf.13722.

        :returns: the final value of the approximated objective function
        """
        # Create environment to suppress all Gurobi output
        env = gp.Env(empty=True)
        env.setParam("OutputFlag", 0)
        env.start()

        model = gp.Model(f"Diamond displacer", env=env)

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

                # Add minimum separation constraint on the pair of obstacles, i.e., δ(o_i, o_j) >= min_sep
                dx = new_xs[i] - new_xs[j]
                dy = new_ys[i] - new_ys[j]

                var_name = f"δ(o'_{i}, o'_{j})"
            else:
                i = constraint.p1.id
                v = constraint.p2

                # Add minimum separation constraint on the obstacle-vertex pair, i.e., δ(o_i, v) >= min_sep
                dx = new_xs[i] - v.x
                dy = new_ys[i] - v.y

                var_name = f"δ(o'_{i}, v_{v.id})"

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
            o.x = new_xs[i].X
            o.y = new_ys[i].X

        return model.ObjVal
