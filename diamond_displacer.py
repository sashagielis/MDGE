import math

import gurobipy as gp

from constraint import ObstaclePairConstraint
from obstacle_displacer import ObstacleDisplacer
from point import Point
from utils import distance


class DiamondDisplacer(ObstacleDisplacer):
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        super().__init__(instance)

    def displace_obstacles(self):
        """
        Computes new obstacle positions by minimizing maximum displacement subject to minimum separation constraints.

        :returns: the final value of the objective function
        """
        model = gp.Model("Displace obstacles")

        d = 0.5 * math.sqrt(2)
        C = [Point(-d, d), Point(d, d), Point(d, -d), Point(-d, -d)]

        new_xs, new_ys = [], []
        displacements = []
        for i in range(len(self.instance.obstacles)):
            o = self.instance.obstacles[i]

            # Create variables for new x- and y-coordinates of obstacles
            new_xs.append(model.addVar(name=f"x(o'_{i})"))
            new_ys.append(model.addVar(name=f"y(o'_{i})"))

            # Add variable equal to displacement, that is, Euclidean distance between current and new obstacle
            displacements.append(model.addVar(name=f"d(o_{i}, o'_{i})"))

            # for c in C:
            #     model.addConstr(displacements[i] >= (c.x * (new_xs[i] - o.x) + c.y * (new_ys[i] - o.y)) / (d ** 2))
            dx = new_xs[i] - o.x
            dy = new_ys[i] - o.y

            a = model.addVars(4, lb=-gp.GRB.INFINITY, name=[f"a_{i}[0]", f"a_{i}[1]", f"a_{i}[2]", f"a_{i}[3]"])
            model.addConstr(a[0] == dx)
            model.addConstr(a[1] == dy)
            model.addConstr(a[2] == gp.abs_(a[0]))
            model.addConstr(a[3] == gp.abs_(a[1]))
            model.addConstr(displacements[i] == a[2] + a[3])

        # Add variable equal to maximum over all displacements and use it as objective to be minimized
        objective = model.addVar(name="max displacement")
        model.addGenConstrMax(objective, displacements)
        model.setObjective(objective, gp.GRB.MINIMIZE)

        # Add constraints to model
        for constraint in self.constraints:
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

            b = model.addVars(4, lb=-gp.GRB.INFINITY, name=[f"b_{i}[0]", f"b_{i}[1]", f"b_{i}[2]", f"b_{i}[3]"])
            model.addConstr(b[0] == dx)
            model.addConstr(b[1] == dy)
            model.addConstr(b[2] == gp.abs_(b[0]))
            model.addConstr(b[3] == gp.abs_(b[1]))
            model.addConstr(b[2] + b[3] >= math.sqrt(2) * constraint.min_separation)

        # Apply optimization to compute new obstacle positions
        model.optimize()

        # Assign computed positions to obstacles
        for i in range(len(self.instance.obstacles)):
            o = self.instance.obstacles[i]
            o.x = new_xs[i].X
            o.y = new_ys[i].X

        return model.ObjVal
