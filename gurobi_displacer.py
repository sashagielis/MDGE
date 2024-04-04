import gurobipy as gp

from constraint import ObstaclePairConstraint
from obstacle_displacer import ObstacleDisplacer
from point import Point
from utils import distance


class GurobiDisplacer(ObstacleDisplacer):
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        super().__init__(instance)

    def execute(self):
        """
        Computes new obstacle positions by minimizing maximum displacement subject to minimum separation constraints.
        Uses Gurobi optimization, which computes a global optimum but is slow (since it solves a non-convex MIQCP).

        :returns: the final value of the objective function
        """
        model = gp.Model("Displace obstacles")

        new_xs, new_ys = [], []
        displacements = []
        for i in range(len(self.instance.obstacles)):
            o = self.instance.obstacles[i]

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

        # Compute minimum separation constraints
        self.compute_constraints()

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

            model.addConstr(dx ** 2 + dy ** 2 >= constraint.min_separation ** 2)

        # Apply optimization to compute new obstacle positions
        model.optimize()

        # Assign computed positions to obstacles
        for i in range(len(self.instance.obstacles)):
            o = self.instance.obstacles[i]
            new_pos = Point(new_xs[i].X, new_ys[i].X)
            o.x = new_pos.x
            o.y = new_pos.y

            # Update displacement cost
            o.displacement = distance(o, new_pos)

        return model.ObjVal
