import math

from functools import partial
from scipy.optimize import minimize

from constraint import ObstaclePairConstraint
from obstacle_displacer import ObstacleDisplacer
from point import Point
from utils import distance


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


class ScipyDisplacer(ObstacleDisplacer):
    def __init__(self, instance):
        """
        :param instance: a SimplifiedInstance object
        """
        super().__init__(instance)

    def execute(self):
        """
        Computes new obstacle positions by minimizing maximum displacement subject to minimum separation constraints.
        Uses SciPy optimization, which is fast but computes a local optimum.

        :returns: the final value of the objective function
        """
        # Compute minimum separation constraints
        self.compute_constraints()

        # Add constraints to model
        cons = []
        for constraint in self.constraints:
            if type(constraint) == ObstaclePairConstraint:
                i = constraint.p1.id
                j = constraint.p2.id

                # Constraints depend on obstacles and total thickness, so we need to evaluate constraints using partial
                # Inspiration: https://stackoverflow.com/questions/27659235/adding-multiple-constraints-to
                # -scipy-minimize-autogenerate-constraint-dictionary-list
                con = partial(obstacle_pair_constraint, index1=i, index2=j, min_sep=constraint.min_separation)
            else:
                i = constraint.p1.id
                v = constraint.p2

                # Constraints depend on pair and total thickness, so we need to evaluate constraints using partial
                # Inspiration: https://stackoverflow.com/questions/27659235/adding-multiple-constraints-to
                # -scipy-minimize-autogenerate-constraint-dictionary-list
                con = partial(obstacle_vertex_constraint, index=i, vertex=v, min_sep=constraint.min_separation)

            cons.append({'type': 'ineq', 'fun': con})

        # Flat list of initial obstacle coordinates
        initial_obstacles = [coordinate for o in self.instance.obstacles for coordinate in [o.x, o.y]]

        # Apply optimization to compute new obstacle positions
        result = minimize(objective, initial_obstacles, args=initial_obstacles, method='SLSQP', constraints=cons)

        # Assign computed positions to obstacles
        flat_new_obstacles = result['x']
        for i in range(0, len(flat_new_obstacles), 2):
            o = self.instance.obstacles[int(i / 2)]
            new_pos = Point(flat_new_obstacles[i], flat_new_obstacles[i + 1])
            o.x = new_pos.x
            o.y = new_pos.y

            # Update displacement cost
            o.displacement = distance(o, new_pos)

        return result['fun']
