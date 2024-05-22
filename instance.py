import os

from diamond_displacer import DiamondDisplacer
from homotopy import Homotopy
from input_parser import read_ipe_instance
from obstacle_displacer import Displacer
from optimal_displacer import OptimalDisplacer
from scipy_displacer import ScipyDisplacer


class Instance:
    """
    An instance of the MDGD problem.
    """
    def __init__(self, instance_name, file):
        if os.path.splitext(file)[1] == '.ipe':
            graph, obstacles, x_range, y_range = read_ipe_instance(file)
        else:
            exit()

        self.name = instance_name
        self.graph = graph
        self.obstacles = obstacles
        self.min_x = x_range[0]
        self.max_x = x_range[1]
        self.min_y = y_range[0]
        self.max_y = y_range[1]

    def __str__(self):
        result = self.name + "\n" + str(self.graph) + "\nObstacles:\n"
        for obstacle in self.obstacles:
            result += f"- {obstacle}\n"

        return result


class SimplifiedInstance(Instance):
    """
    An instance of the simplified MDGD problem.
    """
    def __init__(self, instance_name, file):
        super().__init__(instance_name, file)

    def solve(self, objective, displacement_method):
        """
        Solves the instance.

        :param displacement_method: a Displacer object specifying the ObstacleDisplacer to displace the obstacles
        :param objective: an Objective object specifying the objective function to be minimized
        """
        # Compute shortest homotopic edges
        homotopy = Homotopy(self)
        homotopy.compute_shortest_edges()

        # Choose ObstacleDisplacer
        if displacement_method == Displacer.SCIPY:
            displacer = ScipyDisplacer(self, objective)
        elif displacement_method == Displacer.OPTIMAL:
            displacer = OptimalDisplacer(self, objective)
        elif displacement_method == Displacer.DIAMOND:
            displacer = DiamondDisplacer(self, objective)
        else:
            raise Exception(f"Displacement method {displacement_method.name} not implemented")

        # Displace the obstacles
        displacement_cost = displacer.execute()
        print(f"Displacement cost = {displacement_cost}")

        # TODO: Recompute shortest homotopic edges

        # TODO: compute thick homotopic edges using growing algorithm


class GeneralInstance(Instance):
    """
    An instance of the general MDGD problem.
    """
    def __init__(self, instance_name, file):
        super().__init__(instance_name, file)

    def solve(self):
        # TODO
        return
