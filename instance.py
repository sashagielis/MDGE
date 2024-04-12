import os

from diamond_displacer import DiamondDisplacer
from gurobi_displacer import GurobiDisplacer
from iterative_displacer import IterativeDisplacer
from input_parser import read_ipe_instance
from scipy_displacer import ScipyDisplacer
from sweep_displacer import SweepDisplacer


class Instance:
    """
    An instance of the MDGD problem.
    """
    def __init__(self, instance_name, file):
        if os.path.splitext(file)[1] == '.ipe':
            graph, obstacles = read_ipe_instance(file)
        else:
            exit()

        self.name = instance_name
        self.graph = graph
        self.obstacles = obstacles

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

    def solve(self, displacement_method):
        # TODO: compute shortest homotopic edges

        # Choose displacement method
        if displacement_method == 'scipy':
            displacer = ScipyDisplacer(self)
        elif displacement_method == 'gurobi':
            displacer = GurobiDisplacer(self)
        elif displacement_method == 'iterative':
            displacer = IterativeDisplacer(self)
        elif displacement_method == 'sweep':
            displacer = SweepDisplacer(self)
        elif displacement_method == 'diamond':
            displacer = DiamondDisplacer(self)
        else:
            exit()

        # Displace the obstacles
        displacement_cost = displacer.execute()
        print(f"Displacement cost = {displacement_cost}")

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
