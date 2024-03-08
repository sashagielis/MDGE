from input_parser import read_ipe_instance
from simplified_solver import SimplifiedSolver


class Instance:
    """
    An instance of the MDGD problem.
    """
    def __init__(self, instance_name, file_type):
        if file_type == 'ipe':
            graph, obstacles = read_ipe_instance(f'{instance_name}.ipe')
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
    def __init__(self, instance_name, file_type):
        super().__init__(instance_name, file_type)

    def solve(self):
        solver = SimplifiedSolver(self)
        solver.solve()


class GeneralInstance(Instance):
    """
    An instance of the general MDGD problem.
    """
    def __init__(self, instance_name, file_type):
        super().__init__(instance_name, file_type)

    def solve(self):
        # TODO
        return
