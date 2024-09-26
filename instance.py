import math
import os
import time

from delaunay_displacer import DelaunayDisplacer
from growing_algorithm import GrowingAlgorithm
from homotopy import Homotopy
from input_parser import read_ipe_instance
from obstacle import PointObstacle
from obstacle_displacer import Displacer


class Instance:
    """
    An instance of the MDGE problem.
    """
    def __init__(self, instance_name, **kwargs):
        """
        :param instance_name: name of the instance
        :param kwargs: should either contain the field 'file' or both fields 'graph' and 'obstacles'
        - file: path to file containing the instance
        - graph: a Graph object
        - obstacles: list of Obstacle objects
        """
        self.name = instance_name

        x_range, y_range = None, None

        # If instance is given as file
        if 'file' in kwargs:
            file = kwargs['file']

            if os.path.splitext(file)[1] == '.ipe':
                graph, obstacles, x_range, y_range = read_ipe_instance(file)
            else:
                exit()

        # If instance is given as Graph object and list of Obstacle objects
        else:
            graph = kwargs['graph']
            obstacles = kwargs['obstacles']

        self.graph = graph
        self.obstacles = obstacles

        # Set instance dimensions
        if x_range is None or y_range is None:
            # Compute instance dimensions
            self.compute_instance_dimensions()
        else:
            self.min_x, self.max_x = x_range[0], x_range[1]
            self.min_y, self.max_y = y_range[0], y_range[1]

    def update_dimensions(self, point):
        """
        Updates the dimensions of the instance using the point's coordinates.

        :param point: a Point object
        """
        self.min_x = min(self.min_x, point.x)
        self.max_x = max(self.max_x, point.x)
        self.min_y = min(self.min_y, point.y)
        self.max_y = max(self.max_y, point.y)

    def compute_instance_dimensions(self):
        """
        Computes and sets the dimensions of the instance.
        """
        self.min_x, self.max_x = math.inf, -math.inf
        self.min_y, self.max_y = math.inf, -math.inf

        for vertex in self.graph.vertices:
            # Update instance dimensions
            self.update_dimensions(vertex)

        for edge in self.graph.edges:
            for point in edge.path:
                # Update instance dimensions
                self.update_dimensions(point)

        for obstacle in self.obstacles:
            if type(obstacle) == PointObstacle:
                # Update instance dimensions
                self.update_dimensions(obstacle)
            else:
                for point in obstacle.path:
                    # Update instance dimensions
                    self.update_dimensions(point)

    def __str__(self):
        result = self.name + "\n" + str(self.graph) + "\nObstacles:\n"
        for obstacle in self.obstacles:
            result += f"- {obstacle}\n"
        result = result[:-1]

        return result


class SimplifiedInstance(Instance):
    """
    An instance of the simplified MDGE problem.
    """
    def __init__(self, instance_name, **kwargs):
        super().__init__(instance_name, **kwargs)
        self.homotopy = Homotopy(self)

    def solve(self, objective, displacement_method, displace_vertices=False, print_info=False):
        """
        Solves the instance.

        :param displacement_method: a Displacer object specifying the ObstacleDisplacer to displace the obstacles
        :param objective: an Objective object specifying the objective function to be minimized
        :param displace_vertices: whether the vertices may be displaced
        :param print_info: whether the displacement and growing info should be printed
        :returns: the displacement time, growing time and total algorithm time, or None if no solution was found
        """
        start_time_algorithm = time.time()

        # Compute shortest homotopic edges
        self.homotopy.compute_shortest_edges()

        # Choose ObstacleDisplacer
        if displacement_method == Displacer.DELAUNAY:
            displacer = DelaunayDisplacer(self, objective, displace_vertices)
        else:
            raise Exception(f"Displacement method {displacement_method.name} not implemented")

        if print_info:
            print("\nDisplacing the obstacles...")

        # Displace obstacles
        start_time_displacement = time.time()
        displacement_cost = displacer.execute(keep_prev_constraints=True, print_info=print_info)
        end_time_displacement = time.time()

        if displacement_cost is None:
            return None

        if print_info:
            print(f"Displacement cost = {displacement_cost}")

        # Recompute shortest homotopic edges using updated crossing sequences
        self.homotopy.compute_shortest_edges(use_existing_crossing_sequences=True)

        if print_info:
            print("\nGrowing thick edges...")

        # Compute thick homotopic edges using growing algorithm
        start_time_growing = time.time()
        growing_algo = GrowingAlgorithm(self, 0.1)
        growing_algo.compute_thick_edges(print_events=print_info)
        end_time_growing = time.time()

        end_time_algorithm = time.time()

        # Compute timings
        displacement_time = end_time_displacement - start_time_displacement
        growing_time = end_time_growing - start_time_growing
        algorithm_time = end_time_algorithm - start_time_algorithm

        return displacement_time, growing_time, algorithm_time


class GeneralInstance(Instance):
    """
    An instance of the general MDGE problem.
    """
    def __init__(self, instance_name, **kwargs):
        super().__init__(instance_name, **kwargs)

    def solve(self):
        return
