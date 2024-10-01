from instance import SimplifiedInstance
from obstacle_displacer import Displacer, Objective
from visualizer import visualize

# IPE instances
test_instances = [
    # 'One path small',
    # 'One path large',
    # 'Test shortest homotopic edges',
    # 'Test split event',
    # 'Test merge event',
    # 'Test growing events small',
    # 'Test growing events large',
    # 'Collinear points',
    # 'Collinear points 2',
    # 'Collinear points 3',
    # 'Simplified single paths',
]

experimental_instances = [
    # 'Feasible',
    # 'Infeasible',
    # 'Instance 1',
    # 'Instance 2',
    # 'Instance 3',
    # 'Instance 4',
    # 'Instance 5',
    # 'Polygonal obstacles 1',
    # 'Polygonal obstacles 2',
    # 'Running time test - 1000 obstacles',
    # 'Weird displacement',
]


def solve_instance(instance_name, instance_type, displace_vertices=False, number_of_runs=1, print_run_info=False,
                   x_range=None, y_range=None):
    """
    Solves the given instance.

    :param instance_name: the name of the instance
    :param instance_type: the type of the instance ("test" or "experiment")
    :param displace_vertices: whether the vertices may be displaced
    :param number_of_runs: the number of times the instance should be run
    :param print_run_info: whether the info while running the algorithm should be printed
    :param x_range: the x-range of the plot
    :param y_range: the y-range of the plot
    """
    file = f"instances/{instance_type}/{instance_name}.ipe"
    instance = SimplifiedInstance(instance_name, file=file)

    plot_folder = f"plots/{instance_type}/{instance_name}"

    # Visualize input
    visualize(instance, plot_folder, instance_name, False, False, False, False)

    # Set objective and displacement method
    objective = Objective.TOTAL
    displacement_method = Displacer.DELAUNAY

    # Print info
    dashed_line = "-" * 100
    print(f"\n{dashed_line}")
    print(f"Instance: {instance_name}")
    print(
        f"Info: |V| = {len(instance.graph.vertices)}, |E| = {len(instance.graph.edges)}, |O| = {len(instance.obstacles)}")
    print(f"Objective: minimize {objective.name} displacement")
    print(f"Displacement method: {displacement_method.name}")
    print(f"Displace vertices: {displace_vertices}")
    print(f"{dashed_line}")

    # Solve instance
    print(f"Number of runs: {number_of_runs}")
    total_displacement_time, total_growing_time, total_algorithm_time = 0, 0, 0
    for i in range(number_of_runs):
        instance = SimplifiedInstance(instance_name, file=file)
        timings = instance.solve(objective, displacement_method, displace_vertices, print_run_info)

        # If no solution was found, return
        if timings is None:
            return

        total_displacement_time += timings[0]
        total_growing_time += timings[1]
        total_algorithm_time += timings[2]

    # Print timings
    print(f"\nAverage displacement time: {total_displacement_time / number_of_runs} seconds")
    print(f"Average growing time: {total_growing_time / number_of_runs} seconds")
    print(f"Average algorithm time: {total_algorithm_time / number_of_runs} seconds")

    # Visualize solution
    solution_filename = f"{instance_name} - {objective.name} - {displacement_method.name}"
    visualize(instance, plot_folder, solution_filename, True, False, False, True, x_range, y_range)


def main():
    for instance in test_instances:
        solve_instance(instance, "test")

    for instance in experimental_instances:
        solve_instance(instance, "experiment", False, 1, True)

    # For figures of instance 2:
    # Moving vertices: x_range=(-120, 720), y_range=(390, 860)
    # Max thickness: x_range=(-160, 755), y_range=(375, 885)


if __name__ == "__main__":
    main()
