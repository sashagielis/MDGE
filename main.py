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

report_instances = [
    'Problem definition',
]


def solve_instance(instance_name, instance_type):
    file = f"instances/{instance_type}/{instance_name}.ipe"
    instance = SimplifiedInstance(instance_name, file=file)

    plot_folder = f"plots/{instance_type}/{instance_name}"

    # Visualize input
    visualize(instance, plot_folder, instance_name, False, True, False)

    # Set objective and displacement method
    objective = Objective.TOTAL
    displacement_method = Displacer.DELAUNAY

    # Print info
    dashed_line = "-" * 100
    print(f"\n{dashed_line}")
    print(f"Instance: {instance_name}")
    print(f"Objective: minimize {objective.name} displacement")
    print(f"Displacement method: {displacement_method.name}")
    print(f"{dashed_line}")

    # Solve instance
    instance.solve(objective, displacement_method, True)

    # Visualize solution
    solution_filename = f"{instance_name} - {objective.name} - {displacement_method.name}"
    visualize(instance, plot_folder, solution_filename, True, False, False)


def main():
    for instance_name in test_instances:
        solve_instance(instance_name, "test")

    for instance_name in report_instances:
        solve_instance(instance_name, "report")


if __name__ == "__main__":
    main()
