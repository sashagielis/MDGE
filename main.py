from instance import SimplifiedInstance
from obstacle_displacer import Displacer, Objective
from visualizer import visualize

# IPE instances
instances = [
        'One path small',
        'One path large',
        'Test shortest homotopic edges',
        'Test split event',
        'Test merge event',
        'Test growing events small',
        'Test growing events large',
        'Collinear points',
        'Collinear points 2',
        'Collinear points 3',
        'Simplified single paths',
    ]


def main():
    plot_folder = "plots"
    for instance_name in instances:
        file = f"instances/{instance_name}.ipe"
        instance = SimplifiedInstance(instance_name, file=file)

        instance_folder = f"{plot_folder}/{instance_name}"

        # Visualize input
        visualize(instance, instance_folder, instance_name, False, True, False)

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
        visualize(instance, instance_folder, solution_filename, True, False, False)


if __name__ == "__main__":
    main()
