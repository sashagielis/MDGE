from instance import SimplifiedInstance
from obstacle_displacer import Displacer, Objective
from visualizer import visualize

# IPE instances
ipe_instances = [
        # 'One line small',
        # 'One line large',
        # 'Curve edge case',
        # 'Two lines edge case',
        # 'simplified_instance_from_report',
        'Test shortest homotopic edges',
        # 'Test straight line homotopy',
        'Simplified single paths',
        'Collinear straights',
        'Collinear straights 2',
        'Collinear straights 3',
        'Collinear straights 4',
        'Many collisions',
        'Test'
    ]


def main():
    ipe_plot_folder = "plots/simplified/ipe"

    for instance_name in ipe_instances:
        print(f"Instance: {instance_name}")

        file = f"instances/simplified/ipe/{instance_name}.ipe"
        instance = SimplifiedInstance(instance_name, file=file)
        # print(instance)

        instance_folder = f"{ipe_plot_folder}/{instance_name}"

        # Visualize input
        visualize(instance, instance_folder, instance_name, False, True, False)

        # Set edge thicknesses
        for edge in instance.graph.edges:
            edge.thickness = edge.weight

            edge.v1.radius = edge.thickness / 2
            edge.v2.radius = edge.thickness / 2

        objective = Objective.MAX
        displacement_method = Displacer.DIAMOND
        instance.solve(objective, displacement_method)
        # print(instance)

        # Visualize solution
        solution_folder = f"{instance_folder}/solutions"
        solution_filename = f"{instance_name} - {objective.name} - {displacement_method.name}"
        visualize(instance, solution_folder, solution_filename, True, True)


if __name__ == "__main__":
    main()
