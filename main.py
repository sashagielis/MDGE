from instance import SimplifiedInstance
from obstacle_displacer import Displacer, Objective
from visualizer import visualize

# IPE instances
ipe_instances = {
    'simplified': [
        'One line small',
        # 'One line large',
        # 'Curve edge case',
        # 'Two lines edge case',
        # 'simplified_instance_from_report',
        # 'Test shortest homotopic edges',
        # 'Test straight line homotopy'
    ],
    'general': [
        # 'instance_from_report',
    ]
}


def main():
    ipe_plot_folder = "plots/simplified/ipe"

    for instance_name in ipe_instances['simplified']:
        file = f"instances/simplified/ipe/{instance_name}.ipe"
        instance = SimplifiedInstance(instance_name, file=file)
        # print(instance)

        instance_folder = f"{ipe_plot_folder}/{instance_name}"

        # Visualize input
        visualize(instance, instance_folder, instance_name, False, True)

        # Set edge thicknesses
        for edge in instance.graph.edges:
            edge.thickness = edge.weight

        objective = Objective.MAX
        displacement_method = Displacer.DIAMOND
        instance.solve(objective, displacement_method)
        # print(instance)

        # Visualize solution
        solution_folder = f"{instance_folder}/solutions"
        solution_filename = f"{instance_name} - {objective.name} - {displacement_method.name}"
        visualize(instance, solution_folder, solution_filename, False, True, True)


if __name__ == "__main__":
    main()
