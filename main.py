from instance import SimplifiedInstance
from obstacle_displacer import Displacer, Objective
from visualizer import visualize

# IPE instances
ipe_instances = {
    'simplified': [
        # 'One line small',
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
    for instance_name in ipe_instances['simplified']:
        file = f"instances/simplified/ipe/{instance_name}.ipe"
        instance = SimplifiedInstance(instance_name, file)
        # print(instance)

        ipe_plot_folder = "plots/simplified/ipe"

        # Visualize input
        input_folder = f"{ipe_plot_folder}/input"
        visualize(instance, input_folder, False, True)

        # Set edge thicknesses
        for edge in instance.graph.edges:
            edge.thickness = edge.weight

        displacement_method = Displacer.DIAMOND
        objective = Objective.TOTAL
        instance.solve(displacement_method, objective)
        # print(instance)

        # Visualize output
        output_folder = f"{ipe_plot_folder}/output/{displacement_method.name} displacer/{objective.name} objective"
        visualize(instance, output_folder, False, True)


if __name__ == "__main__":
    main()
