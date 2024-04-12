from instance import SimplifiedInstance
from visualizer import visualize

# IPE instances
ipe_instances = {
    'simplified': [
        'One line small',
        'One line large',
        'Two lines',
        'Curve edge case',
        'Two lines edge case',
        'simplified_instance_from_report',
    ],
    'general': [
        'instance_from_report',
    ]
}


def main():
    for instance_name in ipe_instances['simplified']:
        file = f"instances/simplified/ipe/{instance_name}.ipe"
        instance = SimplifiedInstance(instance_name, file)
        print(instance)

        ipe_plot_folder = "plots/simplified/ipe"

        # Visualize input
        input_folder = f"{ipe_plot_folder}/input"
        visualize(instance, input_folder, False, False)

        # Test visualization large vertices and thick edges
        # for vertex in instance.graph.vertices:
        #     vertex.diameter = 100
        for edge in instance.graph.edges:
            edge.thickness = edge.weight

        displacement_method = 'diamond'
        instance.solve(displacement_method)
        print(instance)

        # Visualize output
        output_folder = f"{ipe_plot_folder}/output/{displacement_method} displacement"
        visualize(instance, output_folder, True, False)


if __name__ == "__main__":
    main()
